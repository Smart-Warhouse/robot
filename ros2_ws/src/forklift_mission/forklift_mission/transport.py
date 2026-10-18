"""MQTT transport abstraction for the fleet client (injectable: paho in production, FakeBroker in tests).

The fleet client never imports paho itself; it talks to a `Transport`:
    configure(client_id, username, password, will) -> connect(host, port, keepalive) -> publish / subscribe
and receives `on_connect()`, `on_disconnect()`, `on_message(topic, payload)` callbacks. Callbacks may arrive on
another thread (paho's network loop): the ROS node funnels them into its executor; the core is not thread-safe.
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass


@dataclass(frozen=True)
class Message:
    topic: str
    payload: str
    qos: int = 0
    retain: bool = False


def topic_matches(sub: str, topic: str) -> bool:
    """MQTT topic filter match with `+` (one level) and `#` (rest)."""
    s, t = sub.split("/"), topic.split("/")
    for i, part in enumerate(s):
        if part == "#":
            return True
        if i >= len(t):
            return False
        if part != "+" and part != t[i]:
            return False
    return len(s) == len(t)


class Transport:
    """Base class; subclasses implement the five methods. Callbacks are assigned by the FleetClient."""

    on_connect: Callable[[], None] = staticmethod(lambda: None)
    on_disconnect: Callable[[], None] = staticmethod(lambda: None)
    on_message: Callable[[str, bytes], None] = staticmethod(lambda t, p: None)

    def configure(self, client_id: str, username: str | None, password: str | None, will: Message) -> None:
        raise NotImplementedError

    def connect(self, host: str, port: int, keepalive: int) -> None:
        raise NotImplementedError

    def publish(self, msg: Message) -> bool:
        raise NotImplementedError

    def subscribe(self, topics: list[tuple[str, int]]) -> None:
        raise NotImplementedError

    def disconnect(self) -> None:
        raise NotImplementedError


class PahoTransport(Transport):
    """paho-mqtt 2.x (pip) or 1.6 (Ubuntu 22.04 apt python3-paho-mqtt). Runs paho's network thread."""

    def __init__(self) -> None:
        import paho.mqtt.client as mqtt  # imported here so the core stays importable without paho

        self._mqtt = mqtt
        self._v2 = hasattr(mqtt, "CallbackAPIVersion")
        self._c = None

    def configure(self, client_id: str, username: str | None, password: str | None, will: Message) -> None:
        mqtt = self._mqtt
        if self._v2:
            self._c = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=client_id, clean_session=True)
            self._c.on_connect = self._on_connect_v2
            self._c.on_disconnect = self._on_disconnect_v2
        else:
            self._c = mqtt.Client(client_id=client_id, clean_session=True)
            self._c.on_connect = self._on_connect_v1
            self._c.on_disconnect = self._on_disconnect_v1
        if username or password:
            self._c.username_pw_set(username, password)
        self._c.will_set(will.topic, will.payload, qos=will.qos, retain=will.retain)
        self._c.reconnect_delay_set(1, 5)
        self._c.on_message = lambda c, u, m: self.on_message(m.topic, m.payload)

    def _on_connect_v2(self, client, userdata, flags, reason_code, properties) -> None:
        if not reason_code.is_failure:
            self.on_connect()

    def _on_disconnect_v2(self, client, userdata, flags, reason_code, properties) -> None:
        self.on_disconnect()

    def _on_connect_v1(self, client, userdata, flags, rc) -> None:
        if rc == 0:
            self.on_connect()

    def _on_disconnect_v1(self, client, userdata, rc) -> None:
        self.on_disconnect()

    def connect(self, host: str, port: int, keepalive: int) -> None:
        self._c.connect_async(host, port, keepalive=keepalive)
        self._c.loop_start()

    def publish(self, msg: Message) -> bool:
        info = self._c.publish(msg.topic, msg.payload, qos=msg.qos, retain=msg.retain)
        return info.rc == self._mqtt.MQTT_ERR_SUCCESS

    def subscribe(self, topics: list[tuple[str, int]]) -> None:
        self._c.subscribe(topics)

    def disconnect(self) -> None:
        if self._c is not None:
            self._c.disconnect()
            self._c.loop_stop()


class ThreadSafeTransport(Transport):
    """Wraps a transport whose callbacks fire on a foreign thread (paho's loop): callbacks are queued and
    delivered by `drain()` from the owner's thread (the ROS timer), so the fleet client stays single-threaded."""

    def __init__(self, inner: Transport) -> None:
        import queue

        self.inner = inner
        self._q: "queue.SimpleQueue[tuple]" = queue.SimpleQueue()
        inner.on_connect = lambda: self._q.put(("connect",))
        inner.on_disconnect = lambda: self._q.put(("disconnect",))
        inner.on_message = lambda topic, payload: self._q.put(("message", topic, payload))

    def drain(self, max_items: int = 1000) -> int:
        import queue

        n = 0
        while n < max_items:
            try:
                item = self._q.get_nowait()
            except queue.Empty:
                break
            n += 1
            if item[0] == "connect":
                self.on_connect()
            elif item[0] == "disconnect":
                self.on_disconnect()
            else:
                self.on_message(item[1], item[2])
        return n

    def configure(self, client_id, username, password, will) -> None:
        self.inner.configure(client_id, username, password, will)

    def connect(self, host, port, keepalive) -> None:
        self.inner.connect(host, port, keepalive)

    def publish(self, msg: Message) -> bool:
        return self.inner.publish(msg)

    def subscribe(self, topics) -> None:
        self.inner.subscribe(topics)

    def disconnect(self) -> None:
        self.inner.disconnect()
