"""In-process MQTT broker for tests and dry runs: wildcards, retained messages, Last Will on an abrupt drop.

Delivery is queued and happens on `pump()`, so a subscriber answering inside its callback (the fake fleet
server granting a lane) cannot re-enter the publisher. QoS is recorded but delivery is always exactly once.
"""

from __future__ import annotations

import json
from collections import deque

from .transport import Message, Transport, topic_matches


class FakeBroker:
    def __init__(self) -> None:
        self.clients: list[FakeTransport] = []
        self.retained: dict[str, Message] = {}
        self.queue: deque[tuple[FakeTransport, Message]] = deque()
        self.log: list[tuple[str, Message]] = []  # (sender client id, message) — everything published

    # --------------------------------------------------------------- client side
    def _connect(self, c: "FakeTransport") -> None:
        if c not in self.clients:
            self.clients.append(c)
        c.connected = True
        c.on_connect()

    def _publish(self, sender: "FakeTransport | None", msg: Message) -> None:
        self.log.append((sender.client_id if sender else "-", msg))
        if msg.retain:
            if msg.payload:
                self.retained[msg.topic] = msg
            else:
                self.retained.pop(msg.topic, None)
        for c in self.clients:
            if c.connected and any(topic_matches(s, msg.topic) for s in c.subs):
                self.queue.append((c, msg))

    def _subscribe(self, c: "FakeTransport", topics: list[tuple[str, int]]) -> None:
        for t, _ in topics:
            c.subs.add(t)
            for r in self.retained.values():
                if topic_matches(t, r.topic):
                    self.queue.append((c, r))

    # --------------------------------------------------------------- test controls
    def pump(self, max_messages: int = 10_000) -> int:
        n = 0
        while self.queue and n < max_messages:
            c, msg = self.queue.popleft()
            if c.connected:
                c.on_message(msg.topic, msg.payload.encode())
            n += 1
        return n

    def drop(self, c: "FakeTransport") -> None:
        """Abrupt loss (Wi-Fi off, process killed): the broker publishes the client's Last Will."""
        if not c.connected:
            return
        c.connected = False
        c.on_disconnect()
        if c.will is not None:
            self._publish(None, c.will)

    def restore(self, c: "FakeTransport") -> None:
        """Reconnect after a drop (paho reconnects automatically); subscriptions are clean (clean_session)."""
        c.subs.clear()
        self._connect(c)

    def published(self, topic_filter: str = "#") -> list[dict]:
        out = []
        for _, m in self.log:
            if topic_matches(topic_filter, m.topic):
                try:
                    body = json.loads(m.payload)
                except json.JSONDecodeError:
                    body = {"_raw": m.payload}
                out.append({"topic": m.topic, "qos": m.qos, "retain": m.retain, **body})
        return out

    def inject(self, topic: str, body: dict, qos: int = 1, retain: bool = False) -> None:
        """Publish as if from another client (e.g. the fleet server)."""
        self._publish(None, Message(topic, json.dumps(body), qos, retain))


class FakeTransport(Transport):
    def __init__(self, broker: FakeBroker) -> None:
        self.broker = broker
        self.client_id = ""
        self.username: str | None = None
        self.password: str | None = None
        self.will: Message | None = None
        self.subs: set[str] = set()
        self.connected = False
        self.keepalive = 0

    def configure(self, client_id: str, username: str | None, password: str | None, will: Message) -> None:
        self.client_id, self.username, self.password, self.will = client_id, username, password, will

    def connect(self, host: str, port: int, keepalive: int) -> None:
        self.keepalive = keepalive
        self.broker._connect(self)

    def publish(self, msg: Message) -> bool:
        if not self.connected:
            return False
        self.broker._publish(self, msg)
        return True

    def subscribe(self, topics: list[tuple[str, int]]) -> None:
        self.broker._subscribe(self, topics)

    def disconnect(self) -> None:
        """Clean disconnect: no Last Will."""
        if self.connected:
            self.connected = False
            self.on_disconnect()
