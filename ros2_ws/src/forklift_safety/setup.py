from setuptools import find_packages, setup

package_name = "forklift_safety"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Ahmed Imad Tarq",
    maintainer_email="ahmed.emadtarq@gmail.com",
    description="Safety supervisor: clamps every command source (Nav2, RL, gamepad) before the ESP32 (SF-03, SF-05..SF-08).",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "safety_supervisor = forklift_safety.supervisor_node:main",
            "fork_state_bridge = forklift_safety.fork_state_bridge:main",
        ],
    },
)
