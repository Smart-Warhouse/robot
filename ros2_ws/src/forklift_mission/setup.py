from setuptools import find_packages, setup

package_name = "forklift_mission"

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
    description="Behavior-Tree mission executor + MQTT fleet client implementing docs/03-mqtt-contract.md (NV-09, NV-10, SF-05).",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "mission_node = forklift_mission.mission_node:main",
            "lift_server = forklift_mission.lift_server_node:main",
        ],
    },
)
