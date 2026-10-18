from setuptools import find_packages, setup

package_name = "forklift_perception"

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
    description="ArUco DICT_4X4_50 pallet/location detection, solvePnP pallet_pose with 5-sample average (PC-03..PC-05), initial pose from the charger marker (NV-04).",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "aruco_node = forklift_perception.aruco_node:main",
            "initial_pose_node = forklift_perception.initial_pose_node:main",
        ],
    },
)
