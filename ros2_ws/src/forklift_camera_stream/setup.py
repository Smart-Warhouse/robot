from setuptools import find_packages, setup

package_name = "forklift_camera_stream"

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
    description="Low-resolution MJPEG stream of the fork camera for manual control, served only in manual mode (UI-08).",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "camera_stream = forklift_camera_stream.camera_stream_node:main",
            "camera_stream_standalone = forklift_camera_stream.standalone:main",
        ],
    },
)
