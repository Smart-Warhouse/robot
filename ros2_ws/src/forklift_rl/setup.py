from setuptools import find_packages, setup

package_name = "forklift_rl"

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
    description="Docking policy runner: 18-dim observation, ONNX/TensorRT policy at 20 Hz, Dock action server; output through the safety supervisor (AI-05, AI-06).",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "dock_server = forklift_rl.dock_node:main",
        ],
    },
)
