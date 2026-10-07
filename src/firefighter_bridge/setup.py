from glob import glob
import os

from setuptools import find_packages, setup

package_name = "firefighter_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        (os.path.join("share", package_name), ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools", "pyserial"],
    zip_safe=True,
    maintainer="thechaelice",
    maintainer_email="thechalice22@gmail.com",
    description="UART bridge between the mobility ESP32 and the ROS 2 graph.",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "bridge_node = firefighter_bridge.bridge_node:main",
        ],
    },
)
