from setuptools import setup, find_packages
from glob import glob
import os

package_name = "behaviour"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(include=["behaviour", "behaviour.*"]),
    data_files=[
        ("share/ament_index/resource_index/packages",
            ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="vagrant",
    maintainer_email="na@example.com",
    description="Package for SOP-Robot Behaviour",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "face_track = behaviour.face_track.node.face_track_node:main",
            "head_movement = behaviour.movement.node.head_movement_node:main",
        ],
    },
)
