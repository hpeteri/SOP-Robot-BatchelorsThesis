from setuptools import setup
from glob import glob
import os

package_name = "perception"

setup(
    name=package_name,
    version="0.0.0",
    packages=["perception"],
    data_files=[
        ("share/ament_index/resource_index/packages",
            ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "predictors"), glob("predictors/*")),
        (os.path.join("share", package_name, "launch"), glob("launch/*")),
        (os.path.join("share", package_name, "models"), glob("models/*"))
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="vagrant",
    maintainer_email="vagrant@todo.todo",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "face_detection = perception.face_detection.face_tracker_node:main",
        ],
    },
)
