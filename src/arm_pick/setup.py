import os

from setuptools import find_packages, setup

package_name = "arm_pick"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", ["config/pick_params.yaml"]),
        ("share/" + package_name + "/launch", ["launch/arm_pick.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="BabyROS",
    maintainer_email="dev@example.com",
    description="Autonomous pick-and-place for the BabyROS arm_6dof",
    license="MIT",
    entry_points={
        "console_scripts": [
            "pick_node = arm_pick.pick_node:main",
        ],
    },
)
