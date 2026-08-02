import os

from setuptools import find_packages, setup

package_name = "arm_teleop"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", ["config/teleop_twist_joy.yaml"]),
        ("share/" + package_name + "/launch",
         ["launch/teleop_joy.launch.py",
          "launch/gripper_action_server.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="BabyROS",
    maintainer_email="dev@example.com",
    description="Gamepad teleop for the BabyROS arm_6dof",
    license="MIT",
    entry_points={
        "console_scripts": [
            "teleop_buttons = arm_teleop.teleop_buttons:main",
            "gripper_action_server = arm_teleop.gripper_action_server:main",
        ],
    },
)
