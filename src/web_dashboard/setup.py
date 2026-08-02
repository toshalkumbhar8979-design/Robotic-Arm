import os

from setuptools import find_packages, setup

package_name = "web_dashboard"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/web.launch.py"]),
        ("share/" + package_name + "/web",
         ["web/index.html",
          "web/css/style.css",
          "web/js/dashboard.js"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="BabyROS",
    maintainer_email="dev@example.com",
    description="Web digital twin dashboard for the BabyROS arm_6dof",
    license="MIT",
    entry_points={
        "console_scripts": [
            "serve_dashboard = web_dashboard.serve_dashboard:main",
        ],
    },
)
