from glob import glob

from setuptools import setup

package_name = "go2_lidar_safety"

setup(
    name=package_name,
    version="1.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/launch", glob("launch/*.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Yusuf Guenena",
    maintainer_email="yusuf.a.guenena@gmail.com",
    description="LiDAR hazard source for the safety arbiter (camera-free hazard context).",
    license="MIT",
    entry_points={
        "console_scripts": [
            "lidar_hazard_node = go2_lidar_safety.lidar_hazard_node:main",
        ],
    },
)
