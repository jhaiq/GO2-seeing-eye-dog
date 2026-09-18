from glob import glob

from setuptools import setup

package_name = "go2_localization"

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
    description="GO2 odometry/LiDAR relay: robot-clock correction, TF, localization validity.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "go2_state_relay_node = go2_localization.state_relay_node:main",
            "calibrate_lidar = go2_localization.calibrate_lidar:main",
            "nav_tf_watchdog = go2_localization.nav_tf_watchdog:main",
        ],
    },
)
