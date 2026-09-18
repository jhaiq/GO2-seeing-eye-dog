from glob import glob

from setuptools import setup

package_name = "go2_sim"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.xml")),
        ("share/" + package_name + "/launch", glob("launch/*.py")),
        ("share/" + package_name + "/worlds", glob("worlds/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Yusuf Guenena",
    maintainer_email="yusuf.a.guenena@gmail.com",
    description="Kinematic GO2 simulator emulating the Sport API, odometry and L1 lidar topics.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "closed_loop_trials = go2_sim.closed_loop_trials:main",
            "go2_kinematic_sim_node = go2_sim.sim_node:main",
        ],
    },
)
