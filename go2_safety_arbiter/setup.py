from setuptools import setup

package_name = "go2_safety_arbiter"

setup(
    name=package_name,
    version="1.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Yusuf Guenena",
    maintainer_email="yusuf.a.guenena@gmail.com",
    description="Deterministic, fail-closed safety arbiter for the GO2 stack.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "safety_arbiter_node = go2_safety_arbiter.safety_arbiter_node:main",
        ],
    },
)
