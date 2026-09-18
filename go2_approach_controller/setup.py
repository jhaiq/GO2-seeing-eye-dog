from setuptools import setup

package_name = "go2_approach_controller"

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
    description="Staged candidate-motion controller for the GO2 stack.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "approach_controller_node = go2_approach_controller.approach_controller_node:main",
            "candidate_stamper_node = go2_approach_controller.candidate_stamper_node:main",
        ],
    },
)
