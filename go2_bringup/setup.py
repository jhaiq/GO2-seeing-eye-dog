from glob import glob

from setuptools import find_packages, setup

package_name = 'go2_bringup'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install every launch file and every config file by glob. The
        # previous per-file list silently omitted anything newly added, which
        # is how a launch file ends up "existing" but not being installed.
        ('share/' + package_name + '/launch', glob('launch/*.launch.py') + glob('launch/*_launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Yusuf Guenena',
    maintainer_email='yusuf.a.guenena@gmail.com',
    description='Canonical bringup and motion-authority launch graph for the GO2 seeing-eye dog.',
    license='MIT',
    entry_points={
        'console_scripts': [],
    },
)
