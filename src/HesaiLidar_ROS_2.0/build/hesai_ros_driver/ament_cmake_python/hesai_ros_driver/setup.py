from setuptools import find_packages
from setuptools import setup

setup(
    name='hesai_ros_driver',
    version='1.5.0',
    packages=find_packages(
        include=('hesai_ros_driver', 'hesai_ros_driver.*')),
)
