from setuptools import find_packages
from setuptools import setup

setup(
    name='turtlebot3_camera_service_interfaces',
    version='0.0.0',
    packages=find_packages(
        include=('turtlebot3_camera_service_interfaces', 'turtlebot3_camera_service_interfaces.*')),
)
