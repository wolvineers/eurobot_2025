from setuptools import find_packages
from setuptools import setup

setup(
    name='camera_robot',
    version='0.0.0',
    packages=find_packages(
        include=('camera_robot', 'camera_robot.*')),
)
