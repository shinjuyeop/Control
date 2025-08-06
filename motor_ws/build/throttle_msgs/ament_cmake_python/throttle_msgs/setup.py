from setuptools import find_packages
from setuptools import setup

setup(
    name='throttle_msgs',
    version='0.0.0',
    packages=find_packages(
        include=('throttle_msgs', 'throttle_msgs.*')),
)
