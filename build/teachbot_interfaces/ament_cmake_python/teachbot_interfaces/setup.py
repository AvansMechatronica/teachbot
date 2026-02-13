from setuptools import find_packages
from setuptools import setup

setup(
    name='teachbot_interfaces',
    version='0.0.1',
    packages=find_packages(
        include=('teachbot_interfaces', 'teachbot_interfaces.*')),
)
