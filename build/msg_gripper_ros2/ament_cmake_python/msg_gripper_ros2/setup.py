from setuptools import find_packages
from setuptools import setup

setup(
    name='msg_gripper_ros2',
    version='1.0.0',
    packages=find_packages(
        include=('msg_gripper_ros2', 'msg_gripper_ros2.*')),
)
