from setuptools import setup

package_name = 'msg_gripper_ros2'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='UCLA Biomechatronics Lab',
    maintainer_email='benforbes@ucla.edu',
    description='ROS2 driver for the MSG gripper with CAN bus control',
    license='MIT',
)
