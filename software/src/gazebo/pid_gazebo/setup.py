from setuptools import setup
from glob import glob

package_name = 'pid_gazebo'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml']),
        (f'share/{package_name}/launch', glob('launch/*.py')),
        (f'share/{package_name}/config', glob('config/*.csv')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sanghyeok',
    maintainer_email='seosanghyuk0@gmail.com',
    description='Dual-channel PID controller for Gazebo Differential Drive',
    license='MIT',
    entry_points={
        'console_scripts': [
            'pid_gazebo_node = pid_gazebo.pid_gazebo_node:main',
            'setpoint_publisher = pid_gazebo.setpoint_publisher:main',
        ],
    },
)
