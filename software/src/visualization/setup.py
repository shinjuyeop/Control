from glob import glob
from setuptools import find_packages, setup

package_name = 'visualization'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='shinjuyeop',
    maintainer_email='shinjp02@gmail.com',
    description='Read-only vehicle monitoring and Foxglove demo helpers.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vehicle_dashboard = visualization.vehicle_dashboard:main',
            'test_publisher = visualization.test_publisher:main',
            'gps_waypoint_publisher = visualization.gps_waypoint_publisher:main',
        ],
    },
)
