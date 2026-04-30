from setuptools import find_packages, setup

package_name = 'throttle_serial_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='shinjuyeop',
    maintainer_email='shinjuyeop@todo.todo',
    description='Read Arduino serial packets and publish ThrottleData',
    license='Apache-2.0',

    entry_points={
        'console_scripts': [
            # name on CLI      =  package.module:function
            'throttle_serial_node = throttle_serial_py.throttle_node:main',
            'rpm_mux              = throttle_serial_py.rpm_mux:main',
        ],
    },
)

