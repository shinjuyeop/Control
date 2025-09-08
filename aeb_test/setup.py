from setuptools import setup

package_name = 'aeb_test'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='shinjuyeop',
    maintainer_email='shinjuyeop@example.com',
    description='Console tool to publish 0/1 to /aeb',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'aeb_pub = aeb_test.aeb_pub:main',
        ],
    },
)

