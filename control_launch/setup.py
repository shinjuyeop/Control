from setuptools import setup
from glob import glob
import os

package_name = 'control_launch'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        # ① 패키지 인덱스 marker (중요!)
        (os.path.join('share', 'ament_index', 'resource_index', 'packages'),
         [os.path.join('resource', package_name)]),

        # ② package.xml
        (os.path.join('share', package_name), ['package.xml']),

        # ③ launch 파일들
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='subin',
    maintainer_email='ju27586@konkuk.ac.kr',
    description='Launch files for control stack',
    license='MIT',
)