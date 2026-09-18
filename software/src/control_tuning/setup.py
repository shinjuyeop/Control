from glob import glob

from setuptools import find_packages, setup

setup(
    name='control_tuning', version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/control_tuning']),
        ('share/control_tuning', ['package.xml', 'README.md']),
        ('share/control_tuning/config', glob('config/*.yaml')),
        ('share/control_tuning/config/control_tuning.presets', glob('config/control_tuning.presets/*.yaml')),
    ],
    install_requires=['setuptools', 'rich', 'PyYAML'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='shinjuyeop', maintainer_email='shinjp02@gmail.com',
    description='Shared vehicle control settings TUI.',
    license='TODO: License declaration',
    entry_points={'console_scripts': [
        'control_tuning_tui = control_tuning.tui:main',
    ]},
)
