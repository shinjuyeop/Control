from glob import glob

from setuptools import find_packages, setup


package_name = "vehicle_test_scenarios"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml", "README.md", "auto_sysid.py"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools", "rich"],
    zip_safe=True,
    maintainer="shinjuyeop",
    maintainer_email="shinjp02@gmail.com",
    description=(
        "Controlled Planning command scenarios for real-vehicle tests."
    ),
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "vehicle_test_scenario_runner = "
            "vehicle_test_scenarios.entrypoint:main",
            "vehicle_test_scenario_node = "
            "vehicle_test_scenarios.scenario_runner:main",
        ],
    },
)
