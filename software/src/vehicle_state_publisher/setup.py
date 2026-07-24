from setuptools import setup

package_name = "vehicle_state_publisher"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="kangmin",
    maintainer_email="kai.driverless@gmail.com",
    description="서브시스템 status 3종을 합성해 /vehicle/state를 발행",
    license="TODO",
    entry_points={
        "console_scripts": [
            "vehicle_state_publisher_node = "
            "vehicle_state_publisher.vehicle_state_publisher_node:main",
        ],
    },
)
