from setuptools import setup

package_name = "vehicle_state_publisher"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="kangmin",
    maintainer_email="kai.driverless@gmail.com",
    description="서브시스템 status를 차량 상태, wheel twist와 diagnostics로 변환",
    license="TODO",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "vehicle_state_publisher_node = "
            "vehicle_state_publisher.vehicle_state_publisher_node:main",
            "diagnostics_publisher = "
            "vehicle_state_publisher.diagnostics_publisher:main",
            "auto_dashboard_summary = "
            "vehicle_state_publisher.auto_dashboard_summary:main",
        ],
    },
)
