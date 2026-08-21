from setuptools import find_packages, setup


package_name = "control_gui_common"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml", "README.md"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="shinjuyeop",
    maintainer_email="shinjp02@gmail.com",
    description="Shared runtime helpers for Control Qt applications.",
    license="TODO: License declaration",
    tests_require=["pytest"],
)
