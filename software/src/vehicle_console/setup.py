from glob import glob
import os

from setuptools import find_packages, setup

package_name = "vehicle_console"
web_files = [
    (os.path.join("share", package_name, "web", os.path.relpath(root, "web/dist")),
     [os.path.join(root, name) for name in names
      if os.path.isfile(os.path.join(root, name))])
    for root, _, names in os.walk("web/dist") if names
]

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md", "requirements-web.txt"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ] + web_files,
    install_requires=["setuptools", "fastapi>=0.115,<1", "uvicorn>=0.30,<1"],
    zip_safe=True,
    maintainer="shinjuyeop",
    maintainer_email="shinjp02@gmail.com",
    description="KAI Vehicle Console: shared browser telemetry and read-only vehicle monitoring.",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={"console_scripts": ["vehicle_console = vehicle_console.vehicle:main"]},
)
