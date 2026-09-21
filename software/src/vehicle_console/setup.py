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
        ("share/" + package_name, [
            "package.xml", "README.md", "HIL_BACKEND.md", "HIL_WEB.md",
            "CONSOLE_PARITY.md", "CONSOLE_VALIDATION.md", "requirements-web.txt",
        ]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/config/scenarios", glob("config/scenarios/*.yaml")),
        ("share/" + package_name + "/vcan_sim", glob("vehicle_console/hil/vcan_sim/*.md")),
        ("share/" + package_name + "/dashboard", glob("dashboard/*.so")),
    ] + web_files,
    install_requires=["setuptools", "fastapi>=0.115,<1", "uvicorn>=0.30,<1", "pyserial", "PyYAML", "Pillow>=9,<13"],
    zip_safe=True,
    maintainer="shinjuyeop",
    maintainer_email="shinjp02@gmail.com",
    description="KAI Vehicle Console: vehicle monitoring, steering fault acknowledgment and controlled HIL validation.",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={"console_scripts": [
        "vehicle_console = vehicle_console.vehicle:main",
        "hil_web_bridge = vehicle_console.hil.hil_web_bridge:main",
        "stm_test_bridge = vehicle_console.hil.stm_test_bridge:main",
        "hil_command_generator = vehicle_console.hil.hil_command_generator:main",
        "hil_scenario_runner = vehicle_console.hil.hil_scenario_runner:main",
        "vcan_sim = vehicle_console.hil.vcan_sim.vcan_sim:main",
    ]},
)
