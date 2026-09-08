from glob import glob
import os

from setuptools import find_packages, setup


package_name = "kaiev26_hil"


def data_files(pattern):
    return [path for path in glob(pattern, recursive=True) if os.path.isfile(path)]


web_files = [
    (os.path.join("share", package_name, "web", os.path.relpath(root, "web/dist")),
     [os.path.join(root, name) for name in names
      if os.path.isfile(os.path.join(root, name))])
    for root, _, names in os.walk("web/dist") if names
]


dashboard_files = [("share/" + package_name + "/dashboard", data_files("dashboard/*.so"))]


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md", "WEB_CONSOLE.md", "requirements-web.txt"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/config/scenarios", glob("config/scenarios/*.yaml")),
        (
            "share/" + package_name + "/vcan_sim",
            data_files("kaiev26_hil/vcan_sim/*.md"),
        ),
    ] + web_files + dashboard_files,
    install_requires=["setuptools", "pyserial", "PyYAML"],
    zip_safe=True,
    maintainer="shinjuyeop",
    maintainer_email="shinjp02@gmail.com",
    description="PC-side ROS 2 tools for STM_TEST HIL and vcan SIL simulation.",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "stm_test_bridge = kaiev26_hil.stm_test_bridge:main",
            "hil_command_generator = kaiev26_hil.hil_command_generator:main",
            "hil_scenario_runner = kaiev26_hil.hil_scenario_runner:main",
            "hil_gui = kaiev26_hil.hil_gui:main",
            "hil_web_bridge = kaiev26_hil.hil_web_bridge:main",
            "vcan_sim = kaiev26_hil.vcan_sim.vcan_sim:main",
        ],
    },
)
