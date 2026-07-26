from glob import glob
from setuptools import setup


package_name = "kaiev26_actuator_command_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/config", glob("config/*.yaml")),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    # 이것이 없으면 colcon test가 pytest를 돌리지 않고 "Ran 0 tests"로
    # 조용히 통과한다. 분할 전 `kaiev26_control`이 그 상태였고
    # `test_control_command_gate.py` 4건은 한 번도 실행되지 않았다.
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="user1",
    maintainer_email="user@example.com",
    description="Sole publisher of the KAIEV26 canonical actuator command.",
    license="TODO",
    entry_points={
        "console_scripts": [
            "actuator_command_bridge_node = "
            "kaiev26_actuator_command_bridge.actuator_command_bridge_node:main",
        ],
    },
)
