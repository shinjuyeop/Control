from __future__ import annotations

import importlib.util
from pathlib import Path

import pytest

SOFTWARE_SRC = Path(__file__).resolve().parents[2]


def _load_launch_description(relative_path: str):
    path = SOFTWARE_SRC / relative_path
    if not path.exists():
        pytest.skip(f"{relative_path} is not available yet")
    spec = importlib.util.spec_from_file_location(path.stem.replace(".", "_"), path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    try:
        spec.loader.exec_module(module)
    except ModuleNotFoundError as exc:
        if exc.name in {"launch", "launch_ros"}:
            pytest.skip("ROS 2 launch modules are not available")
        raise
    return module.generate_launch_description()


def _launch_nodes(relative_path: str):
    launch_description = _load_launch_description(relative_path)
    return [entity for entity in launch_description.entities if type(entity).__name__ == "Node"]


def _parameter_name(parameter_mapping):
    ((parameter_key,),) = parameter_mapping.keys()
    return parameter_key.text


def _assert_nodes_use_parameter(nodes, allowed_parameter_names: set[str]):
    assert nodes
    for node in nodes:
        assert getattr(node, "_Node__arguments") in (None, [])
        parameters = getattr(node, "_Node__parameters")
        assert parameters
        assert _parameter_name(parameters[0]) in allowed_parameter_names


def test_longitudinal_launch_uses_can_iface_parameters_only():
    nodes = _launch_nodes("control_launch/launch/longitudinal.launch.py")
    _assert_nodes_use_parameter(nodes, {"can_iface"})


def test_can_lab_launch_uses_parameters_for_all_control_nodes():
    nodes = _launch_nodes("control_launch/launch/can_lab.launch.py")
    _assert_nodes_use_parameter(nodes, {"can_iface", "can_ifaces", "motor_iface", "steer_iface"})


def test_lateral_launch_uses_can_iface_parameter_for_steering_control():
    nodes = _launch_nodes("control_launch/launch/lateral.launch.py")
    _assert_nodes_use_parameter(nodes, {"can_iface"})
