"""HIL launch profile combinations that protect Web scenario ownership."""

import importlib.util
from pathlib import Path

import pytest
from launch import LaunchContext


LAUNCH_PATH = Path(__file__).resolve().parents[2] / "launch" / "hil.launch.py"
SPEC = importlib.util.spec_from_file_location("vehicle_console_hil_launch", LAUNCH_PATH)
hil_launch = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(hil_launch)


def test_web_rejects_standalone_scenario_file():
    context = LaunchContext()
    context.launch_configurations.update({
        "web": "true",
        "start_bench": "true",
        "scenario_file": "smoke_test.yaml",
    })
    with pytest.raises(ValueError, match="HIL Web Console"):
        hil_launch._launch_actions(context)


def test_headless_bench_preserves_standalone_scenario():
    hil_launch.validate_configuration(
        web=False, start_bench=True, scenario_file="smoke_test.yaml"
    )


def test_web_bench_without_standalone_scenario_is_valid():
    hil_launch.validate_configuration(
        web=True, start_bench=True, scenario_file=""
    )


def test_scenario_is_not_silently_ignored_without_bench():
    with pytest.raises(ValueError, match="start_bench:=true"):
        hil_launch.validate_configuration(
            web=False, start_bench=False, scenario_file="smoke_test.yaml"
        )
