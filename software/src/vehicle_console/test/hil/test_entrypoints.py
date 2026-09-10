from argparse import Namespace

import pytest

from vehicle_console.hil import hil_command_generator, hil_scenario_runner
from vehicle_console.hil.vcan_sim import vcan_sim


class FakeRosNode:
    def __init__(self, events):
        self.events = events

    def destroy_node(self):
        self.events.append("destroy")


@pytest.mark.parametrize(
    ("module", "node_attribute"),
    [
        (hil_command_generator, "HilCommandGenerator"),
        (hil_scenario_runner, "HilScenarioRunner"),
    ],
)
def test_ros_entrypoints_run_the_node_lifecycle(monkeypatch, module, node_attribute):
    events = []
    node = FakeRosNode(events)
    monkeypatch.setattr(module, node_attribute, lambda: node)
    monkeypatch.setattr(
        module.rclpy, "init", lambda *, args=None: events.append(("init", args))
    )
    monkeypatch.setattr(
        module.rclpy, "spin", lambda value: events.append(("spin", value))
    )
    monkeypatch.setattr(module.rclpy, "ok", lambda: True)
    monkeypatch.setattr(module.rclpy, "shutdown", lambda: events.append("shutdown"))

    module.main(args=["--ros-args"])

    assert events == [
        ("init", ["--ros-args"]),
        ("spin", node),
        "destroy",
        "shutdown",
    ]


def test_vcan_entrypoint_runs_headless_demo_and_closes_runtime(monkeypatch):
    events = []

    class FakeRuntime:
        def __init__(self, motor_iface, steer_iface):
            events.append(("open", motor_iface, steer_iface))

        def run_headless_demo(self, duration):
            events.append(("demo", duration))

        def close(self):
            events.append("close")

    monkeypatch.setattr(
        vcan_sim,
        "parse_args",
        lambda: Namespace(
            motor_iface="vcan8",
            steer_iface="vcan9",
            no_gui=True,
            ros_no_gui="false",
            headless_demo=True,
            duration=0.25,
        ),
    )
    monkeypatch.setattr(vcan_sim, "VcanSimRuntime", FakeRuntime)

    assert vcan_sim.main() == 0
    assert events == [
        ("open", "vcan8", "vcan9"),
        ("demo", 0.25),
        "close",
    ]
