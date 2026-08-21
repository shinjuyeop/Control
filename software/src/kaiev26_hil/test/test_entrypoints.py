from kaiev26_hil.hil_command_generator import HilCommandGenerator
from kaiev26_hil.hil_scenario_runner import HilScenarioRunner
from kaiev26_hil.vcan_sim.vcan_sim import main as vcan_sim_main


def test_renamed_entrypoint_symbols_are_importable():
    assert HilCommandGenerator.__name__ == "HilCommandGenerator"
    assert HilScenarioRunner.__name__ == "HilScenarioRunner"
    assert callable(vcan_sim_main)
