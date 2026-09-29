"""Server-owned profiles. A running server never accepts a profile mutation."""

from dataclasses import dataclass


@dataclass(frozen=True)
class Profile:
    name: str
    capabilities: tuple[str, ...]

    def metadata(self):
        return dict(profile=self.name, capabilities=self.capabilities)


OBSERVATION = ("observe_vehicle", "observe_runtime", "observe_diagnostics")
VEHICLE = Profile("vehicle", OBSERVATION)
HIL = Profile("hil", OBSERVATION + (
    "control_bench", "fault_injection", "scenarios", "virtual_display", "virtual_touch",
))
