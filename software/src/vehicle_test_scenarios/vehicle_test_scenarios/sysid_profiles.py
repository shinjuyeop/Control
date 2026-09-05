"""Finite field-test inputs. Speeds are targets; the plant response is recorded."""

from dataclasses import asdict, dataclass
import math

from .scenario_runner import ScenarioSetpoint


CENTER_DEG = -0.56
AMPLITUDE_DEG = 1.0
COUNTDOWN_SEC = 5.0
TAIL_SEC = 2.0


@dataclass(frozen=True)
class RunSettings:
    scenario: int = 1
    max_speed_kph: float = 5.0

    def __post_init__(self):
        if self.scenario not in (1, 2, 3, 4):
            raise ValueError('시나리오는 1..4입니다')
        if not math.isfinite(self.max_speed_kph) or self.max_speed_kph <= 0:
            raise ValueError('최고 목표속도는 0보다 큰 유한한 값이어야 합니다')

    @property
    def duration(self):
        return {1: 20.0, 2: 13.0, 3: 12.0, 4: 17.0}[self.scenario]

    @property
    def needs_cruise(self):
        return self.scenario in (3, 4)

    @property
    def steering_enabled(self):
        return self.scenario in (3, 4)

    def description(self):
        v = self.max_speed_kph
        if self.scenario == 1:
            return f'{v/3:.1f} → 0 → {v/3:.1f} → {2*v/3:.1f} → {v:g} → {2*v/3:.1f} → {v/3:.1f} → 0 km/h'
        if self.scenario == 2:
            return f'0 ↗ {v:g} km/h (3초) · 유지 3초 · ↘ 0 (4초)'
        if self.scenario == 3:
            return f'{v:g} km/h 유지 · 중앙 / 좌 / 중앙 / 우 / 중앙 각 1초 · 감속'
        return f'{v:g} km/h 유지 · 작은 좌우 조향을 점차 빠르게 10초 · 감속'

    def document(self):
        document = asdict(self)
        document.update({
            'speed_unit': 'km/h', 'steering_unit': 'central_equivalent_degrees',
            'steering_enabled': self.steering_enabled,
            'auto_countdown_sec': COUNTDOWN_SEC, 'publish_rate_hz': 20.0,
            'input_duration_sec': self.duration, 'cruise_preparation': self.needs_cruise,
            'cruise_tolerance_kph': 0.5, 'cruise_hold_sec': 1.0,
            'cruise_timeout_sec': 15.0,
            'description': self.description(),
            'segments': segments(self),
        })
        if self.steering_enabled:
            document.update({
                'steering_center_deg': CENTER_DEG, 'steering_amplitude_deg': AMPLITUDE_DEG,
            })
        return document


def baseline(brake=False, *, steering_enabled=False):
    return ScenarioSetpoint(0.0, CENTER_DEG if steering_enabled else None, brake)


def segments(settings):
    """The same segment definitions drive execution and the saved run settings."""
    v = settings.max_speed_kph
    if settings.scenario == 1:
        return [
            {'duration_sec': duration, 'from_kph': fraction * v, 'to_kph': fraction * v}
            for duration, fraction in ((3, 1 / 3), (3, 0), (2, 1 / 3), (2, 2 / 3),
                                       (3, 1), (2, 2 / 3), (2, 1 / 3), (3, 0))
        ]
    if settings.scenario == 2:
        return [
            {'duration_sec': 3, 'from_kph': 0, 'to_kph': v},
            {'duration_sec': 3, 'from_kph': v, 'to_kph': v},
            {'duration_sec': 4, 'from_kph': v, 'to_kph': 0},
            {'duration_sec': 3, 'from_kph': 0, 'to_kph': 0},
        ]
    if settings.scenario == 3:
        steering = [
            {'duration_sec': 1, 'from_kph': v, 'to_kph': v,
             'steering_deg': CENTER_DEG + offset}
            for offset in (0, AMPLITUDE_DEG, 0, -AMPLITUDE_DEG, 0)
        ]
    else:
        steering = [{'duration_sec': 10, 'from_kph': v, 'to_kph': v,
                     'chirp_start_hz': 0.2, 'chirp_end_hz': 1.0,
                     'steering_amplitude_deg': AMPLITUDE_DEG}]
    return steering + [
        {'duration_sec': 4, 'from_kph': v, 'to_kph': 0, 'steering_deg': CENTER_DEG},
        {'duration_sec': 3, 'from_kph': 0, 'to_kph': 0, 'steering_deg': CENTER_DEG},
    ]


def evaluate(schedule, elapsed):
    """Return the input and segment number at a monotonic elapsed time."""
    remaining = max(0.0, elapsed)
    for index, segment in enumerate(schedule):
        duration = segment['duration_sec']
        if remaining < duration:
            speed = segment['from_kph'] + (
                segment['to_kph'] - segment['from_kph']) * remaining / duration
            steering = segment.get('steering_deg')
            if 'chirp_start_hz' in segment:
                f0, f1 = segment['chirp_start_hz'], segment['chirp_end_hz']
                phase = 2 * math.pi * (
                    f0 * remaining + (f1 - f0) * remaining**2 / (2 * duration))
                steering = CENTER_DEG + segment['steering_amplitude_deg'] * math.sin(phase)
            return ScenarioSetpoint(speed, steering), index + 1
        remaining -= duration
    return baseline(steering_enabled='steering_deg' in schedule[-1]), len(schedule)
