"""Scenario catalog shared by the field TUI and its runtime."""

from dataclasses import asdict, dataclass, replace
import math

from .scenario_runner import (
    ScenarioConfig,
    ScenarioSetpoint,
    scenario_cycle_duration_sec,
    scheduled_scenario_setpoint,
    validate_config,
)


CENTER_DEG = -0.56
AMPLITUDE_DEG = 1.0
# steering_actuator.yaml torque 운용값. Run 때 steering_actuator_node에 적용한다(torque 모드만).
TORQUE_P_FULL_SCALE_DEG = 0.5
TORQUE_D_GAIN = 0.0
COUNTDOWN_SEC = 5.0
TAIL_SEC = 2.0
ROUTE_FINISH_DISTANCE_M = 1.0
ROUTE_STOP_SPEED_MPS = 0.05


@dataclass(frozen=True)
class ParameterSpec:
    name: str
    label: str
    unit: str
    integer: bool = False
    note: str = ''


@dataclass(frozen=True)
class ScenarioSpec:
    key: str
    label: str
    category: str
    subgroup: str
    fields: tuple[ParameterSpec, ...]
    defaults: tuple[tuple[str, float | int], ...] = ()


P = ParameterSpec
# P는 "이 오차(deg)에서 토크가 peak에 닿는다" 형식이다: Kp[raw/deg] = peak / 값. 작을수록 P가 세다.
TORQUE_GAIN_FIELDS = (
    P('torque_full_scale_error_deg', 'P · full-scale 오차(작을수록 P↑)', 'deg',
      note='Kp = torque_peak_limit_raw / 값 · torque 모드만 적용'),
    P('torque_kd', 'D 게인 raw/(rad/s)', 'raw', note='0은 감쇠 없음 · torque 모드만 적용'),
)
TORQUE_GAIN_DEFAULTS = (
    ('torque_full_scale_error_deg', TORQUE_P_FULL_SCALE_DEG),
    ('torque_kd', TORQUE_D_GAIN),
)
SCENARIO_SPECS = (
    ScenarioSpec(
        'straight', '일정 속도 직진', '입력 응답 시험', '종방향 입력',
        (P('cruise_speed_kph', '목표속도', 'km/h'),),
        (('cruise_speed_kph', 3.0),),
    ),
    ScenarioSpec(
        'speed_step', '속도 Step', '입력 응답 시험', '종방향 입력',
        (
            P('speed_low_kph', '낮은 속도', 'km/h'),
            P('speed_high_kph', '높은 속도', 'km/h'),
            P('speed_step_interval_sec', '구간 유지시간', 'sec'),
            P('initial_hold_sec', '초기 유지시간', 'sec'),
            P('scenario_repetitions', '반복 횟수', '회', True, '0은 수동 정지까지 반복'),
        ),
        (
            ('speed_low_kph', 1.0), ('speed_high_kph', 3.0),
            ('speed_step_interval_sec', 2.0), ('initial_hold_sec', 0.0),
            ('scenario_repetitions', 4),
        ),
    ),
    ScenarioSpec(
        'speed_ramp', '속도 Ramp', '입력 응답 시험', '종방향 입력',
        (
            P('speed_low_kph', '낮은 속도', 'km/h'),
            P('speed_high_kph', '높은 속도', 'km/h'),
            P('speed_ramp_duration_sec', '편도 변화시간', 'sec'),
            P('initial_hold_sec', '초기 유지시간', 'sec'),
            P('scenario_repetitions', '반복 횟수', '회', True, '0은 수동 정지까지 반복'),
        ),
        (
            ('speed_low_kph', 0.0), ('speed_high_kph', 3.0),
            ('speed_ramp_duration_sec', 5.0), ('initial_hold_sec', 0.0),
            ('scenario_repetitions', 2),
        ),
    ),
    ScenarioSpec(
        'brake_test', '제동 입력', '입력 응답 시험', '종방향 입력',
        (
            P('cruise_speed_kph', '제동 전 목표속도', 'km/h'),
            P('brake_delay_sec', '제동 시작시간', 'sec', note='이후 브레이크 유지'),
        ),
        (('cruise_speed_kph', 3.0), ('brake_delay_sec', 3.0)),
    ),
    ScenarioSpec(
        'steering_step', '조향 양방향 Step', '입력 응답 시험', '횡방향 입력',
        (
            P('cruise_speed_kph', '주행속도', 'km/h'),
            P('steering_center_deg', '센터각', 'deg'),
            P('steering_amplitude_deg', '조향 진폭', 'deg'),
            P('steering_step_interval_sec', '구간 유지시간', 'sec'),
            P('initial_hold_sec', '초기 직진시간', 'sec'),
            P('scenario_repetitions', '반복 횟수', '회', True, '0은 수동 정지까지 반복'),
            *TORQUE_GAIN_FIELDS,
        ),
        (
            ('cruise_speed_kph', 3.0), ('steering_center_deg', CENTER_DEG),
            ('steering_amplitude_deg', AMPLITUDE_DEG),
            ('steering_step_interval_sec', 5.0), ('initial_hold_sec', 0.0),
            ('scenario_repetitions', 3), *TORQUE_GAIN_DEFAULTS,
        ),
    ),
    ScenarioSpec(
        'steering_sine', '조향 Sine · 고정 주기', '입력 응답 시험', '횡방향 입력',
        (
            P('cruise_speed_kph', '주행속도', 'km/h'),
            P('steering_center_deg', '센터각', 'deg'),
            P('steering_amplitude_deg', '조향 진폭', 'deg'),
            P('steering_period_sec', '파형 주기', 'sec'),
            P('initial_hold_sec', '초기 직진시간', 'sec'),
            P('scenario_repetitions', '반복 횟수', '회', True, '0은 수동 정지까지 반복'),
            *TORQUE_GAIN_FIELDS,
        ),
        (
            ('cruise_speed_kph', 3.0), ('steering_center_deg', CENTER_DEG),
            ('steering_amplitude_deg', AMPLITUDE_DEG),
            ('steering_period_sec', 8.0), ('initial_hold_sec', 5.0),
            ('scenario_repetitions', 6), *TORQUE_GAIN_DEFAULTS,
        ),
    ),
    ScenarioSpec(
        'steering_chirp', '조향 Chirp · 주파수 Sweep', '입력 응답 시험', '횡방향 입력',
        (
            P('cruise_speed_kph', '주행속도', 'km/h'),
            P('steering_center_deg', '센터각', 'deg'),
            P('steering_amplitude_deg', '조향 진폭', 'deg'),
            P('chirp_start_hz', '시작 주파수', 'Hz'),
            P('chirp_end_hz', '종료 주파수', 'Hz'),
            P('chirp_duration_sec', 'Sweep 시간', 'sec'),
            P('initial_hold_sec', '초기 직진시간', 'sec'),
            P('scenario_repetitions', '반복 횟수', '회', True, '0은 수동 정지까지 반복'),
            *TORQUE_GAIN_FIELDS,
        ),
        (
            ('cruise_speed_kph', 3.0), ('steering_center_deg', CENTER_DEG),
            ('steering_amplitude_deg', AMPLITUDE_DEG),
            ('chirp_start_hz', 0.2), ('chirp_end_hz', 1.0),
            ('chirp_duration_sec', 10.0), ('initial_hold_sec', 5.0),
            ('scenario_repetitions', 1), *TORQUE_GAIN_DEFAULTS,
        ),
    ),
    ScenarioSpec(
        'route_quali', 'K-City 예선 경로', '경로 추종 시험', '',
        (P('cruise_speed_kph', '최고 목표속도', 'km/h'),),
        (('cruise_speed_kph', 5.0),),
    ),
    ScenarioSpec(
        'route_final', 'K-City 본선 경로', '경로 추종 시험', '',
        (P('cruise_speed_kph', '최고 목표속도', 'km/h'),),
        (('cruise_speed_kph', 5.0),),
    ),
)
SCENARIO_BY_KEY = {spec.key: spec for spec in SCENARIO_SPECS}
SCENARIO_KEYS = tuple(SCENARIO_BY_KEY)


@dataclass(frozen=True)
class RunSettings:
    scenario: str = 'straight'
    cruise_speed_kph: float = 3.0
    speed_low_kph: float = 0.0
    speed_high_kph: float = 3.0
    speed_step_interval_sec: float = 2.0
    speed_ramp_duration_sec: float = 5.0
    steering_center_deg: float = CENTER_DEG
    steering_amplitude_deg: float = AMPLITUDE_DEG
    steering_step_interval_sec: float = 5.0
    steering_period_sec: float = 8.0
    brake_delay_sec: float = 3.0
    initial_hold_sec: float = 0.0
    scenario_repetitions: int = 1
    chirp_start_hz: float = 0.2
    chirp_end_hz: float = 1.0
    chirp_duration_sec: float = 10.0
    torque_full_scale_error_deg: float = TORQUE_P_FULL_SCALE_DEG
    torque_kd: float = TORQUE_D_GAIN
    record_mcap: bool = True

    def __post_init__(self):
        if self.scenario not in SCENARIO_BY_KEY:
            raise ValueError(f'지원하지 않는 시나리오입니다: {self.scenario}')
        if (not math.isfinite(self.torque_full_scale_error_deg)
                or self.torque_full_scale_error_deg <= 0.0):
            raise ValueError('P full-scale 오차는 0보다 큰 유한한 값이어야 합니다')
        if not math.isfinite(self.torque_kd) or self.torque_kd < 0.0:
            raise ValueError('D 게인은 0 이상의 유한한 값이어야 합니다')
        if self.follows_route:
            if not math.isfinite(self.cruise_speed_kph) or self.cruise_speed_kph <= 0:
                raise ValueError('최고 목표속도는 0보다 큰 유한한 값이어야 합니다')
            return
        validate_config(self.to_scenario_config(), 20.0)

    @property
    def spec(self):
        return SCENARIO_BY_KEY[self.scenario]

    @property
    def name(self):
        return self.spec.label

    @property
    def follows_route(self):
        return self.scenario in ('route_quali', 'route_final')

    @property
    def route_filename(self):
        return {
            'route_quali': 'kcity_quali_route.yaml',
            'route_final': 'kcity_final_route.yaml',
        }[self.scenario]

    @property
    def steering_enabled(self):
        return self.follows_route or self.scenario.startswith('steering_')

    @property
    def torque_gains(self):
        """P/D for steering_actuator_node when this test exposes them, else None."""
        names = {field.name for field in TORQUE_GAIN_FIELDS}
        if not names.issubset(field.name for field in self.spec.fields):
            return None
        return {
            'torque_full_scale_error_deg': float(self.torque_full_scale_error_deg),
            'torque_kd': float(self.torque_kd),
        }

    @property
    def needs_cruise(self):
        return self.scenario.startswith('steering_')

    @property
    def max_speed_kph(self):
        if self.scenario in ('speed_step', 'speed_ramp'):
            return self.speed_high_kph
        return self.cruise_speed_kph

    @property
    def duration(self):
        if self.follows_route:
            return None
        cycle = scenario_cycle_duration_sec(self.to_scenario_config())
        if cycle is None or self.scenario_repetitions == 0:
            return None
        return self.initial_hold_sec + self.scenario_repetitions * cycle

    @property
    def record_slug(self):
        return self.scenario

    def to_scenario_config(self):
        engine_scenario = 'steering_sweep' if self.scenario == 'steering_sine' else self.scenario
        return ScenarioConfig(
            scenario=engine_scenario,
            cruise_speed_kph=self.cruise_speed_kph,
            speed_low_kph=self.speed_low_kph,
            speed_high_kph=self.speed_high_kph,
            speed_step_interval_sec=self.speed_step_interval_sec,
            speed_ramp_duration_sec=self.speed_ramp_duration_sec,
            steering_center_deg=self.steering_center_deg,
            steering_amplitude_deg=self.steering_amplitude_deg,
            steering_step_interval_sec=self.steering_step_interval_sec,
            steering_period_sec=self.steering_period_sec,
            chirp_start_hz=self.chirp_start_hz,
            chirp_end_hz=self.chirp_end_hz,
            chirp_duration_sec=self.chirp_duration_sec,
            brake_delay_sec=self.brake_delay_sec,
            initial_hold_sec=self.initial_hold_sec,
            scenario_repetitions=self.scenario_repetitions,
        )

    def description(self):
        if self.follows_route:
            return f'{self.name} · 최고 목표 {self.cruise_speed_kph:g} km/h · 종료점 감속·정지'
        if self.scenario == 'straight':
            return f'{self.cruise_speed_kph:g} km/h 일정 입력 · S/Space로 종료'
        if self.scenario == 'speed_step':
            return (
                f'{self.speed_low_kph:g} ↔ {self.speed_high_kph:g} km/h · '
                f'{self.speed_step_interval_sec:g}초 간격'
            )
        if self.scenario == 'speed_ramp':
            return (
                f'{self.speed_low_kph:g} ↗ {self.speed_high_kph:g} ↘ '
                f'{self.speed_low_kph:g} km/h · 편도 {self.speed_ramp_duration_sec:g}초'
            )
        if self.scenario == 'brake_test':
            return f'{self.cruise_speed_kph:g} km/h · {self.brake_delay_sec:g}초 후 브레이크 유지'
        gains = (f' · P {self.torque_full_scale_error_deg:g}° D {self.torque_kd:g}'
                 if self.torque_gains else '')
        if self.scenario == 'steering_step':
            return (
                f'{self.cruise_speed_kph:g} km/h · 센터/좌/센터/우 · '
                f'진폭 ±{self.steering_amplitude_deg:g}°{gains}'
            )
        if self.scenario == 'steering_sine':
            return (
                f'{self.cruise_speed_kph:g} km/h · ±{self.steering_amplitude_deg:g}° · '
                f'{self.steering_period_sec:g}초 고정 주기{gains}'
            )
        return (
            f'{self.cruise_speed_kph:g} km/h · ±{self.steering_amplitude_deg:g}° · '
            f'{self.chirp_start_hz:g}→{self.chirp_end_hz:g} Hz{gains}'
        )

    def document(self):
        values = asdict(self)
        parameters = {
            field.name: values[field.name]
            for field in self.spec.fields
        }
        document = {
            'scenario': self.scenario,
            'name': self.name,
            'category': self.spec.category,
            'subgroup': self.spec.subgroup,
            'parameters': parameters,
            'description': self.description(),
            'speed_unit': 'km/h',
            'steering_enabled': self.steering_enabled,
            'auto_countdown_sec': COUNTDOWN_SEC,
            'publish_rate_hz': 20.0,
            'record_mcap': self.record_mcap,
        }
        if self.follows_route:
            document['route_file'] = self.route_filename
        else:
            document.update({
                'steering_unit': 'central_equivalent_degrees',
                'input_duration_sec': self.duration,
                'cruise_preparation': self.needs_cruise,
                'cruise_tolerance_kph': 0.5,
                'cruise_hold_sec': 1.0,
                'cruise_timeout_sec': 15.0,
            })
        return document


def default_run_settings(scenario='straight'):
    spec = SCENARIO_BY_KEY[scenario]
    return RunSettings(scenario=scenario, **dict(spec.defaults))


def update_setting(settings, name, value):
    return replace(settings, **{name: value})


def baseline(brake=False, *, settings=None, steering_enabled=None):
    settings = settings or RunSettings()
    enabled = settings.steering_enabled if steering_enabled is None else steering_enabled
    return ScenarioSetpoint(
        0.0,
        settings.steering_center_deg if enabled else None,
        brake,
    )


def input_setpoint(settings, elapsed):
    """Return the selected input and whether its finite schedule is complete."""
    elapsed = max(0.0, elapsed)
    if settings.follows_route:
        raise ValueError('경로 추종 입력은 route worker가 생성합니다')
    return scheduled_scenario_setpoint(settings.to_scenario_config(), elapsed)
