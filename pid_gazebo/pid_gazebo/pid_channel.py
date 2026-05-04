"""Single-axis PID controller with anti-windup (clamp + back-calculation)."""

from __future__ import annotations


class PidChannel:
    """
    Discrete PID with two-stage anti-windup:
      1. Integral clamping to [-i_limit, +i_limit]
      2. Back-calculation: subtract saturation excess from integral each step
    """

    def __init__(
        self,
        kp: float = 1.0,
        ki: float = 0.0,
        kd: float = 0.0,
        out_min: float = -1.0,
        out_max: float = 1.0,
        i_limit: float = 100.0,
    ) -> None:
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.out_min = out_min
        self.out_max = out_max
        self.i_limit = i_limit

        self._i_state: float = 0.0
        self._prev_err: float = 0.0
        self._first: bool = True

    # ------------------------------------------------------------------
    def compute(self, error: float, dt: float) -> float:
        """Return clamped PID output for one time step dt [s]."""
        dt = max(dt, 1e-6)

        if self._first:
            self._prev_err = error
            self._first = False

        p = self.kp * error
        d = self.kd * (error - self._prev_err) / dt
        self._prev_err = error

        self._i_state += error * dt
        # Stage 1: integral clamp
        self._i_state = max(-self.i_limit, min(self.i_limit, self._i_state))

        u_raw = p + self.ki * self._i_state + d
        u_sat = max(self.out_min, min(self.out_max, u_raw))

        # Stage 2: back-calculation
        if abs(self.ki) > 1e-12:
            self._i_state -= (u_raw - u_sat) / self.ki

        return u_sat

    def reset(self) -> None:
        self._i_state = 0.0
        self._prev_err = 0.0
        self._first = True

    def update_gains(
        self,
        kp: float | None = None,
        ki: float | None = None,
        kd: float | None = None,
        out_min: float | None = None,
        out_max: float | None = None,
        i_limit: float | None = None,
    ) -> None:
        if kp is not None:
            self.kp = kp
        if ki is not None:
            self.ki = ki
        if kd is not None:
            self.kd = kd
        if out_min is not None:
            self.out_min = out_min
        if out_max is not None:
            self.out_max = out_max
        if i_limit is not None:
            self.i_limit = i_limit
