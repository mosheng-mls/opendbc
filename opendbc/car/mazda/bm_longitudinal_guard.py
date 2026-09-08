"""Mazda3 BM first-release direct-longitudinal command guard."""

from dataclasses import dataclass

from opendbc.car import DT_CTRL


BM_LONG_MIN_SPEED = 10.0 / 3.6
BM_LONG_HANDOFF_SPEED = 5.0 / 3.6
BM_LONG_HIGH_SPEED = 70.0 / 3.6
BM_LONG_MAX_SPEED = 80.0 / 3.6
BM_ACCEL_MIN = -1.50
BM_ACCEL_MAX = 0.60
BM_HIGH_SPEED_ACCEL_MAX = 0.30
BM_POSITIVE_JERK_MAX = 0.50
BM_NEGATIVE_JERK_MIN = -1.50


@dataclass(frozen=True)
class BMLongitudinalGuardInput:
  requested_accel: float
  v_ego: float
  long_active: bool
  brake_pressed: bool


@dataclass(frozen=True)
class BMLongitudinalGuardOutput:
  accel: float
  low_speed_handoff: bool
  critical_handoff: bool


class BMLongitudinalGuard:
  """Clamp the planner request to the BM V1 evidence envelope.

  This deliberately contains no stop, hold, resume, or target synthesis. Below
  10 km/h it blocks positive acceleration while preserving jerk-limited decel.
  """

  def __init__(self) -> None:
    self._applied_accel = 0.0

  def update(self, inp: BMLongitudinalGuardInput) -> BMLongitudinalGuardOutput:
    long_active = bool(inp.long_active)

    low_speed_handoff = long_active and inp.v_ego < BM_LONG_MIN_SPEED
    critical_handoff = long_active and inp.v_ego < BM_LONG_HANDOFF_SPEED

    if not long_active or inp.brake_pressed:
      # An inactive command must never carry stale acceleration into the next
      # ownership transition. Low speed is not treated as stop completion.
      target_accel = 0.0
      self._applied_accel = 0.0
    else:
      target_accel = min(BM_ACCEL_MAX, max(BM_ACCEL_MIN, float(inp.requested_accel)))
      if BM_LONG_HIGH_SPEED < inp.v_ego <= BM_LONG_MAX_SPEED:
        target_accel = min(target_accel, BM_HIGH_SPEED_ACCEL_MAX)
      if inp.v_ego < BM_LONG_MIN_SPEED or inp.v_ego > BM_LONG_MAX_SPEED:
        target_accel = min(target_accel, 0.0)

      accel_step_up = BM_POSITIVE_JERK_MAX * DT_CTRL
      accel_step_down = abs(BM_NEGATIVE_JERK_MIN) * DT_CTRL
      if target_accel > self._applied_accel:
        self._applied_accel = min(self._applied_accel + accel_step_up, target_accel)
      else:
        self._applied_accel = max(self._applied_accel - accel_step_down, target_accel)

    return BMLongitudinalGuardOutput(
      accel=self._applied_accel,
      low_speed_handoff=low_speed_handoff,
      critical_handoff=critical_handoff,
    )
