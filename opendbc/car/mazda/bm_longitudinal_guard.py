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
BM_LEAD_LOSS_FREEZE_NS = 2_000_000_000
BM_SET_SPEED_FREEZE_NS = 1_500_000_000
BM_ENGAGE_FREEZE_NS = 500_000_000
BM_SET_SPEED_STEP_MS = 1.0


@dataclass(frozen=True)
class BMLongitudinalGuardInput:
  requested_accel: float
  v_ego: float
  long_active: bool
  brake_pressed: bool
  lead_visible: bool
  set_speed: float
  now_nanos: int


@dataclass(frozen=True)
class BMLongitudinalGuardOutput:
  accel: float
  positive_accel_frozen: bool
  low_speed_handoff: bool
  critical_handoff: bool


class BMLongitudinalGuard:
  """Clamp the planner request to the BM V1 evidence envelope.

  This deliberately contains no stop, hold, resume, or target synthesis. Below
  5 km/h it releases commanded acceleration and requires the driver to brake.
  """

  def __init__(self) -> None:
    self._applied_accel = 0.0
    self._was_active = False
    self._last_lead_visible = False
    self._last_set_speed = 0.0
    self._freeze_positive_until_ns = 0

  def _freeze_positive(self, now_nanos: int, duration_ns: int) -> None:
    self._freeze_positive_until_ns = max(self._freeze_positive_until_ns, now_nanos + duration_ns)

  def update(self, inp: BMLongitudinalGuardInput) -> BMLongitudinalGuardOutput:
    now_nanos = max(0, int(inp.now_nanos))
    long_active = bool(inp.long_active)

    if long_active and not self._was_active:
      self._freeze_positive(now_nanos, BM_ENGAGE_FREEZE_NS)
    if long_active and self._last_lead_visible and not inp.lead_visible:
      self._freeze_positive(now_nanos, BM_LEAD_LOSS_FREEZE_NS)
    if long_active and self._was_active and inp.set_speed - self._last_set_speed >= BM_SET_SPEED_STEP_MS:
      self._freeze_positive(now_nanos, BM_SET_SPEED_FREEZE_NS)

    low_speed_handoff = long_active and inp.v_ego < BM_LONG_MIN_SPEED
    critical_handoff = long_active and inp.v_ego < BM_LONG_HANDOFF_SPEED
    positive_accel_frozen = now_nanos < self._freeze_positive_until_ns

    if not long_active or inp.brake_pressed or critical_handoff:
      # An inactive command must never carry stale acceleration into the next
      # ownership transition. At the hard handoff speed, release longitudinal
      # output rather than attempting stop/hold behavior.
      target_accel = 0.0
      self._applied_accel = 0.0
    else:
      target_accel = min(BM_ACCEL_MAX, max(BM_ACCEL_MIN, float(inp.requested_accel)))
      if BM_LONG_HIGH_SPEED < inp.v_ego <= BM_LONG_MAX_SPEED:
        target_accel = min(target_accel, BM_HIGH_SPEED_ACCEL_MAX)
      if inp.v_ego < BM_LONG_MIN_SPEED or inp.v_ego > BM_LONG_MAX_SPEED or positive_accel_frozen:
        target_accel = min(target_accel, 0.0)

      accel_step_up = BM_POSITIVE_JERK_MAX * DT_CTRL
      accel_step_down = abs(BM_NEGATIVE_JERK_MIN) * DT_CTRL
      if target_accel > self._applied_accel:
        self._applied_accel = min(self._applied_accel + accel_step_up, target_accel)
      else:
        self._applied_accel = max(self._applied_accel - accel_step_down, target_accel)

    self._was_active = long_active
    self._last_lead_visible = bool(inp.lead_visible)
    self._last_set_speed = float(inp.set_speed)

    return BMLongitudinalGuardOutput(
      accel=self._applied_accel,
      positive_accel_frozen=positive_accel_frozen,
      low_speed_handoff=low_speed_handoff,
      critical_handoff=critical_handoff,
    )
