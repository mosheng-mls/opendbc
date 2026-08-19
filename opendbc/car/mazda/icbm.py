"""Mazid Mazda ICBM (Intelligent Cruise Button Management) — vehicle TX helper.

SELECTIVE_PORT onto stock openpilot: press SET+/SET− so OEM ACC executes gas/brake.
Full sunnypilot MADS/ICBM settings UI is not present on this lineage.

Invariant (pcmCruise Mazda): ICBM target is the persistent cruise set speed
(vCruise, kph). Planner / follow speed (longitudinalPlan.speeds[0]) must never
drive SET+/SET−, or OEM ACC set speed collapses with the lead vehicle.
"""
from __future__ import annotations

import math

from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.mazda.values import Buttons

# sunnypilot Mazda docs: minimum ~200 ms between simulated presses
MIN_PRESS_INTERVAL_FRAMES = 20  # CarController @ 100 Hz

# Match openpilot selfdrive/car/cruise.py V_CRUISE_UNSET (stored in kph)
V_CRUISE_UNSET_KPH = 255.0


def persistent_cruise_target_ms(v_cruise_kph: float) -> float:
  """Convert persistent cruise set speed (kph) to ICBM target (m/s).

  Returns 0.0 when unset/invalid so MazdaIcbmController skips TX.
  Callers must pass vCruise (or equivalent persistent set), never planner speed.
  """
  if not math.isfinite(v_cruise_kph):
    return 0.0
  if v_cruise_kph <= 0.0 or v_cruise_kph >= V_CRUISE_UNSET_KPH:
    return 0.0
  return float(v_cruise_kph) * CV.KPH_TO_MS


class MazdaIcbmController:
  def __init__(self):
    self.last_press_frame = -MIN_PRESS_INTERVAL_FRAMES

  def update(self, frame: int, *, enabled: bool, cruise_enabled: bool,
             cruise_speed_ms: float, target_speed_ms: float,
             deadband_ms: float = 0.5) -> int | None:
    """Return Buttons.SET_PLUS / SET_MINUS / None for this frame.

    Never returns RESUME or CANCEL — those stay on the CarController OEM paths.
    """
    if not enabled or not cruise_enabled:
      return None
    if target_speed_ms <= 0.0 or cruise_speed_ms <= 0.0:
      return None
    if frame - self.last_press_frame < MIN_PRESS_INTERVAL_FRAMES:
      return None

    delta = target_speed_ms - cruise_speed_ms
    if abs(delta) < deadband_ms:
      return None

    self.last_press_frame = frame
    return Buttons.SET_PLUS if delta > 0.0 else Buttons.SET_MINUS
