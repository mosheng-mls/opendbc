"""Mazid Mazda ICBM (Intelligent Cruise Button Management) — vehicle TX helper.

SELECTIVE_PORT onto stock openpilot: press SET+/SET− so OEM ACC executes gas/brake.
Full sunnypilot MADS/ICBM settings UI is not present on this lineage.
"""
from __future__ import annotations

from opendbc.car.mazda.values import Buttons

# sunnypilot Mazda docs: minimum ~200 ms between simulated presses
MIN_PRESS_INTERVAL_FRAMES = 20  # CarController @ 100 Hz


class MazdaIcbmController:
  def __init__(self):
    self.last_press_frame = -MIN_PRESS_INTERVAL_FRAMES

  def update(self, frame: int, *, enabled: bool, cruise_enabled: bool,
             cruise_speed_ms: float, target_speed_ms: float,
             deadband_ms: float = 0.5) -> int | None:
    """Return Buttons.SET_PLUS / SET_MINUS / None for this frame."""
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
