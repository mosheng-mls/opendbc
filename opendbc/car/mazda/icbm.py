"""Mazid Mazda ICBM (Intelligent Cruise Button Management) — vehicle TX helper.

SELECTIVE_PORT onto stock openpilot: press SET+/SET− so OEM ACC executes gas/brake.
Full sunnypilot MADS/ICBM settings UI is not present on this lineage.

Invariant (pcmCruise Mazda): ordinary ICBM callers use persistent vCruise.
Raw planner/follow speed must never drive SET+/SET−. A temporary target may
only come from a coordinator that separately preserves the driver's base set
speed and restores it after the bounded condition clears.
"""
from __future__ import annotations

import math

from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.mazda.values import Buttons

# sunnypilot Mazda docs: minimum ~200 ms between simulated presses
MIN_PRESS_INTERVAL_FRAMES = 20  # CarController @ 100 Hz
ACK_TIMEOUT_FRAMES = 100
ACK_DELTA_MPS = 0.1

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
  def __init__(self, *, require_ack: bool = False):
    self.require_ack = bool(require_ack)
    self.last_press_frame = -MIN_PRESS_INTERVAL_FRAMES
    self.pending_button: int | None = None
    self.pending_cruise_speed_ms = 0.0

  def reset(self, frame: int) -> None:
    """Clear cross-session timing and require a fresh cooldown before TX."""
    self.last_press_frame = int(frame)
    self.pending_button = None
    self.pending_cruise_speed_ms = 0.0

  def update(self, frame: int, *, enabled: bool, cruise_enabled: bool,
             cruise_speed_ms: float, target_speed_ms: float,
             deadband_ms: float = 0.5) -> int | None:
    """Return Buttons.SET_PLUS / SET_MINUS / None for this frame.

    Never returns RESUME or CANCEL — those stay on the CarController OEM paths.
    """
    if not enabled or not cruise_enabled:
      return None
    if not all(math.isfinite(value) for value in (cruise_speed_ms, target_speed_ms, deadband_ms)):
      return None
    if target_speed_ms <= 0.0 or cruise_speed_ms <= 0.0 or deadband_ms < 0.0:
      return None

    delta = target_speed_ms - cruise_speed_ms
    if self.require_ack and self.pending_button is not None:
      requested_button = None if abs(delta) < deadband_ms else (Buttons.SET_PLUS if delta > 0.0 else Buttons.SET_MINUS)
      acknowledged = (
        self.pending_button == Buttons.SET_PLUS and cruise_speed_ms >= self.pending_cruise_speed_ms + ACK_DELTA_MPS
      ) or (
        self.pending_button == Buttons.SET_MINUS and cruise_speed_ms <= self.pending_cruise_speed_ms - ACK_DELTA_MPS
      )
      if requested_button != self.pending_button:
        # A new risk may reverse a recovery request. Abort the old ACK wait;
        # the normal press interval still prevents a burst.
        self.pending_button = None
      elif acknowledged:
        self.pending_button = None
      elif frame - self.last_press_frame < ACK_TIMEOUT_FRAMES:
        return None
      else:
        # Do not burst after a missing acknowledgement. Start a fresh cooldown
        # and let the caller decide whether the request is still appropriate.
        self.pending_button = None
        self.last_press_frame = int(frame)
        return None

    if frame - self.last_press_frame < MIN_PRESS_INTERVAL_FRAMES:
      return None

    if abs(delta) < deadband_ms:
      return None

    button = Buttons.SET_PLUS if delta > 0.0 else Buttons.SET_MINUS
    self.last_press_frame = frame
    if self.require_ack:
      self.pending_button = button
      self.pending_cruise_speed_ms = float(cruise_speed_ms)
    return button
