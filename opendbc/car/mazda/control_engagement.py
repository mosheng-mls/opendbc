"""Mazid actual-control engagement state (HUD-BRIDGE-002).

READY ≠ ENGAGED ≠ ACTUATING.

LATERAL_ACTIVE is control-authority (CarControl.latActive), not:
  lane lines, FSC, ACC set speed, vCruise, MAIN, MADS-ready, |torque| > 0.

This module does not pack CAN. HudBridge maps the mode to DISPLAY_ONLY 0x440 bits.
"""
from __future__ import annotations

from dataclasses import dataclass

from opendbc.car import structs

VisualAlert = structs.CarControl.HUDControl.VisualAlert
GearShifter = structs.CarState.GearShifter

_PARK_REVERSE = (GearShifter.park, GearShifter.reverse)


class ControlMode:
  OFF = "OFF"
  STANDBY = "STANDBY"
  LATERAL_ACTIVE = "LATERAL_ACTIVE"
  OEM_LONGITUDINAL_ACTIVE = "OEM_LONGITUDINAL_ACTIVE"
  FULL_ASSIST_ACTIVE = "FULL_ASSIST_ACTIVE"  # reserved; never set without direct long
  DRIVER_OVERRIDE = "DRIVER_OVERRIDE"
  TEMP_UNAVAILABLE = "TEMP_UNAVAILABLE"
  TAKEOVER_REQUIRED = "TAKEOVER_REQUIRED"


# HUD CAM_LANEINFO LANE_LINES (DBC: 1 = no lines, 2 = two lines).
LANE_LINES_STANDBY = 1
LANE_LINES_ACTIVE = 2


@dataclass(frozen=True)
class EngagementInputs:
  lat_active: bool = False
  enabled: bool = False
  cruise_available: bool = False
  cruise_enabled: bool = False
  v_cruise_kph: float = 0.0
  hud_set_speed_kph: float = 0.0
  fsc_lane_lines: int = 1
  left_lane_visible: bool = False
  right_lane_visible: bool = False
  actuators_torque: float = 0.0
  steering_pressed: bool = False
  visual_alert: int = VisualAlert.none
  gear: int = GearShifter.drive
  standstill: bool = False
  lkas_allowed_speed: bool = True
  steer_fault_temporary: bool = False
  steer_fault_permanent: bool = False
  brake_pressed: bool = False
  cancel: bool = False


@dataclass(frozen=True)
class EngagementOutput:
  mode: str
  lateral_engaged: bool
  ready: bool
  oem_acc_active: bool
  c4_longitudinal_active: bool
  reason: str


class MazidControlEngagement:
  """Classify C4 control engagement. Unused perception/speed fields are accepted
  so callers cannot accidentally omit them — they must not drive LATERAL_ACTIVE.

  HUD TX is 2 Hz. Enter/exit ticks are in that HUD sample domain, not 100 Hz control.
  Enter: this sample (2 Hz already filters sub-500 ms control blips).
  Exit: this sample (must not keep 'C4 ACTIVE' after real disengage).
  """

  def __init__(self, enter_confirm_ticks: int = 1, exit_hold_ticks: int = 0):
    self.enter_confirm_ticks = max(1, enter_confirm_ticks)
    self.exit_hold_ticks = max(0, exit_hold_ticks)
    self._enter = 0
    self._exit_hold = 0
    self._shown_active = False

  def _raw_want_active(self, inp: EngagementInputs) -> bool:
    if inp.gear in _PARK_REVERSE:
      return False
    return bool(inp.lat_active)

  def update(self, inp: EngagementInputs) -> EngagementOutput:
    # Explicitly unused for ENGAGED. Perception / set speed / torque magnitude
    # are READY or ACTUATING cues, never the engagement latch.
    _ = inp.v_cruise_kph
    _ = inp.hud_set_speed_kph
    _ = inp.fsc_lane_lines
    _ = inp.left_lane_visible
    _ = inp.right_lane_visible
    _ = inp.actuators_torque
    _ = inp.brake_pressed
    _ = inp.cancel

    parked = inp.gear in _PARK_REVERSE
    c4_warn = bool(inp.visual_alert == VisualAlert.steerRequired and inp.lkas_allowed_speed)
    suppress_park_warn = (inp.standstill or parked) and inp.steer_fault_temporary and not inp.lat_active
    if suppress_park_warn:
      c4_warn = False

    want_active = self._raw_want_active(inp)
    if want_active:
      self._enter += 1
      self._exit_hold = self.exit_hold_ticks
    else:
      self._enter = 0
      if self._shown_active and self._exit_hold > 0:
        self._exit_hold -= 1
        want_active = True
      else:
        self._shown_active = False

    lat_engaged = want_active and self._enter >= self.enter_confirm_ticks
    if lat_engaged:
      self._shown_active = True
    elif not want_active:
      lat_engaged = False

    oem_acc = bool(inp.cruise_enabled) and not lat_engaged
    # Stock enabled or MAIN available = READY, not ENGAGED.
    ready = bool(inp.enabled or inp.cruise_available or inp.lat_active) and not parked
    c4_long = False  # no direct longitudinal on this BM

    if parked:
      mode = ControlMode.OFF
      reason = "park_or_reverse"
    elif c4_warn:
      mode = ControlMode.TAKEOVER_REQUIRED
      reason = "steer_capability_or_takeover"
    elif (inp.steer_fault_temporary or inp.steer_fault_permanent) and not lat_engaged:
      mode = ControlMode.TEMP_UNAVAILABLE
      reason = "steer_fault"
    elif lat_engaged and inp.steering_pressed:
      mode = ControlMode.DRIVER_OVERRIDE
      reason = "driver_override_while_lat_active"
    elif lat_engaged:
      mode = ControlMode.LATERAL_ACTIVE
      reason = "lat_active_control_path"
    elif oem_acc:
      mode = ControlMode.OEM_LONGITUDINAL_ACTIVE
      reason = "oem_acc_only"
    elif not parked:
      mode = ControlMode.STANDBY
      reason = "onroad_not_lat_active"
    else:
      mode = ControlMode.OFF
      reason = "off"

    return EngagementOutput(
      mode=mode,
      lateral_engaged=lat_engaged and mode in (ControlMode.LATERAL_ACTIVE, ControlMode.DRIVER_OVERRIDE),
      ready=ready,
      oem_acc_active=oem_acc or (inp.cruise_enabled and lat_engaged),
      c4_longitudinal_active=c4_long,
      reason=reason,
    )
