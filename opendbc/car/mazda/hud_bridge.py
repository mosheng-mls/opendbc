"""Mazid HUD display policy for Mazda GEN1 CAM_LANEINFO (0x440).

DISPLAY POLICY lives here. CAN packing stays in mazdacan.create_alert_command.

Safety / Panda / STEER_MAX / ICBM / 0x243 LKAS_REQUEST are out of scope.
This module only decides HUD *display* bits that Safety already allows on 0x440
without inspecting payload (mazda_tx_hook does not parse 0x440).

HUD-BRIDGE-002: C4-engaged graphic is LANE_LINES=2 driven by MazidControlEngagement
(CarControl.latActive). Perception / ACC set speed must not light that graphic.
"""
from __future__ import annotations

from dataclasses import dataclass

from opendbc.car import structs
from opendbc.car.mazda.control_engagement import (
  ControlMode,
  EngagementInputs,
  LANE_LINES_ACTIVE,
  LANE_LINES_STANDBY,
  MazidControlEngagement,
)

VisualAlert = structs.CarControl.HUDControl.VisualAlert
GearShifter = structs.CarState.GearShifter

# HUD TX is 2 Hz (carcontroller frame % 50). Three ticks ≈ 1.5 s.
MIN_WARN_TICKS = 3


@dataclass(frozen=True)
class HudInputs:
  lat_active: bool = False
  enabled: bool = False
  visual_alert: int = VisualAlert.none
  gear: int = GearShifter.drive
  standstill: bool = False
  lkas_allowed_speed: bool = True
  steer_fault_temporary: bool = False
  steer_fault_permanent: bool = False
  oem_hands_on: bool = False
  cruise_available: bool = False
  cruise_enabled: bool = False
  v_cruise_kph: float = 0.0
  hud_set_speed_kph: float = 0.0
  long_active: bool = False
  openpilot_longitudinal_control: bool = False
  fsc_lane_lines: int = 1
  left_lane_visible: bool = False
  right_lane_visible: bool = False
  actuators_torque: float = 0.0
  steering_pressed: bool = False
  brake_pressed: bool = False
  cancel: bool = False


@dataclass(frozen=True)
class HudOutput:
  steer_required: bool
  ldw: bool
  override_lane_lines: int | None
  line_visible: int | None
  line_not_visible: int | None
  reason: str
  priority: str  # P0 / P1 / P2 / P3 / P4 / NONE
  control_mode: str
  lateral_engaged: bool
  oem_acc_active: bool
  road_seen: bool


class MazidHudBridge:
  """Stateful HUD policy: engagement → OEM graphic, priority, debounce, coexistence."""

  def __init__(self, min_warn_ticks: int = MIN_WARN_TICKS):
    self.min_warn_ticks = min_warn_ticks
    self._warn_hold = 0
    self.engagement = MazidControlEngagement()

  def update(self, inp: HudInputs) -> HudOutput:
    ldw = inp.visual_alert == VisualAlert.ldw
    eng = self.engagement.update(EngagementInputs(
      lat_active=inp.lat_active,
      enabled=inp.enabled,
      cruise_available=inp.cruise_available,
      cruise_enabled=inp.cruise_enabled,
      v_cruise_kph=inp.v_cruise_kph,
      hud_set_speed_kph=inp.hud_set_speed_kph,
      long_active=inp.long_active,
      openpilot_longitudinal_control=inp.openpilot_longitudinal_control,
      fsc_lane_lines=inp.fsc_lane_lines,
      left_lane_visible=inp.left_lane_visible,
      right_lane_visible=inp.right_lane_visible,
      actuators_torque=inp.actuators_torque,
      steering_pressed=inp.steering_pressed,
      visual_alert=inp.visual_alert,
      gear=inp.gear,
      standstill=inp.standstill,
      lkas_allowed_speed=inp.lkas_allowed_speed,
      steer_fault_temporary=inp.steer_fault_temporary,
      steer_fault_permanent=inp.steer_fault_permanent,
      brake_pressed=inp.brake_pressed,
      cancel=inp.cancel,
    ))

    # P0 takeover bits: C4 steerRequired OR OEM hands. Never NAND OEM.
    c4_warn = eng.mode == ControlMode.TAKEOVER_REQUIRED
    want_warn = c4_warn or inp.oem_hands_on
    if want_warn:
      self._warn_hold = self.min_warn_ticks
    steer_required = want_warn or self._warn_hold > 0
    if not want_warn and self._warn_hold > 0:
      self._warn_hold -= 1

    # C4 engaged graphic only when engagement says so. Takeover / unavail /
    # standby / OEM-ACC-only / no-road / off all use LANE_LINES_STANDBY so FSC
    # dual-lines cannot look like 'C4 already took over'.
    # ROAD_SEEN is C4 modelV2 (left/rightLaneVisible), never FSC.
    road_seen = bool(inp.left_lane_visible or inp.right_lane_visible)
    if eng.lateral_engaged and not steer_required:
      override_lane_lines = LANE_LINES_ACTIVE
      line_visible, line_not_visible = 1, 0
    else:
      override_lane_lines = LANE_LINES_STANDBY
      if road_seen:
        line_visible, line_not_visible = 1, 0
      else:
        line_visible, line_not_visible = 0, 1

    if steer_required:
      if c4_warn or inp.oem_hands_on or self._warn_hold > 0:
        if inp.oem_hands_on and not c4_warn:
          priority = "P0"
          reason = "oem_hands_priority"
        elif c4_warn:
          priority = "P0"
          reason = eng.reason
        else:
          priority = "P1"
          reason = "hands_or_hold"
      else:
        priority = "P0"
        reason = "hands_or_hold"
    elif eng.mode == ControlMode.TEMP_UNAVAILABLE:
      priority = "P1"
      reason = eng.reason
    elif eng.lateral_engaged:
      priority = "P2"
      reason = eng.reason
    elif eng.mode == ControlMode.OEM_LONGITUDINAL_ACTIVE:
      priority = "P3"
      reason = eng.reason
    elif eng.mode == ControlMode.STANDBY:
      priority = "P3"
      reason = eng.reason
    elif eng.mode == ControlMode.NO_ROAD:
      priority = "P4"
      reason = eng.reason
    else:
      priority = "P4"
      reason = eng.reason

    return HudOutput(
      steer_required=steer_required,
      ldw=ldw,
      override_lane_lines=override_lane_lines,
      line_visible=line_visible,
      line_not_visible=line_not_visible,
      reason=reason,
      priority=priority,
      control_mode=eng.mode,
      lateral_engaged=eng.lateral_engaged,
      oem_acc_active=eng.oem_acc_active,
      road_seen=road_seen,
    )
