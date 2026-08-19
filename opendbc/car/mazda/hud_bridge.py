"""Mazid HUD display policy for Mazda GEN1 CAM_LANEINFO (0x440).

DISPLAY POLICY lives here. CAN packing stays in mazdacan.create_alert_command.

Safety / Panda / STEER_MAX / ICBM / 0x243 LKAS_REQUEST are out of scope.
This module only decides HUD *display* bits that Safety already allows on 0x440
without inspecting payload (mazda_tx_hook does not parse 0x440).

Lane-line overrides are not applied in this foundation: LANE_LINES is copied
from OEM FSC by the packer. Do not invent HUD graphics the OEM does not have.
"""
from __future__ import annotations

from dataclasses import dataclass

from opendbc.car import structs

VisualAlert = structs.CarControl.HUDControl.VisualAlert
GearShifter = structs.CarState.GearShifter

# HUD TX is 2 Hz (carcontroller frame % 50). Three ticks ≈ 1.5 s.
MIN_WARN_TICKS = 3

_PARK_REVERSE = (GearShifter.park, GearShifter.reverse)


@dataclass(frozen=True)
class HudInputs:
  lat_active: bool = False
  visual_alert: int = VisualAlert.none
  gear: int = GearShifter.drive
  standstill: bool = False
  lkas_allowed_speed: bool = True
  steer_fault_temporary: bool = False
  oem_hands_on: bool = False
  cruise_enabled: bool = False
  left_lane_visible: bool = False
  right_lane_visible: bool = False


@dataclass(frozen=True)
class HudOutput:
  steer_required: bool
  ldw: bool
  # Foundation: packer always copies OEM LANE_LINES. Exposed for tests / future.
  override_lane_lines: int | None
  reason: str
  priority: str  # P0 / P1 / P2 / P3 / NONE


class MazidHudBridge:
  """Stateful HUD policy: priority, debounce, OEM coexistence."""

  def __init__(self, min_warn_ticks: int = MIN_WARN_TICKS):
    self.min_warn_ticks = min_warn_ticks
    self._warn_hold = 0

  def update(self, inp: HudInputs) -> HudOutput:
    ldw = inp.visual_alert == VisualAlert.ldw

    # P0: takeover / steer-capability (same OEM HUD bits as hands-on).
    # Both map to VisualAlert.steerRequired today; distinguish parked
    # LKAS_BLOCK overlay (steerFaultTemporary @ standstill) from on-road warn.
    c4_warn = bool(inp.visual_alert == VisualAlert.steerRequired and inp.lkas_allowed_speed)
    parked_like = inp.standstill or inp.gear in _PARK_REVERSE
    suppress_c4 = parked_like and inp.steer_fault_temporary and not inp.lat_active
    if suppress_c4:
      c4_warn = False

    if parked_like and not inp.lat_active:
      # TEST 6: no false "assist active" HUD while parked / reverse.
      # We do not override LANE_LINES, so this only gates C4 hands bits.
      pass

    # TEST 5: OEM hands-on is not wiped. OR, never NAND.
    want_warn = c4_warn or inp.oem_hands_on

    if want_warn:
      self._warn_hold = self.min_warn_ticks
    steer_required = want_warn or self._warn_hold > 0
    if not want_warn and self._warn_hold > 0:
      self._warn_hold -= 1

    if steer_required and (c4_warn or inp.oem_hands_on or self._warn_hold > 0):
      if inp.lat_active and c4_warn:
        priority = "P0"
        reason = "steer_capability_or_takeover"
      elif inp.oem_hands_on and not c4_warn:
        priority = "P0"
        reason = "oem_hands_priority"
      elif suppress_c4 and inp.oem_hands_on:
        priority = "P0"
        reason = "oem_hands_priority"
      else:
        priority = "P1"
        reason = "hands_or_hold"
    elif inp.lat_active:
      priority = "P2"
      reason = "lat_active_copy_oem_lanes"
    else:
      priority = "P3"
      reason = "idle_copy_oem_lanes"

    # cruise_enabled is accepted so callers can share Vehicle State later.
    # This foundation must not pack ACC into 0x440 (TEST 7).
    _ = inp.cruise_enabled
    _ = inp.left_lane_visible
    _ = inp.right_lane_visible

    return HudOutput(
      steer_required=steer_required,
      ldw=ldw,
      override_lane_lines=None,
      reason=reason,
      priority=priority,
    )
