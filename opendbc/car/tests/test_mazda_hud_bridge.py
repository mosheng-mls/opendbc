"""HUD-BRIDGE-001/002 offline tests. No C4 deploy. No Safety / Panda / ICBM changes."""
from __future__ import annotations

import csv
import os
import unittest

from opendbc.car import structs
from opendbc.car.mazda.control_engagement import ControlMode, LANE_LINES_ACTIVE, LANE_LINES_STANDBY
from opendbc.car.mazda.hud_bridge import HudInputs, MazidHudBridge, MIN_WARN_TICKS
from opendbc.car.mazda.mazdacan import create_alert_command

VisualAlert = structs.CarControl.HUDControl.VisualAlert
GearShifter = structs.CarState.GearShifter

SERIES = r"D:\MazidBench\vehicle\mazid_v02_first_realcar_2026-08-18\analysis\series_10hz.csv"


def _cam(hands=0, lanes=1):
  return {
    "LINE_VISIBLE": 1,
    "LINE_NOT_VISIBLE": 0,
    "LANE_LINES": lanes,
    "BIT1": 0,
    "BIT2": 0,
    "BIT3": 0,
    "NO_ERR_BIT": 1,
    "S1": 0,
    "S1_HBEAM": 0,
    "HANDS_ON_STEER_WARN": hands,
  }


def _num(row, k):
  v = row.get(k, "")
  try:
    return float(v) if v not in ("", "None") else 0.0
  except ValueError:
    return 0.0


def _truth(row, k):
  return str(row.get(k, "")).strip() in ("True", "1", "true")


def _visual(row):
  name = (row.get("hud_visual") or "none").strip()
  if name == "steerRequired":
    return VisualAlert.steerRequired
  if name == "ldw":
    return VisualAlert.ldw
  return VisualAlert.none


def _gear(row):
  g = (row.get("gear") or "drive").strip()
  return getattr(GearShifter, g, GearShifter.drive)


def _from_row(row) -> HudInputs:
  return HudInputs(
    lat_active=_truth(row, "lat_active"),
    enabled=_truth(row, "cc_enabled") or _truth(row, "ss_enabled"),
    visual_alert=_visual(row),
    gear=_gear(row),
    standstill=_truth(row, "standstill"),
    lkas_allowed_speed=True,
    steer_fault_temporary=_truth(row, "steer_fault_tmp"),
    steer_fault_permanent=_truth(row, "steer_fault_perm"),
    oem_hands_on=_num(row, "oem_hands_warn") > 0,
    cruise_available=_truth(row, "cruise_avail"),
    cruise_enabled=_truth(row, "cruise_en"),
    v_cruise_kph=_num(row, "v_cruise"),
    hud_set_speed_kph=_num(row, "hud_set_kph"),
    fsc_lane_lines=int(_num(row, "cam_lane_lines") or 1),
    actuators_torque=_num(row, "act_torque"),
    steering_pressed=_truth(row, "steer_pressed"),
    brake_pressed=_truth(row, "brake"),
  )


class TestMazdaHudBridge(unittest.TestCase):
  def test1_lat_active_false_does_not_mark_active_priority(self):
    b = MazidHudBridge()
    out = b.update(HudInputs(lat_active=False, visual_alert=VisualAlert.none, fsc_lane_lines=2))
    self.assertFalse(out.steer_required)
    self.assertNotEqual(out.priority, "P2")
    self.assertFalse(out.lateral_engaged)
    self.assertEqual(out.override_lane_lines, LANE_LINES_STANDBY)
    self.assertEqual(out.control_mode, ControlMode.STANDBY)

  def test2_lat_active_true_is_stable_copy_oem_lanes(self):
    b = MazidHudBridge()
    last = None
    for _ in range(10):
      last = b.update(HudInputs(lat_active=True, visual_alert=VisualAlert.none, actuators_torque=0.0))
      self.assertEqual(last.priority, "P2")
      self.assertEqual(last.override_lane_lines, LANE_LINES_ACTIVE)
      self.assertFalse(last.steer_required)
      self.assertEqual(last.control_mode, ControlMode.LATERAL_ACTIVE)
    self.assertEqual(last.reason, "lat_active_control_path")

  def test3_steer_capability_warning_has_hud_priority(self):
    b = MazidHudBridge()
    out = b.update(HudInputs(
      lat_active=True,
      visual_alert=VisualAlert.steerRequired,
      lkas_allowed_speed=True,
      steer_fault_temporary=False,
    ))
    self.assertTrue(out.steer_required)
    self.assertEqual(out.priority, "P0")
    self.assertEqual(out.control_mode, ControlMode.TAKEOVER_REQUIRED)
    self.assertEqual(out.override_lane_lines, LANE_LINES_STANDBY)

  def test4_warning_obeys_minimum_display_time(self):
    b = MazidHudBridge(min_warn_ticks=MIN_WARN_TICKS)
    b.update(HudInputs(lat_active=True, visual_alert=VisualAlert.steerRequired))
    still = []
    for _ in range(MIN_WARN_TICKS):
      out = b.update(HudInputs(lat_active=True, visual_alert=VisualAlert.none))
      still.append(out.steer_required)
    self.assertTrue(all(still), still)
    after = b.update(HudInputs(lat_active=True, visual_alert=VisualAlert.none))
    self.assertFalse(after.steer_required)
    self.assertEqual(after.control_mode, ControlMode.LATERAL_ACTIVE)

  def test5_oem_warning_not_overridden_by_c4_suppress(self):
    b = MazidHudBridge()
    out = b.update(HudInputs(
      lat_active=False,
      visual_alert=VisualAlert.steerRequired,
      standstill=True,
      gear=GearShifter.park,
      steer_fault_temporary=True,
      oem_hands_on=True,
    ))
    self.assertTrue(out.steer_required)
    self.assertEqual(out.reason, "oem_hands_priority")
    self.assertEqual(out.control_mode, ControlMode.OFF)

  def test6_park_reverse_no_false_c4_hands(self):
    b = MazidHudBridge()
    park = b.update(HudInputs(
      lat_active=False,
      visual_alert=VisualAlert.steerRequired,
      standstill=True,
      gear=GearShifter.park,
      steer_fault_temporary=True,
      oem_hands_on=False,
    ))
    rev = b.update(HudInputs(
      lat_active=False,
      visual_alert=VisualAlert.steerRequired,
      standstill=False,
      gear=GearShifter.reverse,
      steer_fault_temporary=True,
      oem_hands_on=False,
    ))
    self.assertFalse(park.steer_required)
    self.assertFalse(rev.steer_required)
    self.assertNotEqual(park.control_mode, ControlMode.LATERAL_ACTIVE)
    self.assertNotEqual(rev.control_mode, ControlMode.LATERAL_ACTIVE)

  def test7_acc_flag_does_not_change_lateral_hud(self):
    b = MazidHudBridge()
    a = b.update(HudInputs(lat_active=True, cruise_enabled=False))
    c = b.update(HudInputs(lat_active=True, cruise_enabled=True))
    self.assertEqual(a.steer_required, c.steer_required)
    self.assertEqual(a.override_lane_lines, c.override_lane_lines)
    self.assertEqual(a.ldw, c.ldw)
    self.assertEqual(a.control_mode, ControlMode.LATERAL_ACTIVE)
    self.assertEqual(c.control_mode, ControlMode.LATERAL_ACTIVE)

  def test_packer_copies_oem_lanes_and_does_not_use_ldw_arg(self):
    from opendbc.can import CANPacker
    packer = CANPacker("mazda_3_2019_bm")
    msg_off = create_alert_command(packer, _cam(hands=0, lanes=2), ldw=True, steer_required=False)
    msg_on = create_alert_command(packer, _cam(hands=0, lanes=2), ldw=False, steer_required=True)
    msg_ldw = create_alert_command(packer, _cam(hands=0, lanes=2), ldw=True, steer_required=False)
    self.assertEqual(msg_off[0], 0x440)
    self.assertEqual(msg_on[0], 0x440)
    self.assertEqual(msg_off[1], msg_ldw[1])  # ldw arg is not packed
    self.assertNotEqual(msg_off[1], msg_on[1])
    forced_standby = create_alert_command(
      packer, _cam(hands=0, lanes=2), ldw=False, steer_required=False, lane_lines=LANE_LINES_STANDBY)
    self.assertNotEqual(msg_off[1], forced_standby[1])

  def test_rc_test_01_series_replay_facts(self):
    if not os.path.isfile(SERIES):
      self.skipTest(f"RC_TEST_01 series missing: {SERIES}")
    n = tx_hands = cam_hands = oem_hands = band = dual = 0
    with open(SERIES, newline="", encoding="utf-8") as f:
      for row in csv.DictReader(f):
        n += 1
        if _num(row, "tx_hands"):
          tx_hands += 1
        if _num(row, "cam_hands"):
          cam_hands += 1
        if _num(row, "oem_hands_warn"):
          oem_hands += 1
        v = _num(row, "v_ego_kph")
        if 65 <= v <= 75:
          band += 1
          if abs(_num(row, "tx_lkas")) > 20 and abs(_num(row, "cam_lkas")) > 20:
            dual += 1
    self.assertGreater(n, 30000)
    self.assertGreater(tx_hands, 3000)
    self.assertLess(cam_hands, 100)
    self.assertGreater(oem_hands, 3000)
    self.assertEqual(dual, 0)
    self.assertGreater(band, 2000)


class TestHudBridge002Engagement(unittest.TestCase):
  def test1_lanes_without_lat_is_standby(self):
    out = MazidHudBridge().update(HudInputs(
      lat_active=False, fsc_lane_lines=2, left_lane_visible=True, right_lane_visible=True))
    self.assertEqual(out.control_mode, ControlMode.STANDBY)
    self.assertFalse(out.lateral_engaged)
    self.assertEqual(out.override_lane_lines, LANE_LINES_STANDBY)

  def test2_set_speed_without_lat_is_not_c4_active(self):
    out = MazidHudBridge().update(HudInputs(
      lat_active=False, v_cruise_kph=80.0, hud_set_speed_kph=80.0, fsc_lane_lines=2))
    self.assertNotEqual(out.control_mode, ControlMode.LATERAL_ACTIVE)
    self.assertFalse(out.lateral_engaged)
    self.assertEqual(out.override_lane_lines, LANE_LINES_STANDBY)

  def test3_ready_without_control_is_standby(self):
    out = MazidHudBridge().update(HudInputs(
      lat_active=False, enabled=True, cruise_available=True))
    self.assertEqual(out.control_mode, ControlMode.STANDBY)
    self.assertFalse(out.lateral_engaged)

  def test4_lat_active_control_path_is_lateral_active(self):
    out = MazidHudBridge().update(HudInputs(lat_active=True, enabled=True))
    self.assertEqual(out.control_mode, ControlMode.LATERAL_ACTIVE)
    self.assertTrue(out.lateral_engaged)
    self.assertEqual(out.override_lane_lines, LANE_LINES_ACTIVE)

  def test5_zero_torque_still_lateral_active(self):
    out = MazidHudBridge().update(HudInputs(lat_active=True, actuators_torque=0.0))
    self.assertEqual(out.control_mode, ControlMode.LATERAL_ACTIVE)
    self.assertTrue(out.lateral_engaged)

  def test6_cancel_clears_active(self):
    b = MazidHudBridge()
    b.update(HudInputs(lat_active=True))
    out = b.update(HudInputs(lat_active=False, cancel=True, brake_pressed=True, cruise_enabled=False))
    self.assertNotEqual(out.control_mode, ControlMode.LATERAL_ACTIVE)
    self.assertFalse(out.lateral_engaged)
    self.assertEqual(out.override_lane_lines, LANE_LINES_STANDBY)

  def test7_steer_fault_is_temp_unavailable(self):
    out = MazidHudBridge().update(HudInputs(
      lat_active=False, steer_fault_temporary=True, gear=GearShifter.drive, standstill=False))
    self.assertEqual(out.control_mode, ControlMode.TEMP_UNAVAILABLE)
    self.assertFalse(out.lateral_engaged)

  def test8_steer_capability_is_takeover(self):
    out = MazidHudBridge().update(HudInputs(
      lat_active=True, visual_alert=VisualAlert.steerRequired, lkas_allowed_speed=True))
    self.assertEqual(out.control_mode, ControlMode.TAKEOVER_REQUIRED)
    self.assertTrue(out.steer_required)
    self.assertEqual(out.priority, "P0")
    self.assertEqual(out.override_lane_lines, LANE_LINES_STANDBY)

  def test9_reverse_is_not_active(self):
    out = MazidHudBridge().update(HudInputs(
      lat_active=True, gear=GearShifter.reverse, fsc_lane_lines=2))
    self.assertNotEqual(out.control_mode, ControlMode.LATERAL_ACTIVE)
    self.assertFalse(out.lateral_engaged)
    self.assertEqual(out.override_lane_lines, LANE_LINES_STANDBY)

  def test10_oem_acc_only_keeps_acc_semantics(self):
    out = MazidHudBridge().update(HudInputs(
      lat_active=False, cruise_enabled=True, cruise_available=True, v_cruise_kph=60.0, fsc_lane_lines=2))
    self.assertEqual(out.control_mode, ControlMode.OEM_LONGITUDINAL_ACTIVE)
    self.assertTrue(out.oem_acc_active)
    self.assertFalse(out.lateral_engaged)
    self.assertEqual(out.override_lane_lines, LANE_LINES_STANDBY)

  def test_disengage_is_fast(self):
    b = MazidHudBridge()
    b.update(HudInputs(lat_active=True))
    out = b.update(HudInputs(lat_active=False, enabled=True))
    self.assertEqual(out.control_mode, ControlMode.STANDBY)
    self.assertFalse(out.lateral_engaged)

  def test_rc_test_01_engagement_replay(self):
    if not os.path.isfile(SERIES):
      self.skipTest(f"RC_TEST_01 series missing: {SERIES}")
    b = MazidHudBridge()
    n = false_active = missed_active = standby_with_lanes = 0
    first_lat = None
    with open(SERIES, newline="", encoding="utf-8") as f:
      for i, row in enumerate(csv.DictReader(f)):
        n += 1
        out = b.update(_from_row(row))
        lat = _truth(row, "lat_active")
        parked = _gear(row) in (GearShifter.park, GearShifter.reverse)
        if out.control_mode == ControlMode.LATERAL_ACTIVE and not lat:
          false_active += 1
        if lat and (not parked) and out.control_mode not in (
            ControlMode.LATERAL_ACTIVE, ControlMode.DRIVER_OVERRIDE, ControlMode.TAKEOVER_REQUIRED):
          missed_active += 1
        if (not lat) and int(_num(row, "cam_lane_lines") or 1) == 2:
          if out.override_lane_lines == LANE_LINES_ACTIVE:
            standby_with_lanes += 1
        if lat and first_lat is None:
          first_lat = (i, row.get("wall_cst"), row.get("ss_enabled"), row.get("cruise_en"), row.get("v_cruise"))
    self.assertGreater(n, 30000)
    self.assertEqual(false_active, 0)
    self.assertEqual(missed_active, 0)
    self.assertEqual(standby_with_lanes, 0)
    self.assertIsNotNone(first_lat)
    # First latActive in RC_TEST_01 is MADS (ss_enabled false, no ACC).
    self.assertEqual(str(first_lat[2]), "False")
    self.assertEqual(str(first_lat[3]), "False")


if __name__ == "__main__":
  unittest.main()
