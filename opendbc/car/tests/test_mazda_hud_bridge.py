"""HUD-BRIDGE-001 offline tests. No C4 deploy. No Safety / Panda / ICBM changes."""
from __future__ import annotations

import csv
import os
import unittest

from opendbc.car import structs
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


class TestMazdaHudBridge(unittest.TestCase):
  def test1_lat_active_false_does_not_mark_active_priority(self):
    b = MazidHudBridge()
    out = b.update(HudInputs(lat_active=False, visual_alert=VisualAlert.none))
    self.assertFalse(out.steer_required)
    self.assertNotEqual(out.priority, "P2")
    self.assertIsNone(out.override_lane_lines)

  def test2_lat_active_true_is_stable_copy_oem_lanes(self):
    b = MazidHudBridge()
    last = None
    for _ in range(10):
      last = b.update(HudInputs(lat_active=True, visual_alert=VisualAlert.none))
      self.assertEqual(last.priority, "P2")
      self.assertIsNone(last.override_lane_lines)
      self.assertFalse(last.steer_required)
    self.assertEqual(last.reason, "lat_active_copy_oem_lanes")

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

  def test7_acc_flag_does_not_change_lateral_hud(self):
    b = MazidHudBridge()
    a = b.update(HudInputs(lat_active=True, cruise_enabled=False))
    c = b.update(HudInputs(lat_active=True, cruise_enabled=True))
    self.assertEqual(a.steer_required, c.steer_required)
    self.assertEqual(a.override_lane_lines, c.override_lane_lines)
    self.assertEqual(a.ldw, c.ldw)

  def test_packer_copies_oem_lanes_and_does_not_use_ldw_arg(self):
    # Packing contract: LANE_LINES from cam_msg; unused ldw arg must not invent bits.
    from opendbc.can import CANPacker
    packer = CANPacker("mazda_3_2019_bm")
    msg_off = create_alert_command(packer, _cam(hands=0, lanes=2), ldw=True, steer_required=False)
    msg_on = create_alert_command(packer, _cam(hands=0, lanes=2), ldw=False, steer_required=True)
    msg_ldw = create_alert_command(packer, _cam(hands=0, lanes=2), ldw=True, steer_required=False)
    self.assertEqual(msg_off[0], 0x440)
    self.assertEqual(msg_on[0], 0x440)
    self.assertEqual(msg_off[1], msg_ldw[1])  # ldw arg is not packed
    self.assertNotEqual(msg_off[1], msg_on[1])

  def test_rc_test_01_series_replay_facts(self):
    if not os.path.isfile(SERIES):
      self.skipTest(f"RC_TEST_01 series missing: {SERIES}")
    n = tx_hands = cam_hands = oem_hands = band = dual = 0
    with open(SERIES, newline="", encoding="utf-8") as f:
      for row in csv.DictReader(f):
        n += 1
        def num(k):
          v = row.get(k, "")
          try:
            return float(v) if v not in ("", "None") else 0.0
          except ValueError:
            return 0.0
        if num("tx_hands"):
          tx_hands += 1
        if num("cam_hands"):
          cam_hands += 1
        if num("oem_hands_warn"):
          oem_hands += 1
        v = num("v_ego_kph")
        if 65 <= v <= 75:
          band += 1
          if abs(num("tx_lkas")) > 20 and abs(num("cam_lkas")) > 20:
            dual += 1
    self.assertGreater(n, 30000)
    self.assertGreater(tx_hands, 3000)
    self.assertLess(cam_hands, 100)
    self.assertGreater(oem_hands, 3000)
    self.assertEqual(dual, 0)
    self.assertGreater(band, 2000)


if __name__ == "__main__":
  unittest.main()
