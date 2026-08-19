"""HUD-PROBE-003 offline tests. No Safety / Panda / ICBM / 0x243 changes."""
from __future__ import annotations

import csv
import inspect
import os
import tempfile
import unittest

from opendbc.can import CANPacker, CANParser
from opendbc.car import structs
from opendbc.car.mazda.control_engagement import ControlMode, LANE_LINES_ACTIVE, LANE_LINES_STANDBY
from opendbc.car.mazda.hud_bridge import HudInputs, MazidHudBridge
from opendbc.car.mazda.hud_probe import (
  DISPLAY_ONLY_CONFIRMED,
  GALLERY,
  GALLERY_COUNT,
  GALLERY_IDS,
  MAX_SAFE,
  MAX_SAFE_VISUAL_PAYLOAD_MAP,
  MazidHudProbe,
  TICKS_MAX_SAFE,
  VEGO_ABORT,
  max_safe_conflicts,
)
from opendbc.car.mazda.mazdacan import create_alert_command, create_steering_control
from opendbc.car.mazda.values import CarControllerParams

VisualAlert = structs.CarControl.HUDControl.VisualAlert
GearShifter = structs.CarState.GearShifter

SERIES = r"D:\MazidBench\vehicle\mazid_v02_first_realcar_2026-08-18\analysis\series_10hz.csv"

RESERVED = ("BIT1", "BIT2", "BIT3", "NO_ERR_BIT", "S1", "S1_HBEAM")
FORBIDDEN_GALLERY = ("TJA", "TJA_TRANSITION", "LDW_WARN_LL", "LDW_WARN_RL", "ERR_BIT")


def _cam(**kw):
  base = {
    "LINE_VISIBLE": 0,
    "LINE_NOT_VISIBLE": 1,
    "LANE_LINES": 1,
    "BIT1": 1,
    "BIT2": 1,
    "BIT3": 1,
    "NO_ERR_BIT": 1,
    "S1": 1,
    "S1_HBEAM": 1,
    "HANDS_ON_STEER_WARN": 0,
    "HANDS_ON_STEER_WARN_2": 0,
    "HANDS_WARN_3_BITS": 0,
  }
  base.update(kw)
  return base


def _decode(msg):
  addr, dat, _bus = msg
  cp = CANParser("mazda_3_2019_bm", [("CAM_LANEINFO", 2)], 0)
  cp.update([(0, [(addr, dat, 0)])])
  return dict(cp.vl["CAM_LANEINFO"])


def _pack(disp, cam=None):
  packer = CANPacker("mazda_3_2019_bm")
  cam = cam or _cam()
  return create_alert_command(
    packer, cam, False, False,
    lane_lines=None if disp.copy_oem else disp.lane_lines,
    line_visible=None if disp.copy_oem else disp.line_visible,
    line_not_visible=None if disp.copy_oem else disp.line_not_visible,
    hands_on=None if disp.copy_oem else disp.hands_on,
    hands_on_2=None if disp.copy_oem else disp.hands_on_2,
    hands_warn_3=None if disp.copy_oem else disp.hands_warn_3,
    copy_oem=disp.copy_oem,
  )


def _park_probe():
  td = tempfile.mkdtemp(prefix="hud_probe_")
  return MazidHudProbe(label_path=os.path.join(td, "label"), log_path=os.path.join(td, "log.jsonl"))


def _run_until_gid(p: MazidHudProbe, gid: str, **kw):
  defaults = dict(standstill=True, v_ego=0.0, gear=GearShifter.park, steer_fault_permanent=False, now_ns=0)
  defaults.update(kw)
  for i in range(400):
    defaults["now_ns"] = i
    tick = p.update(**defaults)
    if tick.gid == gid and tick.static:
      return tick
  raise AssertionError(f"never reached {gid}")


class TestHudProbeStaticGallery(unittest.TestCase):
  def test_max_safe_has_no_mutex(self):
    self.assertEqual(max_safe_conflicts(MAX_SAFE), [])
    self.assertEqual(MAX_SAFE.line_visible, 1)
    self.assertEqual(MAX_SAFE.line_not_visible, 0)
    self.assertEqual(MAX_SAFE.lane_lines, 2)
    self.assertEqual(MAX_SAFE_VISUAL_PAYLOAD_MAP["LANE_LINES"], 2)

  def test_gallery_starts_with_max_safe(self):
    self.assertEqual(GALLERY[0].gid, "HUD-G01")
    self.assertEqual(GALLERY[0].name, "MAX_SAFE_VISUAL")
    self.assertEqual(GALLERY[0].ticks, TICKS_MAX_SAFE)
    self.assertIn("HUD-G00", GALLERY_IDS)
    self.assertEqual(GALLERY_COUNT, 16)

  def test_park_allows_gallery(self):
    p = _park_probe()
    t = _run_until_gid(p, "HUD-G01")
    self.assertTrue(t.static)
    self.assertEqual(t.phase, "STATIC")
    self.assertIn("最大显示组合", t.label)

  def test_move_aborts_immediately(self):
    p = _park_probe()
    _run_until_gid(p, "HUD-G01")
    tick = p.update(standstill=False, v_ego=1.0, gear=GearShifter.park,
                    steer_fault_permanent=False, now_ns=99)
    self.assertTrue(p.aborted)
    self.assertEqual(tick.phase, "ABORT")
    self.assertFalse(tick.static)
    later = p.update(standstill=True, v_ego=0.0, gear=GearShifter.park,
                     steer_fault_permanent=False, now_ns=100)
    self.assertEqual(later.phase, "DYNAMIC")
    self.assertFalse(later.static)

  def test_vego_abort_threshold(self):
    p = _park_probe()
    _run_until_gid(p, "HUD-G01")
    tick = p.update(standstill=True, v_ego=VEGO_ABORT + 0.01, gear=GearShifter.park,
                    steer_fault_permanent=False, now_ns=50)
    self.assertEqual(tick.phase, "ABORT")

  def test_reverse_aborts(self):
    p = _park_probe()
    _run_until_gid(p, "HUD-G01")
    tick = p.update(standstill=True, v_ego=0.0, gear=GearShifter.reverse,
                    steer_fault_permanent=False, now_ns=50)
    self.assertEqual(tick.phase, "ABORT")

  def test_drive_aborts(self):
    p = _park_probe()
    _run_until_gid(p, "HUD-G01")
    tick = p.update(standstill=True, v_ego=0.0, gear=GearShifter.drive,
                    steer_fault_permanent=False, now_ns=50)
    self.assertEqual(tick.phase, "ABORT")

  def test_each_gxx_only_allowed_fields(self):
    cam = _cam()
    for item in GALLERY:
      vl = _decode(_pack(item.display, cam))
      for k in RESERVED:
        self.assertEqual(vl[k], cam[k], f"{item.gid} touched reserved {k}")
      for k in FORBIDDEN_GALLERY:
        self.assertEqual(vl.get(k, 0), 0, f"{item.gid} packed {k}")
      if not item.display.copy_oem:
        self.assertIn("LANE_LINES", DISPLAY_ONLY_CONFIRMED)

  def test_unknown_and_reserved_bits_kept(self):
    cam = _cam(BIT1=1, BIT2=0, BIT3=1, S1=0, S1_HBEAM=1, NO_ERR_BIT=1)
    vl = _decode(_pack(MAX_SAFE, cam))
    self.assertEqual(vl["BIT1"], 1)
    self.assertEqual(vl["BIT2"], 0)
    self.assertEqual(vl["BIT3"], 1)
    self.assertEqual(vl["S1"], 0)
    self.assertEqual(vl["S1_HBEAM"], 1)
    self.assertEqual(vl["NO_ERR_BIT"], 1)
    self.assertEqual(vl["TJA"], 0)
    self.assertEqual(vl["TJA_TRANSITION"], 0)

  def test_max_safe_payload_fields(self):
    vl = _decode(_pack(MAX_SAFE, _cam()))
    self.assertEqual(int(vl["LANE_LINES"]), 2)
    self.assertEqual(int(vl["LINE_VISIBLE"]), 1)
    self.assertEqual(int(vl["LINE_NOT_VISIBLE"]), 0)
    self.assertEqual(int(vl["HANDS_ON_STEER_WARN"]), 1)
    self.assertEqual(int(vl["HANDS_ON_STEER_WARN_2"]), 1)
    self.assertEqual(int(vl["HANDS_WARN_3_BITS"]), 7)

  def test_hands_bits_isolated(self):
    g09 = next(g for g in GALLERY if g.gid == "HUD-G09")
    g10 = next(g for g in GALLERY if g.gid == "HUD-G10")
    v9 = _decode(_pack(g09.display))
    v10 = _decode(_pack(g10.display))
    self.assertEqual(int(v9["HANDS_ON_STEER_WARN"]), 1)
    self.assertEqual(int(v9["HANDS_ON_STEER_WARN_2"]), 0)
    self.assertEqual(int(v10["HANDS_ON_STEER_WARN"]), 0)
    self.assertEqual(int(v10["HANDS_ON_STEER_WARN_2"]), 1)

  def test_selected_has_no_independent_signal(self):
    g03 = next(g for g in GALLERY if g.gid == "HUD-G03")
    g06 = next(g for g in GALLERY if g.gid == "HUD-G06")
    self.assertEqual(_pack(g03.display)[1], _pack(g06.display)[1])

  def test_0x243_packer_untouched(self):
    src = inspect.getsource(create_steering_control)
    self.assertIn("LKAS_REQUEST", src)
    self.assertNotIn("hud_probe", src)
    self.assertNotIn("LANE_LINES", src)
    packer = CANPacker("mazda_3_2019_bm")
    lkas = {"BIT_1": 0, "ERR_BIT_1": 0, "ERR_BIT_2": 0}
    msg = create_steering_control(packer, type("CP", (), {"flags": 1})(), 0, 0, lkas)
    self.assertEqual(msg[0], 0x243)

  def test_steer_max_unchanged(self):
    self.assertEqual(CarControllerParams.STEER_MAX, 800)

  def test_label_clears_on_abort(self):
    p = _park_probe()
    _run_until_gid(p, "HUD-G01")
    self.assertTrue(os.path.isfile(p.label_path))
    p.update(standstill=False, v_ego=2.0, gear=GearShifter.park,
             steer_fault_permanent=False, now_ns=9)
    with open(p.label_path, encoding="utf-8") as f:
      self.assertEqual(f.read().strip(), "")


class TestHudProbeDynamic(unittest.TestCase):
  def test_road_seen_not_lat_is_standby(self):
    out = MazidHudBridge().update(HudInputs(
      lat_active=False, left_lane_visible=True, right_lane_visible=True))
    self.assertEqual(out.control_mode, ControlMode.STANDBY)
    self.assertFalse(out.lateral_engaged)
    self.assertEqual(out.override_lane_lines, LANE_LINES_STANDBY)
    self.assertEqual(out.line_visible, 1)
    self.assertTrue(out.road_seen)

  def test_acc_without_lat_is_not_active(self):
    out = MazidHudBridge().update(HudInputs(
      lat_active=False, cruise_enabled=True, cruise_available=True, v_cruise_kph=70.0,
      left_lane_visible=True))
    self.assertNotEqual(out.control_mode, ControlMode.LATERAL_ACTIVE)
    self.assertFalse(out.lateral_engaged)
    self.assertEqual(out.override_lane_lines, LANE_LINES_STANDBY)

  def test_vcruise_without_lat_is_not_active(self):
    out = MazidHudBridge().update(HudInputs(lat_active=False, v_cruise_kph=80.0))
    self.assertNotEqual(out.control_mode, ControlMode.LATERAL_ACTIVE)
    self.assertFalse(out.lateral_engaged)

  def test_lat_active_is_active(self):
    out = MazidHudBridge().update(HudInputs(lat_active=True, actuators_torque=0.0))
    self.assertEqual(out.control_mode, ControlMode.LATERAL_ACTIVE)
    self.assertTrue(out.lateral_engaged)
    self.assertEqual(out.override_lane_lines, LANE_LINES_ACTIVE)

  def test_disengage_clears_active(self):
    b = MazidHudBridge()
    b.update(HudInputs(lat_active=True))
    out = b.update(HudInputs(lat_active=False, v_cruise_kph=70.0, enabled=True))
    self.assertFalse(out.lateral_engaged)
    self.assertEqual(out.override_lane_lines, LANE_LINES_STANDBY)

  def test_temp_unavailable_clears_active(self):
    b = MazidHudBridge()
    b.update(HudInputs(lat_active=True))
    out = b.update(HudInputs(lat_active=False, steer_fault_temporary=True, gear=GearShifter.drive))
    self.assertEqual(out.control_mode, ControlMode.TEMP_UNAVAILABLE)
    self.assertFalse(out.lateral_engaged)
    self.assertEqual(out.override_lane_lines, LANE_LINES_STANDBY)

  def test_takeover_covers_active(self):
    out = MazidHudBridge().update(HudInputs(
      lat_active=True, visual_alert=VisualAlert.steerRequired, lkas_allowed_speed=True))
    self.assertEqual(out.control_mode, ControlMode.TAKEOVER_REQUIRED)
    self.assertTrue(out.steer_required)
    self.assertEqual(out.override_lane_lines, LANE_LINES_STANDBY)

  def test_oem_hands_not_wiped(self):
    out = MazidHudBridge().update(HudInputs(
      lat_active=False, oem_hands_on=True, standstill=True, gear=GearShifter.park,
      steer_fault_temporary=True, visual_alert=VisualAlert.steerRequired))
    self.assertTrue(out.steer_required)

  def test_reverse_not_active(self):
    out = MazidHudBridge().update(HudInputs(lat_active=True, gear=GearShifter.reverse))
    self.assertNotEqual(out.control_mode, ControlMode.LATERAL_ACTIVE)
    self.assertFalse(out.lateral_engaged)


class TestHudProbeRcTest01Replay(unittest.TestCase):
  def test_active_at_mads_not_set_speed(self):
    if not os.path.isfile(SERIES):
      self.skipTest(f"RC_TEST_01 series missing: {SERIES}")
    from opendbc.car.tests.test_mazda_hud_bridge import _from_row, _truth, _num, _gear
    b = MazidHudBridge()
    first_active_wall = None
    first_acc_wall = None
    vcruise_after_exit_still_active = 0
    with open(SERIES, newline="", encoding="utf-8") as f:
      for row in csv.DictReader(f):
        out = b.update(_from_row(row))
        wall = row.get("wall_cst") or ""
        if out.control_mode == ControlMode.LATERAL_ACTIVE and first_active_wall is None:
          first_active_wall = wall
          self.assertFalse(_truth(row, "ss_enabled"))
          self.assertFalse(_truth(row, "cruise_en"))
        if first_acc_wall is None and _num(row, "v_cruise") > 0 and _num(row, "v_cruise") < 250:
          first_acc_wall = wall
        if (not _truth(row, "lat_active")) and _num(row, "v_cruise") > 0 and _num(row, "v_cruise") < 250:
          if out.lateral_engaged or out.override_lane_lines == LANE_LINES_ACTIVE:
            vcruise_after_exit_still_active += 1
        parked = _gear(row) in (GearShifter.park, GearShifter.reverse)
        if _truth(row, "lat_active") and not parked:
          self.assertIn(out.control_mode, (
            ControlMode.LATERAL_ACTIVE, ControlMode.DRIVER_OVERRIDE, ControlMode.TAKEOVER_REQUIRED))
    self.assertIsNotNone(first_active_wall)
    self.assertIn("23:34:38", first_active_wall)
    self.assertIsNotNone(first_acc_wall)
    self.assertGreater(first_acc_wall, first_active_wall)
    self.assertEqual(vcruise_after_exit_still_active, 0)


if __name__ == "__main__":
  unittest.main()
