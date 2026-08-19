"""HUD-PROBE-003 offline tests. No Safety / Panda / ICBM / 0x243 changes."""
from __future__ import annotations

import csv
import inspect
import json
import os
import tempfile
import unittest
from enum import Enum
from types import SimpleNamespace
from unittest.mock import patch

from opendbc.can import CANPacker, CANParser
from opendbc.car import structs
from opendbc.car.mazda import mazdacan
from opendbc.car.mazda.carcontroller import CarController
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
  ProbeTick,
  TICKS_MAX_SAFE,
  VEGO_ABORT,
  max_safe_conflicts,
  normalize_gear,
)
from opendbc.car.mazda.interface import CarInterface
from opendbc.car.mazda.mazdacan import create_alert_command, create_steering_control
from opendbc.car.mazda.values import CAR, DBC, CarControllerParams

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


def _dynamic_gear(value):
  state = structs.CarState.new_message()
  state.gearShifter = value
  return state.gearShifter


def _new_controller():
  cp = CarInterface.get_non_essential_params(CAR.MAZDA_3_2019)
  controller = CarController(DBC[CAR.MAZDA_3_2019], cp)
  controller.hud_probe = None
  return controller


def _controller_io():
  state = structs.CarState.new_message()
  state.gearShifter = GearShifter.park
  state.standstill = True
  state.vEgo = 0.0
  state.vCruise = 0.0
  state.steeringTorque = 0.0
  state.steeringPressed = False
  state.brakePressed = False
  state.steerFaultTemporary = False
  state.steerFaultPermanent = False
  state.cruiseState.available = False
  state.cruiseState.enabled = False
  state.cruiseState.speed = 0.0

  control = structs.CarControl.new_message()
  control.enabled = False
  control.latActive = False
  control.actuators.torque = 0.0

  cs = SimpleNamespace(
    out=state.as_reader(),
    crz_btns_counter=0,
    lkas_allowed_speed=True,
    cam_lkas={"BIT_1": 0, "ERR_BIT_1": 0, "ERR_BIT_2": 0},
    cam_laneinfo=_cam(),
  )
  return control.as_reader(), cs


def _run_controller(controller, cycles=200):
  control, state = _controller_io()
  return [controller.update(control, state, i)[1] for i in range(cycles)]


def _messages(frames, address):
  return [msg for frame in frames for msg in frame if msg[0] == address]


def _lkas_counters(messages):
  parser = CANParser("mazda_3_2019_bm", [("CAM_LKAS", 100)], 0)
  counters = []
  for address, payload, _bus in messages:
    parser.update([(0, [(address, payload, 0)])])
    counters.append(int(parser.vl["CAM_LKAS"]["CTR"]))
  return counters


class _StaticProbe:
  def __init__(self):
    self.calls = 0
    self.failure_reason = ""

  def update(self, **_kwargs):
    self.calls += 1
    item = GALLERY[0]
    return ProbeTick(True, item, item.display, "", "STATIC", item.gid, "gallery")

  def log_tx(self, **_kwargs):
    return True


class _UpdateFailureProbe(_StaticProbe):
  def update(self, **_kwargs):
    self.calls += 1
    raise RuntimeError("injected probe update failure")


class _LogFailureProbe(_StaticProbe):
  def log_tx(self, **_kwargs):
    raise ValueError("injected probe log failure")


class _BridgeFailure:
  def __init__(self):
    self.calls = 0

  def update(self, _inputs):
    self.calls += 1
    raise RuntimeError("injected HudBridge update failure")


class TestHudProbeGearSerialization(unittest.TestCase):
  def test_dynamic_enum_begin_tx_abort_are_json_safe(self):
    with tempfile.TemporaryDirectory(prefix="hud_probe_enum_") as td:
      log_path = os.path.join(td, "probe.jsonl")
      probe = MazidHudProbe(label_path=os.path.join(td, "label"), log_path=log_path)
      park = _dynamic_gear(GearShifter.park)
      drive = _dynamic_gear(GearShifter.drive)
      self.assertEqual(type(park).__name__, "_DynamicEnum")
      with self.assertRaises(TypeError):
        int(park)

      probe.update(standstill=True, v_ego=0.0, gear=park, steer_fault_permanent=False, now_ns=1)
      probe.update(standstill=True, v_ego=0.0, gear=park, steer_fault_permanent=False, now_ns=2)
      self.assertTrue(probe.log_tx(
        gid="HUD-G01", payload_hex="00", v_ego=0.0, gear=park, standstill=True, now_ns=3,
      ))
      tick = probe.update(
        standstill=False, v_ego=1.0, gear=drive, steer_fault_permanent=False, now_ns=4,
      )
      self.assertEqual(tick.phase, "ABORT")
      self.assertEqual(probe.failure_reason, "")

      with open(log_path, encoding="utf-8") as f:
        records = [json.loads(line) for line in f]
      begin = next(r for r in records if r["event"] == "HUD_PROBE_BEGIN")
      tx = next(r for r in records if r["event"] == "HUD_PROBE_TX")
      abort = next(r for r in records if r["event"] == "STATIC_GALLERY_ABORT")
      self.assertEqual(begin["gear"], "park")
      self.assertEqual(tx["gear"], "park")
      self.assertEqual(abort["gear"], "drive")

  def test_normalize_gear_supports_enum_int_and_string(self):
    class OrdinaryGear(Enum):
      PARK = 1

    self.assertEqual(normalize_gear(OrdinaryGear.PARK), "park")
    self.assertEqual(normalize_gear(1), "park")
    self.assertEqual(normalize_gear("park"), "park")
    self.assertEqual(normalize_gear("GearShifter.PARK"), "park")
    self.assertEqual(normalize_gear("4"), "reverse")

    class BrokenGear:
      @property
      def raw(self):
        raise TypeError("broken raw")

    self.assertEqual(normalize_gear(BrokenGear()), "unavailable")

  def test_json_and_file_failures_do_not_escape_probe(self):
    probe = _park_probe()
    with patch("opendbc.car.mazda.hud_probe.json.dumps", side_effect=TypeError("json failure")):
      self.assertFalse(probe.log_tx(
        gid="HUD-G01", payload_hex="00", v_ego=0.0, gear=_dynamic_gear(GearShifter.park),
        standstill=True, now_ns=1,
      ))
    self.assertIn("log:TypeError", probe.failure_reason)

    probe = _park_probe()
    with patch("builtins.open", side_effect=ValueError("write failure")):
      self.assertFalse(probe.log_tx(
        gid="HUD-G01", payload_hex="00", v_ego=0.0, gear=GearShifter.park,
        standstill=True, now_ns=1,
      ))
    self.assertIn("log:ValueError", probe.failure_reason)

    probe = _park_probe()
    with patch("builtins.open", side_effect=RuntimeError("label failure")):
      self.assertFalse(probe._write_label("HUD-G01"))
    self.assertIn("label:RuntimeError", probe.failure_reason)


class TestHudFailureIsolation(unittest.TestCase):
  @classmethod
  def setUpClass(cls):
    cls.baseline_frames = _run_controller(_new_controller())
    cls.baseline_steering_payloads = [msg[1] for msg in _messages(cls.baseline_frames, 0x243)]

  def assert_continuous_control(self, frames, expected_hud_count):
    steering = _messages(frames, 0x243)
    self.assertEqual(len(frames), 200)
    self.assertEqual(len(steering), 200)
    self.assertEqual([msg[1] for msg in steering], self.baseline_steering_payloads)
    self.assertEqual(_lkas_counters(steering), [i % 16 for i in range(200)])
    self.assertEqual(len(_messages(frames, 0x440)), expected_hud_count)
    self.assertTrue(all(sum(msg[0] == 0x440 for msg in frame) <= 1 for frame in frames))
    self.assertEqual(_messages(frames, 0x09D), [])

  def test_real_dynamic_enum_probe_runs_200_cycles(self):
    controller = _new_controller()
    with tempfile.TemporaryDirectory(prefix="hud_probe_controller_") as td:
      log_path = os.path.join(td, "probe.jsonl")
      controller.hud_probe = MazidHudProbe(label_path=os.path.join(td, "label"), log_path=log_path)
      frames = _run_controller(controller)
      self.assert_continuous_control(frames, expected_hud_count=4)
      self.assertFalse(controller.hud_probe_disabled)
      self.assertFalse(controller.hud_deployment_blocked)
      with open(log_path, encoding="utf-8") as f:
        records = [json.loads(line) for line in f]
    self.assertTrue(any(r["event"] == "HUD_PROBE_BEGIN" and r["gear"] == "park" for r in records))
    self.assertTrue(any(r["event"] == "HUD_PROBE_TX" and r["gear"] == "park" for r in records))

  def test_probe_update_failure_latches_and_runs_200_cycles(self):
    controller = _new_controller()
    probe = _UpdateFailureProbe()
    controller.hud_probe = probe
    with self.assertLogs("opendbc.car.mazda.carcontroller", level="ERROR") as logs:
      frames = _run_controller(controller)
    self.assert_continuous_control(frames, expected_hud_count=4)
    self.assertEqual(probe.calls, 1)
    self.assertTrue(controller.hud_probe_disabled)
    self.assertTrue(controller.hud_deployment_blocked)
    self.assertIn("feature=hud_probe.update", controller.hud_last_fault)
    self.assertTrue(any("DEPLOYMENT_BLOCKED" in line for line in logs.output))

  def test_probe_begin_serialization_failure_is_reported_and_latched(self):
    controller = _new_controller()
    with tempfile.TemporaryDirectory(prefix="hud_probe_json_failure_") as td:
      controller.hud_probe = MazidHudProbe(
        label_path=os.path.join(td, "label"), log_path=os.path.join(td, "probe.jsonl"),
      )
      with self.assertLogs("opendbc.car.mazda.carcontroller", level="ERROR"):
        with patch("opendbc.car.mazda.hud_probe.json.dumps", side_effect=TypeError("injected BEGIN serialization failure")):
          frames = _run_controller(controller)
    self.assert_continuous_control(frames, expected_hud_count=4)
    self.assertTrue(controller.hud_probe_disabled)
    self.assertTrue(controller.hud_deployment_blocked)
    self.assertIn("feature=hud_probe.update", controller.hud_last_fault)

  def test_probe_log_failure_latches_and_falls_back_same_tick(self):
    controller = _new_controller()
    probe = _LogFailureProbe()
    controller.hud_probe = probe
    with self.assertLogs("opendbc.car.mazda.carcontroller", level="ERROR"):
      frames = _run_controller(controller)
    self.assert_continuous_control(frames, expected_hud_count=4)
    self.assertEqual(probe.calls, 1)
    self.assertTrue(controller.hud_probe_disabled)
    self.assertIn("feature=hud_probe.log_tx", controller.hud_last_fault)

  def test_probe_candidate_pack_typeerror_falls_back_same_tick(self):
    controller = _new_controller()
    probe = _StaticProbe()
    controller.hud_probe = probe
    real_create_alert = mazdacan.create_alert_command

    def injected(*args, **kwargs):
      if kwargs.get("hands_warn_3") is not None:
        raise TypeError("injected probe candidate pack failure")
      return real_create_alert(*args, **kwargs)

    with self.assertLogs("opendbc.car.mazda.carcontroller", level="ERROR"):
      with patch.object(mazdacan, "create_alert_command", side_effect=injected):
        frames = _run_controller(controller)
    self.assert_continuous_control(frames, expected_hud_count=4)
    self.assertEqual(probe.calls, 1)
    self.assertTrue(controller.hud_probe_disabled)
    self.assertIn("feature=hud_probe.pack", controller.hud_last_fault)

  def test_bridge_update_failure_replays_exact_oem_copy(self):
    controller = _new_controller()
    bridge = _BridgeFailure()
    controller.hud_bridge = bridge
    expected = _pack(next(g.display for g in GALLERY if g.gid == "HUD-G00"), _cam())[1]
    with self.assertLogs("opendbc.car.mazda.carcontroller", level="ERROR"):
      frames = _run_controller(controller)
    self.assert_continuous_control(frames, expected_hud_count=4)
    self.assertEqual(bridge.calls, 1)
    self.assertTrue(controller.hud_enhancement_disabled)
    self.assertTrue(all(msg[1] == expected for msg in _messages(frames, 0x440)))
    self.assertIn("feature=hud_bridge.update", controller.hud_last_fault)

  def test_enhanced_pack_valueerror_replays_exact_oem_copy(self):
    controller = _new_controller()
    real_create_alert = mazdacan.create_alert_command

    def injected(*args, **kwargs):
      if not kwargs.get("copy_oem", False):
        raise ValueError("injected enhanced pack failure")
      return real_create_alert(*args, **kwargs)

    expected = _pack(next(g.display for g in GALLERY if g.gid == "HUD-G00"), _cam())[1]
    with self.assertLogs("opendbc.car.mazda.carcontroller", level="ERROR"):
      with patch.object(mazdacan, "create_alert_command", side_effect=injected):
        frames = _run_controller(controller)
    self.assert_continuous_control(frames, expected_hud_count=4)
    self.assertTrue(controller.hud_enhancement_disabled)
    self.assertTrue(all(msg[1] == expected for msg in _messages(frames, 0x440)))
    self.assertIn("feature=hud_bridge.pack", controller.hud_last_fault)

  def test_transient_oem_copy_failure_recovers_on_next_hud_tick(self):
    controller = _new_controller()
    controller.hud_enhancement_disabled = True
    real_create_alert = mazdacan.create_alert_command
    attempts = 0

    def injected(*args, **kwargs):
      nonlocal attempts
      if kwargs.get("copy_oem", False):
        attempts += 1
        if attempts == 1:
          raise ValueError("injected transient OEM-copy failure")
      return real_create_alert(*args, **kwargs)

    with self.assertLogs("opendbc.car.mazda.carcontroller", level="ERROR"):
      with patch.object(mazdacan, "create_alert_command", side_effect=injected):
        frames = _run_controller(controller)
    self.assert_continuous_control(frames, expected_hud_count=3)
    self.assertEqual(attempts, 4)
    self.assertFalse(controller.hud_oem_copy_fault_active)
    self.assertTrue(controller.hud_deployment_blocked)
    self.assertIn("feature=hud_oem_copy.pack", controller.hud_last_fault)

  def test_persistent_oem_copy_failure_sends_no_fake_or_duplicate(self):
    controller = _new_controller()
    controller.hud_enhancement_disabled = True

    def injected(*_args, **kwargs):
      if kwargs.get("copy_oem", False):
        raise TypeError("injected persistent OEM-copy failure")
      raise AssertionError("enhanced HUD must stay disabled")

    with self.assertLogs("opendbc.car.mazda.carcontroller", level="ERROR") as logs:
      with patch.object(mazdacan, "create_alert_command", side_effect=injected):
        frames = _run_controller(controller)
    self.assert_continuous_control(frames, expected_hud_count=0)
    self.assertTrue(controller.hud_oem_copy_fault_active)
    self.assertTrue(controller.hud_deployment_blocked)
    self.assertEqual(len(controller.hud_faults), 1)
    self.assertEqual(sum("feature=hud_oem_copy.pack" in line for line in logs.output), 1)

  def test_core_steering_packer_exception_still_propagates(self):
    controller = _new_controller()
    controller.hud_probe = _UpdateFailureProbe()
    control, state = _controller_io()
    with patch.object(mazdacan, "create_steering_control", side_effect=RuntimeError("core steering failure")):
      with self.assertRaisesRegex(RuntimeError, "core steering failure"):
        controller.update(control, state, 0)
    self.assertEqual(controller.frame, 0)
    self.assertFalse(controller.hud_deployment_blocked)

  def test_core_icbm_exception_still_propagates(self):
    class BrokenIcbm:
      def update(self, *_args, **_kwargs):
        raise ValueError("core ICBM failure")

    controller = _new_controller()
    controller.icbm = BrokenIcbm()
    control, state = _controller_io()
    with self.assertRaisesRegex(ValueError, "core ICBM failure"):
      controller.update(control, state, 0)
    self.assertEqual(controller.frame, 0)
    self.assertFalse(controller.hud_deployment_blocked)


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
