#!/usr/bin/env python3
"""PC-only gates for the Mazda OEM SET-speed assist request."""
import unittest
from types import SimpleNamespace

from opendbc.can import CANPacker
from opendbc.car import Bus, structs
from opendbc.car.mazda import mazdacan
from opendbc.car.mazda.carcontroller import (
  CarController,
  SET_SPEED_REQUEST_MAX_AGE_NS,
)
from opendbc.car.mazda.carstate import CarState as MazdaCarState
from opendbc.car.mazda.interface import CarInterface
from opendbc.car.mazda.values import CAR, DBC, Buttons


class TestMazdaSetSpeedControl(unittest.TestCase):
  NOW_NS = 10_000_000_000

  @staticmethod
  def controller(*, openpilot_longitudinal: bool = False) -> CarController:
    cp = CarInterface.get_non_essential_params(CAR.MAZDA_3_2019)
    cp.openpilotLongitudinalControl = openpilot_longitudinal
    return CarController(DBC[CAR.MAZDA_3_2019], cp)

  @staticmethod
  def control(*, enabled: bool = True, target_valid: bool = True,
              target_speed: float = 20.0, driver_set_speed: float = 30.0,
              source_mono_time: int = NOW_NS, curve_warning: bool = False):
    cc = structs.CarControl()
    cc.enabled = True
    cc.oemCruiseSetSpeedAssist.enabled = enabled
    cc.oemCruiseSetSpeedAssist.targetValid = target_valid
    cc.oemCruiseSetSpeedAssist.targetSpeed = target_speed
    cc.oemCruiseSetSpeedAssist.driverSetSpeed = driver_set_speed
    cc.oemCruiseSetSpeedAssist.sourceMonoTime = source_mono_time
    cc.oemCruiseSetSpeedAssist.curveWarning = curve_warning
    cc.oemCruiseSetSpeedAssist.curveTargetSpeed = 0.0
    cc.oemCruiseSetSpeedAssist.curveRequiredDecel = 0.0
    cc.oemCruiseSetSpeedAssist.curveTimeToTarget = 0.0
    return cc

  @staticmethod
  def state(**overrides):
    values = {
      "canValid": True,
      "canTimeout": False,
      "cruiseState": SimpleNamespace(available=True, enabled=True, speed=20.0),
      "brakePressed": False,
      "gasPressed": False,
      "buttonEvents": [],
      "gearShifter": structs.CarState.GearShifter.drive,
      "standstill": False,
      "steeringPressed": False,
      "steeringTorque": 0.0,
      "steeringAngleDeg": 0.0,
      "steerFaultTemporary": False,
      "steerFaultPermanent": False,
      "vEgo": 15.0,
      "vEgoRaw": 15.0,
      "vCruise": 72.0,
    }
    values.update(overrides)
    return SimpleNamespace(
      out=SimpleNamespace(**values),
      engine_speed_ms=15.0,
      crz_btns_counter=0,
      bm_radar_startup_ready=False,
      stock_radar_alive=True,
      stock_radar_has_lead=False,
      lkas_allowed_speed=True,
      cam_lkas={"BIT_1": 0, "ERR_BIT_1": 0, "ERR_BIT_2": 0},
      cam_laneinfo={signal: 0 for signal in (
        "LINE_VISIBLE", "LINE_NOT_VISIBLE", "LANE_LINES", "BIT1", "BIT2", "BIT3", "NO_ERR_BIT", "S1", "S1_HBEAM",
      )},
    )

  @staticmethod
  def set_messages(can_sends):
    return [msg for msg in can_sends if msg[0] == 0x09D]

  def test_disabled_request_is_fail_off_without_carstate_dependencies(self):
    controller = self.controller()
    cc = self.control(enabled=False)
    minimal_state = SimpleNamespace(out=SimpleNamespace())
    self.assertIsNone(controller._get_icbm_assist_target(cc, minimal_state, self.NOW_NS))

  def test_hot_off_resets_and_reenable_requires_cooldown(self):
    controller = self.controller()
    controller.frame = 100
    cc = self.control()
    cs = self.state()

    self.assertEqual(controller._get_icbm_assist_target(cc, cs, self.NOW_NS), 20.0)
    self.assertTrue(controller.icbm_assist_enabled)
    self.assertEqual(controller.icbm_assist.last_press_frame, 100)

    controller.frame = 110
    cc.oemCruiseSetSpeedAssist.enabled = False
    self.assertIsNone(controller._get_icbm_assist_target(cc, cs, self.NOW_NS))
    self.assertFalse(controller.icbm_assist_enabled)
    self.assertEqual(controller.icbm_assist.last_press_frame, 110)

    controller.frame = 120
    cc.oemCruiseSetSpeedAssist.enabled = True
    self.assertEqual(controller._get_icbm_assist_target(cc, cs, self.NOW_NS), 20.0)
    self.assertEqual(controller.icbm_assist.last_press_frame, 120)
    self.assertIsNone(controller.icbm_assist.update(
      120, enabled=True, cruise_enabled=True, cruise_speed_ms=15.0, target_speed_ms=20.0,
    ))

  def test_visual_longitudinal_forces_request_off(self):
    controller = self.controller(openpilot_longitudinal=True)
    cc = self.control(enabled=True, target_speed=40.0)
    self.assertIsNone(controller._get_icbm_assist_target(cc, self.state(), self.NOW_NS))
    self.assertFalse(controller.icbm_assist_enabled)

  def test_warning_only_request_never_reaches_set_sender(self):
    controller = self.controller()
    cc = self.control(target_valid=False, curve_warning=True)
    self.assertIsNone(controller._get_icbm_assist_target(cc, self.state(), self.NOW_NS))
    self.assertFalse(controller.icbm_assist_enabled)

  def test_update_emits_set_only_after_hot_enable_cooldown(self):
    controller = self.controller()
    cc = self.control(enabled=True, target_speed=20.0)
    cs = self.state(cruiseState=SimpleNamespace(available=True, enabled=True, speed=15.0))

    controller.frame = 10
    _, can_sends = controller.update(cc.as_reader(), cs, self.NOW_NS)
    self.assertEqual(self.set_messages(can_sends), [])

    controller.frame = 30
    _, can_sends = controller.update(cc.as_reader(), cs, self.NOW_NS)
    self.assertEqual(len(self.set_messages(can_sends)), 1)

    controller.frame = 40
    cc.oemCruiseSetSpeedAssist.enabled = False
    cs.out.vCruise = 54.0
    _, can_sends = controller.update(cc.as_reader(), cs, self.NOW_NS)
    self.assertEqual(self.set_messages(can_sends), [])

  def test_switch_off_preserves_existing_persistent_vcruise_route(self):
    controller = self.controller()
    controller.frame = 0
    cc = self.control(enabled=False)
    cs = self.state(cruiseState=SimpleNamespace(available=True, enabled=True, speed=20.0), vCruise=90.0)
    _, can_sends = controller.update(cc.as_reader(), cs, self.NOW_NS)
    self.assertEqual(
      self.set_messages(can_sends),
      [mazdacan.create_button_cmd(controller.packer, controller.CP, cs.crz_btns_counter, Buttons.SET_PLUS)],
    )

  def test_visual_longitudinal_update_never_emits_set(self):
    controller = self.controller(openpilot_longitudinal=True)
    controller.frame = 1
    cc = self.control(enabled=True, target_speed=40.0)
    cs = self.state(cruiseState=SimpleNamespace(available=True, enabled=True, speed=10.0))
    _, can_sends = controller.update(cc.as_reader(), cs, self.NOW_NS)
    self.assertEqual(self.set_messages(can_sends), [])

  def test_stock_radar_lead_signal_reaches_generic_carstate(self):
    cp = CarInterface.get_non_essential_params(CAR.MAZDA_3_2019)
    cs = MazdaCarState(cp)
    parsers = cs.get_can_parsers(cp)
    initial = cs.update(parsers)  # register lazily parsed DBC messages
    self.assertTrue(initial.stockRadarLead)
    self.assertTrue(cs.stock_radar_has_lead)
    packer = CANPacker(DBC[CAR.MAZDA_3_2019][Bus.pt])

    for raw, expected in ((1, True), (0, False)):
      msg = packer.make_can_msg("CRZ_CTRL", 0, {"RADAR_HAS_LEAD": raw})
      parsers[Bus.pt].update([(self.NOW_NS, [msg])])
      out = cs.update(parsers)
      self.assertEqual(out.stockRadarLead, expected)
      self.assertEqual(cs.stock_radar_has_lead, expected)

  def test_set_plus_raw_state_does_not_change_legacy_button_events(self):
    cp = CarInterface.get_non_essential_params(CAR.MAZDA_3_2019)
    cs = MazdaCarState(cp)
    parsers = cs.get_can_parsers(cp)
    cs.update(parsers)
    packer = CANPacker(DBC[CAR.MAZDA_3_2019][Bus.pt])

    for raw, expected in ((1, True), (0, False)):
      msg = packer.make_can_msg("CRZ_BTNS", 0, {"SET_P": raw})
      parsers[Bus.pt].update([(self.NOW_NS, [msg])])
      out = cs.update(parsers)
      self.assertEqual(out.cruiseSpeedButtonPressed, expected)
      self.assertEqual(list(out.buttonEvents), [])

  def test_each_runtime_gate_fails_closed(self):
    cases = (
      ("target invalid", self.control(target_valid=False), self.state(), self.NOW_NS),
      ("controls disabled", self.control(), self.state(), self.NOW_NS),
      ("CAN invalid", self.control(), self.state(canValid=False), self.NOW_NS),
      ("CAN timeout", self.control(), self.state(canTimeout=True), self.NOW_NS),
      ("MRCC unavailable", self.control(), self.state(cruiseState=SimpleNamespace(available=False, enabled=True, speed=20.0)), self.NOW_NS),
      ("MRCC disabled", self.control(), self.state(cruiseState=SimpleNamespace(available=True, enabled=False, speed=20.0)), self.NOW_NS),
      ("brake", self.control(), self.state(brakePressed=True), self.NOW_NS),
      ("gas", self.control(), self.state(gasPressed=True), self.NOW_NS),
      ("not drive", self.control(), self.state(gearShifter=structs.CarState.GearShifter.park), self.NOW_NS),
      ("driver button", self.control(), self.state(buttonEvents=[SimpleNamespace(pressed=True)]), self.NOW_NS),
      ("any raw cruise button", self.control(), self.state(), self.NOW_NS),
      ("stale", self.control(source_mono_time=self.NOW_NS - SET_SPEED_REQUEST_MAX_AGE_NS - 1), self.state(), self.NOW_NS),
      ("future", self.control(source_mono_time=self.NOW_NS + 50_000_001), self.state(), self.NOW_NS),
      ("nan", self.control(target_speed=float("nan")), self.state(), self.NOW_NS),
      ("target below guard", self.control(target_speed=8.5), self.state(), self.NOW_NS),
      ("target above driver base", self.control(target_speed=31.0, driver_set_speed=30.0), self.state(), self.NOW_NS),
      ("current set nan", self.control(), self.state(cruiseState=SimpleNamespace(available=True, enabled=True, speed=float("nan"))), self.NOW_NS),
      ("low ego speed", self.control(), self.state(vEgo=8.0), self.NOW_NS),
      ("standstill", self.control(), self.state(standstill=True), self.NOW_NS),
      ("stock radar lead", self.control(), self.state(), self.NOW_NS),
    )

    for name, cc, cs, now_ns in cases:
      with self.subTest(name=name):
        if name == "controls disabled":
          cc.enabled = False
        if name == "any raw cruise button":
          cs.cruise_buttons_pressed = True
        if name == "stock radar lead":
          cs.stock_radar_has_lead = True
        controller = self.controller()
        self.assertIsNone(controller._get_icbm_assist_target(cc, cs, now_ns))


if __name__ == "__main__":
  unittest.main()
