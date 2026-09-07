#!/usr/bin/env python3
"""PC-only gates: Mazda OEM path must never inject SET+/SET- onto stock MRCC."""
import unittest
from types import SimpleNamespace

from opendbc.can import CANPacker
from opendbc.car import Bus, structs
from opendbc.car.mazda.carcontroller import CarController
from opendbc.car.mazda.carstate import CarState as MazdaCarState
from opendbc.car.mazda.interface import CarInterface
from opendbc.car.mazda.values import CAR, DBC


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

  def test_assist_on_never_emits_set(self):
    controller = self.controller()
    controller.frame = 30
    cc = self.control(enabled=True, target_speed=20.0)
    cs = self.state(cruiseState=SimpleNamespace(available=True, enabled=True, speed=15.0))
    _, can_sends = controller.update(cc.as_reader(), cs, self.NOW_NS)
    self.assertEqual(self.set_messages(can_sends), [])

  def test_assist_off_never_emits_persistent_vcruise_set(self):
    controller = self.controller()
    controller.frame = 0
    cc = self.control(enabled=False)
    cs = self.state(cruiseState=SimpleNamespace(available=True, enabled=True, speed=20.0), vCruise=90.0)
    _, can_sends = controller.update(cc.as_reader(), cs, self.NOW_NS)
    self.assertEqual(self.set_messages(can_sends), [])

  def test_visual_longitudinal_update_never_emits_set(self):
    controller = self.controller(openpilot_longitudinal=True)
    controller.frame = 1
    cc = self.control(enabled=True, target_speed=40.0)
    cs = self.state(cruiseState=SimpleNamespace(available=True, enabled=True, speed=10.0))
    _, can_sends = controller.update(cc.as_reader(), cs, self.NOW_NS)
    self.assertEqual(self.set_messages(can_sends), [])

  def test_bm_auto_resume_sng_is_disabled(self):
    cp = CarInterface.get_non_essential_params(CAR.MAZDA_3_2019)
    self.assertFalse(cp.autoResumeSng)

  def test_stock_radar_lead_signal_reaches_generic_carstate(self):
    cp = CarInterface.get_non_essential_params(CAR.MAZDA_3_2019)
    cs = MazdaCarState(cp)
    parsers = cs.get_can_parsers(cp)
    initial = cs.update(parsers)  # register lazily parsed DBC messages
    self.assertFalse(initial.stockRadarLead)
    self.assertFalse(cs.stock_radar_has_lead)
    self.assertFalse(cs.stock_radar_lead_valid)
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


if __name__ == "__main__":
  unittest.main()
