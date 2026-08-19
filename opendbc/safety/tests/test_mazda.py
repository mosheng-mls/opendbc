#!/usr/bin/env python3
import unittest

from opendbc.car.structs import CarParams
from opendbc.safety import ALTERNATIVE_EXPERIENCE
from opendbc.safety.tests.libsafety import libsafety_py
import opendbc.safety.tests.common as common
from opendbc.safety.tests.common import CANPackerSafety, make_msg


class TestMazdaSafety(common.CarSafetyTest, common.DriverTorqueSteeringSafetyTest):
  TX_MSGS = [[0x243, 0], [0x09D, 0], [0x440, 0]]
  STANDSTILL_THRESHOLD = 0.1
  RELAY_MALFUNCTION_ADDRS = {0: (0x243, 0x440)}
  FWD_BLACKLISTED_ADDRS = {2: [0x243, 0x440]}

  MAX_RATE_UP = 10
  MAX_RATE_DOWN = 25
  MAX_TORQUE_LOOKUP = [0], [800]

  MAX_RT_DELTA = 300

  DRIVER_TORQUE_ALLOWANCE = 15
  DRIVER_TORQUE_FACTOR = 1

  # Mazda actually does not set any bit when requesting torque
  NO_STEER_REQ_BIT = True

  def setUp(self):
    self.packer = CANPackerSafety("mazda_2017")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.mazda, 0)
    self.safety.init_tests()

  def _torque_meas_msg(self, torque):
    values = {"STEER_TORQUE_MOTOR": torque}
    return self.packer.make_can_msg_safety("STEER_TORQUE", 0, values)

  def _torque_driver_msg(self, torque):
    values = {"STEER_TORQUE_SENSOR": torque}
    return self.packer.make_can_msg_safety("STEER_TORQUE", 0, values)

  def _torque_cmd_msg(self, torque, steer_req=1):
    values = {"LKAS_REQUEST": torque}
    return self.packer.make_can_msg_safety("CAM_LKAS", 0, values)

  def _speed_msg(self, speed):
    values = {"SPEED": speed}
    return self.packer.make_can_msg_safety("ENGINE_DATA", 0, values)

  def _user_brake_msg(self, brake):
    values = {"BRAKE_ON": brake}
    return self.packer.make_can_msg_safety("PEDALS", 0, values)

  def _user_gas_msg(self, gas):
    values = {"PEDAL_GAS": gas}
    return self.packer.make_can_msg_safety("ENGINE_DATA", 0, values)

  def _pcm_status_msg(self, enable):
    values = {"CRZ_ACTIVE": enable}
    return self.packer.make_can_msg_safety("CRZ_CTRL", 0, values)

  def _button_msg(self, resume=False, cancel=False):
    values = {
      "CAN_OFF": cancel,
      "CAN_OFF_INV": (cancel + 1) % 2,
      "RES": resume,
      "RES_INV": (resume + 1) % 2,
    }
    return self.packer.make_can_msg_safety("CRZ_BTNS", 0, values)

  def test_buttons(self):
    # only cancel allows while controls not allowed
    self.safety.set_controls_allowed(0)
    self.assertTrue(self._tx(self._button_msg(cancel=True)))
    self.assertFalse(self._tx(self._button_msg(resume=True)))

    # do not block resume if we are engaged already
    self.safety.set_controls_allowed(1)
    self.assertTrue(self._tx(self._button_msg(cancel=True)))
    self.assertTrue(self._tx(self._button_msg(resume=True)))

  def test_direct_longitudinal_candidates_are_always_blocked(self):
    # Full-DLC candidate payloads, including a complete ISO-TP single-frame
    # UDS diagnostic-session request on the radar address.
    candidate_payloads = {
      0x21B: b"\x00\x10\x00\x20\x04\x00\x01\x55",  # CRZ_INFO / ACCEL_CMD candidate
      0x21C: b"\x08\x00\x06\x00\x01\x01\x00\x5a",  # CRZ_CTRL candidate
      0x764: b"\x02\x10\x03\x00\x00\x00\x00\x00",  # UDS DiagnosticSessionControl(extended)
    }

    for addr, dat in candidate_payloads.items():
      self.assertEqual(len(dat), 8)
      for bus in range(4):
        for controls_allowed in (False, True):
          with self.subTest(addr=hex(addr), bus=bus, controls_allowed=controls_allowed):
            self.safety.set_controls_allowed(controls_allowed)
            self.assertFalse(self._tx(make_msg(bus, addr, dat=dat)))

  def test_legal_tx_boundary_is_preserved(self):
    # Mazda TX remains limited to LKAS, cruise buttons, and HUD.
    self.assertEqual(self.TX_MSGS, [[0x243, 0], [0x09D, 0], [0x440, 0]])
    self.safety.set_controls_allowed(True)
    self.safety.set_desired_torque_last(0)
    self.safety.set_rt_torque_last(0)
    self.safety.set_torque_driver(0, 0)
    self.assertTrue(self._tx(self._torque_cmd_msg(0)))  # 0x243 LKAS
    self.assertTrue(self._tx(self._button_msg(resume=True)))  # 0x09D CRZ_BTNS
    self.assertTrue(self._tx(make_msg(0, 0x440)))  # CAM_LANEINFO

  def _acc_main_msg(self, available, active=False):
    values = {"CRZ_AVAILABLE": int(available), "CRZ_ACTIVE": int(active)}
    return self.packer.make_can_msg_safety("CRZ_CTRL", 0, values)

  def test_mads_off_blocks_torque_without_cruise(self):
    self.safety.set_alternative_experience(0)
    self.safety.set_controls_allowed(0)
    self.safety.set_desired_torque_last(0)
    self.safety.set_rt_torque_last(0)
    self.safety.set_torque_driver(0, 0)
    self.assertFalse(self._tx(self._torque_cmd_msg(100)))

  def test_mads_on_allows_torque_after_acc_main(self):
    self.safety.set_alternative_experience(ALTERNATIVE_EXPERIENCE.ENABLE_MADS)
    self.safety.set_controls_allowed(0)
    self.safety.set_desired_torque_last(0)
    self.safety.set_rt_torque_last(0)
    self.safety.set_torque_driver(0, 0)
    self._rx(self._acc_main_msg(False))
    self.assertFalse(self.safety.get_controls_allowed_lateral())
    self.assertFalse(self._tx(self._torque_cmd_msg(100)))
    self._rx(self._acc_main_msg(True))
    self.assertFalse(self.safety.get_controls_allowed())
    self.assertTrue(self.safety.get_controls_allowed_lateral())
    self.assertTrue(self._tx(self._torque_cmd_msg(10)))


class TestMazdaIgnition(unittest.TestCase):
  TX_MSGS: list = []

  def setUp(self):
    self.safety = libsafety_py.libsafety
    self.safety.init_tests()

  def _msg(self, byte0):
    return make_msg(0, 0x9E, dat=bytes([byte0]) + b"\x00" * 7)

  # 0x9E byte 0 high 3 bits == 6 (0xC0)
  def test_ignition_on(self):
    self.safety.ignition_can_hook(self._msg(0xC0))
    self.assertTrue(self.safety.get_ignition_can())

  def test_ignition_off(self):
    self.safety.ignition_can_hook(self._msg(0xC0))
    self.assertTrue(self.safety.get_ignition_can())
    self.safety.ignition_can_hook(self._msg(0x20))
    self.assertFalse(self.safety.get_ignition_can())


if __name__ == "__main__":
  unittest.main()
