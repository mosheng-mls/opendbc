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


class TestMazdaVisionOnlyRadarSafety(common.SafetyTestBase):
  # Production BM mode combines bit 0 (low-speed steering) and bit 1
  # (vision-only direct longitudinal). Tests must exercise the deployed value.
  DEPLOYED_PARAM = 3
  TX_MSGS = (
    [[0x764, 0]] +
    [[addr, bus] for bus in (0, 2) for addr in (0x21B, 0x21C, 0x499, 0x361, 0x362, 0x363, 0x364, 0x365, 0x366)]
  )
  PROGRAMMING = b"\x02\x10\x02\x00\x00\x00\x00\x00"
  TESTER_PRESENT = b"\x02\x3E\x80\x00\x00\x00\x00\x00"
  DEFAULT = b"\x02\x10\x01\x00\x00\x00\x00\x00"

  def setUp(self):
    self.packer = CANPackerSafety("mazda_2017")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.mazda, self.DEPLOYED_PARAM)
    self.safety.init_tests()

  def _enable_mads(self, mode=ALTERNATIVE_EXPERIENCE.ENABLE_MADS):
    self.safety.set_alternative_experience(mode)
    # sunnypilot's test shim separates storing and applying the setting;
    # the selective Mazid safety shim applies it in set_alternative_experience.
    if hasattr(self.safety, "mads_apply_alternative_experience"):
      self.safety.mads_apply_alternative_experience(mode)

  def _pedals_msg(self, *, acc_main=False, active=False, brake=False):
    return self.packer.make_can_msg_safety("PEDALS", 0, {
      "ACC_OFF": int(acc_main),
      "ACC_ACTIVE": int(active),
      "BRAKE_ON": int(brake),
    })

  @staticmethod
  def _active_crz_info(bus, counter=0):
    dat = bytearray(b"\x01\xff\xe2\x00\x06\x80\x00\x00")
    dat[6] = counter & 0x0F
    dat[7] = 0xFF - (sum(dat[:7]) & 0xFF)
    return make_msg(bus, 0x21B, dat=bytes(dat))

  @staticmethod
  def _active_crz_ctrl(bus):
    return make_msg(bus, 0x21C, dat=b"\x0a\x01\x0b\x20\x00\x00\x10\x00")

  def test_exact_payload_policy(self):
    for controls_allowed in (False, True):
      self.safety.set_controls_allowed(controls_allowed)
      self.assertTrue(self._tx(make_msg(0, 0x764, dat=self.TESTER_PRESENT)))
      self.assertEqual(self._tx(make_msg(0, 0x764, dat=self.PROGRAMMING)), not controls_allowed)
      self.assertEqual(self._tx(make_msg(0, 0x764, dat=self.DEFAULT)), not controls_allowed)

  def test_wrong_bus_dlc_and_payload_are_blocked(self):
    self.safety.set_controls_allowed(False)
    for bus in (1, 2, 3):
      for dat in (self.PROGRAMMING, self.TESTER_PRESENT, self.DEFAULT):
        self.assertFalse(self._tx(make_msg(bus, 0x764, dat=dat)))

    self.assertFalse(self._tx(make_msg(0, 0x764, dat=self.TESTER_PRESENT[:-1])))
    invalid_payloads = (
      b"\x02\x10\x03\x00\x00\x00\x00\x00",
      b"\x02\x28\x83\x01\x00\x00\x00\x00",
      b"\x02\x3E\x00\x00\x00\x00\x00\x00",
      b"\x03\x10\x02\x00\x00\x00\x00\x00",
    )
    for dat in invalid_payloads:
      self.assertFalse(self._tx(make_msg(0, 0x764, dat=dat)))

  def test_single_bit_payload_mutations_are_blocked(self):
    self.safety.set_controls_allowed(False)
    for valid in (self.PROGRAMMING, self.TESTER_PRESENT, self.DEFAULT):
      for bit in range(64):
        mutated = bytearray(valid)
        mutated[bit // 8] ^= 1 << (bit % 8)
        self.assertFalse(self._tx(make_msg(0, 0x764, dat=bytes(mutated))))

  def test_deployed_mode_accepts_only_valid_direct_frames(self):
    self.safety.set_controls_allowed(True)
    for bus in (0, 2):
      self.assertTrue(self._tx(self._active_crz_info(bus)))
      self.assertTrue(self._tx(self._active_crz_ctrl(bus)))

    for bus in (0, 2):
      self.assertFalse(self._tx(make_msg(bus, 0x21B, dat=b"\x00" * 8)))
      self.assertFalse(self._tx(make_msg(bus, 0x21C, dat=b"\x00" * 8)))

    for bus in (1, 3):
      self.assertFalse(self._tx(self._active_crz_info(bus)))
      self.assertFalse(self._tx(self._active_crz_ctrl(bus)))

  def test_pedals_preserve_mads_brake_policy(self):
    self._enable_mads()
    self.assertTrue(self._rx(self._pedals_msg()))
    self.assertFalse(self.safety.get_controls_allowed_lateral())

    self.assertTrue(self._rx(self._pedals_msg(acc_main=True)))
    self.assertTrue(self.safety.get_acc_main_on())
    self.assertTrue(self.safety.get_controls_allowed_lateral())

    self.assertTrue(self._rx(self._pedals_msg(active=True)))
    self.assertTrue(self.safety.get_controls_allowed())
    self.assertTrue(self.safety.get_controls_allowed_lateral())

    # The stock PEDALS bits can both fall during braking. Longitudinal control
    # must disengage, while MADS REMAIN_ACTIVE keeps lateral authority.
    self.assertTrue(self._rx(self._pedals_msg(brake=True)))
    self.assertFalse(self.safety.get_controls_allowed())
    self.assertTrue(self.safety.get_acc_main_on())
    self.assertTrue(self.safety.get_controls_allowed_lateral())

    # A stable main-off state, unlike the brake transient, drops lateral.
    self.assertTrue(self._rx(self._pedals_msg()))
    self.assertTrue(self.safety.get_acc_main_on())
    self.assertTrue(self._rx(self._pedals_msg()))
    self.assertFalse(self.safety.get_acc_main_on())
    self.assertFalse(self.safety.get_controls_allowed_lateral())

  def test_default_mazda_mode_still_blocks_radar_diagnostics(self):
    self.safety.set_safety_hooks(CarParams.SafetyModel.mazda, 0)
    self.safety.init_tests()
    for dat in (self.PROGRAMMING, self.TESTER_PRESENT, self.DEFAULT):
      for bus in range(4):
        for controls_allowed in (False, True):
          self.safety.set_controls_allowed(controls_allowed)
          self.assertFalse(self._tx(make_msg(bus, 0x764, dat=dat)))


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
