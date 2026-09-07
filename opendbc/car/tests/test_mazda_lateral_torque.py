#!/usr/bin/env python3
"""BM lateral torque must not be gated on EPS LKAS_BLOCK at the send boundary."""
import unittest
from types import SimpleNamespace

from opendbc.car import structs
from opendbc.car.mazda.carcontroller import CarController
from opendbc.car.mazda.interface import CarInterface
from opendbc.car.mazda.values import CAR, DBC


class TestMazdaLateralTorque(unittest.TestCase):
  def test_lat_active_applies_torque_when_lkas_blocked(self):
    cp = CarInterface.get_non_essential_params(CAR.MAZDA_3_2019)
    controller = CarController(DBC[CAR.MAZDA_3_2019], cp)
    controller.frame = 5

    cc = structs.CarControl()
    cc.enabled = True
    cc.latActive = True
    cc.actuators.torque = 0.5

    cs = SimpleNamespace(
      out=SimpleNamespace(
        brakePressed=False,
        steeringTorque=0.0,
        vCruise=255.0,
        vEgo=2.0,
        vEgoRaw=2.0,
        standstill=False,
        gearShifter=structs.CarState.GearShifter.drive,
        leftBlinker=False,
        rightBlinker=False,
        steerFaultTemporary=True,
        steerFaultPermanent=False,
        cruiseState=SimpleNamespace(enabled=False, available=False, speed=0.0),
        canValid=True,
        canTimeout=False,
        buttonEvents=[],
      ),
      engine_speed_ms=2.0,
      lkas_blocked=True,
      crz_btns_counter=0,
      cam_lkas={
        "BIT_1": 0,
        "ERR_BIT_1": 0,
        "ERR_BIT_2": 0,
      },
    )

    actuators, _ = controller.update(cc.as_reader(), cs, 0)
    self.assertNotEqual(actuators.torqueOutputCan, 0)
    self.assertNotEqual(actuators.torque, 0.0)


class TestMazdaSteerSmoothness(unittest.TestCase):
  def test_straight_is_slower_than_turn(self):
    from opendbc.car.mazda.values import CarControllerParams as P, SteerEnvelope
    env = SteerEnvelope.TEST
    up_s, down_s = P.get_bm_steer_deltas(10.0, 0.0, env)
    up_t, down_t = P.get_bm_steer_deltas(10.0, 0.06, env)
    self.assertEqual((up_s, down_s), (P.STEER_DELTA_UP_TEST, P.STEER_DELTA_DOWN_TEST))
    self.assertEqual((up_t, down_t), (P.STEER_DELTA_UP_LOW_SPEED, P.STEER_DELTA_DOWN_LOW_SPEED))
    self.assertLess(up_s, up_t)
    self.assertLess(down_s, down_t)

  def test_hold_covers_creep_but_is_below_test_speed(self):
    from opendbc.car.mazda.values import CarControllerParams as P
    self.assertGreaterEqual(P.STEER_HOLD_SPEED_MS, 1.2)
    self.assertLess(P.STEER_HOLD_SPEED_MS, 2.0)

  def test_deadzone_widened_for_highway_wiggle(self):
    from opendbc.car.mazda.values import CarControllerParams as P, SteerEnvelope
    deadzone, curv = P.get_bm_steer_deadzone(SteerEnvelope.TEST)
    self.assertGreaterEqual(deadzone, 30)
    self.assertGreaterEqual(curv, 0.002)
    self.assertEqual(P.get_bm_steer_deadzone(SteerEnvelope.STABLE_1300)[0], 0)

  def test_stable_1300_envelope(self):
    from opendbc.car.mazda.values import CarControllerParams as P, SteerEnvelope
    kph = lambda v: v / 3.6
    env = SteerEnvelope.STABLE_1300
    cases = (
      (8, 0.0, 800), (50, 0.0, 800), (80, 0.0, 800),
      (8, 0.05, 1300), (20, 0.025, 800), (20, 0.0375, 1050), (20, 0.05, 1300),
      (27.5, 0.0375, 925), (27.5, 0.05, 1050), (35, 0.05, 800), (100, 0.05, 800),
      (-5, 0.05, 1300),
    )
    for speed, curvature, expected in cases:
      for sign in (-1, 1):
        with self.subTest(speed=speed, curvature=sign * curvature):
          self.assertEqual(P.get_bm_steer_max(kph(speed), sign * curvature, env), expected)
    for invalid in (float("nan"), float("inf"), float("-inf")):
      with self.subTest(invalid=invalid):
        self.assertEqual(P.get_bm_steer_max(invalid, 0.05, env), 800)
        self.assertEqual(P.get_bm_steer_max(kph(8), invalid, env), 800)

  def test_test_envelope_is_reserved_1500_pack(self):
    from opendbc.car.mazda.values import CarControllerParams as P, SteerEnvelope
    kph = lambda v: v / 3.6
    env = SteerEnvelope.TEST
    self.assertEqual(P.get_bm_steer_max(kph(8), 0.0, env), 800)
    self.assertEqual(P.get_bm_steer_max(kph(50), 0.05, env), 1500)
    self.assertEqual(P.get_bm_steer_max(kph(80), 0.0, env), 800)
    self.assertEqual(P.get_bm_steer_max(kph(80), 0.05, env), 800)
    self.assertEqual(P.get_bm_steer_max(kph(50), 0.0, env), 800)

  def test_stable_1300_keeps_weekend_rates(self):
    from opendbc.car.mazda.values import CarControllerParams as P, SteerEnvelope
    for curvature in (0.0, 0.001, -0.05):
      self.assertEqual(P.get_bm_steer_deltas(10.0, curvature, SteerEnvelope.STABLE_1300), (10, 25))
      self.assertEqual(P.get_bm_steer_deltas(15 / 3.6, curvature, SteerEnvelope.STABLE_1300), (16, 15))
    self.assertEqual(P.get_bm_steer_deltas(float("nan"), 0.0, SteerEnvelope.STABLE_1300), (10, 25))

  def test_optimized_1300_is_separate_and_demand_based(self):
    from opendbc.car.mazda.values import CarControllerParams as P, SteerEnvelope
    env = SteerEnvelope.OPTIMIZED_1300
    self.assertEqual(env, 3)
    self.assertEqual(P.normalize_envelope("3"), env)
    for sign in (-1, 1):
      for curvature, cap in ((0.0, 800), (0.008, 800), (0.013, 1050), (0.018, 1300)):
        with self.subTest(curvature=sign * curvature):
          self.assertEqual(P.get_bm_steer_max(10.0, sign * curvature, env), cap)
          self.assertEqual(P.get_bm_steer_max(10.0, sign * curvature, SteerEnvelope.STABLE_1300), 800)
    self.assertEqual(P.get_bm_steer_max(8 / 3.6, 0.05, env), 1300)
    self.assertEqual(P.get_bm_steer_max(100 / 3.6, 0.0, env), 800)
    for invalid in (float("nan"), float("inf"), float("-inf")):
      self.assertEqual(P.get_bm_steer_max(invalid, 0.05, env), 800)
      self.assertEqual(P.get_bm_steer_max(10.0, invalid, env), 800)

  def test_optimized_1300_straight_and_turn_response(self):
    from opendbc.car.mazda.values import CarControllerParams as P, SteerEnvelope
    env = SteerEnvelope.OPTIMIZED_1300
    self.assertEqual(P.get_bm_steer_deltas(10.0, 0.0, env), (6, 8))
    self.assertEqual(P.get_bm_steer_deltas(10.0, 0.018, env), (10, 25))
    self.assertEqual(P.get_bm_steer_deltas(8 / 3.6, 0.05, env), (16, 15))
    self.assertEqual(P.get_bm_steer_deadzone(env, 10.0, 0.0)[0], 36)
    self.assertEqual(P.get_bm_steer_deadzone(env, 10.0, 0.013)[0], 18)
    self.assertEqual(P.get_bm_steer_deadzone(env, 10.0, 0.018)[0], 0)

  def test_a_gate_opens_on_highway_accel(self):
    from opendbc.car.mazda.values import CarControllerParams as P, SteerEnvelope
    kph = lambda v: v / 3.6
    env = SteerEnvelope.A_GATE
    self.assertEqual(P.get_bm_steer_max(kph(50), 0.05, env), 1500)
    self.assertEqual(P.get_bm_steer_max(kph(100), 0.0005, env), 800)
    self.assertEqual(P.get_bm_steer_max(kph(100), 0.002, env), 1320)
    self.assertEqual(P.get_bm_steer_max(kph(100), 0.003888, env), 1500)
    self.assertEqual(P.get_bm_steer_max(kph(80), 0.004, env), 1500)

  def test_default_envelope_is_stable_1300(self):
    from opendbc.car.mazda.values import CarControllerParams as P, SteerEnvelope
    kph = lambda v: v / 3.6
    self.assertEqual(P.normalize_envelope(None), SteerEnvelope.STABLE_1300)
    self.assertEqual(P.normalize_envelope(99), SteerEnvelope.STABLE_1300)
    self.assertEqual(P.normalize_envelope(2), SteerEnvelope.A_GATE)
    self.assertEqual(P.get_bm_steer_max(kph(8), 0.05), 1300)
    self.assertEqual(P.get_bm_steer_max(kph(8), 0.05, None), 1300)


class TestMazdaBlinkerLkasSuspend(unittest.TestCase):
  def test_one_blinker_suspends(self):
    from opendbc.car.mazda.carcontroller import bm_blinker_suspends_lkas
    self.assertTrue(bm_blinker_suspends_lkas(True, False))
    self.assertTrue(bm_blinker_suspends_lkas(False, True))
    self.assertFalse(bm_blinker_suspends_lkas(False, False))
    self.assertFalse(bm_blinker_suspends_lkas(True, True))

  def test_bleed_does_not_slam(self):
    from opendbc.car.mazda.carcontroller import bleed_steer_to_zero
    self.assertEqual(bleed_steer_to_zero(40, 16), 24)
    self.assertEqual(bleed_steer_to_zero(-40, 16), -24)
    self.assertEqual(bleed_steer_to_zero(10, 16), 0)
    self.assertEqual(bleed_steer_to_zero(0, 16), 0)

  def test_blinker_bleed_from_1500_is_panda_legal(self):
    from opendbc.car.mazda.carcontroller import bleed_steer_to_zero
    from opendbc.car.mazda.values import CarControllerParams as P
    down = P.STEER_BLINKER_DELTA_DOWN
    self.assertEqual(down, 25)
    t = 1500
    steps = 0
    while t != 0 and steps < 80:
      nxt = bleed_steer_to_zero(t, down)
      self.assertLessEqual(abs(t) - abs(nxt), down)
      t = nxt
      steps += 1
    self.assertEqual(t, 0)
    self.assertLessEqual(steps, 60)

  def test_cap_drop_does_not_hard_clip(self):
    clipped = CarController._apply_steer_limits(0, 1400, 0.0, 800)
    self.assertGreater(clipped, 800)
    self.assertLessEqual(1400 - clipped, 8)


if __name__ == "__main__":
  unittest.main()
