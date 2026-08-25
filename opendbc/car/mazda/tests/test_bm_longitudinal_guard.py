import unittest

from opendbc.car.mazda.bm_longitudinal_guard import BMLongitudinalGuard, BMLongitudinalGuardInput


def guard_input(*, speed_kph: float, accel: float = 0.60, active: bool = True,
                brake: bool = False, now_nanos: int = 0) -> BMLongitudinalGuardInput:
  return BMLongitudinalGuardInput(
    requested_accel=accel,
    v_ego=speed_kph / 3.6,
    long_active=active,
    brake_pressed=brake,
    lead_visible=False,
    set_speed=80.0 / 3.6,
    now_nanos=now_nanos,
  )


class TestBMLongitudinalGuard(unittest.TestCase):
  @staticmethod
  def settle(guard: BMLongitudinalGuard, speed_kph: float, frames: int = 300):
    output = None
    for frame in range(frames):
      output = guard.update(guard_input(speed_kph=speed_kph, now_nanos=frame * 10_000_000))
    return output

  def test_70_to_80_caps_positive_accel_at_point_three(self):
    output = self.settle(BMLongitudinalGuard(), 75.0)
    self.assertIsNotNone(output)
    self.assertAlmostEqual(output.accel, 0.30)

  def test_above_80_blocks_positive_but_preserves_decel_demand(self):
    guard = BMLongitudinalGuard()
    positive = self.settle(guard, 80.1)
    self.assertEqual(positive.accel, 0.0)
    negative = guard.update(guard_input(speed_kph=80.1, accel=-0.50, now_nanos=3_010_000_000))
    self.assertLess(negative.accel, 0.0)

  def test_below_five_releases_longitudinal_output(self):
    guard = BMLongitudinalGuard()
    output = guard.update(guard_input(speed_kph=4.9, accel=-0.50))
    self.assertTrue(output.critical_handoff)
    self.assertEqual(output.accel, 0.0)


if __name__ == "__main__":
  unittest.main()
