"""Offline unit tests for Mazid Mazda ICBM vehicle TX helper (DRIVE-MVP-001)."""
import unittest

from opendbc.car.mazda.icbm import MazdaIcbmController, MIN_PRESS_INTERVAL_FRAMES
from opendbc.car.mazda.values import Buttons


class TestMazdaIcbm(unittest.TestCase):
  def test_no_press_when_disabled(self):
    icbm = MazdaIcbmController()
    self.assertIsNone(icbm.update(0, enabled=False, cruise_enabled=True,
                                  cruise_speed_ms=20.0, target_speed_ms=25.0))
    self.assertIsNone(icbm.update(0, enabled=True, cruise_enabled=False,
                                  cruise_speed_ms=20.0, target_speed_ms=25.0))

  def test_set_plus_and_minus(self):
    icbm = MazdaIcbmController()
    self.assertEqual(
      icbm.update(0, enabled=True, cruise_enabled=True, cruise_speed_ms=20.0, target_speed_ms=25.0),
      Buttons.SET_PLUS,
    )
    # rate limit
    self.assertIsNone(
      icbm.update(10, enabled=True, cruise_enabled=True, cruise_speed_ms=20.0, target_speed_ms=25.0),
    )
    self.assertEqual(
      icbm.update(MIN_PRESS_INTERVAL_FRAMES, enabled=True, cruise_enabled=True,
                  cruise_speed_ms=25.0, target_speed_ms=20.0),
      Buttons.SET_MINUS,
    )

  def test_deadband(self):
    icbm = MazdaIcbmController()
    self.assertIsNone(
      icbm.update(0, enabled=True, cruise_enabled=True, cruise_speed_ms=20.0, target_speed_ms=20.3),
    )


if __name__ == "__main__":
  unittest.main()
