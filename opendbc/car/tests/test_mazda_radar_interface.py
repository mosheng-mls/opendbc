import math
import unittest

from opendbc.car.can_definitions import CanData
from opendbc.car.mazda.interface import CarInterface
from opendbc.car.mazda.radar_interface import RadarInterface
from opendbc.car.mazda.values import CAR


PRIMARY_TARGET = bytes.fromhex('213fe1fe9c0ef28c')
NO_TARGET = bytes.fromhex('fff7fefe1fc8010c')
SECONDARY_TARGET = bytes.fromhex('7cc0cff99c80000c')


def radar_can(timestamp_ns: int, address: int, payload: bytes):
  return [(timestamp_ns, [CanData(address, payload, 0)])]


class TestMazdaRadarInterface(unittest.TestCase):
  def setUp(self):
    self.CP = CarInterface.get_non_essential_params(CAR.MAZDA_3_2019)
    self.assertTrue(self.CP.radarUnavailable)
    self.CP.radarUnavailable = False
    self.RI = RadarInterface(self.CP)

  def test_primary_target_lifecycle(self):
    radar_data = self.RI.update(radar_can(0, 0x361, PRIMARY_TARGET))
    self.assertIsNotNone(radar_data)
    self.assertFalse(radar_data.errors.canError)
    self.assertEqual(len(radar_data.points), 1)

    point = radar_data.points[0]
    first_track_id = point.trackId
    self.assertAlmostEqual(point.dRel, 33.1875)
    self.assertTrue(math.isnan(point.yRel))
    self.assertAlmostEqual(point.vRel, -0.75)

    radar_data = self.RI.update(radar_can(100_000_000, 0x361, NO_TARGET))
    self.assertIsNotNone(radar_data)
    self.assertEqual(len(radar_data.points), 0)

    radar_data = self.RI.update(radar_can(200_000_000, 0x361, PRIMARY_TARGET))
    self.assertIsNotNone(radar_data)
    self.assertEqual(len(radar_data.points), 1)
    self.assertGreater(radar_data.points[0].trackId, first_track_id)

  def test_secondary_target_is_not_published(self):
    radar_data = self.RI.update(radar_can(0, 0x363, SECONDARY_TARGET))
    self.assertIsNone(radar_data)
    self.assertEqual(self.RI.pts, {})

  def test_can_timeout_clears_target_once(self):
    radar_data = self.RI.update(radar_can(0, 0x361, PRIMARY_TARGET))
    self.assertEqual(len(radar_data.points), 1)

    radar_data = None
    for frame in range(1, 101):
      radar_data = self.RI.update(radar_can(frame * 100_000_000, 0x202, bytes(8)))
      if radar_data is not None:
        break
    self.assertIsNotNone(radar_data)
    self.assertTrue(radar_data.errors.canError)
    self.assertEqual(len(radar_data.points), 0)
    self.assertEqual(self.RI.pts, {})

    self.assertIsNone(self.RI.update(radar_can(10_100_000_000, 0x202, bytes(8))))

  def test_default_params_keep_candidate_disabled(self):
    CP = CarInterface.get_non_essential_params(CAR.MAZDA_3_2019)
    RI = CarInterface.RadarInterface(CP)
    self.assertTrue(CP.radarUnavailable)
    self.assertIsNone(RI.rcp)

    outputs = [RI.update([]) for _ in range(5)]
    self.assertIsNotNone(outputs[-1])
    self.assertEqual(len(outputs[-1].points), 0)


if __name__ == '__main__':
  unittest.main()
