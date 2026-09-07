import unittest

from opendbc.car.mazda.carstate import CarState


class _Parser:
  def __init__(self, vl_all):
    self.vl_all = vl_all


class TestBMCarStateRadarAlive(unittest.TestCase):
  def test_crz_info_or_track_counts_as_stock_radar(self):
    self.assertFalse(CarState._stock_radar_seen(_Parser({})))
    self.assertFalse(CarState._stock_radar_seen(_Parser({
      "CRZ_INFO": {"CTR1": []},
      "RADAR_TRACK_1": {"LONG_DIST": []},
    })))
    self.assertTrue(CarState._stock_radar_seen(_Parser({
      "CRZ_INFO": {"CTR1": [1]},
      "RADAR_TRACK_1": {"LONG_DIST": []},
    })))
    self.assertTrue(CarState._stock_radar_seen(_Parser({
      "CRZ_INFO": {"CTR1": []},
      "RADAR_TRACK_1": {"LONG_DIST": [33.0]},
    })))


if __name__ == "__main__":
  unittest.main()
