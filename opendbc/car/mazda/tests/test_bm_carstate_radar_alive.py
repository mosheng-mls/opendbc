import unittest
from types import SimpleNamespace as NS

from opendbc.car import Bus
from opendbc.car.mazda.carstate import CarState
from opendbc.car.mazda.values import CAR


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

  def test_op_long_parser_ignores_crz_ctrl_liveness(self):
    cp = NS(openpilotLongitudinalControl=True, carFingerprint=CAR.MAZDA_3_2019)
    parsers = CarState.get_can_parsers(cp)
    names = {state.name: state.ignore_alive for state in parsers[Bus.pt].message_states.values()}
    self.assertTrue(names["CRZ_CTRL"])
    self.assertTrue(names["CRZ_INFO"])
    self.assertTrue(names["RADAR_TRACK_1"])

    oem = CarState.get_can_parsers(NS(openpilotLongitudinalControl=False, carFingerprint=CAR.MAZDA_3_2019))
    self.assertFalse(any(state.name == "CRZ_CTRL" for state in oem[Bus.pt].message_states.values()))

  def test_cancel_latches_available_until_main_rising(self):
    flags = CarState.update_bm_vision_cruise_flags
    available, enabled, latched = flags(True, False, True, False, False, True, True, False, False)
    self.assertFalse(available)
    self.assertFalse(enabled)
    self.assertTrue(latched)
    available, enabled, latched = flags(True, False, False, False, True, True, False, False, False)
    self.assertFalse(available)
    self.assertTrue(latched)
    available, enabled, latched = flags(True, False, False, False, True, False, False, False, False)
    self.assertTrue(available)
    self.assertFalse(latched)

  def test_acc_active_enables_without_radar_silence(self):
    flags = CarState.update_bm_vision_cruise_flags
    available, enabled, latched = flags(True, True, False, False, False, True, True, False, False)
    self.assertTrue(available)
    self.assertTrue(enabled)
    available, enabled, latched = flags(True, False, False, False, False, True, True, True, True)
    self.assertTrue(available)
    self.assertFalse(enabled)
    self.assertFalse(latched)


if __name__ == "__main__":
  unittest.main()
