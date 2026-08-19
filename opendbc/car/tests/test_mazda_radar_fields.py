#!/usr/bin/env python3
"""Offline RADAR-002 checks for evidence-backed Mazda3 BM radar fields."""
import unittest

from opendbc.can import CANParser


class TestMazda3BMRadarFields(unittest.TestCase):
  DBC = "mazda_3_2019_bm"

  @staticmethod
  def parse(address: int, payload_hex: str):
    parser = CANParser(TestMazda3BMRadarFields.DBC, [(address, 10)], 0)
    parser.update([1_000_000_000, [(address, bytes.fromhex(payload_hex), 0)]])
    return parser.vl[address]

  def test_primary_track_known_real_car_payload(self):
    values = self.parse(0x361, "213fe1fe9c0ef28c")
    self.assertAlmostEqual(values["LONG_DIST"], 33.1875)
    self.assertAlmostEqual(values["REL_SPEED"], -0.75)
    self.assertAlmostEqual(values["SPEED_INVERSE"], 60.75)
    self.assertEqual(values["CTR"], 12)

  def test_no_target_sentinel(self):
    values = self.parse(0x362, "fff7fefe1fc8010c")
    self.assertAlmostEqual(values["LONG_DIST"], 255.9375)
    self.assertEqual(values["CTR"], 12)

  def test_additional_track_known_real_car_payloads(self):
    track_3 = self.parse(0x363, "7cc0cff99c80000c")
    track_4 = self.parse(0x364, "290165f73d40000c")
    self.assertAlmostEqual(track_3["LONG_DIST"], 124.75)
    self.assertAlmostEqual(track_3["REL_SPEED"], -3.25)
    self.assertAlmostEqual(track_4["LONG_DIST"], 41.0)
    self.assertAlmostEqual(track_4["REL_SPEED"], -4.4375)
    self.assertEqual(track_3["CTR"], 12)
    self.assertEqual(track_4["CTR"], 12)


if __name__ == "__main__":
  unittest.main()
