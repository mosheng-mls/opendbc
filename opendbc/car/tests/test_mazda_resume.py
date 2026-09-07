#!/usr/bin/env python3
"""Offline LONG-007 fail-closed tests for Mazda automatic RESUME."""
import unittest
from types import SimpleNamespace

from opendbc.car import structs
from opendbc.car.mazda.carcontroller import CarController
from opendbc.car.mazda.interface import CarInterface
from opendbc.car.mazda.values import CAR, DBC


class TestMazdaResumeGate(unittest.TestCase):
  @staticmethod
  def run_resume_frame(candidate):
    cp = CarInterface.get_non_essential_params(candidate)
    controller = CarController(DBC[candidate], cp)
    controller.frame = 5  # RESUME cadence; avoids unrelated 2 Hz HUD work.

    cc = structs.CarControl()
    cc.enabled = True
    cc.latActive = False
    cc.cruiseControl.resume = True

    cs = SimpleNamespace(
      out=SimpleNamespace(
        brakePressed=False,
        standstill=True,
        steeringTorque=0.0,
        vCruise=255.0,
        cruiseState=SimpleNamespace(enabled=False),
      ),
      crz_btns_counter=0,
      cam_lkas={
        "BIT_1": 0,
        "ERR_BIT_1": 0,
        "ERR_BIT_2": 0,
      },
    )
    _, can_sends = controller.update(cc.as_reader(), cs, 0)
    return cp, [msg for msg in can_sends if msg[0] == 0x09D]

  def test_mazda3_2019_has_no_auto_resume_capability(self):
    cp, resume_messages = self.run_resume_frame(CAR.MAZDA_3_2019)
    self.assertFalse(cp.autoResumeSng)
    self.assertEqual(resume_messages, [])

  def test_stop_and_go_mazda_keeps_existing_resume_path(self):
    cp, resume_messages = self.run_resume_frame(CAR.MAZDA_CX5_2022)
    self.assertTrue(cp.autoResumeSng)
    self.assertEqual(len(resume_messages), 1)


if __name__ == "__main__":
  unittest.main()
