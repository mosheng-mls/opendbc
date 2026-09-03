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


if __name__ == "__main__":
  unittest.main()
