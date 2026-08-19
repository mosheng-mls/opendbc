#!/usr/bin/env python3
from opendbc.car import get_safety_config, structs
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.interfaces import CarInterfaceBase
from opendbc.car.mazda.carcontroller import CarController
from opendbc.car.mazda.carstate import CarState
from opendbc.car.mazda.radar_interface import RadarInterface
from opendbc.car.mazda.values import CAR, LKAS_LIMITS, LOW_DEMAND_P_TORQUE_CAP


class CarInterface(CarInterfaceBase):
  CarState = CarState
  CarController = CarController
  RadarInterface = RadarInterface

  @staticmethod
  def _get_params(ret: structs.CarParams, candidate, fingerprint, car_fw, alpha_long, is_release, docs) -> structs.CarParams:
    ret.brand = "mazda"
    ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.mazda)]
    ret.radarUnavailable = True

    # LONG-007: this Mazda3's OEM ACC exits near 30 km/h and has no verified
    # stop-and-go state to resume. Keep generic Mazda auto-resume available for
    # platforms that support it, but fail closed on the 2019 Mazda3 BM.
    if candidate == CAR.MAZDA_3_2019:
      ret.autoResumeSng = False

    # Driving MVP (DRIVE-MVP-001): MAZDA_3_2019 is a first-class drivable platform.
    # Do not reuse CX5_2022 as a live identity substitute — only share capability flags here.
    ret.dashcamOnly = candidate not in (CAR.MAZDA_CX5_2022, CAR.MAZDA_CX9_2021, CAR.MAZDA_3_2019)

    ret.steerActuatorDelay = 0.1
    ret.steerLimitTimer = 0.8

    CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)

    # Product requirement: minSteerSpeed = 0 (not a Safety disable).
    if candidate not in (CAR.MAZDA_CX5_2022, CAR.MAZDA_3_2019):
      ret.minSteerSpeed = LKAS_LIMITS.DISABLE_SPEED * CV.KPH_TO_MS

    ret.centerToFront = ret.wheelbase * 0.41

    return ret

  def get_low_demand_p_torque_cap(self) -> float | None:
    # MAZDA_3_2019 reuses GEN1 MAZDA_3 body specs and CX9 torque substitute.
    # Protocol reuse is not a free pass on lateral gains: low-speed KP * leftover
    # steer angle pegs reverse torque through zero (RC_TEST_01 RC1-LAT-001).
    if self.CP.carFingerprint in (CAR.MAZDA_3, CAR.MAZDA_3_2019):
      return LOW_DEMAND_P_TORQUE_CAP
    return None
