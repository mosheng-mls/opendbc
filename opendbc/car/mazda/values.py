from dataclasses import dataclass, field
from enum import IntFlag

import numpy as np

from opendbc.car import Bus, CarSpecs, DbcDict, PlatformConfig, Platforms
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.structs import CarParams
from opendbc.car.docs_definitions import CarHarness, CarDocs, CarParts
from opendbc.car.fw_query_definitions import FwQueryConfig, Request, StdQueries

Ecu = CarParams.Ecu


# Steer torque limits

class CarControllerParams:
  STEER_MAX = 800                 # default Mazda limit; theoretical max 2047
  STEER_MAX_BM = 1100             # BM low-speed/high-curvature candidate
  STEER_MAX_SPEED_LOOKUP = ([0.0, 20.0 * CV.KPH_TO_MS, 35.0 * CV.KPH_TO_MS], [1100.0, 1100.0, 800.0])
  STEER_MAX_CURVATURE_LOOKUP = ([0.025, 0.05], [0.0, 1.0])
  STEER_DELTA_UP = 10             # torque increase per refresh
  STEER_DELTA_DOWN = 25           # torque decrease per refresh
  STEER_DRIVER_ALLOWANCE = 15     # allowed driver torque before start limiting
  STEER_DRIVER_MULTIPLIER = 1     # weight driver torque
  STEER_DRIVER_FACTOR = 1         # from dbc
  STEER_STEP = 1  # 100 Hz

  def __init__(self, CP):
    pass

  @classmethod
  def get_bm_steer_max(cls, v_ego: float, desired_curvature: float) -> int:
    if not np.isfinite(v_ego) or not np.isfinite(desired_curvature):
      return cls.STEER_MAX
    speed_cap = float(np.interp(max(v_ego, 0.0), cls.STEER_MAX_SPEED_LOOKUP[0], cls.STEER_MAX_SPEED_LOOKUP[1]))
    curve_weight = float(np.interp(abs(desired_curvature), cls.STEER_MAX_CURVATURE_LOOKUP[0],
                                   cls.STEER_MAX_CURVATURE_LOOKUP[1]))
    return int(round(cls.STEER_MAX + curve_weight * (speed_cap - cls.STEER_MAX)))


@dataclass
class MazdaCarDocs(CarDocs):
  package: str = "All"
  car_parts: CarParts = field(default_factory=CarParts.common([CarHarness.mazda]))


@dataclass(frozen=True, kw_only=True)
class MazdaCarSpecs(CarSpecs):
  tireStiffnessFactor: float = 0.7  # not optimized yet


class MazdaFlags(IntFlag):
  # Static flags
  # Gen 1 hardware: same CAN messages and same camera
  GEN1 = 1


class MazdaSafetyFlags(IntFlag):
  BM_LOW_SPEED_STEER = 1
  # Independent opt-in for radar-session and direct-long frames. This must not
  # alias the already deployed BM low-speed steering permission.
  VISION_ONLY_RADAR = 2


@dataclass
class MazdaPlatformConfig(PlatformConfig):
  dbc_dict: DbcDict = field(default_factory=lambda: {Bus.pt: 'mazda_2017'})
  flags: int = MazdaFlags.GEN1


class CAR(Platforms):
  MAZDA_CX5 = MazdaPlatformConfig(
    [MazdaCarDocs("Mazda CX-5 2017-21")],
    MazdaCarSpecs(mass=3655 * CV.LB_TO_KG, wheelbase=2.7, steerRatio=15.5)
  )
  MAZDA_CX9 = MazdaPlatformConfig(
    [MazdaCarDocs("Mazda CX-9 2016-20")],
    MazdaCarSpecs(mass=4217 * CV.LB_TO_KG, wheelbase=3.1, steerRatio=17.6)
  )
  MAZDA_3 = MazdaPlatformConfig(
    [MazdaCarDocs("Mazda 3 2017-18")],
    MazdaCarSpecs(mass=2875 * CV.LB_TO_KG, wheelbase=2.7, steerRatio=14.0)
  )
  # 2019 China BM / Axela GEN1 identity. Specs reuse MAZDA_3 body numbers as closest
  # proven GEN1 3-series values; year range intentionally not expanded beyond evidence.
  # DBC is the Mazid BM dictionary (not global BP mazda_3_2019.dbc).
  # DRIVE-MVP-001: dashcamOnly=false and minSteerSpeed=0 via interface.py (Safety unchanged).
  MAZDA_3_2019 = MazdaPlatformConfig(
    [MazdaCarDocs("Mazda 3 2019")],
    MAZDA_3.specs,
    dbc_dict={Bus.pt: 'mazda_3_2019_bm'},
  )
  MAZDA_6 = MazdaPlatformConfig(
    [MazdaCarDocs("Mazda 6 2017-20")],
    MazdaCarSpecs(mass=3443 * CV.LB_TO_KG, wheelbase=2.83, steerRatio=15.5)
  )
  MAZDA_CX9_2021 = MazdaPlatformConfig(
    [MazdaCarDocs("Mazda CX-9 2021-23", video="https://youtu.be/dA3duO4a0O4")],
    MAZDA_CX9.specs
  )
  MAZDA_CX5_2022 = MazdaPlatformConfig(
    [MazdaCarDocs("Mazda CX-5 2022-25")],
    MAZDA_CX5.specs,
  )


class LKAS_LIMITS:
  STEER_THRESHOLD = 15
  DISABLE_SPEED = 45    # kph
  ENABLE_SPEED = 52     # kph


# RC1-LAT-001: max |P| (normalized torque) when |desired curvature| is low.
# Does not change STEER_MAX / DELTA_UP / DELTA_DOWN. High-curvature demand
# bypasses this cap inside LatControlTorque so LAT-002 authority stays.
LOW_DEMAND_P_TORQUE_CAP = 0.45


class Buttons:
  NONE = 0
  SET_PLUS = 1
  SET_MINUS = 2
  RESUME = 3
  CANCEL = 4


FW_QUERY_CONFIG = FwQueryConfig(
  fw_version_regex=br"[A-Z0-9-]{11,16}\x00{8,13}",
  requests=[
    # TODO: check data to ensure ABS does not skip ISO-TP frames on bus 0
    Request(
      [StdQueries.MANUFACTURER_SOFTWARE_VERSION_REQUEST],
      [StdQueries.MANUFACTURER_SOFTWARE_VERSION_RESPONSE],
      bus=0,
    ),
  ],
)

DBC = CAR.create_dbc_map()
