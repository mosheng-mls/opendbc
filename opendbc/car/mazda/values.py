from dataclasses import dataclass, field
from enum import IntFlag

import numpy as np

from opendbc.car import Bus, CarSpecs, DbcDict, PlatformConfig, Platforms
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.structs import CarParams
from opendbc.car.docs_definitions import CarHarness, CarDocs, CarParts
from opendbc.car.fw_query_definitions import FwQueryConfig, Request, StdQueries

Ecu = CarParams.Ecu


# Steer torque packs (MazdaSteerEnvelope):
#   0 STABLE_1300 — one cap 1300
#   1 TEST        — reserved 1500/800 (kappa gate + 65-70 km/h taper)
#   2 A_GATE      — city kappa gate, highway opens on a = v^2 * |kappa|

class SteerEnvelope:
  STABLE_1300 = 0
  TEST = 1
  A_GATE = 2


class CarControllerParams:
  STEER_MAX = 800                 # non-BM Mazda platforms; also 1500-pack floor
  STEER_MAX_STABLE = 1300
  STEER_MAX_BM = 1500             # TEST / A_GATE peak; panda BM max
  STEER_CURVE_KAPPA = 0.008       # faster rates once the path is a real turn
  STEER_KAPPA0 = 0.003
  STEER_KAPPA1 = 0.008
  STEER_V_FULL_KPH = 65.0
  STEER_V_END_KPH = 70.0
  STEER_A0 = 0.8                  # A_GATE: start opening
  STEER_A1 = 1.8                  # A_GATE: fully open
  STEER_DELTA_UP = 6              # straight / low curvature
  STEER_DELTA_DOWN = 8
  STEER_DELTA_UP_TURN = 16
  STEER_DELTA_DOWN_TURN = 15
  STEER_DELTA_UP_LOW_SPEED = 16   # alias of turn-up (panda BM max_rate_up)
  STEER_DELTA_DOWN_LOW_SPEED = 15
  STEER_DELTA_UP_TEST = 6
  STEER_DELTA_DOWN_TEST = 8
  STEER_BLINKER_DELTA_DOWN = 25   # panda BM max_rate_down; do not linger at 1500
  STEER_DELTA_LOW_SPEED_MAX = 15.0 * CV.KPH_TO_MS
  STEER_DRIVER_ALLOWANCE = 15
  STEER_DRIVER_ALLOWANCE_TEST = 17
  STEER_DRIVER_MULTIPLIER = 1
  STEER_DRIVER_FACTOR = 1
  STEER_STEP = 1  # 100 Hz
  STEER_DEADZONE = 36
  STEER_DEADZONE_TEST = 36
  STEER_DEADZONE_CURVATURE = 0.0025
  STEER_HOLD_SPEED_MS = 1.4  # ~5 kph; standstill bounce still counts as stopped

  def __init__(self, CP):
    pass

  @staticmethod
  def _clip01(x: float) -> float:
    if not np.isfinite(x):
      return 0.0
    return float(np.clip(x, 0.0, 1.0))

  @classmethod
  def normalize_envelope(cls, envelope: int | None) -> int:
    try:
      env = int(envelope)
    except (TypeError, ValueError):
      return SteerEnvelope.STABLE_1300
    if env == SteerEnvelope.TEST:
      return SteerEnvelope.TEST
    if env == SteerEnvelope.A_GATE:
      return SteerEnvelope.A_GATE
    return SteerEnvelope.STABLE_1300

  @classmethod
  def get_bm_steer_max(cls, v_ego: float, desired_curvature: float, envelope: int | None = None) -> int:
    env = cls.normalize_envelope(envelope)
    if env == SteerEnvelope.STABLE_1300:
      return int(cls.STEER_MAX_STABLE)

    v = float(v_ego) if np.isfinite(v_ego) else 0.0
    k = abs(float(desired_curvature)) if np.isfinite(desired_curvature) else 0.0
    v_kph = v * 3.6
    qk = cls._clip01((k - cls.STEER_KAPPA0) / (cls.STEER_KAPPA1 - cls.STEER_KAPPA0))

    if env == SteerEnvelope.TEST:
      if v_kph <= cls.STEER_V_FULL_KPH:
        speed_cap = float(cls.STEER_MAX_BM)
      elif v_kph >= cls.STEER_V_END_KPH:
        speed_cap = float(cls.STEER_MAX)
      else:
        t = (v_kph - cls.STEER_V_FULL_KPH) / (cls.STEER_V_END_KPH - cls.STEER_V_FULL_KPH)
        speed_cap = cls.STEER_MAX_BM + t * (cls.STEER_MAX - cls.STEER_MAX_BM)
      return int(round(cls.STEER_MAX + qk * (speed_cap - cls.STEER_MAX)))

    a = v * v * k
    qa = cls._clip01((a - cls.STEER_A0) / (cls.STEER_A1 - cls.STEER_A0))
    z = cls._clip01((v_kph - cls.STEER_V_FULL_KPH) / (cls.STEER_V_END_KPH - cls.STEER_V_FULL_KPH))
    return int(round(cls.STEER_MAX + (cls.STEER_MAX_BM - cls.STEER_MAX) * ((1.0 - z) * qk + z * qa)))

  @classmethod
  def get_bm_steer_deltas(cls, v_ego: float, desired_curvature: float = 0.0,
                          envelope: int | None = None) -> tuple[int, int]:
    _ = v_ego, envelope
    curve = abs(float(desired_curvature)) if np.isfinite(desired_curvature) else 0.0
    if curve >= cls.STEER_CURVE_KAPPA:
      return cls.STEER_DELTA_UP_TURN, cls.STEER_DELTA_DOWN_TURN
    return cls.STEER_DELTA_UP, cls.STEER_DELTA_DOWN

  @classmethod
  def get_bm_steer_deadzone(cls, envelope: int | None = None) -> tuple[int, float]:
    del envelope
    return cls.STEER_DEADZONE, cls.STEER_DEADZONE_CURVATURE

  @classmethod
  def get_bm_steer_allowance(cls, envelope: int | None = None) -> int:
    if cls.normalize_envelope(envelope) in (SteerEnvelope.TEST, SteerEnvelope.A_GATE):
      return cls.STEER_DRIVER_ALLOWANCE_TEST
    return cls.STEER_DRIVER_ALLOWANCE


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
  # Allows the guarded 0x764 radar diagnostic session and empty radar
  # compatibility frames. It is enabled together with DIRECT_LONG only.
  VISION_ONLY_RADAR = 2
  DIRECT_LONG = 4


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
  # 2019 China BM / Axela GEN1 identity. Reuse the proven GEN1 Mazda 3
  # dimensions, while keeping the evidence-backed BM CAN dictionary separate.
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


class Buttons:
  NONE = 0
  SET_PLUS = 1
  SET_MINUS = 2
  RESUME = 3
  CANCEL = 4


FW_QUERY_CONFIG = FwQueryConfig(
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
