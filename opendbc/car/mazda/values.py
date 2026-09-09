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
#   0 STABLE_1300 — weekend 1300 pack; 800 base and original rates/deadzone
#   1 TEST        — reserved 1500/800 (kappa gate + 65-70 km/h taper)
#   2 A_GATE      — city kappa gate, highway opens on a = v^2 * |kappa|
#   3 OPTIMIZED_1300 — smooth straight-road requests, demand-based 800..1300
#   4 UNIVERSAL_1500 — daily demand pack: 800 straight, kappa opens to 1500,
#     crawl peak 1300, no blinker zero

class SteerEnvelope:
  STABLE_1300 = 0
  TEST = 1
  A_GATE = 2
  OPTIMIZED_1300 = 3
  UNIVERSAL_1500 = 4


class CarControllerParams:
  STEER_MAX = 800                 # non-BM Mazda platforms; also 1500-pack floor
  STEER_MAX_STABLE = 1300
  STEER_MAX_STABLE_SPEED_LOOKUP = ([0.0, 20.0 * CV.KPH_TO_MS, 35.0 * CV.KPH_TO_MS], [1300.0, 1300.0, 800.0])
  STEER_MAX_STABLE_CURVATURE_LOOKUP = ([0.025, 0.05], [0.0, 1.0])
  STEER_OPTIMIZED_A0 = 0.8       # PC candidate: start adding authority
  STEER_OPTIMIZED_A1 = 1.8       # PC candidate: fully open to 1300
  STEER_DELTA_UP_STABLE = 10
  STEER_DELTA_DOWN_STABLE = 25
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
  STEER_DRIVER_ALLOWANCE_BM = 22     # panda BM driver_torque_allowance; blinker yield only
  STEER_DRIVER_MULTIPLIER = 1
  STEER_DRIVER_FACTOR = 1
  STEER_STEP = 1  # 100 Hz
  STEER_DEADZONE = 36
  STEER_DEADZONE_TEST = 36
  STEER_DEADZONE_CURVATURE = 0.0025
  STEER_HOLD_SPEED_MS = 1.4  # ~5 kph; standstill bounce still counts as stopped
  # Pack 4 daily: kappa-only (do not use a=v^2*k; highway speed makes that a 1500 switch).
  STEER_DAILY_SPEED_PEAK_LOOKUP = (
    [0.0, 15.0 * CV.KPH_TO_MS, 35.0 * CV.KPH_TO_MS, 120.0 * CV.KPH_TO_MS],
    [1300.0, 1300.0, 1500.0, 1500.0],
  )
  STEER_DAILY_KAPPA_LOOKUP = ([0.0025, 0.005, 0.007], [0.0, 0.40, 1.0])
  STEER_DAILY_GARAGE_KPH = 20.0
  STEER_DAILY_GARAGE_ANGLE_DEG = 90.0
  STEER_DELTA_UP_DAILY = 10
  STEER_DELTA_DOWN_DAILY = 15

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
    if env == SteerEnvelope.OPTIMIZED_1300:
      return SteerEnvelope.OPTIMIZED_1300
    if env == SteerEnvelope.UNIVERSAL_1500:
      return SteerEnvelope.UNIVERSAL_1500
    return SteerEnvelope.STABLE_1300

  @classmethod
  def get_bm_daily_q(cls, desired_curvature: float) -> float:
    if not np.isfinite(desired_curvature):
      return 0.0
    return float(np.interp(abs(float(desired_curvature)), *cls.STEER_DAILY_KAPPA_LOOKUP))

  @classmethod
  def get_bm_optimized_demand(cls, v_ego: float, desired_curvature: float) -> float:
    if not np.isfinite(v_ego) or not np.isfinite(desired_curvature):
      return 0.0
    v = max(float(v_ego), 0.0)
    a = v * v * abs(float(desired_curvature))
    q = cls._clip01((a - cls.STEER_OPTIMIZED_A0) / (cls.STEER_OPTIMIZED_A1 - cls.STEER_OPTIMIZED_A0))
    return q * q * (3.0 - 2.0 * q)

  @classmethod
  def zeros_lkas_on_blinker(cls, envelope: int | None = None) -> bool:
    # Weekend 1300 and the Saturday-based 1500 pack keep sending. Later 1500
    # packs bleed to 0 while exactly one turn signal is on.
    return cls.normalize_envelope(envelope) not in (
      SteerEnvelope.STABLE_1300,
      SteerEnvelope.UNIVERSAL_1500,
    )

  @classmethod
  def get_bm_steer_max(cls, v_ego: float, desired_curvature: float, envelope: int | None = None,
                       steer_angle_deg: float = 0.0) -> int:
    env = cls.normalize_envelope(envelope)
    if env == SteerEnvelope.UNIVERSAL_1500:
      if not np.isfinite(v_ego) or not np.isfinite(desired_curvature):
        return int(cls.STEER_MAX)
      v = max(float(v_ego), 0.0)
      speed_peak = float(np.interp(v, *cls.STEER_DAILY_SPEED_PEAK_LOOKUP))
      v_kph = v * 3.6
      if (v_kph <= cls.STEER_DAILY_GARAGE_KPH and np.isfinite(steer_angle_deg) and
          abs(float(steer_angle_deg)) >= cls.STEER_DAILY_GARAGE_ANGLE_DEG):
        speed_peak = min(speed_peak, float(cls.STEER_MAX_STABLE))
      q = cls.get_bm_daily_q(desired_curvature)
      return int(round(cls.STEER_MAX + q * (speed_peak - cls.STEER_MAX)))
    if env in (SteerEnvelope.STABLE_1300, SteerEnvelope.OPTIMIZED_1300):
      if not np.isfinite(v_ego) or not np.isfinite(desired_curvature):
        return int(cls.STEER_MAX)
      speed_cap = np.interp(max(v_ego, 0.0), *cls.STEER_MAX_STABLE_SPEED_LOOKUP)
      curve_weight = np.interp(abs(desired_curvature), *cls.STEER_MAX_STABLE_CURVATURE_LOOKUP)
      steer_max = cls.STEER_MAX + curve_weight * (speed_cap - cls.STEER_MAX)
      if env == SteerEnvelope.OPTIMIZED_1300:
        q = cls.get_bm_optimized_demand(v_ego, desired_curvature)
        steer_max = max(steer_max, cls.STEER_MAX + q * (cls.STEER_MAX_STABLE - cls.STEER_MAX))
      return int(round(steer_max))

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
                          envelope: int | None = None, blinker: bool = False) -> tuple[int, int]:
    env = cls.normalize_envelope(envelope)
    curve = abs(float(desired_curvature)) if np.isfinite(desired_curvature) else 0.0
    if env == SteerEnvelope.STABLE_1300:
      if np.isfinite(v_ego) and v_ego <= cls.STEER_DELTA_LOW_SPEED_MAX:
        return cls.STEER_DELTA_UP_LOW_SPEED, cls.STEER_DELTA_DOWN_LOW_SPEED
      return cls.STEER_DELTA_UP_STABLE, cls.STEER_DELTA_DOWN_STABLE
    if env == SteerEnvelope.UNIVERSAL_1500:
      q = cls.get_bm_daily_q(desired_curvature)
      if np.isfinite(v_ego) and v_ego <= cls.STEER_DELTA_LOW_SPEED_MAX:
        up, down = cls.STEER_DELTA_UP_DAILY, cls.STEER_DELTA_DOWN_DAILY
      else:
        up = int(round(cls.STEER_DELTA_UP + q * (cls.STEER_DELTA_UP_DAILY - cls.STEER_DELTA_UP)))
        down = int(round(cls.STEER_DELTA_DOWN + q * (cls.STEER_DELTA_DOWN_DAILY - cls.STEER_DELTA_DOWN)))
      if blinker:
        down = cls.STEER_BLINKER_DELTA_DOWN
      return up, down
    if env == SteerEnvelope.OPTIMIZED_1300:
      if np.isfinite(v_ego) and v_ego <= cls.STEER_DELTA_LOW_SPEED_MAX and curve >= cls.STEER_CURVE_KAPPA:
        return cls.STEER_DELTA_UP_LOW_SPEED, cls.STEER_DELTA_DOWN_LOW_SPEED
      q = cls.get_bm_optimized_demand(v_ego, desired_curvature)
      return (int(round(cls.STEER_DELTA_UP + q * (cls.STEER_DELTA_UP_STABLE - cls.STEER_DELTA_UP))),
              int(round(cls.STEER_DELTA_DOWN + q * (cls.STEER_DELTA_DOWN_STABLE - cls.STEER_DELTA_DOWN))))
    if curve >= cls.STEER_CURVE_KAPPA:
      return cls.STEER_DELTA_UP_TURN, cls.STEER_DELTA_DOWN_TURN
    return cls.STEER_DELTA_UP, cls.STEER_DELTA_DOWN

  @classmethod
  def get_bm_steer_deadzone(cls, envelope: int | None = None, v_ego: float = 0.0,
                           desired_curvature: float = 0.0) -> tuple[int, float]:
    env = cls.normalize_envelope(envelope)
    if env == SteerEnvelope.STABLE_1300:
      return 0, cls.STEER_DEADZONE_CURVATURE
    if env == SteerEnvelope.UNIVERSAL_1500:
      q = cls.get_bm_daily_q(desired_curvature)
      return int(round(cls.STEER_DEADZONE * (1.0 - q))), cls.STEER_DEADZONE_CURVATURE
    if env == SteerEnvelope.OPTIMIZED_1300:
      q = cls.get_bm_optimized_demand(v_ego, desired_curvature)
      return int(round(cls.STEER_DEADZONE * (1.0 - q))), cls.STEER_DEADZONE_CURVATURE
    return cls.STEER_DEADZONE, cls.STEER_DEADZONE_CURVATURE

  @classmethod
  def get_bm_steer_allowance(cls, envelope: int | None = None, blinker: bool = False) -> int:
    env = cls.normalize_envelope(envelope)
    if env == SteerEnvelope.UNIVERSAL_1500 and blinker:
      return cls.STEER_DRIVER_ALLOWANCE_BM
    if env in (SteerEnvelope.TEST, SteerEnvelope.A_GATE, SteerEnvelope.UNIVERSAL_1500):
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


# Cap normalized lateral P only at low desired curvature; the torque controller
# bypasses this cap for high-curvature demand. Preserve the existing local tune.
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
