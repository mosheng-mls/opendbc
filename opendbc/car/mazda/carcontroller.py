from types import SimpleNamespace

from opendbc.can import CANPacker
from opendbc.car import Bus, structs
from opendbc.car.lateral import apply_driver_steer_torque_limits
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.mazda import mazdacan
from opendbc.car.mazda.bm_longitudinal_guard import BM_ACCEL_MIN as BM_DIRECT_LONG_ACCEL_MIN, BM_ACCEL_MAX as BM_DIRECT_LONG_ACCEL_MAX
from opendbc.car.mazda.bm_radar_session import BMRadarSessionInput, BMRadarSessionManager, may_replace_crz, may_replace_radar_tracks
from opendbc.car.mazda.values import CAR, CarControllerParams, Buttons, SteerEnvelope

from opendbc.sunnypilot.car.mazda.icbm import IntelligentCruiseButtonManagementInterface

VisualAlert = structs.CarControl.HUDControl.VisualAlert
BM_VISION_LONG_BUSES = (0, 2)
BM_DIRECT_LONG_ACCEL_DELTA_UP = 0.024
BM_DIRECT_LONG_ACCEL_DELTA_DOWN = 0.030
BM_DIRECT_LONG_COMFORT_DELTA_UP = 0.010  # 0.5 m/s^3 at 50 Hz, matching the supervisor
BM_DIRECT_LONG_ACCEL_OFFSET = 4.096
BM_DIRECT_LONG_ACCEL_SCALE = 1000.0


def bm_direct_long_accel_to_milli(accel: float) -> int:
  accel = max(BM_DIRECT_LONG_ACCEL_MIN, min(BM_DIRECT_LONG_ACCEL_MAX, float(accel)))
  raw = int((accel + BM_DIRECT_LONG_ACCEL_OFFSET) * BM_DIRECT_LONG_ACCEL_SCALE + 0.5)
  return raw - int(BM_DIRECT_LONG_ACCEL_OFFSET * BM_DIRECT_LONG_ACCEL_SCALE)


def slew_bm_direct_long_accel(target_accel: float, last_accel: float) -> float:
  """Slew on the exact milli-m/s² grid decoded by Mazda Safety."""
  target_milli = bm_direct_long_accel_to_milli(target_accel)
  last_milli = bm_direct_long_accel_to_milli(last_accel)
  up_milli = int(BM_DIRECT_LONG_ACCEL_DELTA_UP * BM_DIRECT_LONG_ACCEL_SCALE + 0.5)
  down_milli = int(BM_DIRECT_LONG_ACCEL_DELTA_DOWN * BM_DIRECT_LONG_ACCEL_SCALE + 0.5)
  if target_milli > 0:
    # The upstream ramp can finish while TX is still releasing braking.
    # Bound positive growth against the actual sent command; keep brake release unchanged.
    comfort_up_milli = int(BM_DIRECT_LONG_COMFORT_DELTA_UP * BM_DIRECT_LONG_ACCEL_SCALE + 0.5)
    target_milli = min(target_milli, max(0, last_milli) + comfort_up_milli)
  tx_milli = max(last_milli - down_milli, min(last_milli + up_milli, target_milli))
  return tx_milli / BM_DIRECT_LONG_ACCEL_SCALE


def oem_hands_on_steer_warn(steer_required_alert: bool, lkas_allowed_speed: bool, standstill: bool) -> bool:
  # BM reports LKAS_BLOCK at 0 kph. Do not light OEM "hold wheel" while stopped.
  return bool(steer_required_alert and lkas_allowed_speed and not standstill)


def bm_blinker_suspends_lkas(left_blinker: bool, right_blinker: bool) -> bool:
  # Stock FSC drops LKAS_REQUEST while exactly one turn signal is on. Dual
  # hazards keep authority. This is a send-boundary cut, not BlinkerPause.
  return bool(left_blinker) != bool(right_blinker)


def bleed_steer_to_zero(last_torque: int, delta_down: int) -> int:
  down = max(1, int(delta_down))
  if last_torque > 0:
    return max(0, last_torque - down)
  if last_torque < 0:
    return min(0, last_torque + down)
  return 0


class CarController(CarControllerBase, IntelligentCruiseButtonManagementInterface):
  CANCEL_RETRY_FRAMES = 50  # 0.5 s at the 100 Hz controller rate

  @staticmethod
  def _apply_steer_limits(new_torque, apply_torque_last, driver_torque, steer_max, steer_deltas=None,
                          steer_allowance=None):
    limits = CarControllerParams
    if steer_deltas is not None or steer_allowance is not None:
      limits = SimpleNamespace(
        STEER_DELTA_UP=steer_deltas[0] if steer_deltas is not None else CarControllerParams.STEER_DELTA_UP,
        STEER_DELTA_DOWN=steer_deltas[1] if steer_deltas is not None else CarControllerParams.STEER_DELTA_DOWN,
        STEER_DRIVER_ALLOWANCE=CarControllerParams.STEER_DRIVER_ALLOWANCE if steer_allowance is None else steer_allowance,
        STEER_DRIVER_MULTIPLIER=CarControllerParams.STEER_DRIVER_MULTIPLIER,
        STEER_DRIVER_FACTOR=CarControllerParams.STEER_DRIVER_FACTOR,
      )
    limited_torque = apply_driver_steer_torque_limits(new_torque, apply_torque_last,
                                                      driver_torque, limits, steer_max)
    # Keep requests continuous when the cap decreases, e.g. 1500→800.
    # The legacy pack reapplies its historical hard cap in update().
    down = int(getattr(limits, "STEER_DELTA_DOWN", CarControllerParams.STEER_DELTA_DOWN))
    if limited_torque > steer_max:
      limited_torque = max(steer_max, int(apply_torque_last) - down)
    elif limited_torque < -steer_max:
      limited_torque = min(-steer_max, int(apply_torque_last) + down)
    return int(limited_torque)

  def __init__(self, dbc_names, CP, CP_SP):
    CarControllerBase.__init__(self, dbc_names, CP, CP_SP)
    IntelligentCruiseButtonManagementInterface.__init__(self, CP, CP_SP)
    self.apply_torque_last = 0
    self.bm_low_speed_steer = CP.carFingerprint == CAR.MAZDA_3_2019
    self.packer = CANPacker(dbc_names[Bus.pt])
    self.brake_counter = 0
    self.last_cancel_frame = -self.CANCEL_RETRY_FRAMES
    self.bm_radar_session = BMRadarSessionManager()
    self._radar_shutdown_done = False
    self.bm_long_counter = 0
    self.bm_radar_counter = 0
    self.bm_tx_accel_last = 0.0
    self._steer_envelope = SteerEnvelope.STABLE_1300
    self._blinker_lkas_suspend = False
    self._params = None
    try:
      from openpilot.common.params import Params
      self._params = Params()
    except Exception:
      self._params = None

  def _should_send_cancel(self, cancel_requested, cruise_enabled, brake_pressed):
    # Never send CANCEL after stock ACC has already disengaged. On BM, repeated
    # CANCEL frames after the state transition can also turn cruise MAIN off.
    if not cancel_requested or not cruise_enabled:
      self.brake_counter = 0
      self.last_cancel_frame = self.frame - self.CANCEL_RETRY_FRAMES
      return False

    self.brake_counter += 1
    if brake_pressed and self.brake_counter < 7:
      return False

    if self.frame - self.last_cancel_frame < self.CANCEL_RETRY_FRAMES:
      return False

    self.last_cancel_frame = self.frame
    return True

  def _should_send_bm_resume(self, CC, CS) -> bool:
    # BM declares autoResumeSng=False. Never invent RES onto stock MRCC unless
    # that capability is explicitly enabled for the platform.
    if self.CP.openpilotLongitudinalControl or not self.CP.autoResumeSng:
      return False
    if CC.cruiseControl.cancel or not CC.cruiseControl.resume or self.frame % 5 != 0:
      return False
    if not CS.out.cruiseState.enabled:
      return False
    if CS.out.brakePressed or CS.out.gasPressed:
      return False
    return bool(CS.out.standstill or CS.out.cruiseState.standstill)

  def shutdown_radar_session(self):
    """Return one explicit handback request on card exit; RX recovery is separate."""
    if not self.CP.openpilotLongitudinalControl or self._radar_shutdown_done:
      return []
    self._radar_shutdown_done = True
    self.bm_tx_accel_last = 0.0
    session = self.bm_radar_session.request_immediate_handback()
    return [session.can_msg] if session.can_msg is not None else []

  def _update_bm_vision_longitudinal(self, CC, CS, can_sends):
    if self._radar_shutdown_done:
      return
    session = self.bm_radar_session.update(BMRadarSessionInput(
      requested=True,
      startup_gate_passed=bool(CS.bm_radar_startup_ready),
      stock_radar_alive=bool(CS.stock_radar_alive),
      vehicle_standstill=bool(CS.out.standstill),
      stock_cruise_engaged=bool(CS.out.cruiseState.enabled),
    ))
    if session.can_msg is not None:
      can_sends.append(session.can_msg)

    gas_override = bool(CC.enabled and not CS.out.brakePressed and
                        (CC.cruiseControl.override or CS.out.gasPressed))
    direct_long_command = bool(CC.longActive) or gas_override
    direct_ready = session.direct_longitudinal_ready
    long_engaged = bool(direct_ready and direct_long_command and not CS.out.brakePressed)
    if gas_override or not long_engaged:
      self.bm_tx_accel_last = 0.0

    # Inactive CRZ can fill the vacuum during verification. Empty radar tracks
    # must wait for the full one-second ownership guard on both harness sides.
    if may_replace_radar_tracks(session.state, CS.stock_radar_alive) and self.frame % 10 == 0:
      for bus in BM_VISION_LONG_BUSES:
        can_sends.extend(mazdacan.create_bm_no_target_radar_frames(bus, self.bm_radar_counter))
      self.bm_radar_counter += 1

    if may_replace_crz(session.state, CS.stock_radar_alive) and self.frame % 2 == 0:
      acc_available = bool(CS.out.cruiseState.available)
      gap_setting = int(CC.hudControl.leadDistanceBars) or 2
      tx_engaged = long_engaged if direct_ready else False
      if gas_override or not tx_engaged:
        tx_accel = 0.0
      else:
        target_accel = max(BM_DIRECT_LONG_ACCEL_MIN, min(BM_DIRECT_LONG_ACCEL_MAX, float(CC.actuators.accel)))
        tx_accel = slew_bm_direct_long_accel(target_accel, self.bm_tx_accel_last)
        self.bm_tx_accel_last = tx_accel
      for bus in BM_VISION_LONG_BUSES:
        can_sends.append(mazdacan.create_bm_direct_acc_command(
          bus, self.bm_long_counter, tx_accel, tx_engaged, acc_available,
        ))
        can_sends.append(mazdacan.create_bm_direct_crz_ctrl(
          bus, tx_engaged, acc_available, gap_setting,
        ))
      self.bm_long_counter += 1

  def _read_steer_envelope(self) -> int:
    raw = None
    if self._params is not None:
      try:
        raw = self._params.get("MazdaSteerEnvelope")
      except Exception:
        raw = None
    if raw is None:
      for path in ("/data/mazid/MazdaSteerEnvelope", "/data/params/d/MazdaSteerEnvelope"):
        try:
          with open(path, "rb") as f:
            raw = f.read().strip()
          break
        except Exception:
          continue
      if raw is None:
        return SteerEnvelope.STABLE_1300
    try:
      env = CarControllerParams.normalize_envelope(int(raw))
    except Exception:
      env = SteerEnvelope.STABLE_1300
    if env != self._steer_envelope or not getattr(self, "_steer_envelope_logged", False):
      print(f"MazdaSteerEnvelope apply {self._steer_envelope}->{env}", flush=True)
      self._steer_envelope_logged = True
    return env

  def update(self, CC, CC_SP, CS, now_nanos):
    can_sends = []

    apply_torque = 0
    steer_max = CarControllerParams.STEER_MAX
    steer_deltas = None
    steer_allowance = None
    deadzone = CarControllerParams.STEER_DEADZONE
    v_ego = float(getattr(CS.out, "vEgo", 0.0) or 0.0)
    if self.bm_low_speed_steer:
      if self.frame % 10 == 0:
        self._steer_envelope = self._read_steer_envelope()
      env = self._steer_envelope
      engine_speed_ms = getattr(CS, "engine_speed_ms", getattr(CS.out, "vEgoRaw", 0.0))
      steer_max = CarControllerParams.get_bm_steer_max(engine_speed_ms, CC.actuators.curvature, env)
      steer_deltas = CarControllerParams.get_bm_steer_deltas(engine_speed_ms, CC.actuators.curvature, env)
      if env == SteerEnvelope.OPTIMIZED_1300 and abs(self.apply_torque_last) > CarControllerParams.STEER_MAX_STABLE:
        # Retire inherited 1500-pack torque before using the slower 1300 rates.
        steer_deltas = (steer_deltas[0], CarControllerParams.STEER_DELTA_DOWN_STABLE)
      steer_allowance = CarControllerParams.get_bm_steer_allowance(env)
      deadzone, _ = CarControllerParams.get_bm_steer_deadzone(env, engine_speed_ms, CC.actuators.curvature)

    # Stop / crawl on a straight: hold still. Curvature gate keeps 90°/180 from a crawl.
    desired_curvature = float(CC.actuators.curvature or 0.0)
    hold_steer = bool(
      self.bm_low_speed_steer and
      (CS.out.standstill or v_ego < CarControllerParams.STEER_HOLD_SPEED_MS) and
      abs(desired_curvature) < CarControllerParams.STEER_DEADZONE_CURVATURE
    )
    # The weekend stable pack had no additional send-boundary blinker pause.
    blinker_suspend = (self.bm_low_speed_steer and self._steer_envelope != SteerEnvelope.STABLE_1300 and bm_blinker_suspends_lkas(
      bool(getattr(CS.out, "leftBlinker", False)),
      bool(getattr(CS.out, "rightBlinker", False)),
    ))
    if blinker_suspend != self._blinker_lkas_suspend:
      print(f"BM blinker LKAS suspend {int(self._blinker_lkas_suspend)}->{int(blinker_suspend)}", flush=True)
      self._blinker_lkas_suspend = blinker_suspend

    if CC.latActive and not hold_steer and not blinker_suspend:
      # Pre-GPT BM path: latActive is the send gate. LKAS_BLOCK is reported by
      # EPS at low speed and must not zero torque on its own.
      new_torque = int(round(CC.actuators.torque * steer_max))
      # Highway micro-wiggle: ignore tiny commands when the path is nearly straight.
      if (self.bm_low_speed_steer and
          abs(desired_curvature) < CarControllerParams.STEER_DEADZONE_CURVATURE and
          abs(new_torque) < deadzone):
        new_torque = 0
      apply_torque = self._apply_steer_limits(new_torque, self.apply_torque_last,
                                              CS.out.steeringTorque, steer_max, steer_deltas,
                                              steer_allowance)
      if self.bm_low_speed_steer and env == SteerEnvelope.STABLE_1300:
        apply_torque = max(-steer_max, min(steer_max, apply_torque))
    elif self.bm_low_speed_steer and env == SteerEnvelope.OPTIMIZED_1300 and not CC.latActive:
      # A disabled optimized pack must not keep transmitting a bleed-down tail.
      apply_torque = 0
    elif blinker_suspend:
      # Use a bounded 25-count bleed for an active pack's single turn signal.
      # EPS acceptance of this transition still requires vehicle validation.
      apply_torque = bleed_steer_to_zero(
        self.apply_torque_last, CarControllerParams.STEER_BLINKER_DELTA_DOWN)
    elif hold_steer:
      hold_down = CarControllerParams.STEER_DELTA_DOWN
      if steer_deltas is not None:
        hold_down = steer_deltas[1]
      apply_torque = bleed_steer_to_zero(self.apply_torque_last, hold_down)

    if self.bm_low_speed_steer and not self.CP.openpilotLongitudinalControl:
      if self._should_send_cancel(CC.cruiseControl.cancel, CS.out.cruiseState.enabled, CS.out.brakePressed):
        # Retry conservatively while stock ACC is still active. The enabled-state
        # gate above stops injection as soon as the vehicle reports disengagement.
        can_sends.append(mazdacan.create_button_cmd(self.packer, self.CP, CS.crz_btns_counter, Buttons.CANCEL))
    elif CC.cruiseControl.cancel and not self.CP.openpilotLongitudinalControl:
      # Preserve the upstream cancellation behavior for every non-BM Mazda.
      self.brake_counter += 1
      if self.frame % 10 == 0 and not (CS.out.brakePressed and self.brake_counter < 7):
        can_sends.append(mazdacan.create_button_cmd(self.packer, self.CP, CS.crz_btns_counter, Buttons.CANCEL))
    else:
      self.brake_counter = 0

    if self.bm_low_speed_steer:
      if self._should_send_bm_resume(CC, CS):
        can_sends.append(mazdacan.create_button_cmd(self.packer, self.CP, CS.crz_btns_counter, Buttons.RESUME))
    elif (not self.CP.openpilotLongitudinalControl and not CC.cruiseControl.cancel and
          CC.cruiseControl.resume and self.frame % 5 == 0):
      can_sends.append(mazdacan.create_button_cmd(self.packer, self.CP, CS.crz_btns_counter, Buttons.RESUME))

    self.apply_torque_last = apply_torque

    # send HUD alerts
    if self.frame % 50 == 0:
      ldw = CC.hudControl.visualAlert == VisualAlert.ldw
      steer_required = oem_hands_on_steer_warn(
        CC.hudControl.visualAlert == VisualAlert.steerRequired,
        CS.lkas_allowed_speed,
        CS.out.standstill,
      )
      can_sends.append(mazdacan.create_alert_command(self.packer, CS.cam_laneinfo, ldw, steer_required))

    # send steering command
    can_sends.append(mazdacan.create_steering_control(self.packer, self.CP,
                                                      self.frame, apply_torque, CS.cam_lkas))

    # OEM-MRCC on BM: never inject SET+/SET- (vision assist or sunnypilot ICBM).
    # Stock MRCC remains the only longitudinal authority.
    if self.CP.openpilotLongitudinalControl:
      self._update_bm_vision_longitudinal(CC, CS, can_sends)
    elif not self.bm_low_speed_steer:
      can_sends.extend(IntelligentCruiseButtonManagementInterface.update(
        self, CC_SP, CS, self.packer, self.frame, self.last_button_frame))

    new_actuators = CC.actuators.as_builder()
    new_actuators.torque = apply_torque / steer_max
    new_actuators.torqueOutputCan = apply_torque
    if self.CP.openpilotLongitudinalControl:
      new_actuators.accel = self.bm_tx_accel_last

    self.frame += 1
    return new_actuators, can_sends
