import logging

from opendbc.can import CANPacker
from opendbc.car import Bus
from opendbc.car.lateral import apply_driver_steer_torque_limits
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.mazda import mazdacan
from opendbc.car.mazda.bm_longitudinal_guard import BMLongitudinalGuard, BMLongitudinalGuardInput
from opendbc.car.mazda.bm_radar_session import BMRadarSessionInput, BMRadarSessionManager, BMRadarSessionState
from opendbc.car.mazda.hud_bridge import HudInputs, MazidHudBridge
from opendbc.car.mazda.hud_probe import MazidHudProbe, probe_enabled
from opendbc.car.mazda.icbm import MazdaIcbmController, persistent_cruise_target_ms
from opendbc.car.mazda.values import CAR, CarControllerParams, Buttons

LOGGER = logging.getLogger(__name__)
BM_LONG_BUSES = (0, 2)


class CarController(CarControllerBase):
  CANCEL_RETRY_FRAMES = 50

  def __init__(self, dbc_names, CP):
    super().__init__(dbc_names, CP)
    self.apply_torque_last = 0
    self.packer = CANPacker(dbc_names[Bus.pt])
    self.brake_counter = 0
    self.last_cancel_frame = -self.CANCEL_RETRY_FRAMES
    self.bm_low_speed_steer = CP.carFingerprint == CAR.MAZDA_3_2019
    self.icbm = MazdaIcbmController()
    self.bm_long_guard = BMLongitudinalGuard()
    self.bm_radar_session = BMRadarSessionManager()
    self.bm_long_counter = 0
    self.bm_radar_counter = 0
    self.bm_applied_accel = 0.0
    self.hud_faults: list[str] = []
    self.hud_last_fault = ""
    self.hud_deployment_blocked = False
    self.hud_probe_disabled = False
    self.hud_enhancement_disabled = False
    self.hud_oem_copy_fault_active = False
    self._hud_fault_features: set[str] = set()

    self.hud_bridge = None
    try:
      self.hud_bridge = MazidHudBridge()
    except Exception as exc:
      self._disable_hud_enhancement("hud_bridge.init", exc)

    self.hud_probe = None
    try:
      if probe_enabled():
        self.hud_probe = MazidHudProbe()
    except Exception as exc:
      self._disable_hud_probe("hud_probe.init", exc)

  def _record_hud_fault(self, feature: str, exc: Exception) -> None:
    """Latch one visible deployment-blocking result for an optional HUD failure."""
    self.hud_deployment_blocked = True
    try:
      detail = str(exc)
    except Exception:
      detail = "<unprintable>"
    reason = f"NON_CRITICAL_FEATURE_FAILURE feature={feature} error={type(exc).__name__}:{detail} DEPLOYMENT_BLOCKED"
    self.hud_last_fault = reason

    if feature not in self._hud_fault_features:
      self._hud_fault_features.add(feature)
      self.hud_faults.append(reason)
      try:
        LOGGER.exception(reason)
      except Exception:
        # Failure reporting is itself non-critical and must not escape this boundary.
        pass

  def _disable_hud_probe(self, feature: str, exc: Exception) -> None:
    self._record_hud_fault(feature, exc)
    self.hud_probe_disabled = True
    self.hud_probe = None

  def _disable_hud_enhancement(self, feature: str, exc: Exception) -> None:
    self._record_hud_fault(feature, exc)
    self.hud_enhancement_disabled = True

  @staticmethod
  def _valid_hud_message(msg):
    if msg[0] != 0x440:
      raise ValueError(f"unexpected HUD address: {msg[0]!r}")
    return msg

  def _send_oem_hud_copy(self, cam, can_sends) -> None:
    """Best-effort baseline replay; retry after transient failures on later HUD ticks."""
    try:
      msg = self._valid_hud_message(mazdacan.create_alert_command(
        self.packer, cam, False, False, copy_oem=True,
      ))
    except Exception as exc:
      self.hud_oem_copy_fault_active = True
      self._record_hud_fault("hud_oem_copy.pack", exc)
      return

    recovered = self.hud_oem_copy_fault_active
    self.hud_oem_copy_fault_active = False
    can_sends.append(msg)
    if recovered:
      try:
        LOGGER.warning("NON_CRITICAL_FEATURE_RECOVERY feature=hud_oem_copy.pack deployment_remains_blocked")
      except Exception:
        pass

  def _update_optional_hud(self, CC, CS, now_nanos, cam, can_sends) -> None:
    # Probe-only operations are isolated individually. Any failure discards the
    # candidate and falls through to HudBridge in the same 2 Hz tick.
    if self.hud_probe is not None and not self.hud_probe_disabled:
      probe_tick = None
      try:
        probe_tick = self.hud_probe.update(
          standstill=bool(CS.out.standstill),
          v_ego=float(CS.out.vEgo),
          gear=CS.out.gearShifter,
          steer_fault_permanent=bool(CS.out.steerFaultPermanent),
          now_ns=int(now_nanos),
        )
        probe_failure = getattr(self.hud_probe, "failure_reason", "")
        if probe_failure:
          raise RuntimeError(f"probe reported failure: {probe_failure}")
      except Exception as exc:
        self._disable_hud_probe("hud_probe.update", exc)

      if probe_tick is not None and self.hud_probe is not None:
        try:
          if probe_tick.static and probe_tick.display is not None:
            d = probe_tick.display
            msg = self._valid_hud_message(mazdacan.create_alert_command(
              self.packer, cam, False, False,
              lane_lines=None if d.copy_oem else d.lane_lines,
              line_visible=None if d.copy_oem else d.line_visible,
              line_not_visible=None if d.copy_oem else d.line_not_visible,
              hands_on=None if d.copy_oem else d.hands_on,
              hands_on_2=None if d.copy_oem else d.hands_on_2,
              hands_warn_3=None if d.copy_oem else d.hands_warn_3,
              copy_oem=d.copy_oem,
            ))
          else:
            msg = None
        except Exception as exc:
          self._disable_hud_probe("hud_probe.pack", exc)
          msg = None

        if msg is not None and self.hud_probe is not None:
          try:
            logged = self.hud_probe.log_tx(
              gid=probe_tick.gid,
              payload_hex=msg[1].hex(),
              v_ego=float(CS.out.vEgo),
              gear=CS.out.gearShifter,
              standstill=bool(CS.out.standstill),
              now_ns=int(now_nanos),
            )
            probe_failure = getattr(self.hud_probe, "failure_reason", "")
            if logged is False or probe_failure:
              raise RuntimeError(f"probe log failure: {probe_failure or 'unknown'}")
          except Exception as exc:
            self._disable_hud_probe("hud_probe.log_tx", exc)
          else:
            can_sends.append(msg)
            return

    # HudBridge policy and enhanced packing are also optional, but kept in
    # separate boundaries so a healthy packer can still replay OEM 0x440.
    if not self.hud_enhancement_disabled and self.hud_bridge is not None:
      try:
        hud = self.hud_bridge.update(HudInputs(
          lat_active=bool(CC.latActive),
          enabled=bool(CC.enabled),
          visual_alert=CC.hudControl.visualAlert,
          gear=CS.out.gearShifter,
          standstill=bool(CS.out.standstill),
          lkas_allowed_speed=bool(CS.lkas_allowed_speed),
          steer_fault_temporary=bool(CS.out.steerFaultTemporary),
          steer_fault_permanent=bool(CS.out.steerFaultPermanent),
          oem_hands_on=bool(cam.get("HANDS_ON_STEER_WARN", 0)),
          cruise_available=bool(CS.out.cruiseState.available),
          cruise_enabled=bool(CS.out.cruiseState.enabled),
          v_cruise_kph=float(CS.out.vCruise),
          hud_set_speed_kph=float(CC.hudControl.setSpeed),
          fsc_lane_lines=int(cam.get("LANE_LINES", 1) or 1),
          left_lane_visible=bool(CC.hudControl.leftLaneVisible),
          right_lane_visible=bool(CC.hudControl.rightLaneVisible),
          actuators_torque=float(CC.actuators.torque),
          steering_pressed=bool(CS.out.steeringPressed),
          brake_pressed=bool(CS.out.brakePressed),
          cancel=bool(CC.cruiseControl.cancel),
        ))
      except Exception as exc:
        self._disable_hud_enhancement("hud_bridge.update", exc)
      else:
        try:
          msg = self._valid_hud_message(mazdacan.create_alert_command(
            self.packer, cam, hud.ldw, hud.steer_required,
            lane_lines=hud.override_lane_lines,
            line_visible=hud.line_visible,
            line_not_visible=hud.line_not_visible,
          ))
        except Exception as exc:
          self._disable_hud_enhancement("hud_bridge.pack", exc)
        else:
          can_sends.append(msg)
          return

    self._send_oem_hud_copy(cam, can_sends)

  def _update_bm_longitudinal(self, CC, CS, now_nanos, can_sends) -> None:
    session = self.bm_radar_session.update(BMRadarSessionInput(
      requested=True,
      startup_gate_passed=bool(CS.bm_radar_startup_ready),
      stock_radar_alive=bool(CS.stock_radar_alive),
      vehicle_standstill=bool(CS.out.standstill),
      stock_cruise_engaged=bool(CS.out.cruiseState.enabled),
    ))
    if session.can_msg is not None:
      can_sends.append(session.can_msg)

    direct_ready = session.direct_longitudinal_ready
    planner_active = bool(CC.longActive and direct_ready)
    guarded = self.bm_long_guard.update(BMLongitudinalGuardInput(
      requested_accel=float(CC.actuators.accel),
      v_ego=float(CS.out.vEgo),
      long_active=planner_active,
      brake_pressed=bool(CS.out.brakePressed),
      lead_visible=bool(CC.hudControl.leadVisible),
      set_speed=float(CC.hudControl.setSpeed),
      now_nanos=int(now_nanos),
    ))
    self.bm_applied_accel = guarded.accel

    # Preserve the active cruise mode during a gas override, but command zero.
    # Dropping the active bits mid-override caused a lurch in donor-drive data.
    gas_override = bool(CC.enabled and (CC.cruiseControl.override or CS.out.gasPressed))
    long_engaged = bool(direct_ready and not CS.out.brakePressed and not guarded.critical_handoff and
                        (CC.longActive or gas_override))

    # Once the stock stream disappears, immediately cover the FSC's expected
    # no-target traffic while the one-second ownership verification completes.
    radar_master = (session.state in (BMRadarSessionState.VERIFY_SILENT, BMRadarSessionState.SILENCED) and
                    not CS.stock_radar_alive)
    if radar_master and self.frame % 10 == 0:
      for bus in BM_LONG_BUSES:
        can_sends.extend(mazdacan.create_bm_no_target_radar_frames(bus, self.bm_radar_counter))
      self.bm_radar_counter += 1

    if radar_master and self.frame % 2 == 0:
      acc_available = bool(CS.out.cruiseState.available)
      gap = int(CC.hudControl.leadDistanceBars) or 2
      # Verification frames are strictly inactive; active commands begin only
      # after the stock source has stayed absent for the complete guard period.
      tx_engaged = long_engaged if direct_ready else False
      tx_accel = self.bm_applied_accel if tx_engaged else 0.0
      for bus in BM_LONG_BUSES:
        can_sends.append(mazdacan.create_bm_direct_acc_command(
          bus, self.bm_long_counter, tx_accel, tx_engaged, acc_available,
        ))
        can_sends.append(mazdacan.create_bm_direct_crz_ctrl(
          bus, tx_engaged, acc_available, gap,
        ))
      self.bm_long_counter += 1

  def _should_send_bm_cancel(self, cancel_requested: bool, cruise_enabled: bool, brake_pressed: bool) -> bool:
    # Repeated CANCEL after ACC disengages can also turn cruise MAIN off on BM,
    # which removes the MADS lateral latch. Retry only while ACC is still active.
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

  def update(self, CC, CS, now_nanos):
    can_sends = []

    apply_torque = 0
    steer_max = CarControllerParams.STEER_MAX
    if self.bm_low_speed_steer:
      engine_speed_ms = getattr(CS, "engine_speed_ms", CS.out.vEgoRaw)
      steer_max = CarControllerParams.get_bm_steer_max(engine_speed_ms, CC.actuators.curvature)

    if CC.latActive:
      # calculate steer and also set limits due to driver torque
      new_torque = int(round(CC.actuators.torque * steer_max))
      apply_torque = apply_driver_steer_torque_limits(new_torque, self.apply_torque_last,
                                                      CS.out.steeringTorque, CarControllerParams, steer_max)
      apply_torque = max(-steer_max, min(steer_max, apply_torque))

    if self.bm_low_speed_steer:
      if self._should_send_bm_cancel(CC.cruiseControl.cancel, CS.out.cruiseState.enabled, CS.out.brakePressed):
        can_sends.append(mazdacan.create_button_cmd(self.packer, self.CP, CS.crz_btns_counter, Buttons.CANCEL))
    elif CC.cruiseControl.cancel:
      # If brake is pressed, let us wait >70ms before trying to disable crz to avoid
      # a race condition with the stock system, where the second cancel from openpilot
      # will disable the crz 'main on'. crz ctrl msg runs at 50hz. 70ms allows us to
      # read 3 messages and most likely sync state before we attempt cancel.
      self.brake_counter = self.brake_counter + 1
      if self.frame % 10 == 0 and not (CS.out.brakePressed and self.brake_counter < 7):
        # Cancel Stock ACC if it's enabled while OP is disengaged
        # Send at a rate of 10hz until we sync with stock ACC state
        can_sends.append(mazdacan.create_button_cmd(self.packer, self.CP, CS.crz_btns_counter, Buttons.CANCEL))
    else:
      self.brake_counter = 0

    if not CC.cruiseControl.cancel:
      if self.CP.autoResumeSng and CC.cruiseControl.resume and self.frame % 5 == 0:
        # Mazda Stop and Go requires a RES button (or gas) press if the car stops more than 3 seconds
        # Send Resume button when planner wants car to move
        can_sends.append(mazdacan.create_button_cmd(self.packer, self.CP, CS.crz_btns_counter, Buttons.RESUME))
      elif not self.CP.openpilotLongitudinalControl:
        # Mazid ICBM: SET+/SET− only to match persistent cruise set (vCruise),
        # never hudControl.setSpeed / planner follow speed (RC1-ACC-001).
        if self.frame % 10 == 0:
          button = self.icbm.update(
            self.frame,
            enabled=CC.enabled,
            cruise_enabled=CS.out.cruiseState.enabled,
            cruise_speed_ms=float(CS.out.cruiseState.speed),
            target_speed_ms=persistent_cruise_target_ms(float(CS.out.vCruise)),
          )
          if button is not None:
            can_sends.append(mazdacan.create_button_cmd(self.packer, self.CP, CS.crz_btns_counter, button))

    if self.CP.openpilotLongitudinalControl:
      self._update_bm_longitudinal(CC, CS, now_nanos, can_sends)

    self.apply_torque_last = apply_torque

    # Core 0x243 replay is deliberately outside every optional HUD boundary.
    # Packer errors here must continue to propagate.
    can_sends.append(mazdacan.create_steering_control(self.packer, self.CP,
                                                      self.frame, apply_torque, CS.cam_lkas))

    # send HUD alerts (2 Hz). Policy in HudBridge; packing in mazdacan.
    # DISPLAY_ONLY 0x440 path. Does not change 0x243 torque or ICBM buttons.
    if self.frame % 50 == 0:
      try:
        cam = CS.cam_laneinfo or {}
      except Exception as exc:
        self._disable_hud_enhancement("hud.cam_laneinfo", exc)
        self.hud_oem_copy_fault_active = True
      else:
        self._update_optional_hud(CC, CS, now_nanos, cam, can_sends)

    new_actuators = CC.actuators.as_builder()
    new_actuators.torque = apply_torque / steer_max
    new_actuators.torqueOutputCan = apply_torque
    if self.CP.openpilotLongitudinalControl:
      new_actuators.accel = self.bm_applied_accel

    self.frame += 1
    return new_actuators, can_sends
