from opendbc.can import CANPacker
from opendbc.car import Bus, structs
from opendbc.car.lateral import apply_driver_steer_torque_limits
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.mazda import mazdacan
from opendbc.car.mazda.hud_bridge import HudInputs, MazidHudBridge
from opendbc.car.mazda.hud_probe import MazidHudProbe, probe_enabled
from opendbc.car.mazda.icbm import MazdaIcbmController, persistent_cruise_target_ms
from opendbc.car.mazda.values import CarControllerParams, Buttons


class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP):
    super().__init__(dbc_names, CP)
    self.apply_torque_last = 0
    self.packer = CANPacker(dbc_names[Bus.pt])
    self.brake_counter = 0
    self.icbm = MazdaIcbmController()
    self.hud_bridge = MazidHudBridge()
    self.hud_probe = MazidHudProbe() if probe_enabled() else None

  def update(self, CC, CS, now_nanos):
    can_sends = []

    apply_torque = 0

    if CC.latActive:
      # calculate steer and also set limits due to driver torque
      new_torque = int(round(CC.actuators.torque * CarControllerParams.STEER_MAX))
      apply_torque = apply_driver_steer_torque_limits(new_torque, self.apply_torque_last,
                                                      CS.out.steeringTorque, CarControllerParams)

    if CC.cruiseControl.cancel:
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
      if CC.cruiseControl.resume and self.frame % 5 == 0:
        # Mazda Stop and Go requires a RES button (or gas) press if the car stops more than 3 seconds
        # Send Resume button when planner wants car to move
        can_sends.append(mazdacan.create_button_cmd(self.packer, self.CP, CS.crz_btns_counter, Buttons.RESUME))
      else:
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

    self.apply_torque_last = apply_torque

    # send HUD alerts (2 Hz). Policy in HudBridge; packing in mazdacan.
    # DISPLAY_ONLY 0x440 path. Does not change 0x243 torque or ICBM buttons.
    if self.frame % 50 == 0:
      cam = CS.cam_laneinfo or {}
      probe_tick = None
      if self.hud_probe is not None:
        probe_tick = self.hud_probe.update(
          standstill=bool(CS.out.standstill),
          v_ego=float(CS.out.vEgo),
          gear=CS.out.gearShifter,
          steer_fault_permanent=bool(CS.out.steerFaultPermanent),
          now_ns=int(now_nanos),
        )
      if probe_tick is not None and probe_tick.static and probe_tick.display is not None:
        d = probe_tick.display
        msg = mazdacan.create_alert_command(
          self.packer, cam, False, False,
          lane_lines=None if d.copy_oem else d.lane_lines,
          line_visible=None if d.copy_oem else d.line_visible,
          line_not_visible=None if d.copy_oem else d.line_not_visible,
          hands_on=None if d.copy_oem else d.hands_on,
          hands_on_2=None if d.copy_oem else d.hands_on_2,
          hands_warn_3=None if d.copy_oem else d.hands_warn_3,
          copy_oem=d.copy_oem,
        )
        self.hud_probe.log_tx(
          gid=probe_tick.gid,
          payload_hex=msg[1].hex(),
          v_ego=float(CS.out.vEgo),
          gear=int(CS.out.gearShifter),
          standstill=bool(CS.out.standstill),
          now_ns=int(now_nanos),
        )
        can_sends.append(msg)
      else:
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
        can_sends.append(mazdacan.create_alert_command(
          self.packer, cam, hud.ldw, hud.steer_required,
          lane_lines=hud.override_lane_lines,
          line_visible=hud.line_visible,
          line_not_visible=hud.line_not_visible,
        ))

    # send steering command
    can_sends.append(mazdacan.create_steering_control(self.packer, self.CP,
                                                      self.frame, apply_torque, CS.cam_lkas))

    new_actuators = CC.actuators.as_builder()
    new_actuators.torque = apply_torque / CarControllerParams.STEER_MAX
    new_actuators.torqueOutputCan = apply_torque

    self.frame += 1
    return new_actuators, can_sends
