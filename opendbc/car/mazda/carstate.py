from opendbc.can import CANDefine, CANParser
from opendbc.car import Bus, DT_CTRL, create_button_events, structs
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.interfaces import CarStateBase
from opendbc.car.mazda.values import CAR, DBC, LKAS_LIMITS

ButtonType = structs.CarState.ButtonEvent.Type

FSC_SETTLE_FRAMES = int(10.0 / DT_CTRL)
STOCK_RADAR_ALIVE_FRAMES = int(0.05 / DT_CTRL)
STOCK_RADAR_OWNERSHIP_FRAMES = STOCK_RADAR_ALIVE_FRAMES + int(1.0 / DT_CTRL)
CANCEL_CONTEXT_FRAMES = int(0.5 / DT_CTRL)
STOCK_RADAR_LIVENESS = (
  ("CRZ_INFO", "CTR1"),
  ("RADAR_TRACK_1", "LONG_DIST"),
)


class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)

    can_define = CANDefine(DBC[CP.carFingerprint][Bus.pt])
    self.shifter_values = can_define.dv["GEAR"]["GEAR"]

    self.crz_btns_counter = 0
    self.acc_active_last = False
    self.lkas_allowed_speed = False
    # Raw EPS authority signal. Default blocked until the first valid 0x241.
    self.lkas_blocked = True

    self.distance_button = 0
    self.accel_button = 0
    self.decel_button = 0
    self.set_plus_button = 0
    self.cancel_button = 0
    self.cruise_buttons_pressed = False

    # These attributes always exist because CarController must fail closed
    # before the first CAN update, including on non-longitudinal Mazda ports.
    self.stock_radar_alive = True
    self.stock_radar_has_lead = False
    self.stock_radar_lead_valid = False
    self.bm_radar_startup_ready = False
    self._stock_radar_silent_frames = 0
    self._radar_was_silenced = False
    self._cam_laneinfo_seen = False
    self._fsc_settled_frames = 0
    self._cruise_available = False
    self._cruise_enabled = False
    self._brake_pressed_prev = False
    self._cancel_context_frames = 0

  @staticmethod
  def _stock_radar_seen(cp) -> bool:
    # CRZ_INFO dropping is not enough: the radar can still own 0x361 after 0x21B
    # has gone quiet. Treat either stream as the stock radar still being alive.
    for msg, sig in STOCK_RADAR_LIVENESS:
      try:
        if len(cp.vl_all[msg][sig]) > 0:
          return True
      except (KeyError, TypeError):
        continue
    return False

  def update(self, can_parsers) -> structs.CarState:
    cp = can_parsers[Bus.pt]
    cp_cam = can_parsers[Bus.cam]

    ret = structs.CarState()

    self.parse_wheel_speeds(ret,
      cp.vl["WHEEL_SPEEDS"]["FL"],
      cp.vl["WHEEL_SPEEDS"]["FR"],
      cp.vl["WHEEL_SPEEDS"]["RL"],
      cp.vl["WHEEL_SPEEDS"]["RR"],
    )

    # Match panda speed reading
    speed_kph = cp.vl["ENGINE_DATA"]["SPEED"]
    self.engine_speed_ms = speed_kph * CV.KPH_TO_MS
    ret.standstill = speed_kph <= .1

    can_gear = int(cp.vl["GEAR"]["GEAR"])
    ret.gearShifter = self.parse_gear_shifter(self.shifter_values.get(can_gear, None))

    ret.genericToggle = bool(cp.vl["BLINK_INFO"]["HIGH_BEAMS"])
    ret.leftBlindspot = cp.vl["BSM"]["LEFT_BS_STATUS"] != 0
    ret.rightBlindspot = cp.vl["BSM"]["RIGHT_BS_STATUS"] != 0
    ret.leftBlinker, ret.rightBlinker = self.update_blinker_from_lamp(40, cp.vl["BLINK_INFO"]["LEFT_BLINK"] == 1,
                                                                      cp.vl["BLINK_INFO"]["RIGHT_BLINK"] == 1)

    ret.steeringAngleDeg = cp.vl["STEER"]["STEER_ANGLE"]
    ret.steeringTorque = cp.vl["STEER_TORQUE"]["STEER_TORQUE_SENSOR"]
    ret.steeringPressed = abs(ret.steeringTorque) > LKAS_LIMITS.STEER_THRESHOLD

    ret.steeringTorqueEps = cp.vl["STEER_TORQUE"]["STEER_TORQUE_MOTOR"]
    ret.steeringRateDeg = cp.vl["STEER_RATE"]["STEER_ANGLE_RATE"]

    ret.brakePressed = cp.vl["PEDALS"]["BRAKE_ON"] == 1

    ret.seatbeltUnlatched = cp.vl["SEATBELT"]["DRIVER_SEATBELT"] == 0
    ret.doorOpen = any([cp.vl["DOORS"]["FL"], cp.vl["DOORS"]["FR"],
                        cp.vl["DOORS"]["BL"], cp.vl["DOORS"]["BR"]])

    # TODO: this should be from 0 - 1.
    ret.gasPressed = cp.vl["ENGINE_DATA"]["PEDAL_GAS"] > 0

    # Either due to low speed or hands off
    lkas_blocked = cp.vl["STEER_RATE"]["LKAS_BLOCK"] == 1
    self.lkas_blocked = bool(lkas_blocked)

    if self.CP.minSteerSpeed > 0:
      # LKAS is enabled at 52kph going up and disabled at 45kph going down
      # wait for LKAS_BLOCK signal to clear when going up since it lags behind the speed sometimes
      if speed_kph > LKAS_LIMITS.ENABLE_SPEED and not lkas_blocked:
        self.lkas_allowed_speed = True
      elif speed_kph < LKAS_LIMITS.DISABLE_SPEED:
        self.lkas_allowed_speed = False
    else:
      self.lkas_allowed_speed = True

    if self.CP.openpilotLongitudinalControl:
      # Radar teardown removes radar-owned CRZ_CTRL. PEDALS remains body-owned
      # and carries the wheel/PCM engagement state used by BM before and after
      # the teardown: ACC_OFF is armed, ACC_ACTIVE is engaged.
      acc_armed = cp.vl["PEDALS"]["ACC_OFF"] == 1
      acc_active = cp.vl["PEDALS"]["ACC_ACTIVE"] == 1
      brake_released_edge = not ret.brakePressed and self._brake_pressed_prev
      if cp.vl["CRZ_BTNS"]["CAN_OFF"] == 1:
        self._cancel_context_frames = CANCEL_CONTEXT_FRAMES
      elif self._cancel_context_frames > 0:
        self._cancel_context_frames -= 1
      if acc_armed or acc_active:
        self._cruise_available = True
      elif brake_released_edge or self._cancel_context_frames > 0:
        self._cruise_available = False
      self._cruise_enabled = acc_active

      ret.cruiseState.enabled = self._cruise_enabled

      # Returned TX echoes use bus+128 and never enter this bus-0 parser, so
      # CRZ_INFO / RADAR_TRACK_1 arrivals here represent the physical stock source.
      if self._stock_radar_seen(cp):
        self._stock_radar_silent_frames = 0
      else:
        self._stock_radar_silent_frames += 1
      self.stock_radar_alive = self._stock_radar_silent_frames < STOCK_RADAR_ALIVE_FRAMES
      self._radar_was_silenced |= self._stock_radar_silent_frames >= STOCK_RADAR_OWNERSHIP_FRAMES
      ret.cruiseState.available = self._cruise_available and self._radar_was_silenced and not self.stock_radar_alive
      ret.accFaulted = self._radar_was_silenced and self.stock_radar_alive

      # Wait until the FSC has completed its boot/radar-presence phase before
      # requesting the diagnostic session. Requiring an observed frame avoids
      # treating the parser's initial all-zero values as a stable camera.
      self._cam_laneinfo_seen |= len(cp_cam.vl_all["CAM_LANEINFO"]["LANE_LINES"]) > 0
      laneinfo = cp_cam.vl["CAM_LANEINFO"]
      fsc_settled = self._cam_laneinfo_seen and not any(laneinfo[s] for s in ("NO_ERR_BIT", "BIT2", "ERR_BIT"))
      self._fsc_settled_frames = self._fsc_settled_frames + 1 if fsc_settled else 0
      self.bm_radar_startup_ready = self._fsc_settled_frames >= FSC_SETTLE_FRAMES
    else:
      # TODO: the signal used for available seems to be the adaptive cruise signal, instead of the main on
      #       it should be used for carState.cruiseState.nonAdaptive instead
      ret.cruiseState.available = cp.vl["CRZ_CTRL"]["CRZ_AVAILABLE"] == 1
      ret.cruiseState.enabled = cp.vl["CRZ_CTRL"]["CRZ_ACTIVE"] == 1
      if len(cp.vl_all["CRZ_CTRL"]["RADAR_HAS_LEAD"]) > 0:
        self.stock_radar_lead_valid = True
        self.stock_radar_has_lead = cp.vl["CRZ_CTRL"]["RADAR_HAS_LEAD"] == 1
      ret.stockRadarLead = bool(self.stock_radar_lead_valid and self.stock_radar_has_lead)
    ret.cruiseState.standstill = cp.vl["PEDALS"]["STANDSTILL"] == 1
    ret.cruiseState.speed = cp.vl["CRZ_EVENTS"]["CRZ_SPEED"] * CV.KPH_TO_MS

    # stock lkas should be on
    # TODO: is this needed?
    ret.invalidLkasSetting = cp_cam.vl["CAM_LANEINFO"]["LANE_LINES"] == 0

    if ret.cruiseState.enabled:
      if not self.lkas_allowed_speed and self.acc_active_last:
        self.low_speed_alert = True
      else:
        self.low_speed_alert = False
    ret.lowSpeedAlert = self.low_speed_alert

    # Check if LKAS is disabled due to lack of driver torque when all other states indicate
    # it should be enabled (steer lockout). Don't warn until we actually get lkas active
    # and lose it again, i.e, after initial lkas activation
    ret.steerFaultTemporary = self.lkas_allowed_speed and lkas_blocked

    self.acc_active_last = ret.cruiseState.enabled
    self._brake_pressed_prev = ret.brakePressed

    self.crz_btns_counter = cp.vl["CRZ_BTNS"]["CTR"]

    # camera signals
    self.cam_lkas = cp_cam.vl["CAM_LKAS"]
    self.cam_laneinfo = cp_cam.vl["CAM_LANEINFO"]
    # Camera-side ERR_BIT is advisory for BM: OP TX forces ERR=0 so we do not
    # poison EPS. Do not map it to steerFaultPermanent (that hard-disables
    # latActive until an ignition cycle). Non-BM keeps stock behavior.
    self.cam_lkas_err_bit = cp_cam.vl["CAM_LKAS"]["ERR_BIT_1"] == 1
    if self.CP.carFingerprint == CAR.MAZDA_3_2019:
      ret.steerFaultPermanent = False
    else:
      ret.steerFaultPermanent = self.cam_lkas_err_bit

    # cruise control button events: distance, inc, and dec
    prev_distance_button = self.distance_button
    prev_accel_button = self.accel_button
    prev_decel_button = self.decel_button
    self.distance_button = cp.vl["CRZ_BTNS"]["DISTANCE_LESS"]
    self.accel_button = cp.vl["CRZ_BTNS"]["RES"]
    self.decel_button = cp.vl["CRZ_BTNS"]["SET_M"]
    self.set_plus_button = cp.vl["CRZ_BTNS"]["SET_P"]
    self.cancel_button = cp.vl["CRZ_BTNS"]["CAN_OFF"]
    ret.cruiseSpeedButtonPressed = bool(self.accel_button or self.decel_button or self.set_plus_button)
    self.cruise_buttons_pressed = any(cp.vl["CRZ_BTNS"][signal] for signal in (
      "CAN_OFF", "RES", "SET_P", "SET_M", "DISTANCE_LESS", "DISTANCE_MORE", "MODE_X", "MODE_Y",
    ))

    ret.buttonEvents = [
      *create_button_events(self.distance_button, prev_distance_button, {1: ButtonType.gapAdjustCruise}),
      *create_button_events(self.accel_button, prev_accel_button, {1: ButtonType.accelCruise}),
      *create_button_events(self.decel_button, prev_decel_button, {1: ButtonType.decelCruise}),
    ]

    return ret

  @staticmethod
  def get_can_parsers(CP):
    pt_messages = []
    cam_messages = []
    if CP.openpilotLongitudinalControl:
      # Radar-owned frames disappear after takeover, so they have no liveness
      # requirement. vl_all supplies per-cycle arrival data. CRZ_CTRL is the
      # same family as CRZ_INFO: accessing vl["CRZ_CTRL"] must not trip canError.
      pt_messages.append(("CRZ_INFO", float("nan")))
      pt_messages.append(("CRZ_CTRL", float("nan")))
      pt_messages.append(("RADAR_TRACK_1", float("nan")))
      cam_messages.append(("CAM_LANEINFO", 0))
    return {
      Bus.pt: CANParser(DBC[CP.carFingerprint][Bus.pt], pt_messages, 0),
      Bus.cam: CANParser(DBC[CP.carFingerprint][Bus.pt], cam_messages, 2),
    }
