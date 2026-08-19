from opendbc.car.mazda.values import Buttons, MazdaFlags


def create_steering_control(packer, CP, frame, apply_torque, lkas):

  tmp = apply_torque + 2048

  lo = tmp & 0xFF
  hi = tmp >> 8

  # copy values from camera
  b1 = int(lkas["BIT_1"])
  er1 = int(lkas["ERR_BIT_1"])
  lnv = 0
  ldw = 0
  er2 = int(lkas["ERR_BIT_2"])

  # Some older models do have these, newer models don't.
  # Either way, they all work just fine if set to zero.
  steering_angle = 0
  b2 = 0

  tmp = steering_angle + 2048
  ahi = tmp >> 10
  amd = (tmp & 0x3FF) >> 2
  amd = (amd >> 4) | ((amd & 0xF) << 4)
  alo = (tmp & 0x3) << 2

  ctr = frame % 16
  # bytes:     [    1  ] [ 2 ] [             3               ]  [           4         ]
  csum = 249 - ctr - hi - lo - (lnv << 3) - er1 - (ldw << 7) - (er2 << 4) - (b1 << 5)

  # bytes      [ 5 ] [ 6 ] [    7   ]
  csum = csum - ahi - amd - alo - b2

  if ahi == 1:
    csum = csum + 15

  if csum < 0:
    if csum < -256:
      csum = csum + 512
    else:
      csum = csum + 256

  csum = csum % 256

  values = {}
  if CP.flags & MazdaFlags.GEN1:
    values = {
      "LKAS_REQUEST": apply_torque,
      "CTR": ctr,
      "ERR_BIT_1": er1,
      "LINE_NOT_VISIBLE": lnv,
      "LDW": ldw,
      "BIT_1": b1,
      "ERR_BIT_2": er2,
      "STEERING_ANGLE": steering_angle,
      "ANGLE_ENABLED": b2,
      "CHKSUM": csum
    }

  return packer.make_can_msg("CAM_LKAS", 0, values)


# Copied from FSC. Do not invent TJA / ERR_BIT / LDW_WARN_* / undocumented bits.
_CAM_LANEINFO_COPY = (
  "LINE_VISIBLE",
  "LINE_NOT_VISIBLE",
  "LANE_LINES",
  "BIT1",
  "BIT2",
  "BIT3",
  "NO_ERR_BIT",
  "S1",
  "S1_HBEAM",
)


def create_alert_command(packer, cam_msg: dict, ldw: bool, steer_required: bool,
                         lane_lines: int | None = None,
                         line_visible: int | None = None,
                         line_not_visible: int | None = None,
                         hands_on: bool | None = None,
                         hands_on_2: bool | None = None,
                         hands_warn_3: int | None = None,
                         copy_oem: bool = False):
  """Pack CAM_LANEINFO (0x440). Display-only vs EPS: Safety does not inspect this payload.

  LINE_* / reserved bits are copied from OEM FSC cam_msg unless a DISPLAY_ONLY
  override is supplied (HUD-PROBE-003 gallery / C4 road-seen).
  LANE_LINES: HudBridge may override (HUD-BRIDGE-002 actual-control graphic).
  If lane_lines is None, copy FSC (legacy). Do not pack TJA (unproven).
  `ldw` is accepted for API compatibility but is not packed (LDW_WARN_* stay 0).
  Hands bits come from HudBridge policy unless independently overridden.
  None-defaults keep HUD-BRIDGE-001/002 packing.
  """
  values = {s: cam_msg[s] for s in _CAM_LANEINFO_COPY}
  if copy_oem:
    values["HANDS_WARN_3_BITS"] = int(cam_msg.get("HANDS_WARN_3_BITS", 0) or 0)
    values["HANDS_ON_STEER_WARN"] = bool(cam_msg.get("HANDS_ON_STEER_WARN", 0))
    values["HANDS_ON_STEER_WARN_2"] = bool(cam_msg.get("HANDS_ON_STEER_WARN_2", 0))
    values["LDW_WARN_LL"] = 0
    values["LDW_WARN_RL"] = 0
    _ = ldw
    return packer.make_can_msg("CAM_LANEINFO", 0, values)

  if lane_lines is not None:
    values["LANE_LINES"] = int(lane_lines)
  if line_visible is not None:
    values["LINE_VISIBLE"] = int(line_visible)
  if line_not_visible is not None:
    values["LINE_NOT_VISIBLE"] = int(line_not_visible)

  h1 = bool(steer_required) if hands_on is None else bool(hands_on)
  h2 = bool(steer_required) if hands_on_2 is None else bool(hands_on_2)
  if hands_warn_3 is None:
    w3 = 0b111 if (h1 or h2) else 0
  else:
    w3 = int(hands_warn_3)
  values.update({
    "HANDS_WARN_3_BITS": w3,
    "HANDS_ON_STEER_WARN": h1,
    "HANDS_ON_STEER_WARN_2": h2,
    "LDW_WARN_LL": 0,
    "LDW_WARN_RL": 0,
  })
  _ = ldw  # not packed; see HUD_CAN_FIELD_MAP
  return packer.make_can_msg("CAM_LANEINFO", 0, values)


def create_button_cmd(packer, CP, counter, button):

  can = int(button == Buttons.CANCEL)
  res = int(button == Buttons.RESUME)
  # ICBM / stock cruise: SET+ and SET- (sunnypilot/vendor pattern; Mazid SELECTIVE_PORT)
  inc = int(button == Buttons.SET_PLUS)
  dec = int(button == Buttons.SET_MINUS)

  if CP.flags & MazdaFlags.GEN1:
    values = {
      "CAN_OFF": can,
      "CAN_OFF_INV": (can + 1) % 2,

      "SET_P": inc,
      "SET_P_INV": (inc + 1) % 2,

      "RES": res,
      "RES_INV": (res + 1) % 2,

      "SET_M": dec,
      "SET_M_INV": (dec + 1) % 2,

      "DISTANCE_LESS": 0,
      "DISTANCE_LESS_INV": 1,

      "DISTANCE_MORE": 0,
      "DISTANCE_MORE_INV": 1,

      "MODE_X": 0,
      "MODE_X_INV": 1,

      "MODE_Y": 0,
      "MODE_Y_INV": 1,

      "BIT1": 1,
      "BIT2": 1,
      "BIT3": 1,
      "CTR": (counter + 1) % 16,
    }

    return packer.make_can_msg("CRZ_BTNS", 0, values)
