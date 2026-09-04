from opendbc.car.can_definitions import CanData
from opendbc.car.mazda.values import Buttons, MazdaFlags


# Mazda3 BM direct-long adapter. These byte layouts are isolated from the BM DBC on
# purpose: the stock mazda_2017/BM dictionaries do not contain the 1 mm/s^2 CRZ_INFO
# scaling used by the ZoomPilot candidate that successfully drove this BM vehicle.
# Only ordinary cruise/follow is represented here; stop/hold/resume bits and synthetic
# lead targets stay permanently clear in this first candidate.
BM_CRZ_INFO_ADDR = 0x21B
BM_CRZ_CTRL_ADDR = 0x21C

BM_NO_TARGET_RADAR_STATIC = (0x499, bytes.fromhex("0008c00000000000"))
BM_NO_TARGET_RADAR_TRACKS = (
  (0x361, bytes.fromhex("fff7fefe1fc00080")),
  (0x362, bytes.fromhex("fff7fefe1fc78c80")),
  (0x363, bytes.fromhex("fff7fefe1fc00000")),
  (0x364, bytes.fromhex("fff7fefe1fc00000")),
  (0x365, bytes.fromhex("fff7fe7ffbff3fc0")),
  (0x366, bytes.fromhex("fff7fe7ffbff3fc0")),
)


def bm_crz_info_checksum(dat: bytes) -> int:
  """Return the stock 0x21B inverted checksum.

  The stop bit is excluded by the Mazda radar checksum rule. This adapter never sets
  that bit, but retaining the exact rule keeps the byte contract explicit.
  """
  return (0xFF - ((sum(dat[:7]) - (dat[5] & 0x04)) & 0xFF)) & 0xFF


def create_bm_direct_acc_command(bus: int, counter: int, accel: float,
                                 long_active: bool, acc_available: bool) -> CanData:
  """Create ordinary-cruise CRZ_INFO (0x21B) without stop/hold/resume fields."""
  dat = bytearray(8)
  dat[0] = 0x01  # STATUS
  dat[1] = 0xFF
  dat[2] = 0xE0  # STATIC_1 = 0x7ff
  dat[6] = counter & 0x0F

  if long_active or acc_available:
    # ZoomPilot's proven definition: 13-bit unsigned, factor 0.001, offset -4.096.
    accel = max(-4.096, min(4.095, float(accel)))
    raw_accel = int((accel + 4.096) * 1000.0 + 0.5)
    dat[2] |= (raw_accel >> 11) & 0x03
    dat[3] = (raw_accel >> 3) & 0xFF
    dat[4] = ((raw_accel & 0x07) << 5) | (int(long_active) << 1) | 0x04
    dat[5] = 0x80  # NEW_SIGNAL_7; STOPPING remains clear
  else:
    # Stock radar standby pattern with the MRCC main switch off: ACCEL_CMD raw 8190.
    raw_accel = 0x1FFE
    dat[2] |= (raw_accel >> 11) & 0x03
    dat[3] = (raw_accel >> 3) & 0xFF
    dat[4] = (raw_accel & 0x07) << 5

  dat[7] = bm_crz_info_checksum(dat)
  return CanData(BM_CRZ_INFO_ADDR, bytes(dat), bus)


def create_bm_direct_crz_ctrl(bus: int, long_active: bool, acc_available: bool,
                              gap_setting: int) -> CanData:
  """Create ordinary-cruise CRZ_CTRL (0x21C) with no synthetic radar lead."""
  gap_setting = max(0, min(7, int(gap_setting)))
  dat = bytearray(8)
  dat[0] = 0x02 | (int(long_active) << 3)  # MSG_1_INV, CRZ_ACTIVE
  dat[1] = 0x01  # MSG_1_INV_COPY
  dat[2] = 0x01 | (int(long_active or acc_available) << 1) | (gap_setting << 2)
  if long_active:
    dat[3] = 0x20  # ordinary cruising phase; no stop/hold phase
    dat[6] = 0x10  # ACC_ACTIVE_2
  return CanData(BM_CRZ_CTRL_ADDR, bytes(dat), bus)


def create_bm_no_target_radar_frames(bus: int, counter: int) -> list[CanData]:
  """Create the captured no-object radar keepalive set; never synthesize a lead."""
  frames = [CanData(BM_NO_TARGET_RADAR_STATIC[0], BM_NO_TARGET_RADAR_STATIC[1], bus)]
  ctr = counter & 0x0F
  for addr, dat in BM_NO_TARGET_RADAR_TRACKS:
    frames.append(CanData(addr, dat[:7] + bytes([(dat[7] & 0xF0) | ctr]), bus))
  return frames


def create_steering_control(packer, CP, frame, apply_torque, lkas):

  tmp = apply_torque + 2048

  lo = tmp & 0xFF
  hi = tmp >> 8

  # copy values from camera
  b1 = int(lkas["BIT_1"])
  # Never forward FSC ERR bits to EPS. Echoing ERR_BIT latches OEM LKAS
  # faults and blocks recovery; C4 digests camera errors internally.
  er1 = 0
  lnv = 0
  ldw = 0
  er2 = 0

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


# Copied from FSC. Do not invent TJA / LDW_WARN_* / undocumented bits.
# BIT2 / NO_ERR_BIT / ERR_BIT are forced clean below — same class as LKAS ERR echo.
_CAM_LANEINFO_COPY = (
  "LINE_VISIBLE",
  "LINE_NOT_VISIBLE",
  "LANE_LINES",
  "BIT1",
  "BIT3",
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
  # Stock healthy laneinfo keeps these at 0 (fsc_settled treats any set as unsettled).
  values["BIT2"] = 0
  values["NO_ERR_BIT"] = 0
  values["ERR_BIT"] = 0
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
