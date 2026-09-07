#!/usr/bin/env python3
from opendbc.can import CANParser
from opendbc.car import Bus, structs
from opendbc.car.interfaces import RadarInterfaceBase
from opendbc.car.mazda.values import CAR, DBC


RADAR_TRACK_1 = 0x361
RADAR_FREQUENCY = 10
NO_TARGET_DISTANCE = 0xFFF / 16


def _create_radar_can_parser(CP):
  if CP.carFingerprint != CAR.MAZDA_3_2019:
    return None

  return CANParser(DBC[CP.carFingerprint][Bus.pt], [(RADAR_TRACK_1, RADAR_FREQUENCY)], 0)


class RadarInterface(RadarInterfaceBase):
  """PC-only candidate for the Mazda3 BM longitudinal radar target.

  The target's lateral position and distance reference point are not calibrated.
  CarParams therefore keeps radarUnavailable set, and tests/replay must explicitly
  opt in by clearing it on an isolated CarParams instance.
  """

  def __init__(self, CP):
    super().__init__(CP)
    self.rcp = None if CP.radarUnavailable else _create_radar_can_parser(CP)
    self.radar_seen = False
    self.can_error_active = False

  def update(self, can_packets):
    if self.rcp is None:
      return super().update(None)

    updated_messages = self.rcp.update(can_packets)

    # Once a valid radar frame has been seen, a parser timeout must clear the
    # previous point and publish one fail-closed error frame. Do not repeatedly
    # publish at card's 100 Hz update rate while the radar remains absent.
    if not self.rcp.can_valid:
      if self.radar_seen and not self.can_error_active:
        self.pts.clear()
        self.can_error_active = True
        ret = structs.RadarData()
        ret.errors.canError = True
        return ret
      return None

    if RADAR_TRACK_1 not in updated_messages:
      return None

    self.radar_seen = True
    self.can_error_active = False
    ret = structs.RadarData()
    track = self.rcp.vl[RADAR_TRACK_1]
    distance = track['LONG_DIST']

    if distance >= NO_TARGET_DISTANCE:
      self.pts.pop(RADAR_TRACK_1, None)
    else:
      if RADAR_TRACK_1 not in self.pts:
        point = structs.RadarData.RadarPoint()
        point.trackId = self.track_id
        self.track_id += 1
        self.pts[RADAR_TRACK_1] = point

      point = self.pts[RADAR_TRACK_1]
      point.dRel = distance
      # RADAR-002 did not calibrate lateral position. NaN explicitly marks the
      # point as longitudinal-only; radard filters non-finite points and keeps
      # vision fallback until a later task calibrates lateral position.
      point.yRel = float('nan')
      point.vRel = track['REL_SPEED']

    ret.points = list(self.pts.values())
    return ret
