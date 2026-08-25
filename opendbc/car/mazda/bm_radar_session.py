"""Mazda3 BM experimental vision-only radar session manager.

This module only sequences the radar diagnostic session.  It does not create
longitudinal commands, synthetic radar tracks, or change CarParams/Safety.  A
runtime caller may use ``direct_longitudinal_ready`` only after the stock radar
has been observed silent.
"""

from dataclasses import dataclass
from enum import StrEnum

from opendbc.car import make_tester_present_msg, uds
from opendbc.car.can_definitions import CanData


BM_RADAR_ADDR = 0x764
BM_RADAR_BUS = 0

# The control loop runs at 100 Hz.  Keep diagnostic traffic at 2 Hz, matching
# the only on-car sequence that produced a stable silent session in the donor
# evidence.  These are initial candidates until BM-specific timing is measured.
BM_RADAR_UDS_STEP = 50
BM_RADAR_SILENT_GUARD_FRAMES = 100
BM_RADAR_SILENCE_TIMEOUT_FRAMES = 500
BM_RADAR_HANDBACK_TIMEOUT_FRAMES = 700


def create_bm_radar_session_msg(session_type: uds.SESSION_TYPE) -> CanData:
  """Create one byte-exact, single-frame UDS session-control request."""
  return CanData(BM_RADAR_ADDR, bytes([
    0x02,
    uds.SERVICE_TYPE.DIAGNOSTIC_SESSION_CONTROL,
    int(session_type),
    0x00, 0x00, 0x00, 0x00, 0x00,
  ]), BM_RADAR_BUS)


def create_bm_radar_tester_present_msg() -> CanData:
  """Keep a proven silent session alive without requesting a response."""
  return make_tester_present_msg(BM_RADAR_ADDR, BM_RADAR_BUS, suppress_response=True)


class BMRadarSessionState(StrEnum):
  STOCK = "stock"
  WAITING_GATE = "waiting_gate"
  SILENCING = "silencing"
  VERIFY_SILENT = "verify_silent"
  SILENCED = "silenced"
  HANDBACK = "handback"
  FAULT = "fault"


@dataclass(frozen=True)
class BMRadarSessionInput:
  requested: bool
  startup_gate_passed: bool
  stock_radar_alive: bool
  vehicle_standstill: bool
  stock_cruise_engaged: bool


@dataclass(frozen=True)
class BMRadarSessionOutput:
  state: BMRadarSessionState
  can_msg: CanData | None
  radar_silenced: bool
  direct_longitudinal_ready: bool
  fault_reason: str


class BMRadarSessionManager:
  """Fail-closed ownership handshake for the BM forward radar.

  Takeover starts only while stationary and with stock cruise disengaged.  A
  recovered radar while SILENCED is a two-master hazard, so it latches FAULT
  instead of trying to re-silence while driving.  Clearing a fault requires the
  experimental request to be turned off and the stock radar to be observed
  alive again.
  """

  def __init__(self) -> None:
    self.state = BMRadarSessionState.STOCK
    self._frames_in_state = 0
    self.fault_reason = ""

  def _transition(self, state: BMRadarSessionState, fault_reason: str = "") -> None:
    if state != self.state:
      self.state = state
      self._frames_in_state = 0
    self.fault_reason = fault_reason

  def _message_due(self) -> bool:
    return self._frames_in_state % BM_RADAR_UDS_STEP == 0

  def update(self, inp: BMRadarSessionInput) -> BMRadarSessionOutput:
    if not inp.requested:
      if self.state in (BMRadarSessionState.SILENCING, BMRadarSessionState.VERIFY_SILENT,
                        BMRadarSessionState.SILENCED):
        self._transition(BMRadarSessionState.HANDBACK)
      elif self.state == BMRadarSessionState.WAITING_GATE:
        self._transition(BMRadarSessionState.STOCK)
      elif self.state == BMRadarSessionState.FAULT:
        self._transition(BMRadarSessionState.STOCK if inp.stock_radar_alive else BMRadarSessionState.HANDBACK)
    else:
      if self.state == BMRadarSessionState.STOCK:
        self._transition(BMRadarSessionState.WAITING_GATE)

      if self.state == BMRadarSessionState.WAITING_GATE:
        safe_to_start = inp.startup_gate_passed and inp.vehicle_standstill and not inp.stock_cruise_engaged
        if safe_to_start:
          next_state = BMRadarSessionState.SILENCING if inp.stock_radar_alive else BMRadarSessionState.VERIFY_SILENT
          self._transition(next_state)

      elif self.state == BMRadarSessionState.SILENCING:
        if not inp.vehicle_standstill or inp.stock_cruise_engaged:
          self._transition(BMRadarSessionState.HANDBACK)
        elif not inp.stock_radar_alive:
          self._transition(BMRadarSessionState.VERIFY_SILENT)
        elif self._frames_in_state >= BM_RADAR_SILENCE_TIMEOUT_FRAMES:
          self._transition(BMRadarSessionState.FAULT, "RADAR_SILENCE_TIMEOUT")

      elif self.state == BMRadarSessionState.VERIFY_SILENT:
        if inp.stock_radar_alive:
          self._transition(BMRadarSessionState.HANDBACK, "STOCK_RADAR_REAPPEARED_DURING_VERIFY")
        elif self._frames_in_state >= BM_RADAR_SILENT_GUARD_FRAMES:
          self._transition(BMRadarSessionState.SILENCED)

      elif self.state == BMRadarSessionState.SILENCED and inp.stock_radar_alive:
        self._transition(BMRadarSessionState.FAULT, "STOCK_RADAR_RECOVERED")

    if self.state == BMRadarSessionState.HANDBACK:
      # Emit at least one explicit DEFAULT request before accepting the returned
      # stream; the first alive frame can predate our programming request.
      if self._frames_in_state > 0 and inp.stock_radar_alive:
        self._transition(BMRadarSessionState.STOCK)
      elif self._frames_in_state >= BM_RADAR_HANDBACK_TIMEOUT_FRAMES:
        self._transition(BMRadarSessionState.FAULT, "RADAR_HANDBACK_TIMEOUT")

    can_msg = None
    if self._message_due():
      if self.state == BMRadarSessionState.SILENCING:
        can_msg = create_bm_radar_session_msg(uds.SESSION_TYPE.PROGRAMMING)
      elif self.state in (BMRadarSessionState.VERIFY_SILENT, BMRadarSessionState.SILENCED):
        can_msg = create_bm_radar_tester_present_msg()
      elif self.state == BMRadarSessionState.HANDBACK:
        can_msg = create_bm_radar_session_msg(uds.SESSION_TYPE.DEFAULT)

    radar_silenced = self.state == BMRadarSessionState.SILENCED and not inp.stock_radar_alive
    output = BMRadarSessionOutput(
      state=self.state,
      can_msg=can_msg,
      radar_silenced=radar_silenced,
      direct_longitudinal_ready=radar_silenced and not self.fault_reason,
      fault_reason=self.fault_reason,
    )
    self._frames_in_state += 1
    return output
