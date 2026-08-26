import unittest

from opendbc.car import uds
from opendbc.car.mazda.bm_radar_session import (
  BM_RADAR_ADDR,
  BM_RADAR_BUS,
  BM_RADAR_HANDBACK_TIMEOUT_FRAMES,
  BM_RADAR_SILENT_GUARD_FRAMES,
  BM_RADAR_SILENCE_TIMEOUT_FRAMES,
  BM_RADAR_UDS_STEP,
  BMRadarSessionInput,
  BMRadarSessionManager,
  BMRadarSessionState,
  create_bm_radar_session_msg,
  create_bm_radar_tester_present_msg,
  may_replace_crz,
  may_replace_radar_tracks,
)


def session_input(*, requested=True, gate=True, alive=True, standstill=True, cruise=False):
  return BMRadarSessionInput(
    requested=requested,
    startup_gate_passed=gate,
    stock_radar_alive=alive,
    vehicle_standstill=standstill,
    stock_cruise_engaged=cruise,
  )


class TestBMRadarPayloads(unittest.TestCase):
  def test_exact_payloads(self):
    programming = create_bm_radar_session_msg(uds.SESSION_TYPE.PROGRAMMING)
    default = create_bm_radar_session_msg(uds.SESSION_TYPE.DEFAULT)
    tester_present = create_bm_radar_tester_present_msg()

    self.assertEqual((programming.address, programming.src, programming.dat),
                     (BM_RADAR_ADDR, BM_RADAR_BUS, bytes.fromhex("0210020000000000")))
    self.assertEqual((default.address, default.src, default.dat),
                     (BM_RADAR_ADDR, BM_RADAR_BUS, bytes.fromhex("0210010000000000")))
    self.assertEqual((tester_present.address, tester_present.src, tester_present.dat),
                     (BM_RADAR_ADDR, BM_RADAR_BUS, bytes.fromhex("023e800000000000")))


class TestBMRadarSessionManager(unittest.TestCase):
  def test_default_is_zero_tx(self):
    manager = BMRadarSessionManager()
    out = manager.update(session_input(requested=False))
    self.assertEqual(out.state, BMRadarSessionState.STOCK)
    self.assertIsNone(out.can_msg)
    self.assertFalse(out.direct_longitudinal_ready)

  def test_takeover_waits_for_all_gates(self):
    for kwargs in ({"gate": False}, {"standstill": False}, {"cruise": True}):
      with self.subTest(kwargs=kwargs):
        out = BMRadarSessionManager().update(session_input(**kwargs))
        self.assertEqual(out.state, BMRadarSessionState.WAITING_GATE)
        self.assertIsNone(out.can_msg)

  def test_programming_then_tester_present(self):
    manager = BMRadarSessionManager()
    out = manager.update(session_input())
    self.assertEqual(out.state, BMRadarSessionState.SILENCING)
    self.assertEqual(out.can_msg.dat, bytes.fromhex("0210020000000000"))

    out = manager.update(session_input(alive=False))
    self.assertEqual(out.state, BMRadarSessionState.VERIFY_SILENT)
    self.assertEqual(out.can_msg.dat, bytes.fromhex("023e800000000000"))
    self.assertFalse(out.direct_longitudinal_ready)

    out = None
    for _ in range(BM_RADAR_SILENT_GUARD_FRAMES):
      out = manager.update(session_input(alive=False))
    self.assertEqual(out.state, BMRadarSessionState.SILENCED)
    self.assertTrue(out.direct_longitudinal_ready)

  def test_abort_takeover_if_vehicle_moves(self):
    manager = BMRadarSessionManager()
    manager.update(session_input())
    out = manager.update(session_input(standstill=False, alive=True))
    self.assertEqual(out.state, BMRadarSessionState.HANDBACK)
    self.assertEqual(out.can_msg.dat, bytes.fromhex("0210010000000000"))
    out = manager.update(session_input(requested=False, standstill=False, alive=True))
    self.assertEqual(out.state, BMRadarSessionState.STOCK)

  def test_handback_sends_default_until_radar_returns(self):
    manager = BMRadarSessionManager()
    manager.update(session_input())
    manager.update(session_input(alive=False))
    for _ in range(BM_RADAR_SILENT_GUARD_FRAMES):
      manager.update(session_input(alive=False))

    out = manager.update(session_input(requested=False, alive=False))
    self.assertEqual(out.state, BMRadarSessionState.HANDBACK)
    self.assertEqual(out.can_msg.dat, bytes.fromhex("0210010000000000"))
    self.assertFalse(out.direct_longitudinal_ready)

    out = manager.update(session_input(requested=False, alive=True))
    self.assertEqual(out.state, BMRadarSessionState.STOCK)
    self.assertIsNone(out.can_msg)

  def test_recovered_radar_latches_fault(self):
    manager = BMRadarSessionManager()
    manager.update(session_input())
    manager.update(session_input(alive=False))
    for _ in range(BM_RADAR_SILENT_GUARD_FRAMES):
      manager.update(session_input(alive=False))
    out = manager.update(session_input(alive=True))
    self.assertEqual(out.state, BMRadarSessionState.FAULT)
    self.assertEqual(out.fault_reason, "STOCK_RADAR_RECOVERED")
    self.assertIsNone(out.can_msg)
    self.assertFalse(out.direct_longitudinal_ready)

  def test_silence_timeout_is_fail_closed(self):
    manager = BMRadarSessionManager()
    manager.update(session_input())
    for _ in range(BM_RADAR_SILENCE_TIMEOUT_FRAMES):
      out = manager.update(session_input())
    self.assertEqual(out.state, BMRadarSessionState.FAULT)
    self.assertEqual(out.fault_reason, "RADAR_SILENCE_TIMEOUT")
    self.assertIsNone(out.can_msg)

  def test_handback_timeout_is_fail_closed(self):
    manager = BMRadarSessionManager()
    manager.update(session_input())
    manager.update(session_input(alive=False))
    for _ in range(BM_RADAR_SILENT_GUARD_FRAMES):
      manager.update(session_input(alive=False))
    manager.update(session_input(requested=False, alive=False))
    for _ in range(BM_RADAR_HANDBACK_TIMEOUT_FRAMES):
      out = manager.update(session_input(requested=False, alive=False))
    self.assertEqual(out.state, BMRadarSessionState.FAULT)
    self.assertEqual(out.fault_reason, "RADAR_HANDBACK_TIMEOUT")
    self.assertIsNone(out.can_msg)

  def test_only_radar_diagnostics_are_emitted(self):
    manager = BMRadarSessionManager()
    seen = []
    for _ in range(BM_RADAR_UDS_STEP + 1):
      msg = manager.update(session_input()).can_msg
      if msg is not None:
        seen.append(msg)
    self.assertTrue(seen)
    self.assertTrue(all(msg.address == BM_RADAR_ADDR and msg.src == BM_RADAR_BUS for msg in seen))
    self.assertNotIn(0x21B, [msg.address for msg in seen])
    self.assertNotIn(0x21C, [msg.address for msg in seen])

  def test_empty_radar_tracks_wait_for_silenced(self):
    self.assertFalse(may_replace_radar_tracks(BMRadarSessionState.VERIFY_SILENT, False))
    self.assertTrue(may_replace_radar_tracks(BMRadarSessionState.SILENCED, False))
    self.assertFalse(may_replace_radar_tracks(BMRadarSessionState.SILENCED, True))
    self.assertTrue(may_replace_crz(BMRadarSessionState.VERIFY_SILENT, False))
    self.assertFalse(may_replace_crz(BMRadarSessionState.VERIFY_SILENT, True))

  def test_immediate_handback_emits_default_session(self):
    manager = BMRadarSessionManager()
    manager.update(session_input())
    out = manager.request_immediate_handback()
    self.assertEqual(out.state, BMRadarSessionState.HANDBACK)
    self.assertEqual(out.can_msg.dat, bytes.fromhex("0210010000000000"))
    self.assertFalse(out.direct_longitudinal_ready)

  def test_immediate_handback_from_stock_is_zero_tx(self):
    out = BMRadarSessionManager().request_immediate_handback()
    self.assertEqual(out.state, BMRadarSessionState.STOCK)
    self.assertIsNone(out.can_msg)


if __name__ == "__main__":
  unittest.main()
