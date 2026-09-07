import unittest
from types import SimpleNamespace

from opendbc.car import uds
from opendbc.car.can_definitions import CanData
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


class TestBMRadarControllerSession(unittest.TestCase):
  def setUp(self):
    from opendbc.can import CANPacker
    from opendbc.car import Bus, structs
    from opendbc.car.mazda.carcontroller import CarController
    from opendbc.car.mazda.values import CAR, DBC, MazdaFlags, SteerEnvelope

    # Exercise the actual update/send path with a real packer, without the
    # Params/ICBM constructor or device I/O.
    self.controller = CarController.__new__(CarController)
    self.controller.CP = SimpleNamespace(openpilotLongitudinalControl=True, autoResumeSng=False, flags=MazdaFlags.GEN1)
    self.controller.packer = CANPacker(DBC[CAR.MAZDA_3_2019][Bus.pt])
    self.controller.frame = 0
    self.controller.apply_torque_last = 0
    self.controller.bm_low_speed_steer = True
    self.controller.brake_counter = 0
    self.controller.last_cancel_frame = -50
    self.controller.bm_radar_session = BMRadarSessionManager()
    self.controller._radar_shutdown_done = False
    self.controller.bm_long_counter = 0
    self.controller.bm_radar_counter = 0
    self.controller.bm_tx_accel_last = 0.0
    self.controller._steer_envelope = SteerEnvelope.STABLE_1300
    self.controller._blinker_lkas_suspend = False
    self.controller._read_steer_envelope = lambda: SteerEnvelope.STABLE_1300
    cc = structs.CarControl()
    cc.enabled = True
    cc.longActive = True
    cc.actuators.accel = 0.6
    self.cc = cc.as_reader()
    self.cs = SimpleNamespace(
      bm_radar_startup_ready=True, stock_radar_alive=True, lkas_allowed_speed=True,
      crz_btns_counter=0, engine_speed_ms=0.0,
      out=SimpleNamespace(
        standstill=True, brakePressed=False, gasPressed=False, vEgo=0.0, vEgoRaw=0.0,
        cruiseState=SimpleNamespace(enabled=False, available=True, standstill=True),
      ),
      cam_lkas={"BIT_1": 0},
      cam_laneinfo={signal: 0 for signal in (
        "LINE_VISIBLE", "LINE_NOT_VISIBLE", "LANE_LINES", "BIT1", "BIT3", "S1", "S1_HBEAM",
      )},
    )
    self.radar_addresses = {0x499, *range(0x361, 0x367)}
    self.long_addresses = self.radar_addresses | {0x21B, 0x21C, BM_RADAR_ADDR}

  def step(self):
    return [CanData(*msg) for msg in self.controller.update(self.cc, self.cs, 0)[1]]

  def settle_silent(self):
    self.step()
    self.cs.stock_radar_alive = False
    for _ in range(BM_RADAR_SILENT_GUARD_FRAMES + 1):
      self.step()
    self.assertEqual(self.controller.bm_radar_session.state, BMRadarSessionState.SILENCED)

  def test_update_waits_for_full_silent_guard_before_target_frames(self):
    self.step()
    self.cs.stock_radar_alive = False
    crz = []
    for _ in range(BM_RADAR_SILENT_GUARD_FRAMES):
      msgs = self.step()
      self.assertFalse(self.radar_addresses & {msg.address for msg in msgs})
      crz.extend(msg for msg in msgs if msg.address == 0x21B)
    self.assertEqual(self.controller.bm_radar_session.state, BMRadarSessionState.VERIFY_SILENT)
    self.assertTrue(crz)
    self.assertTrue(all(msg.dat[4] & 0x02 == 0 for msg in crz))
    msgs = [msg for _ in range(10) for msg in self.step()]
    for bus in (0, 2):
      self.assertEqual(self.radar_addresses & {msg.address for msg in msgs if msg.src == bus}, self.radar_addresses)
    self.assertTrue(any(msg.address == 0x21B and msg.dat[4] & 0x02 for msg in msgs))

  def test_stock_mode_update_and_shutdown_emit_no_longitudinal_frames(self):
    self.controller.CP.openpilotLongitudinalControl = False
    for _ in range(110):
      self.assertFalse(self.long_addresses & {msg.address for msg in self.step()})
    self.assertEqual(self.controller.bm_radar_session.state, BMRadarSessionState.STOCK)
    self.assertEqual(self.controller.shutdown_radar_session(), [])

  def test_shutdown_before_takeover_is_zero_tx(self):
    self.cs.bm_radar_startup_ready = False
    self.step()
    self.assertEqual(self.controller.bm_radar_session.state, BMRadarSessionState.WAITING_GATE)
    self.assertEqual(self.controller.shutdown_radar_session(), [])

  def test_shutdown_emits_one_handback_and_stops_future_longitudinal_tx(self):
    self.settle_silent()
    msgs = self.controller.shutdown_radar_session()
    self.assertEqual([(msg.address, msg.src, msg.dat) for msg in msgs],
                     [(BM_RADAR_ADDR, BM_RADAR_BUS, bytes.fromhex("0210010000000000"))])
    self.assertEqual(self.controller.bm_tx_accel_last, 0.0)
    self.assertEqual(self.controller.shutdown_radar_session(), [])
    for _ in range(110):
      self.assertFalse(self.long_addresses & {msg.address for msg in self.step()})

  def test_reappeared_stock_radar_stops_replacement_frames(self):
    self.settle_silent()
    self.cs.stock_radar_alive = True
    for _ in range(110):
      self.assertFalse(self.long_addresses & {msg.address for msg in self.step()})
    self.assertEqual(self.controller.bm_radar_session.state, BMRadarSessionState.FAULT)

  def test_positive_recovery_is_limited_on_actual_encoded_commands(self):
    cc = self.cc.as_builder()
    cc.actuators.accel = -1.5
    self.cc = cc.as_reader()
    self.settle_silent()
    for _ in range(110):
      self.step()
    self.assertEqual(self.controller.bm_tx_accel_last, -1.5)

    cc = self.cc.as_builder()
    cc.actuators.accel = 0.6
    self.cc = cc.as_reader()
    last_milli = -1500
    for _ in range(300):
      for msg in self.step():
        if msg.address == 0x21B and msg.src == 0:
          milli = ((msg.dat[2] & 0x03) << 11 | msg.dat[3] << 3 | msg.dat[4] >> 5) - 4096
          self.assertLessEqual(max(0, milli) - max(0, last_milli), 10)
          if last_milli <= -24:
            self.assertEqual(milli - last_milli, 24)
          self.assertGreaterEqual(milli, -1500)
          self.assertLessEqual(milli, 600)
          last_milli = milli
    self.assertEqual(last_milli, 600)

  def test_slew_preserves_braking_and_negative_accel_release(self):
    from opendbc.car.mazda.carcontroller import slew_bm_direct_long_accel
    for target, previous, expected in (
      (-1.5, 0.6, 0.570), (-1.5, 0.0, -0.030), (-1.5, -0.7, -0.730),
      (-3.5, -1.49, -1.5), (0.0, 0.02, 0.0), (0.3, 0.5, 0.470),
      (0.0, -1.5, -1.476), (0.6, -0.1, -0.076), (-0.05, -0.5, -0.476),
    ):
      with self.subTest(target=target, previous=previous):
        self.assertEqual(slew_bm_direct_long_accel(target, previous), expected)

  def test_positive_slew_respects_zero_crossing_and_milli_grid(self):
    from opendbc.car.mazda.carcontroller import slew_bm_direct_long_accel
    for target, previous, expected in (
      (0.6, -0.024, 0.0), (0.6, -0.012, 0.010), (0.6, 0.0, 0.010),
      (0.6, 0.001, 0.011), (0.004, 0.0, 0.004), (2.0, 0.599, 0.600),
    ):
      with self.subTest(target=target, previous=previous):
        self.assertEqual(slew_bm_direct_long_accel(target, previous), expected)


if __name__ == "__main__":
  unittest.main()
