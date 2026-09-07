"""Offline unit tests for Mazid Mazda ICBM (DRIVE-MVP-001 / RC1-ACC-001)."""
import os
import unittest

from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.mazda.icbm import (
  ACK_TIMEOUT_FRAMES,
  MazdaIcbmController,
  MIN_PRESS_INTERVAL_FRAMES,
  V_CRUISE_UNSET_KPH,
  persistent_cruise_target_ms,
)
from opendbc.car.mazda.values import Buttons

KPH = CV.KPH_TO_MS


def _press(icbm, frame, cruise_kph, target_kph, *, enabled=True, cruise_enabled=True):
  return icbm.update(
    frame,
    enabled=enabled,
    cruise_enabled=cruise_enabled,
    cruise_speed_ms=cruise_kph * KPH,
    target_speed_ms=persistent_cruise_target_ms(target_kph),
  )


class TestMazdaIcbm(unittest.TestCase):
  def test_no_press_when_disabled(self):
    icbm = MazdaIcbmController()
    self.assertIsNone(icbm.update(0, enabled=False, cruise_enabled=True,
                                  cruise_speed_ms=20.0, target_speed_ms=25.0))
    self.assertIsNone(icbm.update(0, enabled=True, cruise_enabled=False,
                                  cruise_speed_ms=20.0, target_speed_ms=25.0))

  def test_set_plus_and_minus(self):
    icbm = MazdaIcbmController()
    self.assertEqual(
      icbm.update(0, enabled=True, cruise_enabled=True, cruise_speed_ms=20.0, target_speed_ms=25.0),
      Buttons.SET_PLUS,
    )
    # rate limit
    self.assertIsNone(
      icbm.update(10, enabled=True, cruise_enabled=True, cruise_speed_ms=20.0, target_speed_ms=25.0),
    )
    self.assertEqual(
      icbm.update(MIN_PRESS_INTERVAL_FRAMES, enabled=True, cruise_enabled=True,
                  cruise_speed_ms=25.0, target_speed_ms=20.0),
      Buttons.SET_MINUS,
    )

  def test_deadband(self):
    icbm = MazdaIcbmController()
    self.assertIsNone(
      icbm.update(0, enabled=True, cruise_enabled=True, cruise_speed_ms=20.0, target_speed_ms=20.3),
    )

  def test_reset_requires_fresh_cooldown(self):
    icbm = MazdaIcbmController()
    self.assertEqual(
      icbm.update(0, enabled=True, cruise_enabled=True, cruise_speed_ms=20.0, target_speed_ms=25.0),
      Buttons.SET_PLUS,
    )
    icbm.reset(5)
    self.assertIsNone(
      icbm.update(5, enabled=True, cruise_enabled=True, cruise_speed_ms=20.0, target_speed_ms=25.0),
    )
    self.assertIsNone(
      icbm.update(5 + MIN_PRESS_INTERVAL_FRAMES - 1, enabled=True, cruise_enabled=True,
                  cruise_speed_ms=20.0, target_speed_ms=25.0),
    )
    self.assertEqual(
      icbm.update(5 + MIN_PRESS_INTERVAL_FRAMES, enabled=True, cruise_enabled=True,
                  cruise_speed_ms=20.0, target_speed_ms=25.0),
      Buttons.SET_PLUS,
    )

  def test_waits_for_oem_ack_before_another_press(self):
    icbm = MazdaIcbmController(require_ack=True)
    self.assertEqual(
      icbm.update(0, enabled=True, cruise_enabled=True, cruise_speed_ms=20.0, target_speed_ms=25.0),
      Buttons.SET_PLUS,
    )
    self.assertIsNone(
      icbm.update(MIN_PRESS_INTERVAL_FRAMES, enabled=True, cruise_enabled=True,
                  cruise_speed_ms=20.0, target_speed_ms=25.0),
    )
    self.assertIsNone(
      icbm.update(ACK_TIMEOUT_FRAMES, enabled=True, cruise_enabled=True,
                  cruise_speed_ms=20.0, target_speed_ms=25.0),
    )
    self.assertEqual(
      icbm.update(ACK_TIMEOUT_FRAMES + MIN_PRESS_INTERVAL_FRAMES, enabled=True, cruise_enabled=True,
                  cruise_speed_ms=20.0, target_speed_ms=25.0),
      Buttons.SET_PLUS,
    )

  def test_legacy_controller_keeps_original_interval_only_behavior(self):
    icbm = MazdaIcbmController()
    self.assertEqual(
      icbm.update(0, enabled=True, cruise_enabled=True, cruise_speed_ms=20.0, target_speed_ms=25.0),
      Buttons.SET_PLUS,
    )
    self.assertEqual(
      icbm.update(MIN_PRESS_INTERVAL_FRAMES, enabled=True, cruise_enabled=True,
                  cruise_speed_ms=20.0, target_speed_ms=25.0),
      Buttons.SET_PLUS,
    )

  def test_ack_allows_next_step_after_minimum_interval(self):
    icbm = MazdaIcbmController(require_ack=True)
    self.assertEqual(
      icbm.update(0, enabled=True, cruise_enabled=True, cruise_speed_ms=20.0, target_speed_ms=25.0),
      Buttons.SET_PLUS,
    )
    self.assertEqual(
      icbm.update(MIN_PRESS_INTERVAL_FRAMES, enabled=True, cruise_enabled=True,
                  cruise_speed_ms=20.2, target_speed_ms=25.0),
      Buttons.SET_PLUS,
    )

  def test_new_risk_reverses_pending_recovery_after_interval(self):
    icbm = MazdaIcbmController(require_ack=True)
    self.assertEqual(
      icbm.update(0, enabled=True, cruise_enabled=True, cruise_speed_ms=20.0, target_speed_ms=25.0),
      Buttons.SET_PLUS,
    )
    self.assertEqual(
      icbm.update(MIN_PRESS_INTERVAL_FRAMES, enabled=True, cruise_enabled=True,
                  cruise_speed_ms=20.0, target_speed_ms=15.0),
      Buttons.SET_MINUS,
    )

  def test_nonfinite_speed_never_emits_button(self):
    for cruise_speed, target_speed in ((float("nan"), 20.0), (20.0, float("nan")), (float("inf"), 20.0)):
      with self.subTest(cruise_speed=cruise_speed, target_speed=target_speed):
        self.assertIsNone(MazdaIcbmController().update(
          0, enabled=True, cruise_enabled=True,
          cruise_speed_ms=cruise_speed, target_speed_ms=target_speed,
        ))

  def test_persistent_target_ignores_planner_and_hud(self):
    # TEST 1: 70 set / planner 50 → ICBM target stays 70, never 50
    target = persistent_cruise_target_ms(70.0)
    planner_ms = 50.0 * KPH
    hud_ms = 50.0 * KPH
    self.assertAlmostEqual(target, 70.0 * KPH)
    self.assertNotAlmostEqual(target, planner_ms)
    self.assertNotAlmostEqual(target, hud_ms)

  def test_unset_and_invalid_target_skips(self):
    self.assertEqual(persistent_cruise_target_ms(V_CRUISE_UNSET_KPH), 0.0)
    self.assertEqual(persistent_cruise_target_ms(0.0), 0.0)
    self.assertEqual(persistent_cruise_target_ms(-1.0), 0.0)
    self.assertEqual(persistent_cruise_target_ms(float("nan")), 0.0)

  def test1_set_70_planner_50_no_set_minus_to_follow_speed(self):
    icbm = MazdaIcbmController()
    # pcmCruise: OEM set == vCruise == 70; planner/hud 50 must not be the target
    self.assertIsNone(_press(icbm, 0, cruise_kph=70.0, target_kph=70.0))
    # old bug: target=planner 50 vs OEM 70 → SET_M
    old = MazdaIcbmController()
    self.assertEqual(
      old.update(0, enabled=True, cruise_enabled=True,
                 cruise_speed_ms=70.0 * KPH, target_speed_ms=50.0 * KPH),
      Buttons.SET_MINUS,
    )

  def test2_driver_plus_not_pulled_back(self):
    # After OEM accepts 70→75, vCruise follows 75. Planner may still be ~70.
    icbm = MazdaIcbmController()
    self.assertIsNone(_press(icbm, 0, cruise_kph=75.0, target_kph=75.0))
    # old bug: chase planner/hud 70 after driver + → SET_M pullback
    old = MazdaIcbmController()
    self.assertEqual(
      old.update(0, enabled=True, cruise_enabled=True,
                 cruise_speed_ms=75.0 * KPH, target_speed_ms=70.0 * KPH),
      Buttons.SET_MINUS,
    )

  def test3_driver_minus_updates_target(self):
    icbm = MazdaIcbmController()
    # After OEM accepts 70→65, persistent target is 65; no extra press when synced
    self.assertIsNone(_press(icbm, 0, cruise_kph=65.0, target_kph=65.0))
    # If C4 persistent set already dropped and OEM lagged, SET_M is correct (driver −)
    self.assertEqual(_press(MazdaIcbmController(), 0, cruise_kph=70.0, target_kph=65.0),
                     Buttons.SET_MINUS)

  def test4_lead_slows_persistent_set_held(self):
    # Vehicle/planner 70→50, OEM set stays 70
    icbm = MazdaIcbmController()
    self.assertIsNone(_press(icbm, 0, cruise_kph=70.0, target_kph=70.0))

  def test5_lead_leaves_no_extra_buttons(self):
    # OEM ACC may accelerate back to 70; ICBM must stay idle so it does not collapse set
    icbm = MazdaIcbmController()
    self.assertIsNone(_press(icbm, 0, cruise_kph=70.0, target_kph=70.0))
    self.assertIsNone(_press(icbm, MIN_PRESS_INTERVAL_FRAMES, cruise_kph=70.0, target_kph=70.0))

  def test6_cancel_no_set_p_or_set_m(self):
    icbm = MazdaIcbmController()
    self.assertIsNone(_press(icbm, 0, 70.0, 80.0, enabled=False))
    self.assertIsNone(_press(icbm, 0, 70.0, 80.0, cruise_enabled=False))

  def test7_resume_not_emitted_by_icbm(self):
    icbm = MazdaIcbmController()
    btn = icbm.update(0, enabled=True, cruise_enabled=True,
                      cruise_speed_ms=20.0, target_speed_ms=25.0)
    self.assertEqual(btn, Buttons.SET_PLUS)
    self.assertNotEqual(btn, Buttons.RESUME)
    self.assertNotEqual(btn, Buttons.CANCEL)
    # ICBM helper never returns RESUME even when speeds match (OEM resume is CarController)
    idle = MazdaIcbmController()
    self.assertIsNone(_press(idle, 0, 70.0, 70.0))

  def test8_metric_imperial_kph_to_ms(self):
    # Internal vCruise is always kph; display unit must not change ICBM SI target
    self.assertAlmostEqual(persistent_cruise_target_ms(70.0), 70.0 * KPH)
    mph70_as_kph = 70.0 * CV.MPH_TO_KPH
    self.assertAlmostEqual(persistent_cruise_target_ms(mph70_as_kph), 70.0 * CV.MPH_TO_MS)

  def test_rc_test_01_seg14_sample_old_set_m_new_idle(self):
    # RC_TEST_01 segment 14 ~23:34:58 CST: OEM 43.1, hud/lp0 39.5
    oem, hud = 43.1, 39.5
    old = MazdaIcbmController()
    self.assertEqual(
      old.update(0, enabled=True, cruise_enabled=True,
                 cruise_speed_ms=oem * KPH, target_speed_ms=hud * KPH),
      Buttons.SET_MINUS,
    )
    new = MazdaIcbmController()
    self.assertIsNone(_press(new, 0, cruise_kph=oem, target_kph=oem))

  def test_rc_test_01_seg15_sample_old_set_m_new_idle(self):
    # RC_TEST_01 segment 15 ~23:36:28 CST: OEM 52.9, hud/lp0 42.7
    oem, hud = 52.9, 42.7
    old = MazdaIcbmController()
    self.assertEqual(
      old.update(0, enabled=True, cruise_enabled=True,
                 cruise_speed_ms=oem * KPH, target_speed_ms=hud * KPH),
      Buttons.SET_MINUS,
    )
    new = MazdaIcbmController()
    self.assertIsNone(_press(new, 0, cruise_kph=oem, target_kph=oem))


def _series_csv_paths():
  return [
    r"D:\MazidBench\vehicle\mazid_v02_first_realcar_2026-08-18\analysis\series_10hz.csv",
    "/mnt/d/MazidBench/vehicle/mazid_v02_first_realcar_2026-08-18/analysis/series_10hz.csv",
  ]


def _truthy(val: str) -> bool:
  return val.strip().lower() in ("1", "true", "yes")


def replay_segment(path: str, segment: int) -> dict:
  """Extracted-state replay of ICBM target choice (not full openpilot replay)."""
  import csv

  old_icbm = MazdaIcbmController()
  new_icbm = MazdaIcbmController()
  old_m = old_p = new_m = new_p = 0
  n = 0
  with open(path, newline="", encoding="utf-8") as f:
    for i, row in enumerate(csv.DictReader(f)):
      if int(row["seg"]) != segment:
        continue
      if not (_truthy(row["cruise_en"]) and _truthy(row["cc_enabled"])):
        continue
      try:
        cruise = float(row["cruise_speed_kph"])
        hud = float(row["hud_set_kph"])
        v_cruise = float(row["v_cruise"])
      except (TypeError, ValueError):
        continue
      n += 1
      frame = i * 10  # CarController samples ICBM at 10 Hz from 100 Hz
      old_btn = old_icbm.update(
        frame, enabled=True, cruise_enabled=True,
        cruise_speed_ms=cruise * KPH, target_speed_ms=hud * KPH,
      )
      new_btn = new_icbm.update(
        frame, enabled=True, cruise_enabled=True,
        cruise_speed_ms=cruise * KPH,
        target_speed_ms=persistent_cruise_target_ms(v_cruise),
      )
      if old_btn == Buttons.SET_MINUS:
        old_m += 1
      elif old_btn == Buttons.SET_PLUS:
        old_p += 1
      if new_btn == Buttons.SET_MINUS:
        new_m += 1
      elif new_btn == Buttons.SET_PLUS:
        new_p += 1
  return {
    "segment": segment,
    "n_acc_enabled": n,
    "BEFORE_FIX_SET_M": old_m,
    "BEFORE_FIX_SET_P": old_p,
    "AFTER_FIX_SET_M": new_m,
    "AFTER_FIX_SET_P": new_p,
  }


class TestRcTest01Replay(unittest.TestCase):
  @classmethod
  def setUpClass(cls):
    cls.csv_path = next((p for p in _series_csv_paths() if os.path.isfile(p)), None)

  def _require_csv(self):
    if not self.csv_path:
      self.skipTest("RC_TEST_01 series_10hz.csv not on this machine")

  def test_seg14_planner_chase_removed(self):
    self._require_csv()
    r = replay_segment(self.csv_path, 14)
    self.assertGreater(r["n_acc_enabled"], 0)
    self.assertGreater(r["BEFORE_FIX_SET_M"], 0, r)
    self.assertEqual(r["AFTER_FIX_SET_M"], 0, r)
    self.assertEqual(r["AFTER_FIX_SET_P"], 0, r)

  def test_seg15_planner_chase_removed(self):
    self._require_csv()
    r = replay_segment(self.csv_path, 15)
    self.assertGreater(r["n_acc_enabled"], 0)
    self.assertGreater(r["BEFORE_FIX_SET_M"], 0, r)
    self.assertEqual(r["AFTER_FIX_SET_M"], 0, r)
    self.assertEqual(r["AFTER_FIX_SET_P"], 0, r)


class TestMazdaIcbmCarController(unittest.TestCase):
  def setUp(self):
    from types import SimpleNamespace

    from opendbc.car import structs
    from opendbc.car.mazda.carcontroller import CarController
    from opendbc.car.mazda.interface import CarInterface
    from opendbc.car.mazda.values import CAR, DBC

    self.cp = CarInterface.get_non_essential_params(CAR.MAZDA_CX5_2022)
    self.controller = CarController(DBC[CAR.MAZDA_CX5_2022], self.cp)
    self.controller.frame = 10  # exercise SET cadence without unrelated HUD work
    self.cc = structs.CarControl()
    self.cc.enabled = True
    self.cc.hudControl.setSpeed = 40.0 * KPH
    self.cs = SimpleNamespace(
      out=SimpleNamespace(
        brakePressed=False, vCruise=70.0,
        cruiseState=SimpleNamespace(enabled=True, speed=70.0 * KPH),
      ),
      crz_btns_counter=0,
      cam_lkas={"BIT_1": 0, "ERR_BIT_1": 0, "ERR_BIT_2": 0},
    )

  def button_messages(self):
    _, messages = self.controller.update(self.cc.as_reader(), self.cs, 0)
    return [msg for msg in messages if msg[0] == 0x09D]

  def expected_button(self, button):
    from opendbc.car.mazda import mazdacan
    return mazdacan.create_button_cmd(self.controller.packer, self.cp, self.cs.crz_btns_counter, button)

  def test_persistent_set_matches_oem_despite_lower_hud(self):
    self.assertEqual(self.button_messages(), [])

  def test_persistent_set_drives_real_buttons_with_existing_cooldown(self):
    self.cs.out.vCruise = 75.0
    self.assertEqual(self.button_messages(), [self.expected_button(Buttons.SET_PLUS)])
    self.controller.frame = 20
    self.assertEqual(self.button_messages(), [])
    self.controller.frame = 30
    self.cs.out.vCruise = 65.0
    self.assertEqual(self.button_messages(), [self.expected_button(Buttons.SET_MINUS)])

  def test_advisory_and_invalid_persistent_set_do_not_inject_buttons(self):
    self.cs.out.vCruise = 75.0
    self.cc.oemCruiseSetSpeedAssist.enabled = True
    self.cc.oemCruiseSetSpeedAssist.targetValid = True
    self.cc.oemCruiseSetSpeedAssist.targetSpeed = 60.0 * KPH
    self.assertEqual(self.button_messages(), [])
    self.cc.oemCruiseSetSpeedAssist.enabled = False
    for invalid in (255.0, 0.0, float("nan")):
      with self.subTest(invalid=invalid):
        self.controller.frame = 10
        self.cs.out.vCruise = invalid
        self.assertEqual(self.button_messages(), [])

  def test_cancel_and_resume_never_add_set_button(self):
    self.cs.out.vCruise = 75.0
    self.cc.cruiseControl.cancel = True
    self.assertEqual(self.button_messages(), [self.expected_button(Buttons.CANCEL)])
    self.controller.frame = 30
    self.cc.cruiseControl.cancel = False
    self.cc.cruiseControl.resume = True
    self.assertEqual(self.button_messages(), [self.expected_button(Buttons.RESUME)])


class TestVisionSetDoesNotAccelerate(unittest.TestCase):
  @staticmethod
  def run_frame(*, target_valid=False, curve_warning=False, target_speed=20.0, driver_set=25.0):
    from types import SimpleNamespace

    from opendbc.car import structs
    from opendbc.car.mazda.carcontroller import CarController
    from opendbc.car.mazda.interface import CarInterface
    from opendbc.car.mazda.values import CAR, DBC

    cp = CarInterface.get_non_essential_params(CAR.MAZDA_3_2019)
    cp.openpilotLongitudinalControl = False
    controller = CarController(DBC[CAR.MAZDA_3_2019], cp)
    controller.frame = 10

    cc = structs.CarControl()
    cc.enabled = True
    cc.latActive = False
    cc.oemCruiseSetSpeedAssist.enabled = True
    cc.oemCruiseSetSpeedAssist.targetValid = target_valid
    cc.oemCruiseSetSpeedAssist.targetSpeed = target_speed
    cc.oemCruiseSetSpeedAssist.driverSetSpeed = driver_set
    cc.oemCruiseSetSpeedAssist.sourceMonoTime = 1
    cc.oemCruiseSetSpeedAssist.curveWarning = curve_warning

    cs = SimpleNamespace(
      out=SimpleNamespace(
        brakePressed=False,
        gasPressed=False,
        steeringTorque=0.0,
        vEgo=25.0,
        vCruise=90.0,
        standstill=False,
        canValid=True,
        canTimeout=False,
        gearShifter=structs.CarState.GearShifter.drive,
        cruiseState=SimpleNamespace(enabled=True, available=True, speed=80.0 * KPH),
        buttonEvents=[],
      ),
      cruise_buttons_pressed=False,
      stock_radar_has_lead=False,
      stock_radar_lead_valid=True,
      crz_btns_counter=0,
      cam_lkas={"BIT_1": 0, "ERR_BIT_1": 0, "ERR_BIT_2": 0},
    )
    _, can_sends = controller.update(cc.as_reader(), cs, 0)
    return [msg for msg in can_sends if msg[0] == 0x09D]

  def test_toggle_on_without_live_target_does_not_restore_vcruise(self):
    self.assertEqual(self.run_frame(target_valid=False), [])

  def test_curve_warning_without_live_target_does_not_restore_vcruise(self):
    self.assertEqual(self.run_frame(curve_warning=True, target_valid=False), [])

  def test_recovery_target_above_cluster_does_not_set_plus(self):
    # Coordinator restore to 90 km/h while cluster is 80 km/h would be SET+.
    self.assertEqual(self.run_frame(target_valid=True, target_speed=90.0 * KPH, driver_set=90.0 * KPH), [])

  def test_valid_lower_target_does_not_set_minus(self):
    self.assertEqual(self.run_frame(target_valid=True, target_speed=20.0, driver_set=25.0), [])


if __name__ == "__main__":
  unittest.main()
