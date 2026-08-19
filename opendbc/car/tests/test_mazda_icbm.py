"""Offline unit tests for Mazid Mazda ICBM (DRIVE-MVP-001 / RC1-ACC-001)."""
import os
import unittest

from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.mazda.icbm import (
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


if __name__ == "__main__":
  unittest.main()
