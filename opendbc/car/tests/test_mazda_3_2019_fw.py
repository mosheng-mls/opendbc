"""Offline FW fingerprint regression for 2019 China Mazda3 BM (MAZDA19-010)."""
import json
import unittest
from pathlib import Path

from opendbc.car.structs import CarParams
from opendbc.car.fw_versions import match_fw_to_car, match_fw_to_car_exact, match_fw_to_car_fuzzy, build_fw_dict
from opendbc.car.mazda.values import CAR, DBC
from opendbc.car import Bus
from opendbc.car.fingerprints import FW_VERSIONS
from opendbc.car.mazda.interface import CarInterface
from opendbc.can.dbc import DBC as DbcFile

CarFw = CarParams.CarFw
Ecu = CarParams.Ecu

# Authoritative live bytes (24) from route 00000006--2f926f3978
FW = {
  "engine": bytes.fromhex("505344432d3138384b322d42000000000000000000000000"),
  "transmission": bytes.fromhex("505344442d32315053312d44000000000000000000000000"),
  "eps": bytes.fromhex("4b4253542d33323130582d412d3030000000000000000000"),
  "abs": bytes.fromhex("424b44322d34333741532d302d3032000000000000000000"),
  "fwdRadar": bytes.fromhex("4b3133312d3637584b322d46000000000000000000000000"),
  "fwdCamera": bytes.fromhex("475348372d3637584b322d53000000000000000000000000"),
}

ADDR = {
  "engine": 0x7e0,
  "transmission": 0x7e1,
  "eps": 0x730,
  "abs": 0x760,
  "fwdRadar": 0x764,
  "fwdCamera": 0x706,
}

EVIDENCE = Path(r"C:\Users\Administrator\MazidBench\vehicle\vendor_real_drive_2026-08-18\analysis\mazda19_001_fw_extract.json")

CARSTATE_MSGS = [
  "WHEEL_SPEEDS", "ENGINE_DATA", "GEAR", "BLINK_INFO", "BSM", "STEER",
  "STEER_TORQUE", "STEER_RATE", "PEDALS", "SEATBELT", "DOORS",
  "CRZ_CTRL", "CRZ_EVENTS", "CRZ_BTNS", "CAM_LANEINFO", "CAM_LKAS",
]


def mazda_fw(ecu_name: str, fw: bytes, brand: str = "mazda", logging: bool = False) -> CarFw:
  return CarFw(
    ecu=getattr(Ecu, ecu_name),
    fwVersion=fw,
    brand=brand,
    address=ADDR[ecu_name],
    subAddress=0,
    logging=logging,
  )


def six_authoritative() -> list[CarFw]:
  return [mazda_fw(k, FW[k]) for k in FW]


def subset(keys: list[str]) -> list[CarFw]:
  return [mazda_fw(k, FW[k]) for k in keys]


def candidates(car_fw: list[CarFw], *, exact: bool = True, fuzzy: bool = True) -> tuple[bool, set[str]]:
  return match_fw_to_car(car_fw, "", allow_exact=exact, allow_fuzzy=fuzzy, log=False)


class TestMazda32019Fw(unittest.TestCase):
  def test_platform_and_dbc_mapping(self):
    self.assertIn(CAR.MAZDA_3_2019, CAR)
    self.assertEqual(DBC[CAR.MAZDA_3_2019][Bus.pt], "mazda_3_2019_bm")
    self.assertIn(CAR.MAZDA_3_2019, FW_VERSIONS)
    self.assertEqual(len(FW_VERSIONS[CAR.MAZDA_3_2019]), 6)

  def test_control_capability_driving_mvp(self):
    # DRIVE-MVP-001 product targets (Safety model unchanged)
    CP = CarInterface.get_non_essential_params(CAR.MAZDA_3_2019)
    self.assertFalse(CP.dashcamOnly)
    self.assertEqual(CP.minSteerSpeed, 0.0)
    self.assertEqual(CP.safetyConfigs[0].safetyModel, CarParams.SafetyModel.mazda)

  def test_real6_exact(self):
    exact, matches = candidates(six_authoritative())
    self.assertTrue(exact)
    self.assertEqual(matches, {CAR.MAZDA_3_2019})

  def test_real14_with_junk(self):
    self.assertTrue(EVIDENCE.is_file(), "missing MazidBench FW extract")
    data = json.loads(EVIDENCE.read_text(encoding="utf-8"))
    car_fw = []
    for row in data["params_cp"]["carFw"]:
      car_fw.append(CarFw(
        ecu=getattr(Ecu, row["ecu"]),
        fwVersion=bytes.fromhex(row["fw_hex"]),
        brand=row["brand"],
        address=row["address"],
        subAddress=row.get("subAddress", 0) or 0,
        logging=bool(row.get("logging", False)),
      ))
    self.assertEqual(len(car_fw), 14)
    exact, matches = candidates(car_fw)
    self.assertTrue(exact)
    self.assertEqual(matches, {CAR.MAZDA_3_2019})

  def test_scenarios(self):
    # Expectations follow live match_fw_to_car (exact then fuzzy).
    cases = {
      "ALL_SIX": (list(FW), {CAR.MAZDA_3_2019}),
      "PCM_MISSING": ([k for k in FW if k != "engine"], {CAR.MAZDA_3_2019}),  # fuzzy ABS+TCM
      "TCM_MISSING": ([k for k in FW if k != "transmission"], {CAR.MAZDA_3_2019}),  # exact
      "ABS_MISSING": ([k for k in FW if k != "abs"], {CAR.MAZDA_3_2019}),  # fuzzy PCM+TCM
      "SHARED_ONLY": (["eps", "fwdRadar", "fwdCamera"], set()),
      "PCM_ONLY": (["engine"], set()),  # one unique ECU cannot fuzzy; exact missing essentials
      "PCM_TCM": (["engine", "transmission"], {CAR.MAZDA_3_2019}),  # fuzzy
      "PCM_TCM_ABS": (["engine", "transmission", "abs"], {CAR.MAZDA_3_2019}),  # fuzzy/exact-ish
    }

    for name, (keys, expected) in cases.items():
      with self.subTest(name=name):
        _, matches = candidates(subset(keys))
        matches = {str(m) for m in matches}
        expected_s = {str(e) for e in expected}
        self.assertEqual(matches, expected_s, f"{name}: matches={matches}")

  def test_extra_ecu_0x720(self):
    fw = six_authoritative()
    fw.append(CarFw(ecu=Ecu.unknown, fwVersion=b"\x00", brand="mazda", address=0x720, subAddress=0))
    exact, matches = candidates(fw)
    self.assertEqual(matches, {CAR.MAZDA_3_2019})

  def test_shared_only_no_false_positive(self):
    exact, matches = candidates(subset(["eps", "fwdRadar", "fwdCamera"]))
    self.assertEqual(matches, set())

  def test_existing_platform_regression(self):
    # One FW row per ECU from each existing platform should not become MAZDA_3_2019
    for platform in (CAR.MAZDA_3, CAR.MAZDA_CX5_2022, CAR.MAZDA_CX9_2021):
      with self.subTest(platform=str(platform)):
        fw = []
        for (ecu, addr, sub), versions in FW_VERSIONS[platform].items():
          fw.append(CarFw(ecu=ecu, fwVersion=versions[0], brand="mazda",
                          address=addr, subAddress=0 if sub is None else sub))
        exact, matches = candidates(fw)
        self.assertNotIn(CAR.MAZDA_3_2019, matches)
        self.assertIn(platform, matches)

  def test_uniqueness(self):
    for label, fw in (("PCM", FW["engine"]), ("ABS", FW["abs"]), ("TCM", FW["transmission"])):
      hits = []
      for car, table in FW_VERSIONS.items():
        for versions in table.values():
          if fw in versions:
            hits.append(str(car))
      with self.subTest(label=label):
        self.assertEqual(hits, [str(CAR.MAZDA_3_2019)], f"{label} hits={hits}")

  def test_carstate_static_dbc(self):
    dbc_name = DBC[CAR.MAZDA_3_2019][Bus.pt]
    dbc = DbcFile(dbc_name)
    missing = [m for m in CARSTATE_MSGS if m not in dbc.name_to_msg]
    self.assertEqual(missing, [], f"missing msgs: {missing}")
    # Controller TX/RX message names used by stock Mazda CarController
    for msg in ("CAM_LKAS", "CAM_LANEINFO", "CRZ_BTNS"):
      self.assertIn(msg, dbc.name_to_msg)

  def test_scenario_exact_vs_fuzzy(self):
    # Record exact vs fuzzy source for handoff (live matcher semantics).
    cases = {
      "ALL_SIX": list(FW),
      "PCM_MISSING": [k for k in FW if k != "engine"],
      "TCM_MISSING": [k for k in FW if k != "transmission"],
      "ABS_MISSING": [k for k in FW if k != "abs"],
      "SHARED_ONLY": ["eps", "fwdRadar", "fwdCamera"],
      "PCM_ONLY": ["engine"],
      "PCM_TCM": ["engine", "transmission"],
      "PCM_TCM_ABS": ["engine", "transmission", "abs"],
    }
    for name, keys in cases.items():
      with self.subTest(name=name):
        exact_flag, matches = candidates(subset(keys))
        # Shared-only / PCM-only must not match; others match MAZDA_3_2019
        if name in ("SHARED_ONLY", "PCM_ONLY"):
          self.assertEqual(matches, set())
        else:
          self.assertEqual(matches, {CAR.MAZDA_3_2019})
          if name in ("ALL_SIX", "TCM_MISSING"):
            self.assertTrue(exact_flag)
          elif name in ("PCM_MISSING", "ABS_MISSING", "PCM_TCM", "PCM_TCM_ABS"):
            # Fuzzy path when essential ECU(s) missing from live set
            self.assertFalse(exact_flag)


if __name__ == "__main__":
  unittest.main()
