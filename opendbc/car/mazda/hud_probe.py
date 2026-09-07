"""HUD-PROBE-003: parked DISPLAY_ONLY gallery + abort-on-move.

Only 0x440 fields classified DISPLAY_ONLY_CONFIRMED. Does not touch 0x243 / 0x09D / 0x21C.
TJA / BIT1/2/3 / S1 / NO_ERR_BIT / LDW_WARN_* are not written (reserved/unknown/unproven).

LINE_SELECTED is not a separate DBC signal. Gallery G06–G08 use LANE_LINES 3/4/2
(the OEM left/right/both enum). Expect the same glass as G03–G05 if no extra highlight exists.
"""
from __future__ import annotations

import json
import os
import time
from dataclasses import dataclass

from opendbc.car import structs

GearShifter = structs.CarState.GearShifter

_GEAR_NAME_BY_RAW = {
  GearShifter.unknown: "unknown",
  GearShifter.park: "park",
  GearShifter.drive: "drive",
  GearShifter.neutral: "neutral",
  GearShifter.reverse: "reverse",
  GearShifter.sport: "sport",
  GearShifter.low: "low",
  GearShifter.brake: "brake",
  GearShifter.eco: "eco",
  GearShifter.manumatic: "manumatic",
}

PROBE_MARKER = "/data/openpilot_mazid_hud_probe_03/.mazid_hud_probe"
LABEL_PATH = "/dev/shm/mazid_hud_probe_label"
LOG_PATH = "/data/mazid_hud_probe_03.log"

# 0x440 is packed at 2 Hz (frame % 50). 10 ticks ≈ 5 s; 18 ticks ≈ 9 s.
TICKS_ITEM = 10
TICKS_MAX_SAFE = 18
VEGO_ABORT = 0.35  # m/s; ~1.3 km/h
STATIC_CONFIRM_TICKS = 2

DISPLAY_ONLY_CONFIRMED = (
  "LANE_LINES",
  "LINE_VISIBLE",
  "LINE_NOT_VISIBLE",
  "HANDS_ON_STEER_WARN",
  "HANDS_ON_STEER_WARN_2",
  "HANDS_WARN_3_BITS",
)

# DBC LANE_LINES: 1=no lines, 2=two, 3=left, 4=right.
LL_NONE = 1
LL_BOTH = 2
LL_LEFT = 3
LL_RIGHT = 4


def probe_enabled() -> bool:
  if os.environ.get("MAZID_HUD_PROBE") == "1":
    return True
  return os.path.isfile(PROBE_MARKER)


def normalize_gear(gear: object) -> str:
  """Return one stable, JSON-safe gear name for capnp and ordinary values."""
  try:
    if gear is None:
      return "none"
    if isinstance(gear, str):
      text = gear
    elif isinstance(gear, bool):
      return "true" if gear else "false"
    elif isinstance(gear, int):
      return _GEAR_NAME_BY_RAW.get(gear, f"raw:{gear}")
    else:
      raw = getattr(gear, "raw", None)
      if raw is not None:
        raw_int = int(raw)
        return _GEAR_NAME_BY_RAW.get(raw_int, f"raw:{raw_int}")

      name = getattr(gear, "name", None)
      if isinstance(name, str):
        text = name
      else:
        value = getattr(gear, "value", None)
        if isinstance(value, int) and not isinstance(value, bool):
          return _GEAR_NAME_BY_RAW.get(value, f"raw:{value}")
        text = str(gear)

    token = text.strip().lower().rsplit(".", 1)[-1]
    if token in _GEAR_NAME_BY_RAW.values():
      return token
    try:
      raw_int = int(token)
    except (TypeError, ValueError):
      return token or "empty"
    return _GEAR_NAME_BY_RAW.get(raw_int, f"raw:{raw_int}")
  except Exception:
    # Probe diagnostics must never be able to terminate CarController.
    return "unavailable"


def _safe_json_default(value: object) -> str:
  try:
    return str(value)
  except Exception:
    return "<unserializable>"


@dataclass(frozen=True)
class ProbeDisplay:
  """None means 'do not override; copy FSC / HudBridge default'."""
  copy_oem: bool = False
  lane_lines: int | None = None
  line_visible: int | None = None
  line_not_visible: int | None = None
  hands_on: bool | None = None
  hands_on_2: bool | None = None
  hands_warn_3: int | None = None


@dataclass(frozen=True)
class GalleryItem:
  gid: str
  name: str
  label_cn: str
  ticks: int
  display: ProbeDisplay
  expected_visual: str


def _vis(ll, vis, nvis, h1, h2, w3) -> ProbeDisplay:
  return ProbeDisplay(
    copy_oem=False,
    lane_lines=ll,
    line_visible=vis,
    line_not_visible=nvis,
    hands_on=h1,
    hands_on_2=h2,
    hands_warn_3=w3,
  )


# MAX_SAFE_VISUAL: compatible combo only (LINE_VISIBLE xor LINE_NOT_VISIBLE).
MAX_SAFE = _vis(LL_BOTH, 1, 0, True, True, 0b111)
MAX_SAFE_VISUAL_PAYLOAD_MAP = {
  "LANE_LINES": LL_BOTH,
  "LINE_VISIBLE": 1,
  "LINE_NOT_VISIBLE": 0,
  "HANDS_ON_STEER_WARN": 1,
  "HANDS_ON_STEER_WARN_2": 1,
  "HANDS_WARN_3_BITS": 0b111,
}

GALLERY: tuple[GalleryItem, ...] = (
  GalleryItem("HUD-G01", "MAX_SAFE_VISUAL", "最大显示组合", TICKS_MAX_SAFE, MAX_SAFE,
              "最多同时亮：双线 + 可见 + 双手提示。不含互斥 NOT_VISIBLE，不含 TJA。"),
  GalleryItem("HUD-G00", "OEM_BASELINE", "原厂原始显示", TICKS_ITEM, ProbeDisplay(copy_oem=True),
              "完全复制 FSC 0x440，C4 不叠显示。"),
  GalleryItem("HUD-G02", "NO_C4_LANES", "无线", TICKS_ITEM, _vis(LL_NONE, 0, 1, False, False, 0),
              "LANE_LINES=1，NOT_VISIBLE=1，无双手。"),
  GalleryItem("HUD-G03", "LEFT_VISIBLE", "左线", TICKS_ITEM, _vis(LL_LEFT, 1, 0, False, False, 0),
              "LANE_LINES=3 + LINE_VISIBLE。无独立左可见位。"),
  GalleryItem("HUD-G04", "RIGHT_VISIBLE", "右线", TICKS_ITEM, _vis(LL_RIGHT, 1, 0, False, False, 0),
              "LANE_LINES=4 + LINE_VISIBLE。"),
  GalleryItem("HUD-G05", "BOTH_VISIBLE", "双线可见", TICKS_ITEM, _vis(LL_BOTH, 1, 0, False, False, 0),
              "LANE_LINES=2，无双手。ACTIVE 候选。"),
  GalleryItem("HUD-G06", "LEFT_SELECTED", "左线选中", TICKS_ITEM, _vis(LL_LEFT, 1, 0, False, False, 0),
              "DBC 无 LINE_SELECTED；与 G03 同载荷。若玻璃相同则该字段不存在。"),
  GalleryItem("HUD-G07", "RIGHT_SELECTED", "右线选中", TICKS_ITEM, _vis(LL_RIGHT, 1, 0, False, False, 0),
              "与 G04 同载荷。"),
  GalleryItem("HUD-G08", "BOTH_SELECTED", "双线选中", TICKS_ITEM, _vis(LL_BOTH, 1, 0, False, False, 0),
              "与 G05 同载荷。"),
  GalleryItem("HUD-G09", "HANDS_ON_1_ONLY", "双手提示1", TICKS_ITEM, _vis(LL_NONE, 0, 1, True, False, 0b111),
              "仅 HANDS_ON_STEER_WARN。"),
  GalleryItem("HUD-G10", "HANDS_ON_2_ONLY", "双手提示2", TICKS_ITEM, _vis(LL_NONE, 0, 1, False, True, 0b111),
              "仅 HANDS_ON_STEER_WARN_2。"),
  GalleryItem("HUD-G11", "HANDS_ON_BOTH", "双手提示两组", TICKS_ITEM, _vis(LL_NONE, 0, 1, True, True, 0b111),
              "两组 HANDS_ON + WARN_3_BITS。"),
  GalleryItem("HUD-G12", "BOTH_VISIBLE_HANDS", "双线+双手", TICKS_ITEM, _vis(LL_BOTH, 1, 0, True, True, 0b111),
              "G05 + G11。"),
  GalleryItem("HUD-G13", "BOTH_SELECTED_HANDS", "双线选中+双手", TICKS_ITEM, _vis(LL_BOTH, 1, 0, True, True, 0b111),
              "与 G12 同载荷（无独立 SELECTED）。"),
  GalleryItem("HUD-G14", "STANDBY_CANDIDATE", "待命普通线", TICKS_ITEM, _vis(LL_NONE, 1, 0, False, False, 0),
              "LINE_VISIBLE=1 且 LANE_LINES=1。STANDBY/看见道路候选。"),
  GalleryItem("HUD-G15", "NO_ROAD_CANDIDATE", "未见道路", TICKS_ITEM, _vis(LL_NONE, 0, 1, False, False, 0),
              "与 G02 同载荷。NO_ROAD 候选。"),
)

GALLERY_COUNT = len(GALLERY)
GALLERY_IDS = tuple(g.gid for g in GALLERY)


def max_safe_conflicts(disp: ProbeDisplay) -> list[str]:
  bad = []
  if disp.copy_oem:
    return bad
  if disp.line_visible == 1 and disp.line_not_visible == 1:
    bad.append("LINE_VISIBLE+LINE_NOT_VISIBLE")
  if disp.lane_lines in (LL_LEFT, LL_RIGHT, LL_BOTH) and disp.line_not_visible == 1:
    bad.append("drawn_lane+LINE_NOT_VISIBLE")
  return bad


@dataclass
class ProbeTick:
  static: bool
  item: GalleryItem | None
  display: ProbeDisplay | None
  label: str
  phase: str  # STATIC | DYNAMIC | ABORT | DONE
  gid: str
  reason: str


def _fields(d: ProbeDisplay) -> dict:
  if d.copy_oem:
    return {"copy_oem": True}
  return {
    "LANE_LINES": d.lane_lines,
    "LINE_VISIBLE": d.line_visible,
    "LINE_NOT_VISIBLE": d.line_not_visible,
    "HANDS_ON_STEER_WARN": int(bool(d.hands_on)),
    "HANDS_ON_STEER_WARN_2": int(bool(d.hands_on_2)),
    "HANDS_WARN_3_BITS": d.hands_warn_3,
  }


class MazidHudProbe:
  def __init__(self, label_path: str = LABEL_PATH, log_path: str = LOG_PATH):
    self.label_path = label_path
    self.log_path = log_path
    self._i = 0
    self._hold = 0
    self._confirm = 0
    self.aborted = False
    self.finished = False
    self.started = False
    self._last_gid = ""
    self._item_t0 = 0
    self.failure_reason = ""

  def _mark_failure(self, stage: str, exc: Exception) -> None:
    if self.failure_reason:
      return
    try:
      detail = str(exc)
    except Exception:
      detail = "<unprintable>"
    self.failure_reason = f"{stage}:{type(exc).__name__}:{detail}"

  def _write_label(self, text: str) -> bool:
    try:
      d = os.path.dirname(self.label_path)
      if d:
        os.makedirs(d, exist_ok=True)
      with open(self.label_path, "w", encoding="utf-8") as f:
        f.write(text)
      return True
    except Exception as exc:
      self._mark_failure("label", exc)
      return False

  def _log(self, rec: dict) -> bool:
    try:
      record = dict(rec)
      record["wall_unix"] = time.time()
      line = json.dumps(record, ensure_ascii=False, default=_safe_json_default)
      with open(self.log_path, "a", encoding="utf-8") as f:
        f.write(line + "\n")
      return True
    except Exception as exc:
      self._mark_failure("log", exc)
      return False

  def log_tx(self, *, gid: str, payload_hex: str, v_ego: float, gear: object,
             standstill: bool, now_ns: int) -> bool:
    return self._log({
      "event": "HUD_PROBE_TX",
      "HUD_PROBE_PHASE_ID": gid,
      "payload_hex": payload_hex,
      "v_ego": v_ego,
      "gear": normalize_gear(gear),
      "standstill": standstill,
      "mono_ns": now_ns,
    })

  def _static_ok(self, *, standstill: bool, v_ego: float, gear: object,
                 steer_fault_permanent: bool) -> bool:
    if steer_fault_permanent:
      return False
    if normalize_gear(gear) in ("drive", "reverse"):
      return False
    if not standstill:
      return False
    if abs(v_ego) > VEGO_ABORT:
      return False
    return True

  def update(self, *, standstill: bool, v_ego: float, gear: object,
             steer_fault_permanent: bool, now_ns: int = 0) -> ProbeTick:
    if self.aborted or self.finished:
      self._write_label("")
      return ProbeTick(False, None, None, "", "DYNAMIC", "", "gallery_over")

    if not self._static_ok(standstill=standstill, v_ego=v_ego, gear=gear,
                           steer_fault_permanent=steer_fault_permanent):
      if self.started:
        self.aborted = True
        self._log({"event": "STATIC_GALLERY_ABORT", "gid": self._last_gid,
                   "v_ego": v_ego, "gear": normalize_gear(gear), "standstill": standstill, "mono_ns": now_ns})
        self._write_label("")
        return ProbeTick(False, None, None, "", "ABORT", self._last_gid, "move_or_gear")
      self._confirm = 0
      self._write_label("")
      return ProbeTick(False, None, None, "", "DYNAMIC", "", "not_static")

    self._confirm += 1
    if not self.started:
      if self._confirm < STATIC_CONFIRM_TICKS:
        return ProbeTick(False, None, None, "", "STATIC", "", "confirm")
      self.started = True
      self._i = 0
      self._hold = 0
      self._item_t0 = now_ns
      item0 = GALLERY[0]
      self._last_gid = ""
      self._log({"event": "HUD_PROBE_START", "gid": item0.gid, "name": item0.name, "mono_ns": now_ns})

    item = GALLERY[self._i]
    if item.gid != self._last_gid:
      if self._last_gid:
        self._log({"event": "HUD_PROBE_END", "gid": self._last_gid, "endMonoTime": now_ns})
      self._last_gid = item.gid
      self._item_t0 = now_ns
      self._log({"event": "HUD_PROBE_BEGIN", "gid": item.gid, "name": item.name,
                 "HUD_PROBE_PHASE_ID": item.gid, "HUD_PROBE_STATE_NAME": item.name,
                 "startMonoTime": now_ns, "fields": _fields(item.display),
                 "v_ego": v_ego, "gear": normalize_gear(gear), "standstill": standstill})

    total = GALLERY_COUNT
    label = f"HUD {item.gid} / {total}\n{item.label_cn}"
    self._write_label(label)

    self._hold += 1
    if self._hold >= item.ticks:
      self._hold = 0
      if self._i + 1 >= GALLERY_COUNT:
        self.finished = True
        self._log({"event": "HUD_PROBE_END", "gid": item.gid, "endMonoTime": now_ns})
        self._log({"event": "HUD_PROBE_DONE", "mono_ns": now_ns})
        self._write_label("")
        return ProbeTick(False, item, item.display, "", "DONE", item.gid, "gallery_complete")
      self._i += 1

    return ProbeTick(True, item, item.display, label, "STATIC", item.gid, "gallery")


assert not max_safe_conflicts(MAX_SAFE)
assert GALLERY[0].gid == "HUD-G01"
