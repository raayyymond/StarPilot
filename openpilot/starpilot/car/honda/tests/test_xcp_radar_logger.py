"""Tests for the read-only Honda Bosch-A XCP Array B 20 Hz diagnostic logger."""
from __future__ import annotations

from types import SimpleNamespace

from opendbc.car.honda.values import CAR, HondaFlags
import openpilot.starpilot.car.honda.xcp_radar_logger as xcp_mod
from openpilot.starpilot.car.honda.xcp_radar_logger import (
  NUM_SLOTS,
  REC0,
  STRIDE,
  XCP_BUS,
  XCP_DTO,
  XcpRadarLogger,
  decode_flags,
  key_for_seed,
)


def _write_slot(ram: bytearray, slot: int, cfg: dict) -> None:
  base = slot * STRIDE
  ram[base + 2 : base + 4] = int(cfg["age"]).to_bytes(2, "big")
  ram[base + 4 : base + 6] = (0x2000).to_bytes(2, "big")
  ram[base + 6 : base + 8] = int(cfg["sub"]).to_bytes(2, "big")
  ram[base + 8] = slot
  ram[base + 16 : base + 18] = int(round(cfg["x"] * 128)).to_bytes(2, "big", signed=True)
  ram[base + 18 : base + 20] = int(round(cfg["vx"] * 256)).to_bytes(2, "big", signed=True)
  ram[base + 20 : base + 22] = int(round(cfg["ax"] * 2048)).to_bytes(2, "big", signed=True)
  ram[base + 22 : base + 24] = int(round(cfg["y"] * 128)).to_bytes(2, "big", signed=True)
  ram[base + 24 : base + 26] = (0x0010).to_bytes(2, "big")
  ram[base + 26 : base + 28] = int(round(cfg["vy"] * 256)).to_bytes(2, "big", signed=True)
  ram[base + 106 : base + 108] = int(cfg["flags"]).to_bytes(2, "big")
  ram[base + 127] = cfg["qual"]
  ram[base + 128] = cfg["cls"]


def _capture_events(monkeypatch) -> list[dict]:
  rows: list[dict] = []

  def _fake_event(name: str, **kwargs):
    if name == "xcp_array_b_20hz":
      rows.append(kwargs)

  monkeypatch.setattr(xcp_mod.cloudlog, "event", _fake_event)
  return rows


def test_decode_flags_and_seed_key():
  assert key_for_seed(b"\x12\x34\x56\x78") != bytes(4)
  assert decode_flags(0xF040, 0x0400, 20) == {
    "stage": "measured",
    "measured": True,
    "conf_band": "low",
    "ghost": False,
    "in_path": True,
    "trust_accel": False,
  }
  assert decode_flags(0xE0C0, 0x0400, 20) == {
    "stage": "measured",
    "measured": True,
    "conf_band": "med",
    "ghost": False,
    "in_path": True,
    "trust_accel": True,
  }
  assert decode_flags(0xE840, 0x0400, 10)["conf_band"] == "high"
  assert decode_flags(0xA0C0, 0x0400, 20)["stage"] == "coasting"
  assert decode_flags(0xA0C0, 0x0400, 20)["trust_accel"] is False
  assert decode_flags(0x6080, 0x0000, 0)["stage"] == "birth"
  assert decode_flags(0xE0E0, 0x0400, 20)["ghost"] is True
  assert decode_flags(0xE0C0, 0x0400, 4)["trust_accel"] is False


def test_non_honda_bosch_disabled():
  cp = SimpleNamespace(brand="honda", flags=int(HondaFlags.BOSCH | HondaFlags.BOSCH_RADARLESS), passive=False)
  logger = XcpRadarLogger(cp)
  assert not logger.enabled
  assert logger.next_can_sends(1_000_000_000) == []


def test_20hz_scheduler_all_slots_matching_and_track_id(monkeypatch):
  rows = _capture_events(monkeypatch)
  ram = bytearray(NUM_SLOTS * STRIDE)
  active_defs = {
    2: {"x": 18.0, "vx": -0.5, "ax": -1.2, "y": 0.1, "vy": 0.0, "flags": 0xE0C0, "sub": 0x0400, "age": 40, "qual": 25, "cls": 82},
    7: {"x": 42.0, "vx": 1.0, "ax": 0.1, "y": 0.3, "vy": 0.0, "flags": 0xE840, "sub": 0x0400, "age": 80, "qual": 30, "cls": 88},
    13: {"x": 25.0, "vx": -0.2, "ax": 0.0, "y": -3.6, "vy": 0.0, "flags": 0xE0C0, "sub": 0x0000, "age": 15, "qual": 22, "cls": 55},
    19: {"x": 60.0, "vx": 2.0, "ax": 0.0, "y": 3.5, "vy": 0.0, "flags": 0xF040, "sub": 0x0000, "age": 12, "qual": 10, "cls": 25},
    22: {"x": 31.0, "vx": -1.5, "ax": -0.5, "y": 3.7, "vy": 0.1, "flags": 0xE0C0, "sub": 0x0000, "age": 60, "qual": 28, "cls": 75},
    31: {"x": 11.0, "vx": -3.0, "ax": -2.0, "y": -4.0, "vy": 0.2, "flags": 0xE0C0, "sub": 0x0000, "age": 50, "qual": 24, "cls": 79},
    34: {"x": 75.0, "vx": 0.5, "ax": 0.0, "y": -7.0, "vy": 0.0, "flags": 0xE0C0, "sub": 0x0000, "age": 10, "qual": 18, "cls": 60},
    38: {"x": 52.0, "vx": -0.8, "ax": -0.3, "y": 7.2, "vy": -0.1, "flags": 0xA0C0, "sub": 0x0000, "age": 35, "qual": 26, "cls": 77},
  }
  for s, cfg in active_defs.items():
    _write_slot(ram, s, cfg)

  cp = SimpleNamespace(brand="honda", flags=int(HondaFlags.BOSCH), passive=False)
  logger = XcpRadarLogger(cp)
  assert logger.enabled

  t = 1_000_000_000
  assert logger.next_can_sends(t) == []
  t = logger._tx_armed_ns + 600_000_000

  for _ in range(4):
    sends = logger.next_can_sends(t)
    if sends:
      pkt = bytes(sends[0].dat)
      pid = pkt[0]
      if pid == 0xFF:
        rep = b"\xFF\x05\x01\x08\x00\x08\x01\x01"
      elif pid == 0xFD:
        rep = b"\xFF\x00\x05\x00\x00\x00"
      elif pid == 0xF8:
        rep = b"\xFF\x00\x12\x34\x56\x78"
      elif pid == 0xF7:
        assert pkt[2:6] == key_for_seed(b"\x12\x34\x56\x78")
        rep = b"\xFF\x01"
      else:
        raise AssertionError(f"Unexpected pid {pid:#x}")
      logger.observe_can([(t, [(XCP_DTO, rep, XCP_BUS)])], t + 9_000_000, v_ego=15.0)
    t += 10_000_000

  assert logger.state == "READY"
  rd = SimpleNamespace(points=[
    SimpleNamespace(trackId=17, dRel=15.3, yRel=0.1, vRel=-0.5),
    SimpleNamespace(trackId=29, dRel=22.4, yRel=-3.5, vRel=-0.2),
  ])
  logger.observe_radar_tracks(rd, t)

  probed_slots: set[int] = set()
  lead_kin_reads = 0
  for i in range(400):
    if i == 250:
      active_defs[31]["age"] = 1
      active_defs[31]["x"] = 14.0
      _write_slot(ram, 31, active_defs[31])
    if i % 2 == 0:
      logger.observe_radar_tracks(rd, t)
    sends = logger.next_can_sends(t)
    if sends:
      pkt = bytes(sends[0].dat)
      sz = pkt[1]
      addr = int.from_bytes(pkt[4:8], "big")
      off = addr - REC0
      s = off // STRIDE
      probed_slots.add(s)
      if s == 2 and (off % STRIDE) == 16:
        lead_kin_reads += 1
      rep = b"\xFF" + bytes(ram[off : off + sz])
      logger.observe_can([(t, [(XCP_DTO, rep, XCP_BUS)])], t + 9_000_000, v_ego=15.0)
    t += 10_000_000

  logger.close()

  assert probed_slots == set(range(NUM_SLOTS))
  assert lead_kin_reads >= 65
  assert 75 <= len(rows) <= 85
  assert all(r["debug"] is True for r in rows)
  by_slot = {tgt["slot"]: tgt for tgt in rows[-1]["targets"]}
  assert set(by_slot.keys()) == set(active_defs.keys())
  assert by_slot[2]["op_track_id"] == 17
  assert by_slot[13]["op_track_id"] == 29
  assert rows[-1]["lead"]["slot"] == 2
  assert len({tgt["radar_track_id"] for r in rows for tgt in r["targets"] if tgt["slot"] == 31}) >= 2


def _unlock_logger(logger: XcpRadarLogger, t_start: int, v_ego: float = 15.0) -> int:
  t = t_start
  logger.next_can_sends(t)
  t = logger._tx_armed_ns + 600_000_000
  for _ in range(4):
    sends = logger.next_can_sends(t)
    if sends:
      pkt = bytes(sends[0].dat)
      pid = pkt[0]
      if pid == 0xFF:
        rep = b"\xFF\x05\x01\x08\x00\x08\x01\x01"
      elif pid == 0xFD:
        rep = b"\xFF\x00\x05\x00\x00\x00"
      elif pid == 0xF8:
        rep = b"\xFF\x00\x12\x34\x56\x78"
      elif pid == 0xF7:
        rep = b"\xFF\x01"
      else:
        raise AssertionError(f"Unexpected pid {pid:#x}")
      logger.observe_can([(t, [(XCP_DTO, rep, XCP_BUS)])], t + 19_000_000, v_ego=v_ego)
    t += 20_000_000
  assert logger.state == "READY"
  return t


def test_stationary_clutter_does_not_starve_primary_lead_at_20ms_rtt(monkeypatch):
  rows = _capture_events(monkeypatch)
  ram = bytearray(NUM_SLOTS * STRIDE)
  # Slot 5 is the real moving lead; slot 3 is closer stationary in-path clutter; slots 10..29 are roadside stationary clutter
  _write_slot(ram, 5, {"x": 28.0, "vx": -1.0, "ax": -1.5, "y": 0.2, "vy": 0.0, "flags": 0xE0C0, "sub": 0x0400, "age": 40, "qual": 25, "cls": 82})
  _write_slot(ram, 3, {"x": 12.0, "vx": -15.0, "ax": 0.0, "y": 0.1, "vy": 0.0, "flags": 0xE0C0, "sub": 0x0400, "age": 12, "qual": 20, "cls": 60})
  for s in range(10, 30):
    _write_slot(ram, s, {"x": 15.0 + s, "vx": -15.0, "ax": 0.0, "y": -4.0, "vy": 0.0, "flags": 0xE0C0, "sub": 0x0000, "age": 10, "qual": 15, "cls": 50})

  cp = SimpleNamespace(brand="honda", flags=int(HondaFlags.BOSCH), passive=False)
  logger = XcpRadarLogger(cp)
  t = _unlock_logger(logger, 1_000_000_000, v_ego=15.0)

  # 5 seconds at 20 ms RTT (50 reads/sec)
  lead_kin_times: list[int] = []
  for i in range(250):
    if i >= 25:
      # Once slot 5 is discovered, publish openpilot track shifted by -2.9 m
      logger.observe_radar_tracks(SimpleNamespace(points=[SimpleNamespace(trackId=91, dRel=25.1, yRel=0.2, vRel=-1.0)]), t)
    sends = logger.next_can_sends(t)
    if sends:
      pkt = bytes(sends[0].dat)
      sz = pkt[1]
      off = int.from_bytes(pkt[4:8], "big") - REC0
      s = off // STRIDE
      if s == 5 and (off % STRIDE) == 16:
        lead_kin_times.append(t + 19_000_000)
      rep = b"\xFF" + bytes(ram[off : off + sz])
      logger.observe_can([(t, [(XCP_DTO, rep, XCP_BUS)])], t + 19_000_000, v_ego=15.0)
    t += 20_000_000

  logger.close()
  assert len(lead_kin_times) >= 60
  # Check post-warmup gaps between primary lead kinematics reads stay bounded (< 90 ms) despite 21 clutter slots
  post_warmup_gaps_ms = [(b - a) / 1e6 for a, b in zip(lead_kin_times[5:-1], lead_kin_times[6:], strict=True)]
  assert max(post_warmup_gaps_ms) <= 90.0

  matched_rows = [r for r in rows if r["lead"] is not None and r["lead"]["slot"] == 5]
  assert len(matched_rows) >= 50
  assert all(r["lead"]["op_track_id"] == 91 for r in matched_rows[-20:])


def test_offset_matching_and_moving_lead_fallback(monkeypatch):
  rows = _capture_events(monkeypatch)
  ram = bytearray(NUM_SLOTS * STRIDE)
  # Closer stationary in-path clutter (slot 1) vs distant highway moving lead (slot 4 at x=106.0, op dRel=100.0)
  _write_slot(ram, 1, {"x": 8.0, "vx": -15.0, "ax": 0.0, "y": 0.0, "vy": 0.0, "flags": 0xE0C0, "sub": 0x0400, "age": 20, "qual": 20, "cls": 60})
  _write_slot(ram, 4, {"x": 106.0, "vx": -1.0, "ax": -0.5, "y": -0.3, "vy": 0.0, "flags": 0xE0C0, "sub": 0x0400, "age": 30, "qual": 25, "cls": 85})

  cp = SimpleNamespace(brand="honda", flags=int(HondaFlags.BOSCH), passive=False)
  logger = XcpRadarLogger(cp)
  t = _unlock_logger(logger, 1_000_000_000, v_ego=15.0)

  # First 2.0s without op_tracks (verifies moving-lead fallback skips closer stationary clutter in slot 1)
  for i in range(150):
    if i >= 100:
      logger.observe_radar_tracks(SimpleNamespace(points=[SimpleNamespace(trackId=55, dRel=100.0, yRel=-0.3, vRel=-1.0)]), t)
    sends = logger.next_can_sends(t)
    if sends:
      pkt = bytes(sends[0].dat)
      sz = pkt[1]
      off = int.from_bytes(pkt[4:8], "big") - REC0
      rep = b"\xFF" + bytes(ram[off : off + sz])
      logger.observe_can([(t, [(XCP_DTO, rep, XCP_BUS)])], t + 19_000_000, v_ego=15.0)
    t += 20_000_000

  logger.close()
  # Once both slot 1 and slot 4 have flags, lead should pick slot 4 (both via moving fallback and via op_tracks range-proportional tolerance)
  rows_both = [r for r in rows if {tgt["slot"] for tgt in r["targets"]} >= {1, 4}]
  assert rows_both
  assert all(r["lead"]["slot"] == 4 for r in rows_both)
  assert rows_both[-1]["lead"]["op_track_id"] == 55
  assert rows_both[-1]["lead"]["op_dRel"] == 100.0
  assert rows_both[-1]["lead"]["op_match_dist_m"] <= 6.1


def test_tolerance_stationary_flat_and_moving_far_range():
  cp = SimpleNamespace(brand="honda", flags=int(HondaFlags.BOSCH), passive=False)
  logger = XcpRadarLogger(cp)
  targets = [
    {"slot": 1, "dRel_m": 60.0, "yRel_m": 3.0, "vRel_mps": -15.0, "op_track_id": None, "op_dRel": None},
    {"slot": 2, "dRel_m": 58.0, "yRel_m": -3.0, "vRel_mps": -14.8, "op_track_id": None, "op_dRel": None},
    {"slot": 3, "dRel_m": 92.0, "yRel_m": 0.1, "vRel_mps": -1.0, "op_track_id": None, "op_dRel": None},
  ]
  op_tracks = [
    {"tid": 10, "dRel": 54.0, "yRel": 3.0, "vRel": -15.0},
    {"tid": 11, "dRel": 54.0, "yRel": -3.0, "vRel": -14.8},
    {"tid": 12, "dRel": 78.0, "yRel": 0.1, "vRel": -1.0},
  ]
  matched_slots, op_lead_slot = logger._match_openpilot_tracks(targets, op_tracks, v_ego=15.0)
  assert targets[0]["op_track_id"] is None
  assert targets[1]["op_track_id"] == 11
  assert targets[1]["op_dRel"] == 54.0
  assert targets[2]["op_track_id"] == 12
  assert targets[2]["op_dRel"] == 78.0
  assert op_lead_slot == 3
  assert matched_slots == [3, 2]


def test_ui_shm_snapshot_filters_roadside_stationary_clutter_and_unlinks(tmp_path):
  import json
  shm_path = tmp_path / "xcp_radar_points.json"
  cp = SimpleNamespace(brand="honda", flags=int(HondaFlags.BOSCH), passive=False)
  logger = XcpRadarLogger(cp, shm_path=shm_path)
  targets = [
    {"dRel_m": 20.0, "yRel_m": 3.5, "vRel_mps": -15.0, "vyRel_mps": 0.0, "age": 10, "ghost": False, "lat_updated_ns": 5_000_000_000, "kin_updated_ns": 5_000_000_000},   # stationary off-path clutter -> excluded
    {"dRel_m": 15.0, "yRel_m": 0.5, "vRel_mps": -15.0, "vyRel_mps": 0.0, "age": 10, "ghost": False, "lat_updated_ns": 5_000_000_000, "kin_updated_ns": 5_000_000_000},   # stationary in-path -> included
    {"dRel_m": 45.0, "yRel_m": -3.2, "vRel_mps": -2.0, "vyRel_mps": 1.0, "age": 10, "ghost": False, "lat_updated_ns": 4_900_000_000, "kin_updated_ns": 4_950_000_000},   # moving + time-aligned -> (44.9, -3.1)
    {"dRel_m": 30.0, "yRel_m": 0.0, "vRel_mps": 0.0, "vyRel_mps": 0.0, "age": 10, "ghost": False, "lat_updated_ns": 0, "kin_updated_ns": 5_000_000_000},               # uninitialized Y -> excluded
    {"dRel_m": 35.0, "yRel_m": -3.0, "vRel_mps": 0.0, "vyRel_mps": 0.0, "age": 10, "ghost": True, "lat_updated_ns": 5_000_000_000, "kin_updated_ns": 5_000_000_000},     # ghost -> excluded
    {"dRel_m": 40.0, "yRel_m": -3.0, "vRel_mps": 0.0, "vyRel_mps": 0.0, "age": 2, "ghost": False, "lat_updated_ns": 5_000_000_000, "kin_updated_ns": 5_000_000_000},     # newborn age<3 -> excluded
    {"dRel_m": -4.0, "yRel_m": 0.0, "vRel_mps": 0.0, "vyRel_mps": 0.0, "age": 10, "ghost": False, "lat_updated_ns": 5_000_000_000, "kin_updated_ns": 5_000_000_000},      # behind bumper -> excluded
  ]
  logger._write_ui_snapshot(5_000_000_000, 15.0, targets)
  data = json.loads(shm_path.read_text())
  assert data == {"mono_ns": 5_000_000_000, "points": [[15.0, 0.5], [44.9, -3.1]]}
  logger.close()
  assert not shm_path.exists()


def test_cutin_lateral_y_vy_prioritized_over_qual_on_adjacent_cars(monkeypatch):
  _capture_events(monkeypatch)
  ram = bytearray(NUM_SLOTS * STRIDE)
  # Lead (slot 2), secondary adjacent car (slot 6), and third moving adjacent car (slot 9), plus 10 stationary clutter slots
  _write_slot(ram, 2, {"x": 25.0, "vx": -0.5, "ax": 0.0, "y": 0.1, "vy": 0.0, "flags": 0xE0C0, "sub": 0x0400, "age": 40, "qual": 25, "cls": 82})
  _write_slot(ram, 6, {"x": 20.0, "vx": 1.0, "ax": 0.0, "y": -3.1, "vy": 0.4, "flags": 0xE0C0, "sub": 0x0000, "age": 20, "qual": 22, "cls": 75})
  _write_slot(ram, 9, {"x": 38.0, "vx": 0.8, "ax": 0.0, "y": 3.2, "vy": -0.3, "flags": 0xE0C0, "sub": 0x0000, "age": 18, "qual": 21, "cls": 74})
  for s in range(15, 25):
    _write_slot(ram, s, {"x": 15.0 + s, "vx": -15.0, "ax": 0.0, "y": -5.0, "vy": 0.0, "flags": 0xE0C0, "sub": 0x0000, "age": 10, "qual": 15, "cls": 50})

  cp = SimpleNamespace(brand="honda", flags=int(HondaFlags.BOSCH), passive=False)
  logger = XcpRadarLogger(cp)
  t = _unlock_logger(logger, 1_000_000_000, v_ego=15.0)
  counts = {s: {16: 0, 22: 0, 127: 0} for s in (2, 6, 9)}
  rd = SimpleNamespace(points=[
    SimpleNamespace(trackId=1, dRel=25.0, yRel=0.1, vRel=-0.5),
    SimpleNamespace(trackId=2, dRel=20.0, yRel=-3.1, vRel=1.0),
  ])
  # 5 seconds at 20 ms RTT (50 reads/sec)
  for i in range(250):
    if i >= 20:
      logger.observe_radar_tracks(rd, t)
    sends = logger.next_can_sends(t)
    if sends:
      pkt = bytes(sends[0].dat)
      sz = pkt[1]
      off = int.from_bytes(pkt[4:8], "big") - REC0
      s = off // STRIDE
      fld = off % STRIDE
      if i >= 50 and s in counts and fld in counts[s]:
        counts[s][fld] += 1
      rep = b"\xFF" + bytes(ram[off : off + sz])
      logger.observe_can([(t, [(XCP_DTO, rep, XCP_BUS)])], t + 19_000_000, v_ego=15.0)
    t += 20_000_000
  logger.close()

  # Over 4s post-warmup: lead keeps ~25 Hz kin (16) and ~5 Hz lat (22); both adjacent moving cars get >=2.5 Hz (22) and more (22) than (127)
  assert counts[2][16] >= 80
  assert counts[2][22] >= 16
  assert counts[6][22] > counts[6][127] and counts[6][22] >= 10
  assert counts[9][22] > counts[9][127] and counts[9][22] >= 10 and counts[9][16] >= 8


def test_standing_traffic_and_adjacent_lat_not_starved_by_wide_traffic(monkeypatch, tmp_path):
  import json
  _capture_events(monkeypatch)
  ram = bytearray(NUM_SLOTS * STRIDE)
  # Lead (slot 2), adjacent moving lanes (slots 5, 8, 11), standing car (slot 12, born age=1), wide oncoming traffic |y|>6m (slots 14..21)
  _write_slot(ram, 2, {"x": 45.0, "vx": -1.0, "ax": 0.0, "y": 0.2, "vy": 0.0, "flags": 0xE0C0, "sub": 0x0400, "age": 40, "qual": 25, "cls": 82})
  for idx, s in enumerate((5, 8, 11)):
    _write_slot(ram, s, {"x": 30.0 + idx * 8.0, "vx": 0.5, "ax": 0.0, "y": -3.2 if idx % 2 == 0 else 3.4, "vy": 0.0,
                         "flags": 0xE0C0, "sub": 0x0000, "age": 25, "qual": 22, "cls": 75})
  _write_slot(ram, 12, {"x": 65.0, "vx": -15.0, "ax": 0.0, "y": 0.3, "vy": 0.0, "flags": 0xE0C0, "sub": 0x0400, "age": 1, "qual": 24, "cls": 80})
  for idx, s in enumerate(range(14, 22)):
    _write_slot(ram, s, {"x": 55.0 + idx * 6.0, "vx": -30.0, "ax": 0.0, "y": 8.0 if idx % 2 == 0 else -8.5, "vy": 0.0,
                         "flags": 0xE0C0, "sub": 0x0000, "age": 20, "qual": 20, "cls": 70})

  shm_path = tmp_path / "xcp_radar_points.json"
  cp = SimpleNamespace(brand="honda", flags=int(HondaFlags.BOSCH), passive=False)
  logger = XcpRadarLogger(cp, shm_path=shm_path)
  t = _unlock_logger(logger, 1_000_000_000, v_ego=15.0)
  rd = SimpleNamespace(points=[SimpleNamespace(trackId=1, dRel=45.0, yRel=0.2, vRel=-1.0)])

  max_adj_lat_age_s = 0.0
  standing_visible_at = None
  for i in range(400):  # 8.0 s at 20 ms RTT
    if i == 15:
      _write_slot(ram, 12, {"x": 62.0, "vx": -15.0, "ax": 0.0, "y": 0.3, "vy": 0.0, "flags": 0xE0C0, "sub": 0x0400, "age": 5, "qual": 24, "cls": 80})
    if i >= 10:
      logger.observe_radar_tracks(rd, t)
    sends = logger.next_can_sends(t)
    if sends:
      pkt = bytes(sends[0].dat)
      sz = pkt[1]
      off = int.from_bytes(pkt[4:8], "big") - REC0
      rep = b"\xFF" + bytes(ram[off : off + sz])
      logger.observe_can([(t, [(XCP_DTO, rep, XCP_BUS)])], t + 19_000_000, v_ego=15.0)
    if i >= 260:  # post-discovery steady state across 11 active moving + 1 standing slots
      for adj in (5, 8, 11):
        lat_ts = logger.slot_cache[adj]["lat_updated_ns"]
        assert lat_ts > 0
        max_adj_lat_age_s = max(max_adj_lat_age_s, (t - lat_ts) / 1e9)
    if standing_visible_at is None and shm_path.exists():
      pts = json.loads(shm_path.read_text())["points"]
      if any(55.0 <= px <= 63.0 and abs(py - 0.3) < 0.5 for px, py in pts):
        standing_visible_at = i * 0.02
    t += 20_000_000

  logger.close()
  assert max_adj_lat_age_s <= 0.9
  assert standing_visible_at is not None and standing_visible_at <= 7.5


def test_stale_measured_downgraded_and_trailer_offset_lead_match(monkeypatch):
  rows = _capture_events(monkeypatch)
  cp = SimpleNamespace(brand="honda", flags=int(HondaFlags.BOSCH), passive=False)
  logger = XcpRadarLogger(cp)
  logger.state = "READY"
  logger.last_v_ego = 9.8
  t0 = 10_000_000_000

  # Slot 33: s4 tid 22 style lead (dd = 5.93 m > 4.5 m, trailer-compensated dd_eff = 2.54 m)
  logger.slot_cache[33].update({
    "X_raw": int(round(34.16 * 128)),
    "Vx_raw": int(round(-2.45 * 256)),
    "Ax_raw": int(round(-0.8 * 2048)),
    "Y_raw": int(round(0.44 * 128)),
    "Vy_raw": 0,
    "age": 120,
    "flags": 0xE0C0,
    "subflags": 0x0400,
    "qual": 25,
    "cls_conf": 80,
    "radar_track_id": 403,
    "kin_updated_ns": t0 - 25_000_000,
    "lat_updated_ns": t0 - 25_000_000,
    "age_updated_ns": t0 - 25_000_000,
    "flags_updated_ns": t0 - 25_000_000,
    "updated_ns": t0 - 25_000_000,
    "last_rtt_ns": 10_000_000,
  })
  logger.active_slots[33] = logger.slot_cache[33]

  # Slot 10: background measured slot with stale kinematics (> 200 ms old)
  logger.slot_cache[10].update({
    "X_raw": int(round(20.84 * 128)),
    "Vx_raw": int(round(-2.14 * 256)),
    "Ax_raw": 0,
    "Y_raw": int(round(3.43 * 128)),
    "Vy_raw": 0,
    "age": 90,
    "flags": 0xE0C0,
    "subflags": 0x0000,
    "qual": 20,
    "cls_conf": 70,
    "radar_track_id": 407,
    "kin_updated_ns": t0 - 350_000_000,
    "lat_updated_ns": t0 - 25_000_000,
    "age_updated_ns": t0 - 25_000_000,
    "flags_updated_ns": t0 - 25_000_000,
    "updated_ns": t0 - 25_000_000,
    "last_rtt_ns": 10_000_000,
  })
  logger.active_slots[10] = logger.slot_cache[10]

  logger.observe_radar_tracks(
    SimpleNamespace(points=[SimpleNamespace(trackId=22, dRel=28.23, yRel=0.18, vRel=-1.04)]),
    t0,
  )
  logger._maybe_log_20hz(t0)
  assert len(rows) == 1
  by_slot = {tgt["slot"]: tgt for tgt in rows[0]["targets"]}
  assert by_slot[33]["stage"] == "measured"
  assert by_slot[33]["fw_stage"] == "measured"
  assert by_slot[33]["measured"] is True
  assert by_slot[33]["trust_accel"] is True
  assert by_slot[33]["kin_age_ms"] == 25.0
  assert by_slot[33]["op_track_id"] == 22
  assert rows[0]["lead"]["slot"] == 33
  assert rows[0]["lead"]["op_track_id"] == 22
  assert rows[0]["lead"]["lead_source"] == "op_lead"

  assert by_slot[10]["stage"] == "stale"
  assert by_slot[10]["fw_stage"] == "measured"
  assert by_slot[10]["measured"] is False
  assert by_slot[10]["trust_accel"] is False
  assert by_slot[10]["kin_age_ms"] == 350.0

  # When op_tracks has no lead, fallback sets lead_source == 'xcp_fallback'
  logger.observe_radar_tracks(SimpleNamespace(points=[]), t0 + 100_000_000)
  logger._maybe_log_20hz(t0 + 100_000_000)
  assert len(rows) == 2
  assert rows[1]["lead"]["slot"] == 33
  assert rows[1]["lead"]["op_track_id"] is None
  assert rows[1]["lead"]["lead_source"] == "xcp_fallback"
  assert logger._next_read_task_locked(t0 + 100_000_000) is not None
  logger.close()


def test_fast_inactive_probe_when_op_lead_unmatched():
  cp = SimpleNamespace(brand="honda", flags=int(HondaFlags.BOSCH), passive=False)
  logger = XcpRadarLogger(cp)
  logger.state = "READY"
  logger.last_v_ego = 16.0
  t0 = 10_000_000_000
  # Seed a non-matching active target so prio_slots is non-empty, plus an unmatched in-path op_track
  logger.slot_cache[2].update({
    "X_raw": int(round(30.0 * 128)),
    "Vx_raw": int(round(-1.0 * 256)),
    "Y_raw": int(round(-3.5 * 128)),
    "age": 20,
    "flags": 0xE0C0,
    "flags_updated_ns": t0,
    "age_updated_ns": t0,
    "lat_updated_ns": t0,
    "kin_updated_ns": t0,
    "qual_updated_ns": t0,
  })
  logger.active_slots[2] = logger.slot_cache[2]
  logger._op_matched_slots = []
  logger.observe_radar_tracks(SimpleNamespace(points=[SimpleNamespace(trackId=41, dRel=83.0, yRel=-0.2, vRel=-12.0)]), t0)

  # After 3 reads (INACTIVE_PROBE_PERIOD_NO_LEAD), should probe an inactive slot (offset 16, size 6 on slot != 2)
  logger._read_seq = 2
  logger._last_probe_seq = 0
  task = logger._next_read_task_locked(t0 + 5_000_000)
  assert task is not None and task[0] != 2 and task[1:] == (16, 6)
  logger.close()


def test_wayb_1hz_probe_reads_a150_bases_and_preserves_lead_rate(monkeypatch):
  rows = _capture_events(monkeypatch)
  ram = bytearray(NUM_SLOTS * STRIDE)
  _write_slot(ram, 4, {"x": 32.0, "vx": -1.5, "ax": -0.8, "y": 0.1, "vy": 0.0, "flags": 0xE0C0, "sub": 0x0400, "age": 35, "qual": 25, "cls": 85})

  assert not any(name.startswith("WAYB_PTR_") for name in dir(xcp_mod))
  assert not hasattr(xcp_mod, "WAYB_SEL_CAM_OFFSET")
  t1_base = 0x40032AEC
  out18_base = 0x4003681C
  p_base = 0x400397FC
  p_win_start = 0x400397DC
  t1_slot = 7
  assert xcp_mod.WAYB_DEFAULT_T1_BASE == t1_base
  assert xcp_mod.WAYB_DEFAULT_OUT18_BASE == out18_base
  assert xcp_mod.WAYB_DEFAULT_P_BASE == p_base
  assert xcp_mod.WAYB_P_WIN_START == p_win_start

  # Mock bounded A150 SRAM for P (0x400397FC), P-window (0x400397DC..0x400398DE), T1[7], and OUT18[0..31]+16.
  t1_slot7 = bytearray(bytes((i & 0xFF) for i in range(104)))
  t1_slot7[24:30] = (
    int(round(32.0 * 128)).to_bytes(2, "big", signed=True)
    + (0).to_bytes(2, "big", signed=True)
    + int(round(-1.5 * 256)).to_bytes(2, "big", signed=True)
  )
  t1_slot7[40:44] = int(round(-15.0 * 256)).to_bytes(2, "big", signed=True) + (0).to_bytes(2, "big", signed=True)
  t1_slot7[86:92] = (0x8800).to_bytes(2, "big") + bytes([0, 0, 0, 4])
  out18_table = {s: bytes([0, 0]) for s in range(32)}
  out18_table[t1_slot] = bytes([22, 0x03])
  p_win_mem = bytes(((i + 0x10) & 0xFF) for i in range(37 * 7))

  cp = SimpleNamespace(brand="honda", flags=int(HondaFlags.BOSCH), passive=False)
  logger = XcpRadarLogger(cp, probe_wayb=True)
  assert logger._wayb_probe_enabled is True
  t = _unlock_logger(logger, 1_000_000_000, v_ego=15.0)
  rd = SimpleNamespace(points=[SimpleNamespace(trackId=22, dRel=28.6, yRel=0.1, vRel=-1.5)])

  wayb_p0_reads = 0
  lead_kin_reads = 0
  for i in range(180):  # 3.6 s at 20 ms RTT
    if i >= 15:
      logger.observe_radar_tracks(rd, t)
    sends = logger.next_can_sends(t)
    if sends:
      pkt = bytes(sends[0].dat)
      sz = pkt[1]
      addr = int.from_bytes(pkt[4:8], "big")
      if REC0 <= addr < REC0 + NUM_SLOTS * STRIDE:
        off = addr - REC0
        if (off // STRIDE) == 4 and (off % STRIDE) == 16 and i >= 20:
          lead_kin_reads += 1
        rep = b"\xFF" + bytes(ram[off : off + sz])
      elif addr == p_base + 2 and sz == 7:
        wayb_p0_reads += 1
        rep = b"\xFF" + bytes([0x00, 0x01, 0x00, 0x02, 5, t1_slot, 0])
      elif (t1_base + 104 * t1_slot) <= addr and (addr + sz) <= (t1_base + 104 * (t1_slot + 1)):
        off = addr - (t1_base + 104 * t1_slot)
        rep = b"\xFF" + bytes(t1_slot7[off : off + sz])
      elif sz == 2 and ((addr - out18_base) % 18) == 16 and 0 <= ((addr - out18_base) // 18) < 32:
        s = (addr - out18_base) // 18
        rep = b"\xFF" + out18_table[s]
      elif sz == 7 and p_win_start <= addr and (addr + sz) <= p_win_start + len(p_win_mem) and ((addr - p_win_start) % 7) == 0:
        off = addr - p_win_start
        rep = b"\xFF" + p_win_mem[off : off + sz]
      else:
        raise AssertionError(f"Unexpected XCP probe address {addr:#x} size={sz}")
      logger.observe_can([(t, [(XCP_DTO, rep, XCP_BUS)])], t + 19_000_000, v_ego=15.0)
    t += 20_000_000

  assert logger.state == "READY"
  logger.close()

  assert 3 <= wayb_p0_reads <= 4
  assert lead_kin_reads >= 45
  wb = rows[-1].get("wayb_probe")
  assert wb is not None
  assert "ptrs_raw" not in wb
  assert "sel_cam_id" not in wb and "sel_flags" not in wb and "sel_state" not in wb
  assert wb["t1_base"] == hex(t1_base)
  assert wb["out18_base"] == hex(out18_base)
  assert wb["p_base"] == hex(p_base)
  assert wb["p0_valid"] == 5
  assert wb["p0_t1_slot"] == t1_slot
  assert wb["p0_slot_queried"] == t1_slot
  assert wb["t1_x_raw"] == int(round(32.0 * 128))
  assert wb["t1_vx28_raw"] == int(round(-1.5 * 256))
  assert wb["t1_vx40_raw"] == int(round(-15.0 * 256))
  assert wb["t1_flags"] == 0x8800
  assert wb["t1_valid"] == 4
  assert wb["out18_cam_id"] == 22
  assert wb["out18_cam_flags"] == 0x03
  assert len(wb["out18_scan"]) == 4
  assert len(wb["p_win"]) == 2
  assert all(len(bytes.fromhex(hx)) == 7 for _, hx in wb["p_win"])
  td_slot, td_off, td_hex = wb["t1_dump"]
  assert td_slot == t1_slot
  assert len(bytes.fromhex(td_hex)) == min(7, 104 - td_off)
  assert wb["last_err"] is None
  assert wb["last_err_addr"] is None


def test_wayb_1hz_probe_idle_oob_slot_and_error_isolation(monkeypatch):
  rows = _capture_events(monkeypatch)
  ram = bytearray(NUM_SLOTS * STRIDE)
  _write_slot(ram, 4, {"x": 32.0, "vx": -1.5, "ax": -0.8, "y": 0.1, "vy": 0.0, "flags": 0xE0C0, "sub": 0x0400, "age": 35, "qual": 25, "cls": 85})

  t1_base = 0x40032AEC
  out18_base = 0x4003681C
  p_win_start = 0x400397DC
  slot0_t1 = bytes(104)

  cp = SimpleNamespace(brand="honda", flags=int(HondaFlags.BOSCH), passive=False)
  logger = XcpRadarLogger(cp, probe_wayb=True)
  t = _unlock_logger(logger, 1_000_000_000, v_ego=15.0)
  rd = SimpleNamespace(points=[SimpleNamespace(trackId=22, dRel=28.6, yRel=0.1, vRel=-1.5)])

  # Round 1: idle p0 (p0_valid=0, p0_t1_slot=255) -> must query slot 0 (slot_source="idle0")
  # Round 2: out-of-range slot (p0_valid=1, p0_t1_slot=32) -> must clamp/fallback to slot 0 (slot_source="idle0")
  # Round 3: XCP 0xFE error on p0 -> must log last_err="fe22", last_err_addr="0x400397fe", clear _wayb_queue, stay READY
  p0_replies = [
    b"\xFF" + bytes([0x00, 0x00, 0x00, 0x00, 0, 255, 0]),
    b"\xFF" + bytes([0x00, 0x01, 0x00, 0x02, 1, 32, 0]),
    b"\xFE\x22",
  ]
  p0_round = 0
  seen_slot0_t1 = 0
  seen_slot0_out18 = 0

  for i in range(160):  # 3.2 s at 20 ms RTT
    if i >= 10:
      logger.observe_radar_tracks(rd, t)
    sends = logger.next_can_sends(t)
    if sends:
      pkt = bytes(sends[0].dat)
      sz = pkt[1]
      addr = int.from_bytes(pkt[4:8], "big")
      if REC0 <= addr < REC0 + NUM_SLOTS * STRIDE:
        off = addr - REC0
        rep = b"\xFF" + bytes(ram[off : off + sz])
      elif addr == 0x400397FE and sz == 7:
        rep = p0_replies[min(p0_round, len(p0_replies) - 1)]
        p0_round += 1
      elif t1_base <= addr and (addr + sz) <= t1_base + 104:
        off = addr - t1_base
        if off == 24:
          seen_slot0_t1 += 1
        rep = b"\xFF" + slot0_t1[off : off + sz]
      elif sz == 2 and ((addr - out18_base) % 18) == 16 and 0 <= ((addr - out18_base) // 18) < 32:
        if addr == out18_base + 16:
          seen_slot0_out18 += 1
        rep = b"\xFF" + bytes(2)
      elif sz == 7 and p_win_start <= addr and (addr + sz) <= p_win_start + 37 * 7 and ((addr - p_win_start) % 7) == 0:
        rep = b"\xFF" + bytes(7)
      else:
        raise AssertionError(f"Out-of-bounds or unexpected Way B address {addr:#x} size={sz}")
      logger.observe_can([(t, [(XCP_DTO, rep, XCP_BUS)])], t + 19_000_000, v_ego=15.0)
    t += 20_000_000

  assert logger.state == "READY"
  assert p0_round >= 3
  assert seen_slot0_t1 == 2
  assert seen_slot0_out18 == 3  # 2 core out18 reads + 1 round-1 out18_scan read on slot 0
  assert len(logger._wayb_queue) == 0
  logger.close()

  wb = rows[-1].get("wayb_probe")
  assert wb is not None
  assert wb["p0_slot_queried"] == 0
  assert wb["slot_source"] == "idle0"
  assert wb["last_err"] == "fe22"
  assert wb["last_err_addr"] == "0x400397fe"
  assert wb["updated_ns"] > 0

  # Verify first-read 0xFE error alone sets updated_ns > 0 and emits wayb_probe
  rows.clear()
  logger2 = XcpRadarLogger(cp, probe_wayb=True)
  t2 = _unlock_logger(logger2, 1_000_000_000, v_ego=15.0)
  for _ in range(30):
    sends = logger2.next_can_sends(t2)
    if sends:
      pkt = bytes(sends[0].dat)
      sz = pkt[1]
      addr = int.from_bytes(pkt[4:8], "big")
      if REC0 <= addr < REC0 + NUM_SLOTS * STRIDE:
        rep = b"\xFF" + bytes(ram[addr - REC0 : addr - REC0 + sz])
      elif addr == 0x400397FE:
        rep = b"\xFE\x22"
      else:
        raise AssertionError(f"Unexpected address {addr:#x}")
      logger2.observe_can([(t2, [(XCP_DTO, rep, XCP_BUS)])], t2 + 19_000_000, v_ego=15.0)
    t2 += 20_000_000
  assert logger2.state == "READY"
  assert len(logger2._wayb_queue) == 0
  logger2.close()
  wb2 = rows[-1].get("wayb_probe")
  assert wb2 is not None and wb2["last_err"] == "fe22" and wb2["last_err_addr"] == "0x400397fe" and wb2["updated_ns"] > 0

  # Verify 0xFE error on first p_win chunk does not block core/scan/dump reads or stall next round's cursor
  rows.clear()
  logger3 = XcpRadarLogger(cp, probe_wayb=True)
  t3 = _unlock_logger(logger3, 1_000_000_000, v_ego=15.0)
  pwin_seen_round2: list[int] = []
  saw_r1_core = False
  saw_r1_scan = False
  saw_r1_dump = False
  for _ in range(95):  # 1.9 s -> completes rounds 1 and 2 before round 3 starts at +2.0 s
    sends = logger3.next_can_sends(t3)
    if sends:
      pkt = bytes(sends[0].dat)
      sz = pkt[1]
      addr = int.from_bytes(pkt[4:8], "big")
      if REC0 <= addr < REC0 + NUM_SLOTS * STRIDE:
        rep = b"\xFF" + bytes(ram[addr - REC0 : addr - REC0 + sz])
      elif addr == 0x400397FE and sz == 7:
        rep = b"\xFF" + bytes([0, 0, 0, 0, 0, 255, 0])
      elif t1_base <= addr and (addr + sz) <= t1_base + 104:
        off = addr - t1_base
        if off == 24:
          saw_r1_core = True
        if off == 0 and sz == 7:
          saw_r1_dump = True
        rep = b"\xFF" + slot0_t1[off : off + sz]
      elif sz == 2 and ((addr - out18_base) % 18) == 16 and 0 <= ((addr - out18_base) // 18) < 32:
        if addr != out18_base + 16:
          saw_r1_scan = True
        rep = b"\xFF" + bytes(2)
      elif addr == p_win_start and sz == 7:
        rep = b"\xFE\x22"
      elif sz == 7 and p_win_start < addr and (addr + sz) <= p_win_start + 37 * 7 and ((addr - p_win_start) % 7) == 0:
        pwin_seen_round2.append(addr)
        rep = b"\xFF" + bytes(7)
      else:
        raise AssertionError(f"Unexpected address {addr:#x} size={sz}")
      logger3.observe_can([(t3, [(XCP_DTO, rep, XCP_BUS)])], t3 + 19_000_000, v_ego=15.0)
    t3 += 20_000_000
  assert logger3.state == "READY"
  logger3.close()
  assert saw_r1_core and saw_r1_scan and saw_r1_dump
  assert pwin_seen_round2 == [0x400397EA, 0x400397F1]
  wb3 = rows[-1].get("wayb_probe")
  assert wb3 is not None and wb3["last_err"] == "fe22" and wb3["last_err_addr"] == "0x400397dc"
  assert [a for a, _ in wb3["p_win"]] == ["0x400397ea", "0x400397f1"]


def test_wayb_1hz_out18_scan_locks_inlane_lead_slot_and_releases_on_change(monkeypatch):
  rows = _capture_events(monkeypatch)
  ram = bytearray(NUM_SLOTS * STRIDE)
  _write_slot(ram, 4, {"x": 28.4, "vx": -1.0, "ax": -0.2, "y": 0.2, "vy": 0.0, "flags": 0xE0C0, "sub": 0x0400, "age": 35, "qual": 25, "cls": 85})

  t1_base = 0x40032AEC
  out18_base = 0x4003681C
  p_win_start = 0x400397DC
  lead_slot = 13

  t1_tables: dict[int, bytearray] = {s: bytearray(104) for s in range(32)}
  t1_tables[lead_slot][24:30] = (
    int(round(25.0 * 128)).to_bytes(2, "big", signed=True)
    + (0).to_bytes(2, "big", signed=True)
    + int(round(0.2 * 128)).to_bytes(2, "big", signed=True)
  )
  t1_tables[lead_slot][40:44] = int(round(-1.0 * 256)).to_bytes(2, "big", signed=True) + (0).to_bytes(2, "big", signed=True)
  t1_tables[lead_slot][86:92] = (0x4C50).to_bytes(2, "big") + bytes([0, 0, 0, 4])

  out18_table: dict[int, bytes] = {s: bytes([0, 0]) for s in range(32)}
  out18_table[lead_slot] = bytes([42, 0x40])

  cp = SimpleNamespace(brand="honda", flags=int(HondaFlags.BOSCH), passive=False)
  logger = XcpRadarLogger(cp, probe_wayb=True)
  t = _unlock_logger(logger, 1_000_000_000, v_ego=15.0)
  rd = SimpleNamespace(points=[SimpleNamespace(trackId=42, dRel=25.0, yRel=0.2, vRel=-1.0)])

  # Phase 1: 6.2 s (6 rounds) while p0 is idle -> round 4 scans slots 12..15, locking slot=13 on rounds 5..6
  phase1_snapshots: list[dict] = []
  last_snap_upd = 0
  for i in range(310):
    if i >= 10:
      logger.observe_radar_tracks(rd, t)
    sends = logger.next_can_sends(t)
    if sends:
      pkt = bytes(sends[0].dat)
      sz = pkt[1]
      addr = int.from_bytes(pkt[4:8], "big")
      if REC0 <= addr < REC0 + NUM_SLOTS * STRIDE:
        rep = b"\xFF" + bytes(ram[addr - REC0 : addr - REC0 + sz])
      elif addr == 0x400397FE and sz == 7:
        rep = b"\xFF" + bytes([0, 0, 0, 0, 0, 255, 0])
      elif t1_base <= addr and (addr + sz) <= t1_base + 32 * 104:
        s = (addr - t1_base) // 104
        off = (addr - t1_base) % 104
        assert off + sz <= 104
        rep = b"\xFF" + bytes(t1_tables[s][off : off + sz])
      elif sz == 2 and ((addr - out18_base) % 18) == 16 and 0 <= ((addr - out18_base) // 18) < 32:
        rep = b"\xFF" + out18_table[(addr - out18_base) // 18]
      elif sz == 7 and p_win_start <= addr and (addr + sz) <= p_win_start + 37 * 7 and ((addr - p_win_start) % 7) == 0:
        rep = b"\xFF" + bytes(7)
      else:
        raise AssertionError(f"Unexpected address {addr:#x} size={sz}")
      logger.observe_can([(t, [(XCP_DTO, rep, XCP_BUS)])], t + 19_000_000, v_ego=15.0)
    wb_cur = rows[-1].get("wayb_probe") if rows else None
    if wb_cur and wb_cur["updated_ns"] != last_snap_upd:
      last_snap_upd = wb_cur["updated_ns"]
      phase1_snapshots.append(wb_cur)
    t += 20_000_000

  locked = [w for w in phase1_snapshots if w.get("slot_source") == "lead_scan"]
  assert len(locked) >= 2
  last_locked = locked[-1]
  assert last_locked["lead_slot"] == lead_slot
  assert last_locked["lead_cam_id"] == 42
  assert last_locked["p0_slot_queried"] == lead_slot
  assert last_locked["t1_x_raw"] == int(round(25.0 * 128))
  assert last_locked["t1_vx40_raw"] == int(round(-1.0 * 256))
  assert last_locked["t1_flags"] == 0x4C50
  assert last_locked["t1_dump"][0] == lead_slot

  # Phase 2: clear OUT18[13] and run 2 more rounds -> must release lock back to idle0 / slot 0
  out18_table[lead_slot] = bytes([0, 0])
  for _ in range(110):
    logger.observe_radar_tracks(rd, t)
    sends = logger.next_can_sends(t)
    if sends:
      pkt = bytes(sends[0].dat)
      sz = pkt[1]
      addr = int.from_bytes(pkt[4:8], "big")
      if REC0 <= addr < REC0 + NUM_SLOTS * STRIDE:
        rep = b"\xFF" + bytes(ram[addr - REC0 : addr - REC0 + sz])
      elif addr == 0x400397FE and sz == 7:
        rep = b"\xFF" + bytes([0, 0, 0, 0, 0, 255, 0])
      elif t1_base <= addr and (addr + sz) <= t1_base + 32 * 104:
        s = (addr - t1_base) // 104
        off = (addr - t1_base) % 104
        assert off + sz <= 104
        rep = b"\xFF" + bytes(t1_tables[s][off : off + sz])
      elif sz == 2 and ((addr - out18_base) % 18) == 16 and 0 <= ((addr - out18_base) // 18) < 32:
        rep = b"\xFF" + out18_table[(addr - out18_base) // 18]
      elif sz == 7 and p_win_start <= addr and (addr + sz) <= p_win_start + 37 * 7 and ((addr - p_win_start) % 7) == 0:
        rep = b"\xFF" + bytes(7)
      else:
        raise AssertionError(f"Unexpected address {addr:#x} size={sz}")
      logger.observe_can([(t, [(XCP_DTO, rep, XCP_BUS)])], t + 19_000_000, v_ego=15.0)
    t += 20_000_000

  wb_rel = rows[-1].get("wayb_probe")
  assert wb_rel["lead_slot"] is None
  assert wb_rel["lead_cam_id"] is None
  assert wb_rel["slot_source"] == "idle0"
  assert wb_rel["p0_slot_queried"] == 0

  # Phase 3: run 15+ rounds and verify T1 dump covers all 104 bytes (off=98 -> 6 B)
  dump_slices: dict[int, int] = {}
  for _ in range(780):  # +15.6 s (15+ full 1 Hz rounds)
    logger.observe_radar_tracks(rd, t)
    sends = logger.next_can_sends(t)
    if sends:
      pkt = bytes(sends[0].dat)
      sz = pkt[1]
      addr = int.from_bytes(pkt[4:8], "big")
      if REC0 <= addr < REC0 + NUM_SLOTS * STRIDE:
        rep = b"\xFF" + bytes(ram[addr - REC0 : addr - REC0 + sz])
      elif addr == 0x400397FE and sz == 7:
        rep = b"\xFF" + bytes([0, 0, 0, 0, 0, 255, 0])
      elif t1_base <= addr and (addr + sz) <= t1_base + 32 * 104:
        s = (addr - t1_base) // 104
        off = (addr - t1_base) % 104
        assert off + sz <= 104
        rep = b"\xFF" + bytes(t1_tables[s][off : off + sz])
      elif sz == 2 and ((addr - out18_base) % 18) == 16 and 0 <= ((addr - out18_base) // 18) < 32:
        rep = b"\xFF" + out18_table[(addr - out18_base) // 18]
      elif sz == 7 and p_win_start <= addr and (addr + sz) <= p_win_start + 37 * 7 and ((addr - p_win_start) % 7) == 0:
        rep = b"\xFF" + bytes(7)
      else:
        raise AssertionError(f"Unexpected address {addr:#x} size={sz}")
      logger.observe_can([(t, [(XCP_DTO, rep, XCP_BUS)])], t + 19_000_000, v_ego=15.0)
    td = rows[-1].get("wayb_probe", {}).get("t1_dump")
    if td is not None:
      dump_slices[td[1]] = len(bytes.fromhex(td[2]))
    t += 20_000_000

  logger.close()
  assert dump_slices == {**dict.fromkeys(range(0, 98, 7), 7), 98: 6}
  assert sum(dump_slices.values()) == 104


def test_match_openpilot_tracks_keeps_locked_closing_lead_and_railed_u11():
  cp = SimpleNamespace(brand="honda", flags=int(HondaFlags.BOSCH), passive=False)
  logger = XcpRadarLogger(cp)
  op_tracks = [{"tid": 34, "dRel": 45.0, "yRel": -0.1, "vRel": 2.5}]

  # Initial acquisition with dv = 4.5 m/s (> 3.5) must still reject when not previously locked.
  t_wide = [{"slot": 11, "dRel_m": 44.5, "yRel_m": -0.2, "vRel_mps": -2.0, "lat_updated_ns": 1}]
  matched, op_lead_slot = logger._match_openpilot_tracks(t_wide, op_tracks, v_ego=11.0)
  assert matched == [] and op_lead_slot is None and t_wide[0].get("op_track_id") is None

  # Railed U11 (o["vRel"] <= -11.5) matches on initial acquisition up to dv <= 25.0.
  t_rail = [{"slot": 12, "dRel_m": 45.0, "yRel_m": 0.0, "vRel_mps": -17.5, "lat_updated_ns": 1}]
  matched_rail, op_lead_rail = logger._match_openpilot_tracks(
    t_rail, [{"tid": 40, "dRel": 44.5, "yRel": 0.0, "vRel": -12.0}], v_ego=22.0
  )
  assert matched_rail == [12] and op_lead_rail == 12

  # Acquire moving lead within dv < 3.5, then keep locked across closing_lag = 6.0 m/s (dv = 6.0 > 3.5).
  t_acq = [{"slot": 11, "dRel_m": 44.5, "yRel_m": -0.2, "vRel_mps": 0.5, "lat_updated_ns": 1}]
  matched1, lead1 = logger._match_openpilot_tracks(t_acq, op_tracks, v_ego=11.0)
  assert matched1 == [11] and lead1 == 11

  t_brake = [{"slot": 11, "dRel_m": 44.0, "yRel_m": -0.2, "vRel_mps": -3.5, "lat_updated_ns": 1}]
  matched2, lead2 = logger._match_openpilot_tracks(t_brake, op_tracks, v_ego=11.0)
  assert matched2 == [11] and lead2 == 11 and t_brake[0].get("op_track_id") == 34

  # Stationary target, wide lateral dy >= 1.5, zero yRel_m, or out-of-lane |yRel| > 1.5 must NOT use
  # the locked continuation window.
  t_stat = [{"slot": 11, "dRel_m": 44.0, "yRel_m": -0.2, "vRel_mps": -11.0, "lat_updated_ns": 1}]
  matched3, _ = logger._match_openpilot_tracks(t_stat, op_tracks, v_ego=11.0)
  assert matched3 == []

  logger._match_openpilot_tracks(t_acq, op_tracks, v_ego=11.0)
  t_adj = [{"slot": 11, "dRel_m": 44.0, "yRel_m": 1.6, "vRel_mps": -3.5, "lat_updated_ns": 1}]
  matched4, _ = logger._match_openpilot_tracks(t_adj, op_tracks, v_ego=11.0)
  assert matched4 == []

  op_y0 = [{"tid": 34, "dRel": 44.5, "yRel": 0.0, "vRel": 2.5}]
  t_y0_acq = [{"slot": 11, "dRel_m": 44.5, "yRel_m": 0.1, "vRel_mps": 0.5, "lat_updated_ns": 1}]
  logger._match_openpilot_tracks(t_y0_acq, op_y0, v_ego=11.0)
  t_y0_brake = [{"slot": 11, "dRel_m": 44.0, "yRel_m": 0.0, "vRel_mps": -3.5, "lat_updated_ns": 1}]
  matched5, _ = logger._match_openpilot_tracks(t_y0_brake, op_y0, v_ego=11.0)
  assert matched5 == []

  op_out = [{"tid": 35, "dRel": 44.5, "yRel": 1.6, "vRel": 2.5}]
  t_out_acq = [{"slot": 12, "dRel_m": 44.5, "yRel_m": 1.6, "vRel_mps": 0.5, "lat_updated_ns": 1}]
  logger._match_openpilot_tracks(t_out_acq, op_out, v_ego=11.0)
  t_out_brake = [{"slot": 12, "dRel_m": 44.0, "yRel_m": 1.6, "vRel_mps": -3.5, "lat_updated_ns": 1}]
  matched6, _ = logger._match_openpilot_tracks(t_out_brake, op_out, v_ego=11.0)
  assert matched6 == []
  logger.close()


def test_fast_assist_shm_publishes_on_primary_lead_kin_arrival_rate_limited(monkeypatch, tmp_path):
  import json
  rows = _capture_events(monkeypatch)
  ram = bytearray(NUM_SLOTS * STRIDE)
  _write_slot(ram, 5, {"x": 30.0, "vx": -1.5, "ax": -1.0, "y": 0.1, "vy": 0.0, "flags": 0xE0C0, "sub": 0x0400, "age": 35, "qual": 25, "cls": 85})

  ui_path = tmp_path / "xcp_radar_points.json"
  assist_path = tmp_path / "xcp_radar_assist.json"
  cp = SimpleNamespace(brand="honda", flags=int(HondaFlags.BOSCH), passive=False)
  logger = XcpRadarLogger(cp, shm_path=ui_path, assist_shm_path=assist_path, assist_fast_publish=True)
  t = _unlock_logger(logger, 1_000_000_000, v_ego=15.0)
  rd = SimpleNamespace(points=[SimpleNamespace(trackId=12, dRel=30.0, yRel=0.1, vRel=-1.5)])

  # Initialize slot 5 so it becomes prio_slots[0] and reaches first 50 ms epoch log
  for _ in range(15):
    logger.observe_radar_tracks(rd, t)
    sends = logger.next_can_sends(t)
    if sends:
      pkt = bytes(sends[0].dat)
      sz = pkt[1]
      off = int.from_bytes(pkt[4:8], "big") - REC0
      rep = b"\xFF" + bytes(ram[off : off + sz])
      logger.observe_can([(t, [(XCP_DTO, rep, XCP_BUS)])], t + 10_000_000, v_ego=15.0)
    t += 15_000_000

  assert assist_path.exists()
  # Sync cleanly to a fresh 50 ms epoch boundary ahead of any initialization timestamps.
  epoch_ns = max(t, logger.last_log_ns + 60_000_000)
  logger.observe_radar_tracks(rd, epoch_ns)
  logger.observe_can([], epoch_ns, v_ego=15.0)
  assert logger.last_log_ns == epoch_ns
  assert json.loads(assist_path.read_text())["mono_ns"] == epoch_ns
  events_before = len(rows)

  # Drive a direct priority-lead kinematics reply only 22 ms after the 50 ms log epoch (< 50 ms EPOCH_NS):
  # assist SHM must update immediately with new vRel, without emitting a second cloudlog.event.
  t_mid = epoch_ns + 22_000_000
  _write_slot(ram, 5, {"x": 29.8, "vx": -3.25, "ax": -3.0, "y": 0.1, "vy": 0.0, "flags": 0xE0C0, "sub": 0x0400, "age": 36, "qual": 25, "cls": 85})
  with logger._lock:
    logger.in_flight = ("F4", 5, 16, 6, t_mid - 8_000_000)
  rep_mid = b"\xFF" + bytes(ram[5 * STRIDE + 16 : 5 * STRIDE + 22])
  logger.observe_can([(t_mid, [(XCP_DTO, rep_mid, XCP_BUS)])], t_mid, v_ego=15.0)

  assert len(rows) == events_before
  assist_mid = json.loads(assist_path.read_text())
  assert assist_mid["mono_ns"] == t_mid
  assert assist_mid["tracks_by_op_id"]["12"]["vRel_mps"] == -3.25

  # Another reply only 10 ms later (< 20 ms ASSIST_SHM_MIN_INTERVAL_NS) is rate-limited until the next 20 ms window.
  t_soon = t_mid + 10_000_000
  _write_slot(ram, 5, {"x": 29.7, "vx": -3.75, "ax": -3.0, "y": 0.1, "vy": 0.0, "flags": 0xE0C0, "sub": 0x0400, "age": 36, "qual": 25, "cls": 85})
  with logger._lock:
    logger.in_flight = ("F4", 5, 16, 6, t_soon - 5_000_000)
  rep_soon = b"\xFF" + bytes(ram[5 * STRIDE + 16 : 5 * STRIDE + 22])
  logger.observe_can([(t_soon, [(XCP_DTO, rep_soon, XCP_BUS)])], t_soon, v_ego=15.0)
  assert len(rows) == events_before
  assert json.loads(assist_path.read_text())["mono_ns"] == t_mid

  logger.close()


def test_fast_assist_shm_gated_by_param_and_skipped_on_card_loop_when_io_thread_active(monkeypatch, tmp_path):
  import json

  class _FakeParams:
    def __init__(self, root):
      self.root = root

    def get_param_path(self, key: str) -> str:
      return str(self.root / key)

  _capture_events(monkeypatch)
  ram = bytearray(NUM_SLOTS * STRIDE)
  _write_slot(ram, 5, {"x": 30.0, "vx": -1.5, "ax": -1.0, "y": 0.1, "vy": 0.0, "flags": 0xE0C0, "sub": 0x0400, "age": 35, "qual": 25, "cls": 85})

  params_dir = tmp_path / "params"
  params_dir.mkdir()
  (params_dir / "IsReleaseBranch").write_bytes(b"0")
  (params_dir / "AlphaHondaXcpRadarAssist").write_bytes(b"0")
  fake_params = _FakeParams(params_dir)

  cp = SimpleNamespace(
    brand="honda",
    carFingerprint=CAR.HONDA_CIVIC_BOSCH,
    flags=int(HondaFlags.BOSCH),
    alternativeExperience=0,
    openpilotLongitudinalControl=True,
    pcmCruise=True,
    radarUnavailable=False,
    notCar=False,
    passive=False,
    dashcamOnly=False,
  )

  ui_off = tmp_path / "points_off.json"
  assist_off = tmp_path / "assist_off.json"
  logger_no_params = XcpRadarLogger(cp, shm_path=ui_off, assist_shm_path=assist_off)
  assert logger_no_params._assist_fast_publish_enabled is False
  logger_no_params.close()

  logger_off = XcpRadarLogger(cp, params=fake_params, shm_path=ui_off, assist_shm_path=assist_off)
  assert logger_off._assist_fast_publish_enabled is False

  t = _unlock_logger(logger_off, 2_000_000_000, v_ego=15.0)
  rd = SimpleNamespace(points=[SimpleNamespace(trackId=12, dRel=30.0, yRel=0.1, vRel=-1.5)])
  for _ in range(15):
    logger_off.observe_radar_tracks(rd, t)
    sends = logger_off.next_can_sends(t)
    if sends:
      pkt = bytes(sends[0].dat)
      sz = pkt[1]
      off = int.from_bytes(pkt[4:8], "big") - REC0
      rep = b"\xFF" + bytes(ram[off : off + sz])
      logger_off.observe_can([(t, [(XCP_DTO, rep, XCP_BUS)])], t + 10_000_000, v_ego=15.0)
    t += 15_000_000

  # 20 Hz _write_ui_snapshot still runs for V-ASM side_sightings even when AlphaHondaXcpRadarAssist is Off.
  assert assist_off.exists()
  epoch_ns = max(t, logger_off.last_log_ns + 60_000_000)
  logger_off.observe_radar_tracks(rd, epoch_ns)
  logger_off.observe_can([], epoch_ns, v_ego=15.0)
  assert json.loads(assist_off.read_text())["mono_ns"] == epoch_ns

  # Mid-epoch F4 reply must NOT trigger 50 Hz fast-publish when assist toggle is Off.
  t_mid = epoch_ns + 25_000_000
  _write_slot(ram, 5, {"x": 29.7, "vx": -3.5, "ax": -3.0, "y": 0.1, "vy": 0.0, "flags": 0xE0C0, "sub": 0x0400, "age": 36, "qual": 25, "cls": 85})
  with logger_off._lock:
    logger_off.in_flight = ("F4", 5, 16, 6, t_mid - 5_000_000)
  rep_mid = b"\xFF" + bytes(ram[5 * STRIDE + 16 : 5 * STRIDE + 22])
  logger_off.observe_can([(t_mid, [(XCP_DTO, rep_mid, XCP_BUS)])], t_mid, v_ego=15.0)
  assert logger_off._assist_shm_dirty is False
  assert json.loads(assist_off.read_text())["mono_ns"] == epoch_ns
  logger_off.close()

  # With AlphaHondaXcpRadarAssist=1, fast publish enables, and observe_can skips _maybe_publish_assist_shm
  # whenever the dedicated IO thread owns fast publishing.
  (params_dir / "AlphaHondaXcpRadarAssist").write_bytes(b"1")
  logger_on = XcpRadarLogger(
    cp,
    params=fake_params,
    shm_path=tmp_path / "points_on.json",
    assist_shm_path=tmp_path / "assist_on.json",
  )
  assert logger_on._assist_fast_publish_enabled is True
  publish_calls = 0

  def _count_publish(ns: int) -> None:
    nonlocal publish_calls
    publish_calls += 1

  monkeypatch.setattr(logger_on, "_maybe_publish_assist_shm", _count_publish)
  logger_on.state = "READY"
  logger_on._use_io_thread = True
  logger_on._assist_shm_dirty = True
  logger_on.observe_can([], epoch_ns + 25_000_000, v_ego=15.0)
  assert publish_calls == 0
  logger_on.close()
