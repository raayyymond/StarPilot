"""Read-only 20 Hz XCP Array B logger for Honda Bosch-A (36802-TBA-A150).

Observes 0x1EC00179 on CAN bus 1 and emits non-blocking single-in-flight
CONNECT/GET_STATUS/GET_SEED/UNLOCK/SHORT_UPLOAD requests on 0x1EC00079 (bus 1)
via an event-driven XCP I/O thread (~100 reads/sec hardware ceiling).
Never modifies CarState, RadarData, CarControl, or longitudinal/lateral control.
"""
from __future__ import annotations

import atexit
from collections import deque
import json
import math
import os
from pathlib import Path
import threading
import time

from openpilot.common.swaglog import cloudlog
from opendbc.car import CanData
from opendbc.car.honda.values import HondaFlags

XCP_BUS = 1
XCP_CTO = 0x1EC00079
XCP_DTO = 0x1EC00179

ARRAY_B_BASE = 0x400371A4
REC0 = 0x400371B4
STRIDE = 136
NUM_SLOTS = 40
MAX_ACTIVE_TARGETS = 40
LEAD_PRIORITY_COUNT = 2

EPOCH_NS = 50_000_000          # 20 Hz output logging cadence
LEAD_KIN_INTERVAL_NS = 18_000_000  # ~25 Hz (every 40 ms) at 20 ms RTT, ~33 Hz at 10 ms RTT
LEAD_META_INTERVAL_NS = 250_000_000  # ~4 Hz alternating age / flags (~2 Hz each) for primary lead
LEAD_LAT_INTERVAL_NS = 180_000_000   # ~5.5 Hz Y / Vy for primary lead
LEAD_QUAL_INTERVAL_NS = 1_000_000_000
LEAD2_KIN_INTERVAL_NS = 110_000_000  # ~8 Hz kinematics for secondary priority target
LEAD2_LAT_INTERVAL_NS = 150_000_000  # ~6.7 Hz Y / Vy for secondary priority target
LEAD2_META_INTERVAL_NS = 800_000_000
OTHER_LAT_INTERVAL_NS = 250_000_000  # ~4 Hz Y / Vy target cadence for moving cut-in traffic
OTHER_META_INTERVAL_NS = 1_000_000_000
OTHER_QUAL_INTERVAL_NS = 1_000_000_000
OTHER_MAX_STALE_NS = 350_000_000
OTHER_META_MAX_STALE_NS = 1_000_000_000
FAR_OTHER_KIN_INTERVAL_NS = 700_000_000
FAR_OTHER_LAT_INTERVAL_NS = 700_000_000
FAR_OTHER_META_INTERVAL_NS = 1_200_000_000
NEWBORN_AGE_REFRESH_NS = 100_000_000  # re-poll newborn track age < 3 quickly so UI unlocks promptly
NEAR_MOVING_Y_RAW = 768               # ±6.0 m (1/128 m/lsb) keeps adjacent lanes ahead of wide oncoming cars
STATIONARY_IN_PATH_Y_RAW = 384        # ±3.0 m fast age < 3 refresh for oncoming standing traffic
STATIONARY_INIT_YIELD_READS = 8       # yield 1 read per 8 non-lead cycles to init waiting stopped slots without starving adjacent lanes
STATIONARY_MAX_STALE_NS = 3_000_000_000
INACTIVE_PROBE_PERIOD_WITH_LEAD = 10
INACTIVE_PROBE_PERIOD_NO_LEAD = 3
OP_TRACKS_MAX_AGE_NS = 500_000_000
MEASURED_MAX_KIN_AGE_NS = 200_000_000  # 200 ms freshness ceiling for stage=='measured'
ASSIST_SHM_MIN_INTERVAL_NS = 20_000_000  # 50 Hz max assist-only SHM publish on primary-lead kin arrival
OP_MATCH_LOCKED_CLOSING_MAX_DV_MPS = 8.0
OP_MATCH_LOCKED_CLOSING_MAX_DY_M = 1.5
OP_MATCH_LOCKED_CLOSING_MAX_ABS_Y_M = 1.5
OP_MATCH_LOCKED_MIN_VLEAD_MPS = -0.5
ARRAY_B_TRAILER_OFFSET_M = 434.0 / 128.0  # 3.390625 m A150 Array B -> Array A range offset

F4_TIMEOUT_NS = 150_000_000     # single SHORT_UPLOAD timeout
HANDSHAKE_RETRY_NS = 2_000_000_000
STARTUP_DELAY_NS = 500_000_000

# 1 Hz Way B internal-fusion diagnostic probe (Bosch-A 36802-TBA-A150 verified SRAM bases).
# Offline route evidence (bob-unify-car):
# - T1=0x40032AEC and OUT18=0x4003681C (from 8f88b00741) were confirmed on routes
#   000000f4--618185d0bb and 000000f5--6e8e645f02; 8343dbb36f's +0x20 shift moved off both tables.
# - P=0x400397FC (-4 from 8343dbb36f's 0x40039800) and full 26-segment scan of
#   000000fa--148773dcd2 (948d8c2f5, 1483 rounds, 0 XCP errors):
#   - T1+86 bit 0x0010 matched OUT18+16 != 0 on 1481/1483 rounds.
#   - T1+28 (t1_vx28_raw, name retained for log compatibility) is lateral Y at 1/128 m
#     (p50 err = 0.035 m vs radarTracks yRel, n=168); T1+40 is vRel at 1/256 m/s (p50 |err| = 0.189 m/s, n=124).
#   - P[0] (0x400397FC) tracked yRel > 0 (left-side) targets on 124/124 matched rounds.
#   - T1+3414 = 0x40033842 (T1[32]+86) returned static sel_cam_id=7, sel_state=255 on all 75/75 nonzero rounds; removed.
# - Added bounded 1 Hz discovery reads: 4 OUT18[0..31]+16 scan reads/round (full 32-slot sweep in 8 s) to lock onto
#   the live in-lane lead slot; 2 P-window 7 B reads/round across 0x400397DC..0x400398DE (37 chunks = 259 B,
#   full sweep in 18.5 s); 1 rotating 7 B chunk/round across T1[slot]+0..103 (15 chunks = 104 B).
WAYB_PROBE_INTERVAL_NS = 1_000_000_000
WAYB_DEFAULT_T1_BASE = 0x40032AEC
WAYB_DEFAULT_OUT18_BASE = 0x4003681C
WAYB_DEFAULT_P_BASE = 0x400397FC
WAYB_T1_STRIDE = 104
WAYB_OUT18_STRIDE = 18
WAYB_T1_NUM_SLOTS = 32
WAYB_OUT18_SCAN_PER_ROUND = 4
WAYB_P_WIN_START = WAYB_DEFAULT_P_BASE - 0x20  # 0x400397DC (-0x20..+0xE2 around P[0], inside valid BSS 0x400396FC..0x40039A64)
WAYB_P_WIN_CHUNK_SIZE = 7
WAYB_P_WIN_NUM_CHUNKS = 37  # 37 * 7 = 259 B -> 0x400397DC..0x400398DE
WAYB_P_WIN_CHUNKS_PER_ROUND = 2
WAYB_T1_DUMP_CHUNK_SIZE = 7

XCP_UI_SHM_PATH = Path("/dev/shm/xcp_radar_points.json") if Path("/dev/shm").is_dir() else None
XCP_ASSIST_SHM_PATH = Path("/dev/shm/xcp_radar_assist.json") if Path("/dev/shm").is_dir() else None
_DEFAULT_ASSIST_SHM_PATH = object()
_CLOCK_BOOTTIME = getattr(time, "CLOCK_BOOTTIME", None)


def _boot_minus_mono_ns() -> int:
  if _CLOCK_BOOTTIME is not None:
    try:
      return max(0, time.clock_gettime_ns(_CLOCK_BOOTTIME) - time.monotonic_ns())
    except OSError:
      pass
  return 0


def _write_atomic_json(path: Path, data: dict) -> None:
  payload = json.dumps(data, separators=(",", ":")).encode("utf-8")
  tmp = path.with_name(f".{path.name}.{os.getpid()}.tmp")
  tmp.write_bytes(payload)
  os.replace(tmp, path)


def key_for_seed(seed_be: bytes) -> bytes:
  value = int.from_bytes(seed_be, "big")
  for _ in range(7):
    carry = value >> 31
    value = ((value << 1) | carry) & 0xFFFFFFFF
    if not carry:
      break
    value ^= 0xFB312378
  return value.to_bytes(4, "little")


def decode_flags(flags: int, subflags: int, age: int) -> dict:
  upper = flags & 0xE000
  if upper == 0xE000:
    stage = "measured"
  elif upper == 0xA000:
    stage = "coasting"
  elif upper == 0x6000:
    stage = "birth"
  else:
    stage = "other"

  if (flags & 0x0F00) != 0:
    conf_band = "high"
  elif (flags & 0x0080) != 0:
    conf_band = "med"
  elif (flags & 0x1000) != 0:
    conf_band = "low"
  else:
    conf_band = "none"

  measured = bool(flags & 0x4000)
  ghost = bool(flags & 0x0020)
  in_path = bool(subflags & 0x0400)
  trust_accel = bool(upper == 0xE000 and (flags & 0x1020) == 0 and age >= 5)
  return {
    "stage": stage,
    "measured": measured,
    "conf_band": conf_band,
    "ghost": ghost,
    "in_path": in_path,
    "trust_accel": trust_accel,
  }


def _empty_slot_record() -> dict:
  return {
    "X_raw": 0,
    "Vx_raw": 0,
    "Ax_raw": 0,
    "Y_raw": 0,
    "Vy_raw": 0,
    "raw24": 0x8000,
    "age": 0,
    "prev_age": None,
    "marker": 0,
    "subflags": 0,
    "flags": 0,
    "qual": 0,
    "cls_conf": 0,
    "radar_track_id": 0,
    "kin_updated_ns": 0,
    "lat_updated_ns": 0,
    "age_updated_ns": 0,
    "flags_updated_ns": 0,
    "qual_updated_ns": 0,
    "updated_ns": 0,
    "last_rtt_ns": 0,
  }


class XcpRadarLogger:
  def __init__(
    self,
    CP,
    *,
    send_lock: threading.Lock | None = None,
    pm=None,
    params=None,
    shm_path: Path | None = XCP_UI_SHM_PATH,
    assist_shm_path: Path | None | object = _DEFAULT_ASSIST_SHM_PATH,
    assist_fast_publish: bool | None = None,
    probe_wayb: bool | None = None,
  ) -> None:
    if assist_fast_publish is not None:
      self._assist_fast_publish_enabled = bool(assist_fast_publish)
    elif params is not None:
      try:
        from openpilot.starpilot.car.honda.xcp_radar_assist import enabled as xcp_assist_enabled
        self._assist_fast_publish_enabled = bool(xcp_assist_enabled(params, CP))
      except Exception:
        self._assist_fast_publish_enabled = False
    else:
      self._assist_fast_publish_enabled = False
    self.enabled = False
    self.state = "DISABLED"
    self.in_flight = None  # (cmd_type, slot, offset, size, sent_ns)
    self._init_queue: deque[tuple[int, int, int]] = deque()
    self._init_queued: set[tuple[int, int, int]] = set()
    self.active_slots: dict[int, dict] = {}
    self.slot_cache: dict[int, dict] = {s: _empty_slot_record() for s in range(NUM_SLOTS)}
    self.scan_cursor = 0
    self._rr_cursor = 0
    self._read_seq = 0
    self._last_probe_seq = -INACTIVE_PROBE_PERIOD_WITH_LEAD
    self._stationary_init_skip_count = 0
    self._next_radar_track_id = 1

    self.last_handshake_tx_ns = -HANDSHAKE_RETRY_NS
    self.last_log_ns = 0
    self.start_ns = time.monotonic_ns()
    self.consecutive_f4_timeouts = 0
    self.seed_bytes: bytes | None = None
    self.last_v_ego = 0.0

    self._op_tracks: list[dict] = []
    self._op_tracks_ns = 0
    self._op_matched_slots: list[int] = []
    self._prev_op_slot_by_tid: dict[int, int] = {}
    self._last_assist_shm_ns: int = 0
    self._assist_shm_dirty: bool = False

    self._lock = threading.RLock()
    self._send_lock = send_lock
    self._pm = pm
    self._wayb_probe_enabled = bool(probe_wayb if probe_wayb is not None else (pm is not None))
    self._wayb_queue: deque[tuple[str, int, int]] = deque()
    self._wayb_last_round_ns = 0
    self._wayb_t1_base = WAYB_DEFAULT_T1_BASE
    self._wayb_out18_base = WAYB_DEFAULT_OUT18_BASE
    self._wayb_p_base = WAYB_DEFAULT_P_BASE
    self._wayb_lead_slot: int | None = None
    self._wayb_lead_cam_id: int | None = None
    self._wayb_out18_cursor = 0
    self._wayb_pwin_cursor = 0
    self._wayb_t1_dump_off = 0
    self._wayb_state: dict = self._empty_wayb_state()
    self._shm_path = Path(shm_path) if shm_path is not None else None
    if assist_shm_path is _DEFAULT_ASSIST_SHM_PATH:
      self._assist_shm_path = XCP_ASSIST_SHM_PATH if shm_path is XCP_UI_SHM_PATH else None
    else:
      self._assist_shm_path = Path(assist_shm_path) if assist_shm_path is not None else None

    self._tx_armed = False
    self._tx_armed_ns = 0
    self._stop_event = threading.Event()
    self._tx_event = threading.Event()
    self._io_thread: threading.Thread | None = None
    self._use_io_thread = False
    self._closed = False

    try:
      if (
        "REPLAY" not in os.environ
        and getattr(CP, "brand", None) == "honda"
        and bool(CP.flags & HondaFlags.BOSCH)
        and not bool(CP.flags & (HondaFlags.BOSCH_RADARLESS | HondaFlags.BOSCH_CANFD))
        and not bool(CP.passive)
      ):
        self.enabled = True
        self.state = "NEED_CONNECT"
        atexit.register(self.close)
        if self._pm is not None:
          self._use_io_thread = True
          self._io_thread = threading.Thread(target=self._io_loop, name="xcp_radar_io", daemon=True)
          self._io_thread.start()
    except Exception:
      self.enabled = False
      self.close()

  def close(self) -> None:
    if self._closed:
      return
    self._closed = True
    self._stop_event.set()
    self._tx_event.set()
    io_thread = self._io_thread
    if io_thread is not None and io_thread is not threading.current_thread():
      io_thread.join(timeout=0.5)
    for path in (self._shm_path, self._assist_shm_path):
      if path is not None:
        try:
          path.unlink(missing_ok=True)
        except OSError:
          pass

  def _alloc_track_id_locked(self) -> int:
    tid = self._next_radar_track_id
    self._next_radar_track_id += 1
    return tid

  def _is_moving_target_locked(self, rec: dict) -> bool:
    if self.last_v_ego < 3.0:
      return True
    return abs(self.last_v_ego + rec["Vx_raw"] / 256.0) >= 1.5

  def _enqueue_init_locked(self, slot: int, *, moving: bool = True) -> None:
    if moving or self.last_v_ego < 3.0:
      # Push moving targets to front so Y/Vy + age arrive before flags unlocks emission
      for task in ((slot, 106, 2), (slot, 2, 6), (slot, 22, 6)):
        if task not in self._init_queued:
          self._init_queued.add(task)
          self._init_queue.appendleft(task)
    else:
      # Queue lateral read first; only near/in-path stationary targets enqueue age+flags to protect moving bandwidth
      task = (slot, 22, 6)
      if task not in self._init_queued:
        self._init_queued.add(task)
        self._init_queue.append(task)

  @staticmethod
  def _empty_wayb_state() -> dict:
    return {
      "t1_base": hex(WAYB_DEFAULT_T1_BASE),
      "out18_base": hex(WAYB_DEFAULT_OUT18_BASE),
      "p_base": hex(WAYB_DEFAULT_P_BASE),
      "p0_b2_b5": None,
      "p0_valid": None,
      "p0_t1_slot": None,
      "p0_b8": None,
      "p0_slot_queried": None,
      "t1_x_raw": None,
      "t1_u26": None,
      "t1_vx28_raw": None,
      "t1_vx40_raw": None,
      "t1_vy42_raw": None,
      "t1_flags": None,
      "t1_valid": None,
      "out18_cam_id": None,
      "out18_cam_flags": None,
      "slot_source": None,       # "lead_scan" | "p0" | "idle0"
      "lead_slot": None,         # T1/OUT18 slot locked via out18_scan matching current in-lane op_lead tid
      "lead_cam_id": None,       # matched op_lead tid
      "out18_scan": [],          # [[slot, cam_id, cam_flags], ...] from current 1 Hz round (reassigned, never mutated in place)
      "p_win": [],               # [[addr_hex, data_hex], ...] from current 1 Hz round (reassigned, never mutated in place)
      "t1_dump": None,           # [slot, offset, data_hex] single 7B (or 6B at off=98) slice of queried T1[slot]
      "last_err": None,
      "last_err_addr": None,     # hex(addr) of most recent WAYB 0xFE error
      "updated_ns": 0,
    }

  def _clear_session_locked(self, target_state: str = "NEED_CONNECT") -> None:
    self.state = target_state
    self.in_flight = None
    self._init_queue.clear()
    self._init_queued.clear()
    self._wayb_queue.clear()
    self._wayb_last_round_ns = 0
    self._wayb_lead_slot = None
    self._wayb_lead_cam_id = None
    self.seed_bytes = None
    self.consecutive_f4_timeouts = 0
    self._prev_op_slot_by_tid.clear()
    self._assist_shm_dirty = False

  def _reset_slot_locked(self, slot: int) -> None:
    self.active_slots.pop(slot, None)
    self.slot_cache[slot] = _empty_slot_record()

  def _handle_f4_reply_locked(self, slot: int, offset: int, size: int, payload: bytes, now_ns: int, sent_ns: int) -> None:
    if len(payload) < 1 + size or not (0 <= slot < NUM_SLOTS):
      return
    data = payload[1 : 1 + size]
    rec = self.slot_cache[slot]
    rtt_ns = max(0, now_ns - sent_ns)
    rec["updated_ns"] = now_ns
    rec["last_rtt_ns"] = rtt_ns

    if offset == 16 and size == 6:
      rec["X_raw"] = int.from_bytes(data[0:2], "big", signed=True)
      rec["Vx_raw"] = int.from_bytes(data[2:4], "big", signed=True)
      rec["Ax_raw"] = int.from_bytes(data[4:6], "big", signed=True)
      rec["kin_updated_ns"] = now_ns
    elif offset == 22 and size == 6:
      rec["Y_raw"] = int.from_bytes(data[0:2], "big", signed=True)
      rec["raw24"] = int.from_bytes(data[2:4], "big", signed=False)
      rec["Vy_raw"] = int.from_bytes(data[4:6], "big", signed=True)
      rec["lat_updated_ns"] = now_ns
      if slot in self.active_slots and not self._is_moving_target_locked(rec):
        if rec["X_raw"] > 0 and (bool(rec["subflags"] & 0x0400) or abs(rec["Y_raw"]) < STATIONARY_IN_PATH_Y_RAW):
          for follow in ((slot, 2, 6), (slot, 106, 2)):
            if follow not in self._init_queued and (
              (follow[1] == 2 and rec["age_updated_ns"] == 0) or (follow[1] == 106 and rec["flags_updated_ns"] == 0)
            ):
              self._init_queued.add(follow)
              self._init_queue.append(follow)
    elif offset == 2 and size == 6:
      new_age = int.from_bytes(data[0:2], "big", signed=False)
      rec["marker"] = int.from_bytes(data[2:4], "big", signed=False)
      rec["subflags"] = int.from_bytes(data[4:6], "big", signed=False)
      prev_age = rec["prev_age"]
      if slot in self.active_slots and prev_age is not None and new_age < prev_age:
        rec["radar_track_id"] = self._alloc_track_id_locked()
      rec["age"] = new_age
      rec["prev_age"] = new_age
      rec["age_updated_ns"] = now_ns
    elif offset == 106 and size == 2:
      rec["flags"] = int.from_bytes(data[0:2], "big", signed=False)
      rec["flags_updated_ns"] = now_ns
    elif offset == 127 and size == 2:
      rec["qual"] = int(data[0])
      rec["cls_conf"] = int(data[1])
      rec["qual_updated_ns"] = now_ns

    flags = rec["flags"]
    flags_read = rec["flags_updated_ns"] > 0
    is_valid_bit13 = bool(flags & 0x2000)
    nonzero_kin = (rec["X_raw"] != 0 or rec["Vx_raw"] != 0 or rec["Ax_raw"] != 0)
    nonzero_any = nonzero_kin or (rec["Y_raw"] != 0 or rec["Vy_raw"] != 0)

    if nonzero_kin or (flags_read and is_valid_bit13):
      if slot not in self.active_slots and len(self.active_slots) < MAX_ACTIVE_TARGETS:
        if rec["radar_track_id"] == 0:
          rec["radar_track_id"] = self._alloc_track_id_locked()
        self.active_slots[slot] = rec
        self._enqueue_init_locked(slot, moving=self._is_moving_target_locked(rec))
      elif slot in self.active_slots:
        self.active_slots[slot] = rec

    if slot in self.active_slots:
      if not nonzero_kin and flags_read and not is_valid_bit13:
        self._reset_slot_locked(slot)
      elif not nonzero_any and flags_read and not is_valid_bit13:
        self._reset_slot_locked(slot)
      elif flags_read and not is_valid_bit13 and rec["flags_updated_ns"] >= rec["kin_updated_ns"]:
        self._reset_slot_locked(slot)
      elif offset == 16 and not nonzero_kin and (slot, 106, 2) not in self._init_queued:
        task = (slot, 106, 2)
        self._init_queued.add(task)
        self._init_queue.append(task)

    if (
      offset == 16
      and size == 6
      and slot in self.active_slots
      and self._assist_shm_path is not None
      and self._assist_fast_publish_enabled
    ):
      if (self._op_matched_slots and slot == self._op_matched_slots[0]) or slot in self._select_priority_slots_locked()[:1]:
        self._assist_shm_dirty = True

  def _process_dto_locked(self, data: bytes, now_ns: int) -> None:
    if not data or self.in_flight is None:
      return
    pid = data[0]
    if pid not in (0xFF, 0xFE):
      return
    cmd_type, slot, offset, size, sent_ns = self.in_flight
    self.in_flight = None
    if pid == 0xFE:
      if cmd_type == "WAYB":
        self._wayb_state["last_err"] = data.hex()
        self._wayb_state["last_err_addr"] = hex(offset)
        self._wayb_state["updated_ns"] = now_ns
        self._wayb_queue.clear()
        return
      self._clear_session_locked("NEED_CONNECT")
      return
    self.consecutive_f4_timeouts = 0
    if cmd_type == "CONNECT":
      self.state = "NEED_STATUS"
    elif cmd_type == "STATUS":
      if len(data) >= 6 and (data[2] & 0x04) == 0:
        self.state = "READY"
      else:
        self.state = "NEED_SEED"
    elif cmd_type == "SEED":
      if len(data) >= 6:
        self.seed_bytes = bytes(data[2:6])
        self.state = "NEED_UNLOCK"
      else:
        self.state = "NEED_SEED"
    elif cmd_type == "UNLOCK":
      self.state = "READY"
    elif cmd_type == "F4":
      self._handle_f4_reply_locked(slot, offset, size, data, now_ns, sent_ns)
    elif cmd_type == "WAYB":
      self._handle_wayb_reply_locked(slot, size, data, now_ns)

  def _current_op_lead_tid_locked(self, now_ns: int) -> int | None:
    if now_ns - self._op_tracks_ns > OP_TRACKS_MAX_AGE_NS or not self._op_tracks:
      return None
    in_path = [o for o in self._op_tracks if o["dRel"] > 0.0 and abs(o["yRel"]) < 2.0]
    if not in_path:
      return None
    return int(min(in_path, key=lambda o: o["dRel"])["tid"])

  def _handle_wayb_reply_locked(self, tag: str, size: int, payload: bytes, now_ns: int) -> None:
    if len(payload) < 1 + size:
      return
    data = payload[1 : 1 + size]
    wb = self._wayb_state
    if tag == "p0" and size == 7:
      wb["p0_b2_b5"] = int.from_bytes(data[0:4], "big")
      p0_valid = int(data[4])
      p0_t1_slot = int(data[5])
      wb["p0_valid"] = p0_valid
      wb["p0_t1_slot"] = p0_t1_slot
      wb["p0_b8"] = int(data[6])
      cur_lead_tid = self._current_op_lead_tid_locked(now_ns)
      if cur_lead_tid is None or cur_lead_tid != self._wayb_lead_cam_id:
        self._wayb_lead_slot = None
        self._wayb_lead_cam_id = None
      wb["lead_slot"] = self._wayb_lead_slot
      wb["lead_cam_id"] = self._wayb_lead_cam_id
      if self._wayb_lead_slot is not None and 0 <= self._wayb_lead_slot < WAYB_T1_NUM_SLOTS:
        slot = self._wayb_lead_slot
        slot_source = "lead_scan"
      elif p0_valid != 0 and 0 <= p0_t1_slot < WAYB_T1_NUM_SLOTS:
        slot = p0_t1_slot
        slot_source = "p0"
      else:
        slot = 0
        slot_source = "idle0"
      wb["p0_slot_queried"] = slot
      wb["slot_source"] = slot_source
      wb["out18_scan"] = []
      wb["p_win"] = []
      wb["updated_ns"] = now_ns
      t1_slot_base = self._wayb_t1_base + WAYB_T1_STRIDE * slot
      out18_slot_base = self._wayb_out18_base + WAYB_OUT18_STRIDE * slot
      # Queue verified core reads first, then OUT18 lead scan & T1 struct dump, then p_win discovery last
      # so any 0xFE on p_win cannot drop core/scan/dump reads.
      for item in (
        ("t1_kin", t1_slot_base + 24, 6),
        ("t1_vx", t1_slot_base + 40, 4),
        ("t1_meta", t1_slot_base + 86, 6),
        ("out18", out18_slot_base + 16, 2),
      ):
        self._wayb_queue.append(item)
      for _ in range(WAYB_OUT18_SCAN_PER_ROUND):
        s = self._wayb_out18_cursor
        self._wayb_out18_cursor = (self._wayb_out18_cursor + 1) % WAYB_T1_NUM_SLOTS
        self._wayb_queue.append((f"o18s:{s}", self._wayb_out18_base + WAYB_OUT18_STRIDE * s + 16, 2))
      dump_off = self._wayb_t1_dump_off
      dump_sz = min(WAYB_T1_DUMP_CHUNK_SIZE, WAYB_T1_STRIDE - dump_off)
      next_off = dump_off + WAYB_T1_DUMP_CHUNK_SIZE
      self._wayb_t1_dump_off = next_off if next_off < WAYB_T1_STRIDE else 0
      self._wayb_queue.append((f"t1d:{slot}:{dump_off}", t1_slot_base + dump_off, dump_sz))
      for _ in range(WAYB_P_WIN_CHUNKS_PER_ROUND):
        chunk_idx = self._wayb_pwin_cursor
        self._wayb_pwin_cursor = (self._wayb_pwin_cursor + 1) % WAYB_P_WIN_NUM_CHUNKS
        addr = WAYB_P_WIN_START + chunk_idx * WAYB_P_WIN_CHUNK_SIZE
        self._wayb_queue.append((f"pwin:{addr:x}", addr, WAYB_P_WIN_CHUNK_SIZE))
    elif tag == "t1_kin" and size == 6:
      wb["t1_x_raw"] = int.from_bytes(data[0:2], "big", signed=True)
      wb["t1_u26"] = int.from_bytes(data[2:4], "big", signed=True)
      wb["t1_vx28_raw"] = int.from_bytes(data[4:6], "big", signed=True)
      wb["updated_ns"] = now_ns
    elif tag == "t1_vx" and size == 4:
      wb["t1_vx40_raw"] = int.from_bytes(data[0:2], "big", signed=True)
      wb["t1_vy42_raw"] = int.from_bytes(data[2:4], "big", signed=True)
      wb["updated_ns"] = now_ns
    elif tag == "t1_meta" and size == 6:
      wb["t1_flags"] = int.from_bytes(data[0:2], "big", signed=False)
      wb["t1_valid"] = int(data[5])
      wb["updated_ns"] = now_ns
    elif tag == "out18" and size == 2:
      cam_id = int(data[0])
      cam_flags = int(data[1])
      wb["out18_cam_id"] = cam_id
      wb["out18_cam_flags"] = cam_flags
      cur_lead_tid = self._current_op_lead_tid_locked(now_ns)
      if wb.get("slot_source") == "lead_scan":
        if cur_lead_tid is None or cam_id != cur_lead_tid:
          self._wayb_lead_slot = None
          self._wayb_lead_cam_id = None
          wb["lead_slot"] = None
          wb["lead_cam_id"] = None
      elif cur_lead_tid is not None and cam_id != 0 and cam_id == cur_lead_tid:
        q_slot = wb.get("p0_slot_queried")
        if isinstance(q_slot, int) and 0 <= q_slot < WAYB_T1_NUM_SLOTS:
          self._wayb_lead_slot = q_slot
          self._wayb_lead_cam_id = cam_id
          wb["lead_slot"] = q_slot
          wb["lead_cam_id"] = cam_id
      wb["updated_ns"] = now_ns
    elif tag.startswith("o18s:") and size == 2:
      scan_slot = int(tag.split(":", 1)[1])
      cam_id = int(data[0])
      cam_flags = int(data[1])
      wb["out18_scan"] = [*wb["out18_scan"], [scan_slot, cam_id, cam_flags]]
      cur_lead_tid = self._current_op_lead_tid_locked(now_ns)
      if cur_lead_tid is not None and cam_id != 0 and cam_id == cur_lead_tid:
        self._wayb_lead_slot = scan_slot
        self._wayb_lead_cam_id = cam_id
        wb["lead_slot"] = scan_slot
        wb["lead_cam_id"] = cam_id
      wb["updated_ns"] = now_ns
    elif tag.startswith("t1d:"):
      _, s_str, off_str = tag.split(":", 2)
      wb["t1_dump"] = [int(s_str), int(off_str), data.hex()]
      wb["updated_ns"] = now_ns
    elif tag.startswith("pwin:"):
      addr = int(tag.split(":", 1)[1], 16)
      wb["p_win"] = [*wb["p_win"], [hex(addr), data.hex()]]
      wb["updated_ns"] = now_ns

  def _primary_lead_urgent_locked(self, now_ns: int) -> bool:
    prio_slots = self._select_priority_slots_locked()
    if not prio_slots:
      return False
    rec = self.slot_cache[prio_slots[0]]
    return bool(
      now_ns - rec["kin_updated_ns"] >= LEAD_KIN_INTERVAL_NS
      or rec["age_updated_ns"] == 0
      or (rec["age"] < 3 and now_ns - rec["age_updated_ns"] >= NEWBORN_AGE_REFRESH_NS)
      or rec["flags_updated_ns"] == 0
    )

  def _next_wayb_task_locked(self, now_ns: int) -> tuple[str, int, int] | None:
    if not self._wayb_probe_enabled:
      return None
    if self._wayb_queue:
      return self._wayb_queue.popleft()
    if self._wayb_last_round_ns == 0 or now_ns - self._wayb_last_round_ns >= WAYB_PROBE_INTERVAL_NS:
      self._wayb_last_round_ns = now_ns
      return ("p0", self._wayb_p_base + 2, 7)
    return None

  def _next_inactive_probe_locked(self) -> tuple[int, int, int] | None:
    for _ in range(NUM_SLOTS):
      s = self.scan_cursor
      self.scan_cursor = (self.scan_cursor + 1) % NUM_SLOTS
      if s not in self.active_slots:
        return (s, 16, 6)
    return None

  def _select_priority_slots_locked(self) -> list[int]:
    prio: list[int] = []
    for s in self._op_matched_slots:
      if s in self.active_slots and s not in prio:
        prio.append(s)
        if len(prio) >= LEAD_PRIORITY_COUNT:
          return prio

    def _prio_key(item: tuple[int, dict]) -> tuple[int, int, int]:
      s, rec = item
      x_raw = rec["X_raw"]
      y_raw = rec["Y_raw"]
      is_moving = self._is_moving_target_locked(rec)
      in_path = bool(rec["subflags"] & 0x0400) or (rec["lat_updated_ns"] > 0 and x_raw > 0 and abs(y_raw) < 320)
      forward = x_raw > 0
      tier = 0 if (in_path and forward and is_moving) else (1 if (in_path and forward) else (2 if (forward and is_moving) else 3))
      return (tier, x_raw if forward else abs(x_raw) + 100_000, s)

    for s, _ in sorted(self.active_slots.items(), key=_prio_key):
      if s not in prio:
        prio.append(s)
        if len(prio) >= LEAD_PRIORITY_COUNT:
          break
    return prio

  def _pop_init_task_locked(self, *, allow_stationary: bool) -> tuple[int, int, int] | None:
    while self._init_queue:
      front = self._init_queue[0]
      if front not in self._init_queued or front[0] not in self.active_slots:
        self._init_queue.popleft()
        self._init_queued.discard(front)
        continue
      if not allow_stationary and not self._is_moving_target_locked(self.slot_cache[front[0]]):
        return None
      self._init_queue.popleft()
      self._init_queued.discard(front)
      if allow_stationary:
        self._stationary_init_skip_count = 0
      return front
    return None

  def _has_pending_init_locked(self) -> bool:
    while self._init_queue:
      front = self._init_queue[0]
      if front in self._init_queued and front[0] in self.active_slots:
        return True
      self._init_queue.popleft()
      self._init_queued.discard(front)
    return False

  def _stale_group_task_locked(
    self,
    group: list[int],
    now_ns: int,
    max_stale_ns: int,
    meta_stale_ns: int,
    lat_stale_ns: int | None = None,
    *,
    fast_newborn_age: bool = False,
  ) -> tuple[int, int, int] | None:
    if not group:
      return None
    best_ratio = 0.0
    best_task: tuple[int, int, int] | None = None

    def _consider(ratio: float, task: tuple[int, int, int]) -> None:
      nonlocal best_ratio, best_task
      if ratio > best_ratio:
        best_ratio = ratio
        best_task = task

    stale_kin_slot = min(group, key=lambda s: (self.slot_cache[s]["kin_updated_ns"], s))
    kin_age = now_ns - self.slot_cache[stale_kin_slot]["kin_updated_ns"]
    if kin_age >= max_stale_ns:
      _consider(kin_age / max_stale_ns, (stale_kin_slot, 16, 6))

    if lat_stale_ns is not None:
      stale_lat_slot = min(group, key=lambda s: (self.slot_cache[s]["lat_updated_ns"], s))
      lat_ts = self.slot_cache[stale_lat_slot]["lat_updated_ns"]
      if lat_ts == 0:
        _consider(2_000_000.0, (stale_lat_slot, 22, 6))
      else:
        lat_age = now_ns - lat_ts
        if lat_age >= lat_stale_ns:
          _consider(lat_age / lat_stale_ns, (stale_lat_slot, 22, 6))

    if fast_newborn_age:
      newborn = [
        s for s in group
        if self.slot_cache[s]["age"] < 3 and (
          self._is_moving_target_locked(self.slot_cache[s])
          or (self.slot_cache[s]["lat_updated_ns"] > 0 and abs(self.slot_cache[s]["Y_raw"]) < STATIONARY_IN_PATH_Y_RAW)
        )
      ]
      if newborn:
        stale_newborn = min(newborn, key=lambda s: (self.slot_cache[s]["age_updated_ns"], s))
        age_ts = self.slot_cache[stale_newborn]["age_updated_ns"]
        if age_ts == 0:
          _consider(1_000_000.0, (stale_newborn, 2, 6))
        else:
          nb_age = now_ns - age_ts
          if nb_age >= NEWBORN_AGE_REFRESH_NS:
            _consider(nb_age / NEWBORN_AGE_REFRESH_NS, (stale_newborn, 2, 6))

    if best_task is not None:
      return best_task

    stale_meta_slot = min(
      group,
      key=lambda s: (min(self.slot_cache[s]["age_updated_ns"], self.slot_cache[s]["flags_updated_ns"]), s),
    )
    rec = self.slot_cache[stale_meta_slot]
    meta_age = now_ns - min(rec["age_updated_ns"], rec["flags_updated_ns"])
    if meta_age >= meta_stale_ns:
      return (stale_meta_slot, 2, 6) if rec["age_updated_ns"] <= rec["flags_updated_ns"] else (stale_meta_slot, 106, 2)

    return None

  def _round_robin_task_locked(self, others: list[int], now_ns: int) -> tuple[int, int, int] | None:
    if not others:
      return None
    oldest_kin_slot = min(others, key=lambda s: (self.slot_cache[s]["kin_updated_ns"], s))
    rec = self.slot_cache[oldest_kin_slot]
    kin_ts = rec["kin_updated_ns"]
    if rec["flags_updated_ns"] == 0:
      return (oldest_kin_slot, 106, 2)
    if rec["age_updated_ns"] == 0 or (rec["age"] < 3 and now_ns - rec["age_updated_ns"] >= NEWBORN_AGE_REFRESH_NS):
      return (oldest_kin_slot, 2, 6)
    for field_key, interval_ns, task in (
      ("lat_updated_ns", OTHER_LAT_INTERVAL_NS, (oldest_kin_slot, 22, 6)),
      ("flags_updated_ns", OTHER_META_INTERVAL_NS, (oldest_kin_slot, 106, 2)),
      ("age_updated_ns", OTHER_META_INTERVAL_NS, (oldest_kin_slot, 2, 6)),
      ("qual_updated_ns", OTHER_QUAL_INTERVAL_NS, (oldest_kin_slot, 127, 2)),
    ):
      ts = rec[field_key]
      if ts == 0 or (now_ns - ts >= interval_ns and ts < kin_ts):
        return task
    return (oldest_kin_slot, 16, 6)

  def _next_read_task_locked(self, now_ns: int) -> tuple[int, int, int] | None:
    self._read_seq += 1

    prio_slots = self._select_priority_slots_locked()
    prio_set = set(prio_slots)

    # Primary lead (prio_slots[0]) sits strictly above init queue and inactive probing
    if prio_slots:
      s = prio_slots[0]
      rec = self.slot_cache[s]
      if now_ns - rec["kin_updated_ns"] >= LEAD_KIN_INTERVAL_NS:
        return (s, 16, 6)
      if rec["age_updated_ns"] == 0 or (rec["age"] < 3 and now_ns - rec["age_updated_ns"] >= NEWBORN_AGE_REFRESH_NS):
        return (s, 2, 6)
      if rec["flags_updated_ns"] == 0:
        return (s, 106, 2)
      if now_ns - max(rec["age_updated_ns"], rec["flags_updated_ns"]) >= LEAD_META_INTERVAL_NS:
        return (s, 2, 6) if rec["age_updated_ns"] <= rec["flags_updated_ns"] else (s, 106, 2)
      if now_ns - rec["lat_updated_ns"] >= LEAD_LAT_INTERVAL_NS:
        return (s, 22, 6)
      if now_ns - rec["qual_updated_ns"] >= LEAD_QUAL_INTERVAL_NS:
        return (s, 127, 2)

    # Adaptive inactive slot discovery (1/10 when tracking a matched lead, 1/3 if no lead or op_tracks lead is unmatched)
    has_unmatched_op_in_path = (
      not self._op_matched_slots
      and (now_ns - self._op_tracks_ns <= OP_TRACKS_MAX_AGE_NS)
      and any(o["dRel"] > 0.0 and abs(o["yRel"]) < 2.0 for o in self._op_tracks)
    )
    probe_period = (
      INACTIVE_PROBE_PERIOD_NO_LEAD
      if (not prio_slots or has_unmatched_op_in_path)
      else INACTIVE_PROBE_PERIOD_WITH_LEAD
    )
    if (self._read_seq - self._last_probe_seq) >= probe_period and len(self.active_slots) < NUM_SLOTS:
      probe = self._next_inactive_probe_locked()
      if probe is not None:
        self._last_probe_seq = self._read_seq
        return probe

    # Newly promoted moving slots run immediately ahead of other traffic
    if task := self._pop_init_task_locked(allow_stationary=False):
      return task

    lead1 = prio_slots[0] if prio_slots else -1
    others_all = [s for s in sorted(self.active_slots) if s != lead1]
    moving_others = [s for s in others_all if self._is_moving_target_locked(self.slot_cache[s])]
    stationary_others = [s for s in others_all if s not in moving_others]
    near_moving_others = [
      s for s in moving_others
      if self.slot_cache[s]["lat_updated_ns"] == 0 or abs(self.slot_cache[s]["Y_raw"]) <= NEAR_MOVING_Y_RAW
    ]
    far_moving_others = [s for s in moving_others if s not in near_moving_others]

    # Periodic anti-starvation yield so standing traffic ahead still initializes Y/age/flags in busy moving traffic
    if self._has_pending_init_locked():
      if self._stationary_init_skip_count >= STATIONARY_INIT_YIELD_READS or not near_moving_others:
        if task := self._pop_init_task_locked(allow_stationary=True):
          return task
      self._stationary_init_skip_count += 1
    else:
      self._stationary_init_skip_count = 0

    # Near-path / adjacent moving lanes stay fresh on both kinematics and Y/Vy (wide oncoming traffic separated below)
    if task := self._stale_group_task_locked(
      near_moving_others, now_ns, OTHER_MAX_STALE_NS, OTHER_META_MAX_STALE_NS,
      lat_stale_ns=OTHER_LAT_INTERVAL_NS, fast_newborn_age=True,
    ):
      return task

    # Secondary priority lead (prio_slots[1]): ~10 Hz kinematics, ~6.7 Hz Y/Vy, slower age/flags/qual
    if len(prio_slots) > 1:
      s = prio_slots[1]
      rec = self.slot_cache[s]
      if now_ns - rec["kin_updated_ns"] >= LEAD2_KIN_INTERVAL_NS:
        return (s, 16, 6)
      if rec["age_updated_ns"] == 0 or (rec["age"] < 3 and now_ns - rec["age_updated_ns"] >= NEWBORN_AGE_REFRESH_NS):
        return (s, 2, 6)
      if rec["flags_updated_ns"] == 0:
        return (s, 106, 2)
      if rec["lat_updated_ns"] == 0 or now_ns - rec["lat_updated_ns"] >= LEAD2_LAT_INTERVAL_NS:
        return (s, 22, 6)
      if now_ns - max(rec["age_updated_ns"], rec["flags_updated_ns"]) >= LEAD2_META_INTERVAL_NS:
        return (s, 2, 6) if rec["age_updated_ns"] <= rec["flags_updated_ns"] else (s, 106, 2)
      if rec["qual_updated_ns"] == 0 or now_ns - rec["qual_updated_ns"] >= OTHER_QUAL_INTERVAL_NS:
        return (s, 127, 2)

    if task := self._stale_group_task_locked(
      far_moving_others, now_ns, FAR_OTHER_KIN_INTERVAL_NS, FAR_OTHER_META_INTERVAL_NS,
      lat_stale_ns=FAR_OTHER_LAT_INTERVAL_NS,
    ):
      return task

    # Near-path stationary newborn slots refresh age >= 3 promptly; bulk roadside clutter stays at low cadence
    if task := self._stale_group_task_locked(
      stationary_others, now_ns, STATIONARY_MAX_STALE_NS, 2 * STATIONARY_MAX_STALE_NS, fast_newborn_age=True,
    ):
      return task

    if task := self._round_robin_task_locked([s for s in near_moving_others if s not in prio_set], now_ns):
      return task
    if task := self._round_robin_task_locked([s for s in far_moving_others if s not in prio_set], now_ns):
      return task

    if task := self._pop_init_task_locked(allow_stationary=True):
      return task

    # If no active slots need updating right now, sweep inactive slots faster
    if len(self.active_slots) < NUM_SLOTS:
      probe = self._next_inactive_probe_locked()
      if probe is not None:
        self._last_probe_seq = self._read_seq
        return probe

    if prio_slots:
      return (prio_slots[0], 16, 6)
    return None

  def _build_next_cto_locked(self, now_ns: int) -> bytes | None:
    if not self._tx_armed:
      return None
    if now_ns - self._tx_armed_ns < STARTUP_DELAY_NS:
      return None

    if self.in_flight is not None:
      cmd_type, _, _, _, sent_ns = self.in_flight
      timeout = F4_TIMEOUT_NS if cmd_type in ("F4", "WAYB") else HANDSHAKE_RETRY_NS
      if now_ns - sent_ns < timeout:
        return None
      self.in_flight = None
      if cmd_type in ("F4", "WAYB"):
        self.consecutive_f4_timeouts += 1
        if self.consecutive_f4_timeouts >= 5:
          self._clear_session_locked("NEED_CONNECT")
          return None
      else:
        self._clear_session_locked("NEED_CONNECT")
        return None

    if self.state == "NEED_CONNECT":
      if now_ns - self.last_handshake_tx_ns < HANDSHAKE_RETRY_NS:
        return None
      self.last_handshake_tx_ns = now_ns
      self.in_flight = ("CONNECT", -1, 0, 0, now_ns)
      return b"\xFF\x00" + bytes(6)

    if self.state == "NEED_STATUS":
      self.in_flight = ("STATUS", -1, 0, 0, now_ns)
      return b"\xFD" + bytes(7)

    if self.state == "NEED_SEED":
      self.in_flight = ("SEED", -1, 0, 0, now_ns)
      return b"\xF8\x00\x04" + bytes(5)

    if self.state == "NEED_UNLOCK":
      if self.seed_bytes is None or len(self.seed_bytes) != 4:
        self.state = "NEED_SEED"
        return None
      key = key_for_seed(self.seed_bytes)
      self.seed_bytes = None
      self.in_flight = ("UNLOCK", -1, 0, 0, now_ns)
      return b"\xF7\x04" + key + bytes(2)

    if self.state == "READY":
      if not self._primary_lead_urgent_locked(now_ns):
        wb_task = self._next_wayb_task_locked(now_ns)
        if wb_task is not None:
          tag, addr, size = wb_task
          pkt = bytes([0xF4, size, 0x00, 0x00]) + addr.to_bytes(4, "big")
          self.in_flight = ("WAYB", tag, addr, size, now_ns)
          return pkt
      task = self._next_read_task_locked(now_ns)
      if task is None:
        return None
      slot, offset, size = task
      addr = REC0 + STRIDE * slot + offset
      pkt = bytes([0xF4, size, 0x00, 0x00]) + addr.to_bytes(4, "big")
      self.in_flight = ("F4", slot, offset, size, now_ns)
      return pkt

    return None

  def _send_cto(self, pkt: bytes) -> None:
    if self._pm is None:
      return
    from openpilot.selfdrive.pandad import can_list_to_can_capnp

    frame = can_list_to_can_capnp([(XCP_CTO, pkt, XCP_BUS)], msgtype="sendcan", valid=True)
    if self._send_lock is not None:
      with self._send_lock:
        self._pm.send("sendcan", frame)
    else:
      self._pm.send("sendcan", frame)

  def _io_loop(self) -> None:
    import openpilot.cereal.messaging as messaging
    from openpilot.selfdrive.pandad import can_capnp_to_list

    can_sock = None
    try:
      while not self._stop_event.is_set() and not self._tx_armed:
        self._tx_event.wait(timeout=0.05)
        self._tx_event.clear()

      if self._stop_event.is_set():
        return

      can_sock = messaging.sub_sock("can", timeout=5)
      while not self._stop_event.is_set():
        can_strs = messaging.drain_sock_raw(can_sock, wait_for_one=True)
        now_ns = time.monotonic_ns()
        if can_strs:
          can_list = can_capnp_to_list(can_strs)
          with self._lock:
            for _, packet in can_list:
              for addr, dat, src in packet:
                if src == XCP_BUS and addr == XCP_DTO:
                  self._process_dto_locked(bytes(dat), now_ns)
            pkt = self._build_next_cto_locked(now_ns)
        else:
          with self._lock:
            pkt = self._build_next_cto_locked(now_ns)
        if pkt is not None:
          self._send_cto(pkt)
        if can_strs:
          self._maybe_publish_assist_shm(now_ns)
    except Exception:
      pass
    finally:
      if can_sock is not None:
        try:
          del can_sock
        except Exception:
          pass

  def _match_openpilot_tracks(self, targets: list[dict], op_tracks: list[dict], v_ego: float = 0.0) -> tuple[list[int], int | None]:
    with self._lock:
      if not targets or not op_tracks:
        self._prev_op_slot_by_tid = {}
        return [], None
      prev_slot_by_tid = self._prev_op_slot_by_tid
      candidates: list[tuple[float, int, int, float]] = []
      for ti, t in enumerate(targets):
        if int(t.get("lat_updated_ns", 1)) <= 0:
          continue
        is_stationary = v_ego >= 3.0 and abs(v_ego + t["vRel_mps"]) < 0.7
        for oi, o in enumerate(op_tracks):
          dd = abs(t["dRel_m"] - o["dRel"])
          dy = abs(t["yRel_m"] - o["yRel"])
          dv = abs(t["vRel_mps"] - o["vRel"])
          if is_stationary:
            dd_eff = dd
            max_dd = 4.5
          else:
            allow_trailer = t["dRel_m"] >= o["dRel"] and o["dRel"] >= 25.0 and dy < 1.0
            dd_eff = min(dd, abs((t["dRel_m"] - ARRAY_B_TRAILER_OFFSET_M) - o["dRel"])) if allow_trailer else dd
            r = max(t["dRel_m"], o["dRel"], 0.0)
            max_dd = max(5.5, (0.20 if r > 70.0 else 0.14) * r)
          prev_matched_same = prev_slot_by_tid.get(o["tid"]) == t["slot"]
          closing_lag = o["vRel"] - t["vRel_mps"]
          rail_ok = o["vRel"] <= -11.5 and t["vRel_mps"] <= o["vRel"] + 1.0 and dv <= 25.0
          locked_closing_ok = (
            prev_matched_same
            and not is_stationary
            and (v_ego < 3.0 or (v_ego + t["vRel_mps"]) >= OP_MATCH_LOCKED_MIN_VLEAD_MPS)
            and t["yRel_m"] != 0.0
            and abs(o["yRel"]) <= OP_MATCH_LOCKED_CLOSING_MAX_ABS_Y_M
            and dy < OP_MATCH_LOCKED_CLOSING_MAX_DY_M
            and 0.0 < closing_lag <= OP_MATCH_LOCKED_CLOSING_MAX_DV_MPS
          )
          if dd_eff < max_dd and dy < 2.2 and (dv < 3.5 or rail_ok or locked_closing_ok):
            cost = dd_eff * dd_eff + (2.0 * dy) * (2.0 * dy) + (0.8 * dv) * (0.8 * dv)
            dist_m = math.hypot(dd, dy)
            candidates.append((cost, ti, oi, dist_m))
      candidates.sort(key=lambda item: item[0])

      used_targets: set[int] = set()
      used_op: set[int] = set()
      op_in_path = [o for o in op_tracks if o["dRel"] > 0.0 and abs(o["yRel"]) < 2.0]
      op_lead_tid = min(op_in_path, key=lambda o: o["dRel"])["tid"] if op_in_path else None
      lead_matched_slots: list[int] = []
      matched_slots: list[int] = []
      op_lead_slot: int | None = None
      new_op_slot_by_tid: dict[int, int] = {}

      for _, ti, oi, dist_m in candidates:
        if ti in used_targets or oi in used_op:
          continue
        used_targets.add(ti)
        used_op.add(oi)
        tid = op_tracks[oi]["tid"]
        targets[ti]["op_track_id"] = tid
        targets[ti]["op_dRel"] = op_tracks[oi]["dRel"]
        targets[ti]["op_match_dist_m"] = round(dist_m, 3)
        slot = targets[ti]["slot"]
        new_op_slot_by_tid[tid] = slot
        if tid == op_lead_tid:
          lead_matched_slots.insert(0, slot)
          op_lead_slot = slot
        else:
          matched_slots.append(slot)
      self._prev_op_slot_by_tid = new_op_slot_by_tid
      return lead_matched_slots + matched_slots, op_lead_slot

  def _collect_targets_locked(self, now_ns: int) -> tuple[float, list[dict], list[dict], int | None]:
    v_ego = self.last_v_ego
    op_tracks = list(self._op_tracks) if (now_ns - self._op_tracks_ns <= OP_TRACKS_MAX_AGE_NS) else []

    targets: list[dict] = []
    for slot, rec in self.active_slots.items():
      if rec["flags_updated_ns"] <= 0:
        continue
      flags = rec["flags"]
      if (flags & 0x2000) == 0:
        continue
      X = rec["X_raw"]
      Y = rec["Y_raw"]
      Vx = rec["Vx_raw"]
      Vy = rec["Vy_raw"]
      Ax = rec["Ax_raw"]
      if X == 0 and Y == 0 and Vx == 0 and Vy == 0:
        continue
      subflags = rec["subflags"]
      age = rec["age"]
      dec = decode_flags(flags, subflags, age)
      kin_age_ns = max(0, now_ns - rec["kin_updated_ns"])
      kin_age_ms = round(kin_age_ns / 1_000_000.0, 2)
      is_fresh_kin = rec["kin_updated_ns"] > 0 and kin_age_ns <= MEASURED_MAX_KIN_AGE_NS
      if dec["stage"] == "measured" and not is_fresh_kin:
        stage = "stale"
        measured = False
        trust_accel = False
      else:
        stage = dec["stage"]
        measured = dec["measured"]
        trust_accel = dec["trust_accel"]
      targets.append({
        "slot": slot,
        "radar_track_id": rec["radar_track_id"],
        "dRel_m": round(X / 128.0, 4),
        "vRel_mps": round(Vx / 256.0, 4),
        "yRel_m": round(Y / 128.0, 4),
        "vyRel_mps": round(Vy / 256.0, 4),
        "ax_mps2": round(Ax / 2048.0, 4),
        "flags": flags,
        "subflags": subflags,
        "age": age,
        "qual": rec["qual"],
        "cls_conf": rec["cls_conf"],
        "stage": stage,
        "fw_stage": dec["stage"],
        "measured": measured,
        "conf_band": dec["conf_band"],
        "ghost": dec["ghost"],
        "in_path": dec["in_path"],
        "trust_accel": trust_accel,
        "kin_age_ms": kin_age_ms,
        "op_track_id": None,
        "op_dRel": None,
        "op_match_dist_m": None,
        "kin_updated_ns": rec["kin_updated_ns"],
        "lat_updated_ns": rec["lat_updated_ns"],
        "age_updated_ns": rec["age_updated_ns"],
        "flags_updated_ns": rec["flags_updated_ns"],
        "updated_ns": rec["updated_ns"],
        "last_rtt_ms": round(rec["last_rtt_ns"] / 1_000_000.0, 2),
      })

    matched_slots, op_lead_slot = self._match_openpilot_tracks(targets, op_tracks, v_ego)
    self._op_matched_slots = matched_slots
    return v_ego, op_tracks, targets, op_lead_slot

  def _maybe_log_20hz(self, now_ns: int) -> None:
    with self._lock:
      if self.state != "READY":
        return
      if now_ns - self.last_log_ns < EPOCH_NS:
        return
      self.last_log_ns = now_ns
      v_ego, op_tracks, targets, op_lead_slot = self._collect_targets_locked(now_ns)
      wayb_probe = (
        dict(self._wayb_state)
        if self._wayb_probe_enabled and self._wayb_state["updated_ns"] > 0
        else None
      )
      if self._assist_shm_path is not None:
        self._last_assist_shm_ns = now_ns
        self._assist_shm_dirty = False

    targets.sort(key=lambda t: (abs(t["dRel_m"]), t["slot"]))
    matched_lead = next((t for t in targets if t["slot"] == op_lead_slot), None) if op_lead_slot is not None else None
    if matched_lead is not None:
      lead = dict(matched_lead)
      lead["lead_source"] = "op_lead"
    else:
      in_path = [t for t in targets if t["dRel_m"] > 0.0 and abs(t["yRel_m"]) < 2.0]
      if v_ego >= 3.0:
        moving_in_path = [t for t in in_path if abs(v_ego + t["vRel_mps"]) >= 1.5]
        fallback = min(moving_in_path or in_path, key=lambda t: t["dRel_m"]) if in_path else None
      else:
        fallback = min(in_path, key=lambda t: t["dRel_m"]) if in_path else None
      if fallback is not None:
        lead = dict(fallback)
        lead["lead_source"] = "xcp_fallback"
      else:
        lead = None
    event_kwargs = dict(
      mono_ns=now_ns,
      v_ego=round(v_ego, 3),
      lead=lead,
      targets=targets,
      op_tracks=op_tracks,
      debug=True,
    )
    if wayb_probe is not None:
      event_kwargs["wayb_probe"] = wayb_probe
    cloudlog.event("xcp_array_b_20hz", **event_kwargs)
    self._write_ui_snapshot(now_ns, v_ego, targets)

  @staticmethod
  def _build_assist_payload(now_ns: int, targets: list[dict]) -> dict:
    tracks_by_op_id: dict[str, dict] = {}
    shm_targets: list[dict] = []
    for t in targets:
      if (
        t.get("stage") == "measured"
        and not t.get("ghost", False)
        and int(t.get("lat_updated_ns", 1)) > 0
      ):
        entry = {
          "slot": t["slot"],
          "rtid": t["radar_track_id"],
          "age": t["age"],
          "stage": t["stage"],
          "ghost": t["ghost"],
          "flags": t["flags"],
          "dRel_m": t["dRel_m"],
          "yRel_m": t["yRel_m"],
          "vRel_mps": t["vRel_mps"],
          "ax_mps2": t["ax_mps2"],
          "kin_age_ms": t["kin_age_ms"],
          "kin_updated_ns": int(t.get("kin_updated_ns", 0)),
          "lat_valid": True,
          "trust_accel": t["trust_accel"],
          "op_track_id": t.get("op_track_id"),
        }
        shm_targets.append(entry)
        op_tid = t.get("op_track_id")
        if op_tid is not None:
          tracks_by_op_id[str(op_tid)] = {k: v for k, v in entry.items() if k != "op_track_id"}
    return {
      "mono_ns": int(now_ns),
      "boot_offset_ns": int(_boot_minus_mono_ns()),
      "tracks_by_op_id": tracks_by_op_id,
      "targets": shm_targets,
    }

  def _maybe_publish_assist_shm(self, now_ns: int) -> None:
    with self._lock:
      if (
        self._assist_shm_path is None
        or not self._assist_fast_publish_enabled
        or self.state != "READY"
        or not self._assist_shm_dirty
        or (now_ns - self._last_assist_shm_ns) < ASSIST_SHM_MIN_INTERVAL_NS
      ):
        return
      self._assist_shm_dirty = False
      self._last_assist_shm_ns = now_ns
      _, _, targets, _ = self._collect_targets_locked(now_ns)
      assist_path = self._assist_shm_path
    try:
      _write_atomic_json(assist_path, self._build_assist_payload(now_ns, targets))
    except Exception:
      pass

  def _write_ui_snapshot(self, now_ns: int, v_ego: float, targets: list[dict]) -> None:
    if self._shm_path is None and self._assist_shm_path is None:
      return
    try:
      if self._shm_path is not None:
        points: list[list[float]] = []
        for t in targets:
          if t.get("lat_updated_ns", 1) <= 0 or t.get("ghost", False) or t.get("age", 99) < 3:
            continue
          dt_kin = min(max(now_ns - t.get("kin_updated_ns", now_ns), 0), 300_000_000) / 1e9
          dt_lat = min(max(now_ns - t.get("lat_updated_ns", now_ns), 0), 300_000_000) / 1e9
          d_ui = t["dRel_m"] + t.get("vRel_mps", 0.0) * dt_kin
          y_ui = t["yRel_m"] + t.get("vyRel_mps", 0.0) * dt_lat
          if d_ui > 0.0 and (v_ego < 3.0 or abs(v_ego + t["vRel_mps"]) >= 1.5 or abs(y_ui) < 2.2):
            points.append([round(d_ui, 2), round(y_ui, 2)])
        _write_atomic_json(self._shm_path, {"mono_ns": int(now_ns), "points": points})
      if self._assist_shm_path is not None:
        _write_atomic_json(self._assist_shm_path, self._build_assist_payload(now_ns, targets))
    except Exception:
      pass

  def observe_radar_tracks(self, RD, now_ns: int) -> None:
    if not self.enabled or RD is None:
      return
    try:
      tracks = [
        {
          "tid": int(pt.trackId),
          "dRel": round(float(pt.dRel), 3),
          "yRel": round(float(pt.yRel), 3),
          "vRel": round(float(pt.vRel), 3),
        }
        for pt in RD.points
      ]
      with self._lock:
        self._op_tracks = tracks
        self._op_tracks_ns = int(now_ns)
    except Exception:
      pass

  def observe_can(self, can_list, now_ns: int, v_ego: float = 0.0) -> None:
    if not self.enabled:
      return
    try:
      with self._lock:
        self.last_v_ego = float(v_ego)
        if not self._use_io_thread:
          for _, packet in can_list:
            for addr, dat, src in packet:
              if src == XCP_BUS and addr == XCP_DTO:
                self._process_dto_locked(bytes(dat), now_ns)
      self._maybe_log_20hz(now_ns)
      if not self._use_io_thread:
        self._maybe_publish_assist_shm(now_ns)
    except Exception:
      pass

  def next_can_sends(self, now_ns: int) -> list[CanData]:
    if not self.enabled:
      return []
    try:
      with self._lock:
        if not self._tx_armed:
          self._tx_armed = True
          self._tx_armed_ns = int(now_ns)
          self._tx_event.set()
        if self._use_io_thread:
          return []
        pkt = self._build_next_cto_locked(now_ns)
        return [CanData(XCP_CTO, pkt, XCP_BUS)] if pkt is not None else []
    except Exception:
      pass
    return []
