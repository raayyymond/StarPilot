"""Fork gate around the Bosch-A Array B logger (xcp_radar_logger.py, from bob-unify-car; the one fork change in it
is that the kinematic read offsets are module values, set here per firmware).

The logger reads the radar's internal track list at 20 Hz and only records it (rlog event xcp_array_b_20hz).
This module decides whether it runs at all, and with which layout:

  - only when the master toggle (AlphaHondaXcpRadarAssist) is on. That one switch gates both the polling here and the
    fusion into long control in radard, so XCP is read and used together or not at all; off, the radar is sent nothing.
    The radar must also have XCP enabled on its own side first (a bench UDS write, kept in the firmware kit), or it
    never answers;
  - only on a verified Bosch-A car (the logger itself would start on every Bosch Honda with a radar);
  - only for a radar firmware with an Array B address to read. The address and the record layout move between
    builds (A150 0x400371A4, A160 0x400371C4, the Accord's TVA-A170 0x40034F94 with an 8-byte header), so a firmware
    with no entry is logged and not polled;
  - a firmware whose address is not known yet can list several candidates, and the recorder takes them in turn, one
    slice each, with an array_b_recorder_layout event at every change. No firmware does today;
  - a candidate can also be a raw memory window (xcp_raw_window.py), recorded as bytes for offline decoding when the
    record format itself is in question. No firmware uses one today; it is kept as the tool for the next unknown;
  - the 1 Hz internal-fusion probe only on the firmware its addresses were read from;
  - it stops for the rest of the drive after repeated lost sessions, or when the radar never answers;
  - twice a second it also records the logger's whole slot table, unfiltered (rlog event xcp_array_b_slots). The 20 Hz
    event only carries slots that passed the logger's validity rules, so it cannot show why a car is missing from it:
    on the first Accord drives (2026-11-22) the lead had no track for 10-40 s at a time while the car's own object
    list held it. The table shows each slot's raw words, how old each read is, whether the logger counts it active,
    and how often it was dropped and with which flags;
  - what is read when is array_b_read_order.py's: the slots that answer to radard's lead first, the rest of the list by
    what each slot is to long control. The table event carries the vision lead and the slots chosen for it;
  - the radar's CONNECT reply is logged once (array_b_recorder_connect): its resource byte says whether this firmware
    offers XCP DAQ, which the panda does not let through today.
"""
from __future__ import annotations

from openpilot.common.swaglog import cloudlog

from typing import NamedTuple

RADAR_ADDRESS = 0x18DAB0F1

A150_BASE = 0x400371A4  # live on the Civic (Peter)
A160_BASE = 0x400371C4  # static, from the Civic A160 image
A170_BASE = 0x40034F94  # live on the Accord (36802-TVA-A170), verified 2026-11-21 via XCP

# Record layout per firmware: (header_size, kin_offset, lat_offset).
#   header_size: REC0 = base + header_size (offset of the first record from the array base).
#   kin_offset:  SHORT_UPLOAD offset within a record for X/Vx/Ax.
#   lat_offset:  SHORT_UPLOAD offset within a record for Y/raw_sentinel/Vy.
# Civic TBA (A150/A160): 16-byte header, kinematics at +16/+22.
# Accord TVA (A170): 8-byte header, every kinematic channel shifted +2 -> +18/+24. A170 is the
#   R6.88 predecessor of the A180 image whose static map pins this same layout at 0x40034FB4
#   (A170 verified live at A180's base shifted -0x20).
_CIVIC_LAYOUT = (0x10, 16, 22)
_TVA_LAYOUT = (0x08, 18, 24)

ARRAY_B_LAYOUTS_BY_FIRMWARE: dict[bytes, tuple[int, int, int]] = {
  b"36802-TBA-A150": _CIVIC_LAYOUT,
  b"36802-TBA-A160": _CIVIC_LAYOUT,
  b"36802-TVA-A170": _TVA_LAYOUT,
}


class RawWindow(NamedTuple):  # same fields as xcp_raw_window.RawWindow, which is only imported once a car needs it
  address: int
  length: int
  chunk: int


# radar firmware -> Array B base(s) or raw windows. More than one: not known yet, recorded in turn.
ARRAY_B_BASES_BY_FIRMWARE: dict[bytes, tuple[int | RawWindow, ...]] = {
  b"36802-TBA-A150": (A150_BASE,),
  b"36802-TBA-A160": (A160_BASE,),
  b"36802-TVA-A170": (A170_BASE,),
}
INTERNAL_FUSION_PROBE_FIRMWARE = frozenset({b"36802-TBA-A150"})

SLICE_NS = 120_000_000_000              # time on one candidate address before the next
SLOT_TABLE_NS = 500_000_000             # unfiltered slot table cadence
MAX_SESSION_LOSSES = 5                  # sessions that reached READY and were dropped
CONNECT_DEADLINE_NS = 30_000_000_000    # armed this long without one READY session


def radar_firmware(CP) -> bytes | None:
  for fw in getattr(CP, "carFw", ()):
    if str(fw.ecu).rsplit(".", 1)[-1] == "fwdRadar" and int(fw.address) == RADAR_ADDRESS and not bool(fw.logging):
      return bytes(fw.fwVersion).rstrip(b"\x00")
  return None


def supported(CP) -> bool:
  from opendbc.car.honda.bosch_a_radar import VERIFIED_BOSCH_A_CARS
  return (getattr(CP, "brand", None) == "honda" and CP.carFingerprint in VERIFIED_BOSCH_A_CARS
          and not bool(CP.radarUnavailable) and not bool(CP.passive))


def _xcp_enabled(CP, params) -> bool:
  """The master toggle. It gates the polling here and, in radard, the fusion into control: XCP reads and is used
  together, or not at all. Off means we send the radar nothing. Fail-closed, and never raises."""
  try:
    from openpilot.starpilot.car.honda.xcp_radar_assist import enabled as xcp_assist_enabled
    return bool(xcp_assist_enabled(params, CP))
  except Exception:
    cloudlog.exception("array_b_recorder: toggle read failed")
    return False


def _age_ms(now_ns: int, stamp_ns: int) -> int:
  return -1 if stamp_ns <= 0 else int((now_ns - stamp_ns) // 1_000_000)


class SlotTable:
  """The logger's per-slot state as it stands, for the xcp_array_b_slots event. Reads only; sends nothing."""

  def __init__(self, logger):
    self.logger = logger
    self.drops: dict[int, list[int]] = {}   # slot -> [times dropped, flags at the last drop, X at the last drop]
    self.last_ns = 0
    inner = logger._reset_slot_locked

    def reset(slot: int) -> None:
      rec = logger.slot_cache[slot]
      seen = self.drops.setdefault(slot, [0, 0, 0])
      seen[0] += 1
      seen[1], seen[2] = int(rec["flags"]), int(rec["X_raw"])
      inner(slot)
    logger._reset_slot_locked = reset

  def rows(self, now_ns: int) -> dict:
    logger = self.logger
    with logger._lock:
      slots = []
      for slot, rec in sorted(logger.slot_cache.items()):
        drops = self.drops.get(slot, (0, 0, 0))
        slots.append([
          slot, int(slot in logger.active_slots),
          rec["X_raw"], rec["Vx_raw"], rec["Ax_raw"], rec["Y_raw"], rec["Vy_raw"],
          rec["age"], rec["subflags"], rec["flags"], rec["qual"],
          _age_ms(now_ns, rec["kin_updated_ns"]), _age_ms(now_ns, rec["lat_updated_ns"]),
          _age_ms(now_ns, rec["age_updated_ns"]), _age_ms(now_ns, rec["flags_updated_ns"]),
          drops[0], drops[1], drops[2],
        ])
      return dict(mono_ns=int(now_ns), v_ego=round(float(logger.last_v_ego), 3), reads=int(logger._read_seq),
                  init_queue=len(logger._init_queue), priority=list(logger._select_priority_slots_locked()),
                  op_matched=list(logger._op_matched_slots), slots=slots)

  def log(self, now_ns: int, more=dict) -> None:
    """`more` gives extra fields, asked for after the rows (reading the priority list settles the read order)."""
    if self.logger.state != "READY" or now_ns - self.last_ns < SLOT_TABLE_NS:
      return
    self.last_ns = int(now_ns)
    cloudlog.event("xcp_array_b_slots", **self.rows(now_ns), **more())


def slot_table(logger) -> SlotTable | None:
  """A table for a logger that keeps one (the raw window logger and a broken one do not)."""
  try:
    return SlotTable(logger) if hasattr(logger, "slot_cache") else None
  except Exception:
    cloudlog.exception("array_b_recorder: no slot table")
    return None


class ModelLead:
  """The model's first lead as (d, y, v, prob), y left positive like the radar's; None when no new frame came."""

  def __init__(self):
    import openpilot.cereal.messaging as messaging
    self.recv = messaging.recv_one_or_none
    self.sock = messaging.sub_sock("modelV2", conflate=True)

  def __call__(self) -> tuple[float, float, float, float] | None:
    msg = self.recv(self.sock)
    if msg is None:
      return None
    leads = msg.modelV2.leadsV3
    if len(leads) == 0:
      return (0.0, 0.0, 0.0, 0.0)
    lead = leads[0]
    return (float(lead.x[0]), -float(lead.y[0]), float(lead.v[0]), float(lead.prob))

  def close(self) -> None:
    self.sock = None


def read_order(logger):
  """The reading order for a logger that keeps a slot table, or None (the logger then reads in its own order)."""
  try:
    if not hasattr(logger, "slot_cache"):
      return None
    from openpilot.starpilot.car.honda import array_b_read_order, xcp_radar_logger
    return array_b_read_order.ReadOrder(logger, xcp_radar_logger)
  except Exception:
    cloudlog.exception("array_b_recorder: no read order")
    return None


def log_connect_reply(logger) -> None:
  """Log the radar's first CONNECT reply. Reads what already arrives; sends nothing."""
  if not hasattr(logger, "_process_dto_locked"):
    return
  inner = logger._process_dto_locked
  seen = []

  def process(data: bytes, now_ns: int) -> None:
    try:
      if not seen and data and data[0] == 0xFF and logger.in_flight is not None and logger.in_flight[0] == "CONNECT":
        seen.append(True)
        cloudlog.event("array_b_recorder_connect", reply=bytes(data).hex(), daq=bool(len(data) > 1 and data[1] & 0x04))
    except Exception:
      seen.append(True)
    inner(data, now_ns)
  logger._process_dto_locked = process


class ArrayBRecorder:
  """The logger's card-facing surface, plus the candidate rotation and the give-up rule."""

  def __init__(self, make_logger, bases: tuple[int, ...], lead_source=None):
    self.make_logger = make_logger   # base -> a started logger
    self.bases = bases
    self.lead_source = lead_source   # () -> (d, y, v, prob) of the vision lead, or None when nothing new
    self.index = 0
    self.logger = make_logger(bases[0])
    self._dress()
    self.slice_start_ns = 0
    self.armed_ns = 0
    self.was_ready = False
    self.ever_ready = False
    self.session_losses = 0
    self.stopped = False

  def _dress(self) -> None:
    self.slot_table = slot_table(self.logger)
    self.order = read_order(self.logger)
    try:
      log_connect_reply(self.logger)
    except Exception:
      cloudlog.exception("array_b_recorder: no connect log")

  def _see_lead(self, now_ns: int) -> None:
    if self.order is None:
      return
    lead = None
    if self.lead_source is not None:
      try:
        lead = self.lead_source()
      except Exception:
        cloudlog.exception("array_b_recorder: vision lead failed")       # the order goes on from the CAN tracks alone
        self.lead_source = None
    with self.logger._lock:
      self.order.observe(lead, now_ns)

  def _stop(self, reason: str) -> None:
    self.stopped = True
    self.logger.enabled = False
    self.logger.close()
    cloudlog.event("array_b_recorder_stopped", reason=reason, session_losses=self.session_losses,
                   ever_ready=self.ever_ready)

  def _next_layout(self, now_ns: int) -> None:
    self.logger.enabled = False
    self.logger.close()
    self.index = (self.index + 1) % len(self.bases)
    self.logger = self.make_logger(self.bases[self.index])
    self._dress()
    self.slice_start_ns = int(now_ns)
    self.was_ready = False
    log_layout(self.bases[self.index], now_ns)

  def _watch(self, now_ns: int) -> None:
    if self.stopped:
      return
    ready = self.logger.state == "READY"
    if ready:
      self.ever_ready = True
    elif self.was_ready:
      self.session_losses += 1
    self.was_ready = ready
    never_answered = self.armed_ns and not self.ever_ready and now_ns - self.armed_ns > CONNECT_DEADLINE_NS
    if self.session_losses >= MAX_SESSION_LOSSES:
      self._stop("lost_sessions")
    elif never_answered:
      self._stop("no_answer")
    elif len(self.bases) > 1 and self.slice_start_ns and now_ns - self.slice_start_ns > SLICE_NS:
      try:
        self._next_layout(now_ns)
      except Exception:
        cloudlog.exception("array_b_recorder: layout change failed")
        self.stopped = True

  def observe_can(self, can_list, now_ns: int, v_ego: float = 0.0) -> None:
    if self.stopped:
      return
    self._see_lead(now_ns)
    self.logger.observe_can(can_list, now_ns, v_ego=v_ego)
    if self.slot_table is not None:
      try:
        self.slot_table.log(now_ns, self.order.state if self.order is not None else dict)
      except Exception:
        cloudlog.exception("array_b_recorder: slot table failed")
        self.slot_table = None
    self._watch(now_ns)

  def next_can_sends(self, now_ns: int):
    if self.stopped:
      return []
    if not self.armed_ns:
      self.armed_ns = self.slice_start_ns = int(now_ns)
    return self.logger.next_can_sends(now_ns)

  def observe_radar_tracks(self, RD, now_ns: int) -> None:
    if not self.stopped:
      self.logger.observe_radar_tracks(RD, now_ns)

  def close(self) -> None:
    self.logger.close()
    if hasattr(self.lead_source, "close"):
      self.lead_source.close()


def log_layout(layout: int | RawWindow, now_ns: int) -> None:
  raw = isinstance(layout, RawWindow)
  cloudlog.event("array_b_recorder_layout", base=layout.address if raw else layout, raw=raw, mono_ns=int(now_ns))


def build(CP, *, send_lock, pm, params) -> ArrayBRecorder | None:
  """The recorder for this car, or None. Never raises: card must start without it."""
  try:
    if not supported(CP):
      return None
    firmware = radar_firmware(CP)
    bases = ARRAY_B_BASES_BY_FIRMWARE.get(firmware, ()) if firmware is not None else ()
    xcp_on = _xcp_enabled(CP, params)
    cloudlog.event("array_b_recorder", firmware=None if firmware is None else firmware.decode(errors="replace"),
                   bases=[b.address if isinstance(b, RawWindow) else b for b in bases],
                   polling=bool(bases) and xcp_on, xcp_enabled=xcp_on)
    if not bases:
      return None
    if not xcp_on:
      return None                 # master toggle off: we ask the radar for nothing

    from openpilot.starpilot.car.honda import xcp_radar_logger

    def make_logger(base: int | RawWindow):
      if isinstance(base, RawWindow):
        from openpilot.starpilot.car.honda.xcp_raw_window import RawWindowLogger
        return RawWindowLogger(CP, window=base, send_lock=send_lock, pm=pm, params=params)
      header_size, kin_offset, lat_offset = ARRAY_B_LAYOUTS_BY_FIRMWARE[firmware]
      xcp_radar_logger.ARRAY_B_BASE = base
      xcp_radar_logger.REC0 = base + header_size
      xcp_radar_logger.KIN_OFFSET = kin_offset
      xcp_radar_logger.LAT_OFFSET = lat_offset
      return xcp_radar_logger.XcpRadarLogger(CP, send_lock=send_lock, pm=pm, params=params,
                                             probe_wayb=firmware in INTERNAL_FUSION_PROBE_FIRMWARE)

    try:
      lead_source = ModelLead()
    except Exception:
      cloudlog.exception("array_b_recorder: no model lead")
      lead_source = None
    recorder = ArrayBRecorder(make_logger, bases, lead_source)
    if not recorder.logger.enabled:
      recorder.close()
      return None
    log_layout(bases[0], 0)
    return recorder
  except Exception:
    cloudlog.exception("array_b_recorder: not started")
    return None
