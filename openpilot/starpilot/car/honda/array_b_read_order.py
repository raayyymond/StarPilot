"""Which Array B field the logger reads next. Changes what is read when; sends nothing new.

The radar answers about 48 reads a second on the Accord, for 40 slots of five fields each, and radard only fuses a slot
whose range and speed were read within 200 ms. The logger's own order spent two reads in three on its chosen lead and
left the rest of the list 5-8 s old (routes 00000071/72, 2026-11-23), so a new lead sat unread and the choice stayed
on cars that had left the lane.

Here every (slot, field) has a period set by what the slot is to long control, and the most overdue one is read:

  lead   the slots that answer to what radard may take as its lead. Range and speed every 60 ms, which is how often the
         radar renews a track (a faster read returns the same words), and ahead of everything else when due, but never
         twice running: with two leads or a slow radar the rest of the list still gets every other read.
  near   an object ahead in or beside the lane, up to the lead: what becomes the lead next.
  far    any other object.
  empty  a free slot, read to see it fill.
  freed  a slot the radar emptied that the logger still lists: its flags are read so the logger drops it.

What radard may take as its lead: the CAN track that answers to the vision lead, the nearest in-path CAN track short
of it, and the vision lead itself while no CAN track answers to it. A slot is matched to a CAN track as tightly as
radard pairs them, to vision loosely. With neither, the logger's own first choice is the lead.

The radar often holds two tracks on one car, the second 4-9 m further on with the same speed; the nearer one costs less
and takes the match. Array B ranges read about 1.5 m longer than the CAN track of the same car (1.3-1.7 m on 71/72).

The logger file stays as it came from bob-unify-car: the order is wrapped on the instance and handed back for good on
any failure.

Undriven. Replayed against 71/72 (the real logger over the slots as recorded, the CAN lead given a slot where none held
it), a track radard would fuse sat on its lead 81% / 86% of the time, against 78% / 61% for the logger's own order, and
78% / 85% at 38 reads a second. The replay compares orders; it does not predict the car: its slots are only what the
logger's own order happened to read, and it puts the logger's own order at 82% / 65% lead coverage where the car
measured 58% / 58%. About 11% / 3% is lost to the radar's own flags whatever is read.
"""
from __future__ import annotations

from openpilot.common.swaglog import cloudlog

KIN, LAT, AGE, FLAGS, QUAL = range(5)
STAMP = ("kin_updated_ns", "lat_updated_ns", "age_updated_ns", "flags_updated_ns", "qual_updated_ns")
MS = 1_000_000
#        range/speed  lateral  age    flags  quality      ms between reads, 0 = not read
LEAD = (60 * MS, 250 * MS, 500 * MS, 300 * MS, 2000 * MS)
NEAR = (300 * MS, 800 * MS, 2000 * MS, 1500 * MS, 4000 * MS)
FAR = (3000 * MS, 6000 * MS, 6000 * MS, 4000 * MS, 0)
EMPTY = (3000 * MS, 0, 0, 0, 0)
FREED = (3000 * MS, 0, 0, 3000 * MS, 0)
UNANSWERED_NS = 200 * MS      # a field asked for this long ago and still unread is asked for again

VISION_MIN_PROB = 0.5
VISION_MAX_AGE_NS = 300 * MS
CAN_MAX_AGE_NS = 500 * MS
CAMERA_AHEAD_M = 1.5          # vision x minus a CAN track's range on the same car
ARRAY_B_AHEAD_M = 1.5         # Array B X minus the CAN track's range on the same car
SECOND_TRACK_M = 9.0
MAX_KIN_AGE_S = 3.0           # an older read says too little about where the car is now
UNREAD_DY_M = 1.0             # cost of a slot whose lateral position was never read
HELD_BONUS = 2.0              # a slot already matched keeps the match against an equal one
UNFIT_COST = 4.0              # a track radard would refuse (not measured, or flagged) loses to one it would take
LANE_HALF_M = 2.5
BESIDE_HALF_M = 4.5
NO_LEAD_REACH_M = 120.0
LEAD_COUNT = 2
GHOST = 0x0020
#               range gate (m, share of range)   speed   lateral
TIGHT = (4.0, 0.10, 3.0, 1.5)       # against a CAN track
LOOSE = (5.0, 0.20, 5.0, 2.0)       # against vision
IN_PATH_DY_M = 1.5


def can_track(tracks, lead: tuple[float, float, float]) -> tuple[float, float, float] | None:
  """The CAN track that answers to the vision lead (d, y, v_rel), as (d, y, v_rel)."""
  d, y, v = lead
  best = None
  for o in tracks:
    dd, dy, dv = abs(o["dRel"] - (d - CAMERA_AHEAD_M)), abs(o["yRel"] - y), abs(o["vRel"] - v)
    if dd > max(4.0, 0.15 * d) or dy > IN_PATH_DY_M or dv > 4.0:
      continue
    cost = dd + 2.0 * dy + dv
    if best is None or cost < best[0]:
      best = (cost, (o["dRel"], o["yRel"], o["vRel"]))
  return None if best is None else best[1]


def references(tracks, lead: tuple[float, float, float] | None) -> list[tuple[float, float, float, bool]]:
  """What radard may take as its lead, nearest first, as (d, y, v_rel, from a CAN track)."""
  can = can_track(tracks, lead) if lead is not None else None
  reach = NO_LEAD_REACH_M if lead is None else (can[0] - 1.0 if can is not None else lead[0] - CAMERA_AHEAD_M)
  nearer = sorted((o["dRel"], o["yRel"], o["vRel"], True) for o in tracks if 0.0 < o["dRel"] < reach and abs(o["yRel"]) < IN_PATH_DY_M)
  if lead is None:
    return nearer[:1]
  return nearer[:LEAD_COUNT - 1] + [(*can, True) if can is not None else (*lead, False)]


def match(slots, ref: tuple[float, float, float, bool], now_ns: int, held=(), taken=()) -> int | None:
  """The slot that answers to one reference."""
  d, y, v_rel, tight = ref
  gate_m, gate_share, max_dv, max_dy = TIGHT if tight else LOOSE
  best = None
  for slot, rec in slots:
    if rec["kin_updated_ns"] <= 0 or rec["X_raw"] <= 0 or slot in taken:
      continue
    flags = rec["flags"] if rec["flags_updated_ns"] > 0 else None
    age = (now_ns - rec["kin_updated_ns"]) / 1e9
    if age > MAX_KIN_AGE_S or (flags is not None and flags & GHOST):
      continue
    v = rec["Vx_raw"] / 256.0
    x = rec["X_raw"] / 128.0 + v * age
    lat = rec["lat_updated_ns"] > 0
    dy = abs(rec["Y_raw"] / 128.0 - y) if lat else UNREAD_DY_M
    gate = max(gate_m, gate_share * d) + 0.5 * age
    if tight:
      ahead = x - d - ARRAY_B_AHEAD_M
      near, far = -gate, gate
    else:
      ahead = x - (d - CAMERA_AHEAD_M)
      near, far = -gate, max(SECOND_TRACK_M + 0.5 * age, gate)
    if not near <= ahead <= far or abs(v - v_rel) > max_dv or (lat and dy > max_dy):
      continue
    fit = flags is None or (flags & 0xF020) == 0xE000
    cost = abs(ahead) + 2.0 * dy + abs(v - v_rel) - (HELD_BONUS if slot in held else 0.0) + (0.0 if fit else UNFIT_COST)
    if best is None or cost < best[0]:
      best = (cost, slot)
  return None if best is None else best[1]


class ReadOrder:
  """Wraps one logger's read order and priority choice. Every method the logger calls runs under its lock."""

  def __init__(self, logger, layout):
    self.logger = logger
    self.layout = layout          # the module that carries KIN_OFFSET / LAT_OFFSET / NUM_SLOTS
    self.reads = (layout.KIN_OFFSET, 6), (layout.LAT_OFFSET, 6), (2, 6), (106, 2), (127, 2)
    self.asked = [[0] * 5 for _ in range(layout.NUM_SLOTS)]     # when each field was last asked for
    self.vision: tuple[float, float, float, float] | None = None   # d, y, v, prob as vision gave them
    self.vision_ns = 0
    self.now_ns = 0
    self.leads: list[int] = []    # the slots that answer to radard's lead, nearest first
    self.unanswered = 0           # references no slot answers to
    self.lead_first = False       # the last read was a lead's range and speed
    self.failed = False
    self.own_priority = logger._select_priority_slots_locked
    self.own_task = logger._next_read_task_locked
    logger._select_priority_slots_locked = self.priority
    logger._next_read_task_locked = self.task

  def release(self) -> None:
    """Hand the order back to the logger for good. The read loop must never stop on this module."""
    if not self.failed:
      cloudlog.exception("array_b_read_order: handed back to the logger")
    self.failed = True
    self.leads = []
    self.logger._select_priority_slots_locked = self.own_priority
    self.logger._next_read_task_locked = self.own_task

  def observe(self, lead: tuple[float, float, float, float] | None, now_ns: int) -> None:
    self.now_ns = max(self.now_ns, int(now_ns))
    if lead is not None:
      self.vision, self.vision_ns = lead, int(now_ns)

  def wanted(self) -> tuple[float, float, float] | None:
    """The vision lead as (d, y, v_rel), while it is fresh and confident."""
    if self.vision is None or self.now_ns - self.vision_ns > VISION_MAX_AGE_NS or self.vision[3] < VISION_MIN_PROB:
      return None
    return self.vision[0], self.vision[1], self.vision[2] - float(self.logger.last_v_ego)

  def priority(self) -> list[int]:
    own = self.own_priority()
    if self.failed:
      return own
    try:
      return self.choose(own)
    except Exception:
      self.release()
      return own

  def choose(self, own: list[int]) -> list[int]:
    logger = self.logger
    lead = self.wanted()
    tracks = logger._op_tracks if self.now_ns - logger._op_tracks_ns <= CAN_MAX_AGE_NS else ()
    refs = references(tracks, lead)
    if not refs:
      self.leads, self.unanswered = own[:1], 0
      return own
    cache = logger.slot_cache
    slots = [(s, cache[s]) for s in range(self.layout.NUM_SLOTS)]
    leads: list[int] = []
    for ref in refs:
      slot = match(slots, ref, self.now_ns, self.leads, leads)
      if slot is not None:
        leads.append(slot)
    self.leads, self.unanswered = leads, len(refs) - len(leads)
    reach = refs[-1][0] + max(SECOND_TRACK_M, LOOSE[1] * refs[-1][0])
    rest = [s for s in own if s not in leads and 0 < cache[s]["X_raw"] / 128.0 <= reach
            and (cache[s]["lat_updated_ns"] <= 0 or abs(cache[s]["Y_raw"] / 128.0) < LANE_HALF_M)]
    return (leads + rest)[:max(self.layout.LEAD_PRIORITY_COUNT, len(leads))]

  def task(self, now_ns: int):
    if not self.failed:
      try:
        return self.next(int(now_ns))
      except Exception:
        self.release()
    return self.own_task(now_ns)

  def periods(self, slot: int, rec, reach: float) -> tuple[int, int, int, int, int]:
    if slot in self.leads:
      return LEAD
    if rec["X_raw"] == 0 and rec["Vx_raw"] == 0:
      return FREED if slot in self.logger.active_slots else EMPTY
    half = BESIDE_HALF_M if self.logger._is_moving_target_locked(rec) else LANE_HALF_M
    if 0 < rec["X_raw"] / 128.0 <= reach and (rec["lat_updated_ns"] <= 0 or abs(rec["Y_raw"] / 128.0) < half):
      return NEAR
    return FAR

  def next(self, now_ns: int) -> tuple[int, int, int]:
    logger = self.logger
    self.now_ns = max(self.now_ns, now_ns)
    logger._read_seq += 1
    logger._init_queue.clear()        # the logger queues a new slot's unread fields; here they are simply due
    logger._init_queued.clear()
    self.choose(self.own_priority())
    lead = self.wanted()
    reach = NO_LEAD_REACH_M if lead is None else lead[0] + max(SECOND_TRACK_M, LOOSE[1] * lead[0])
    best = (-1.0, 0, KIN)
    for slot in range(self.layout.NUM_SLOTS):
      rec = logger.slot_cache[slot]
      periods = self.periods(slot, rec, reach)
      asked = self.asked[slot]
      for field, period in enumerate(periods):
        if not period:
          continue
        if periods is LEAD and field == KIN and self.lead_first:
          continue                            # never twice running: the rest of the list gets every other read
        waited = now_ns - asked[field]
        if field != KIN and rec[STAMP[field]] == 0 and waited > UNANSWERED_NS:
          due = 1e6 - field                   # never read for the object now in the slot
        elif not asked[field]:
          due = 1e5
        else:
          due = waited / period
          if periods is LEAD and field == KIN and due >= 1.0:
            due += 1e9                        # the lead's range and speed come before everything
        if due > best[0]:
          best = (due, slot, field)
    _, slot, field = best
    self.lead_first = field == KIN and slot in self.leads
    self.asked[slot][field] = now_ns
    return (slot, *self.reads[field])

  def state(self) -> dict:
    age = -1 if not self.vision_ns else int((self.now_ns - self.vision_ns) // MS)
    return dict(vision=None if self.vision is None else [round(v, 2) for v in self.vision], vision_age_ms=age,
                lead_slots=list(self.leads), unanswered=self.unanswered, order_failed=self.failed)
