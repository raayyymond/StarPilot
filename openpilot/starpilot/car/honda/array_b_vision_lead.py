"""Reading order for the Array B logger, led by the vision lead. Changes which slot is read when; sends nothing new.

The radar answers about 48 reads a second on the Accord, for 40 slots of four fields each, so the logger has to choose
what to read. Its own choice follows the car's object list (the CAN tracks) and, without a match, the nearest moving
slot in path. On routes 00000071/72 (2026-11-23) that left the lead without a fresh track 42% of the time:

  - a slot is only re-read every 5-8 s (median) unless it is the chosen lead, so a new lead sat unread;
  - the choice stayed on the matched car after it had left the lane (3.4 m to the side), then went to a car at 138 m
    while the lead was at 80 m;
  - half of the 24 reads a second spent on the lead returned the same words: the radar renews a track about every 60 ms.

So, on the logger instance (its file stays as it came from bob-unify-car):

  - every third read goes to the slot whose kinematics are oldest, whatever the logger thinks of it, so the whole list
    is seen about every 2.5 s; every second read while vision has a lead and no slot answers to it;
  - when vision holds a confident lead, the slot that sits where vision sees it and moves like it is read first. With
    no such slot, a slot keeps its priority only if it is in lane and not beyond the lead. With no vision lead the
    logger's own choice stands.

The radar often holds two tracks on one car, the second 4-9 m further on with the same speed. Both answer to the lead;
the nearer one costs less and takes the choice as soon as it has been read.
"""
from __future__ import annotations

VISION_MIN_PROB = 0.5
VISION_MAX_AGE_NS = 300_000_000
CAMERA_AHEAD_M = 1.5          # vision x minus Array B X on the same car, the usual track (routes 71/72)
SECOND_TRACK_M = 9.0          # the radar often holds a second track on the same car 4-9 m further on; it counts too
MAX_KIN_AGE_S = 3.0           # an older read says too little about where the car is now
MAX_DV_MPS = 5.0
MAX_DY_M = 2.0
UNREAD_DY_M = 1.0             # cost of a slot whose lateral position was never read
HELD_BONUS = 2.0              # the slot already chosen keeps the choice against an equal one
LANE_HALF_M = 2.5
GHOST = 0x0020

SWEEP_PERIOD = 3
SEARCH_PERIOD = 2
SWEEP_MIN_AGE_NS = 1_000_000_000


def range_gate(d: float) -> float:
  return max(5.0, 0.2 * d)


def pick(slots, lead: tuple[float, float, float], now_ns: int, held: int | None = None) -> int | None:
  """The slot that answers to the vision lead (d, y, v_rel; metres ahead of the camera, left positive), or None."""
  d, y, v_rel = lead
  best = None
  for slot, rec in slots:
    if rec["kin_updated_ns"] <= 0 or rec["X_raw"] <= 0:
      continue
    if rec["flags_updated_ns"] > 0 and rec["flags"] & GHOST:
      continue
    age = (now_ns - rec["kin_updated_ns"]) / 1e9
    if age > MAX_KIN_AGE_S:
      continue
    v = rec["Vx_raw"] / 256.0
    ahead = rec["X_raw"] / 128.0 + v * age - (d - CAMERA_AHEAD_M)
    dd = abs(ahead)
    dv = abs(v - v_rel)
    dy = abs(rec["Y_raw"] / 128.0 - y) if rec["lat_updated_ns"] > 0 else UNREAD_DY_M
    if ahead < -range_gate(d) - 0.5 * age or ahead > max(SECOND_TRACK_M, range_gate(d)) + 0.5 * age or dv > MAX_DV_MPS or dy > MAX_DY_M:
      continue
    cost = dd + 2.0 * dy + dv - (HELD_BONUS if slot == held else 0.0)
    if best is None or cost < best[0]:
      best = (cost, slot)
  return None if best is None else best[1]


def in_lane_up_to(rec, d: float) -> bool:
  if rec["X_raw"] <= 0 or rec["X_raw"] / 128.0 > d + max(SECOND_TRACK_M, range_gate(d)):
    return False
  return rec["lat_updated_ns"] <= 0 or abs(rec["Y_raw"] / 128.0) < LANE_HALF_M


class VisionLead:
  """Wraps one logger's priority choice and read order. Every method the logger calls runs under its lock."""

  def __init__(self, logger, layout):
    self.logger = logger
    self.layout = layout          # the module that carries KIN_OFFSET / NUM_SLOTS / LEAD_PRIORITY_COUNT
    self.lead: tuple[float, float, float, float] | None = None   # d, y, v, prob as vision gave them
    self.lead_ns = 0
    self.now_ns = 0
    self.slot: int | None = None
    self.reads = 0
    self.sweeps = 0
    self.failed = False
    self.own_priority = logger._select_priority_slots_locked
    self.own_task = logger._next_read_task_locked
    logger._select_priority_slots_locked = self.priority
    logger._next_read_task_locked = self.task

  def release(self) -> None:
    """Hand the choice back to the logger for good. The read loop must never stop on this module."""
    self.failed = True
    self.slot = None
    self.logger._select_priority_slots_locked = self.own_priority
    self.logger._next_read_task_locked = self.own_task

  def observe(self, lead: tuple[float, float, float, float] | None, now_ns: int) -> None:
    self.now_ns = max(self.now_ns, int(now_ns))
    if lead is not None:
      self.lead, self.lead_ns = lead, int(now_ns)

  def wanted(self) -> tuple[float, float, float] | None:
    """The vision lead as (d, y, v_rel), while it is fresh and confident."""
    if self.lead is None or self.now_ns - self.lead_ns > VISION_MAX_AGE_NS or self.lead[3] < VISION_MIN_PROB:
      return None
    return self.lead[0], self.lead[1], self.lead[2] - float(self.logger.last_v_ego)

  def priority(self) -> list[int]:
    own = self.own_priority()
    if self.failed:
      return own
    try:
      return self.led(own)
    except Exception:
      self.release()
      return own

  def led(self, own: list[int]) -> list[int]:
    lead = self.wanted()
    if lead is None:
      self.slot = None
      return own
    active = self.logger.active_slots
    self.slot = pick(active.items(), lead, self.now_ns, self.slot)
    if self.slot is None:
      return [s for s in own if in_lane_up_to(active[s], lead[0])]
    return ([self.slot] + [s for s in own if s != self.slot])[:self.layout.LEAD_PRIORITY_COUNT]

  def task(self, now_ns: int):
    if not self.failed:
      try:
        sweep = self.sweep(now_ns)
        if sweep is not None:
          return sweep
      except Exception:
        self.release()
    return self.own_task(now_ns)

  def sweep(self, now_ns: int):
    self.now_ns = max(self.now_ns, int(now_ns))
    self.reads += 1
    first = self.led(self.own_priority())[:1]
    searching = self.wanted() is not None and self.slot is None
    if self.reads % (SEARCH_PERIOD if searching else SWEEP_PERIOD) == 0:
      cache = self.logger.slot_cache
      slot = min((s for s in range(self.layout.NUM_SLOTS) if s not in first), key=lambda s: (cache[s]["kin_updated_ns"], s))
      if now_ns - cache[slot]["kin_updated_ns"] >= SWEEP_MIN_AGE_NS:
        self.logger._read_seq += 1
        self.sweeps += 1
        return (slot, self.layout.KIN_OFFSET, 6)
    return None

  def state(self) -> dict:
    age = -1 if not self.lead_ns else int((self.now_ns - self.lead_ns) // 1_000_000)
    return dict(vision=None if self.lead is None else [round(v, 2) for v in self.lead], vision_age_ms=age,
                vision_slot=self.slot, sweeps=self.sweeps, vision_failed=self.failed)
