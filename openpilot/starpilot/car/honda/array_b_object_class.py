"""What a radar firmware's Array B flag word says about an object's class, put in the terms the lead gate reads.

The logger and radard's gate (both from bob-unify-car) read the flag word the way the Civic's radar writes it: bit
0x1000 marks a low-confidence track, which radard refuses, and a slot's range is the range of the CAN track on the
same car.

On the Accord's TVA-A170 the bits 0x1F00 are the object's class, one bit at a time. Routes 00000075 / 00000077
(2026-11-24), every slot whose flags were read within 250 ms of its range, against the CAN track it sits on:

  class    slots   range past the CAN track   on it / 5-9 m past it
  0x0800   226     +0.9 m                     81% / 12%
  0x0400   115     +0.5 m                     90% /  2%
  0x1000   105     -0.4 m                     88% /  1%
  0x0200    88     +5.8 m                     13% / 70%
  0x0100    14     +5.5 m                      0% / 93%

So 0x1000 is a class here, not a verdict. It is a young track (median age 31-43 against about 150), seen mostly
under 15 m and behind a slow car. Its Doppler speed agrees with the CAN track as well as any other class does (median
0.28-0.41 m/s apart against 0.39-0.44, 95% within 1.1 against 1.7), and refusing it was 20% / 30% of the time
radard's radar lead went without Array B.

And a 0x0200 or 0x0100 object gives its range from a point 5.8 m further on. One track changes class while it
follows one car: its range steps 5.7-6.2 m in a single read, the speed unbroken, the CAN track and the vision lead
unmoved. Read as a second car further on, it was refused for 42% / 25% of that time.

Both are translated where the logger collects its targets, before it pairs them with the CAN tracks, so the
pairing, the 20 Hz event and the snapshot radard reads all carry the same range and the same flags. The words as the
radar sent them stay in each target (flags_wire, range_ahead_m) and in the 2 Hz slot table. The flags are read less
often than the range, so for a read or two after a class change the range is still on the old reference; radard
refuses the slot for that long, and array_b_read_order asks for the flags at once when a range steps.

A firmware with no entry is left as read.

Replayed through radard on the two routes (the car's own radarTracks, model and snapshots; the replay's lead is the
car's on every frame, its speed within 0.004 m/s median): Array B on the radar lead 53.5% -> 72.2% and 74.2% -> 87.0%
of the time, under 15 m 20% -> 86% and 28% -> 73%. The newly fed lead speed sits a median 0.25-0.54 m/s from the CAN
filter's, where the already fed one sits 0.48. The replay only has the reads the car made, so it cannot show what
the read order's part adds: a far-class lead was in the order's lead set 26% of the time against 95%. Undriven.
"""
from __future__ import annotations

from typing import NamedTuple

from openpilot.common.swaglog import cloudlog


class Reading(NamedTuple):
  not_a_verdict: int    # class bits the gate would read as its low-confidence flag
  far_classes: int      # classes whose range is given from a point further on
  far_m: float          # how much further


BY_FIRMWARE: dict[bytes, Reading] = {
  b"36802-TVA-A170": Reading(not_a_verdict=0x1000, far_classes=0x0300, far_m=5.8),
}


def reading(firmware: bytes | None) -> Reading | None:
  return BY_FIRMWARE.get(firmware) if firmware is not None else None


def range_ahead_m(how: Reading | None, flags: int | None) -> float:
  """How far past the car's CAN range this slot's range is given, by its class."""
  return how.far_m if how is not None and flags is not None and flags & how.far_classes else 0.0


def gate_flags(how: Reading | None, flags: int) -> int:
  """The flag word with this firmware's class bits taken out of the gate's verdict bits."""
  return flags if how is None else flags & ~how.not_a_verdict


def translate(how: Reading, target: dict) -> None:
  flags = int(target["flags"])
  ahead = range_ahead_m(how, flags)
  target["flags_wire"] = flags
  target["range_ahead_m"] = ahead
  target["flags"] = gate_flags(how, flags)
  if ahead:
    target["dRel_m"] = round(float(target["dRel_m"]) - ahead, 4)


def dress(logger, how: Reading | None) -> None:
  """Translate each target as the logger collects it. Wrapped on the instance; handed back for good on any failure."""
  if how is None or not hasattr(logger, "_match_openpilot_tracks"):
    return
  inner = logger._match_openpilot_tracks

  def paired(targets, op_tracks, v_ego=0.0):
    try:
      for target in targets:
        if "flags_wire" not in target:
          translate(how, target)
    except Exception:
      cloudlog.exception("array_b_object_class: targets left as read")
      logger._match_openpilot_tracks = inner
    return inner(targets, op_tracks, v_ego)
  logger._match_openpilot_tracks = paired
