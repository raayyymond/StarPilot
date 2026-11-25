"""The Accord radar's object classes: one sits where the gate reads a verdict, two give their range from further on."""
from __future__ import annotations

from types import SimpleNamespace

from openpilot.starpilot.car.honda import array_b_object_class as object_class
from openpilot.starpilot.car.honda import array_b_read_order as order_mod
from openpilot.starpilot.car.honda.radar import _xcp_target_qualifies
from openpilot.starpilot.car.honda.tests import test_array_b_recorder as helpers
from openpilot.starpilot.car.honda.tests.test_array_b_recorder import _restore_layout  # noqa: F401  (autouse fixture)
import openpilot.starpilot.car.honda.xcp_radar_logger as xcp_mod

NOW = helpers.NOW
ACCORD = object_class.BY_FIRMWARE[b"36802-TVA-A170"]
CAR, UNCLASSED, LONG = 0xE840, 0xF040, 0xE240       # flag words as the Accord's radar sends them


def test_only_a_listed_firmware_is_translated():
  assert object_class.reading(b"36802-TBA-A150") is None and object_class.reading(None) is None
  assert object_class.range_ahead_m(None, LONG) == 0.0 and object_class.gate_flags(None, UNCLASSED) == UNCLASSED
  assert object_class.range_ahead_m(ACCORD, LONG) == 5.8 and object_class.range_ahead_m(ACCORD, 0xE140) == 5.8
  assert object_class.range_ahead_m(ACCORD, CAR) == 0.0 and object_class.range_ahead_m(ACCORD, None) == 0.0
  assert object_class.gate_flags(ACCORD, UNCLASSED) == 0xE040 and object_class.gate_flags(ACCORD, 0xE260) == 0xE260


def test_a_target_keeps_the_words_the_radar_sent():
  target = dict(flags=LONG, dRel_m=26.3)
  object_class.translate(ACCORD, target)
  assert target == dict(flags=LONG, dRel_m=20.5, flags_wire=LONG, range_ahead_m=5.8)
  target = dict(flags=UNCLASSED, dRel_m=11.0)
  object_class.translate(ACCORD, target)
  assert target == dict(flags=0xE040, dRel_m=11.0, flags_wire=UNCLASSED, range_ahead_m=0.0)


def _object(logger, slot, now, *, x_m, flags, y_m=0.0, v_rel=-1.0):
  helpers._car(logger, slot, now, x_m=x_m, y_m=y_m, v_rel=v_rel)
  logger._handle_f4_reply_locked(slot, 2, 6, b"\xff" + (50).to_bytes(2, "big") + bytes(4), now, now)      # age 50
  logger._handle_f4_reply_locked(slot, 106, 2, b"\xff" + flags.to_bytes(2, "big"), now, now)


def test_radard_takes_what_the_accord_logger_publishes(monkeypatch):
  built = helpers._accord_recorder(monkeypatch)
  try:
    logger = built.logger
    logger.last_v_ego = 20.0
    _object(logger, 4, NOW, x_m=11.0, flags=UNCLASSED, v_rel=-1.0)      # a young track close ahead
    _object(logger, 9, NOW, x_m=26.25, flags=LONG, y_m=3.4, v_rel=-2.0)  # a long vehicle one lane over, 20.45 m away
    helpers._can(logger, NOW, (10.5, 0.0, -1.0), (20.0, 3.4, -2.0))
    with logger._lock:
      _, _, targets, lead_slot = logger._collect_targets_locked(NOW)
    by_slot = {t["slot"]: t for t in targets}
    assert by_slot[9]["dRel_m"] == 20.45 and by_slot[9]["flags_wire"] == LONG and by_slot[9]["range_ahead_m"] == 5.8
    assert by_slot[4]["flags"] == 0xE040 and by_slot[4]["flags_wire"] == UNCLASSED
    assert by_slot[4]["op_track_id"] == 0 and by_slot[9]["op_track_id"] == 1 and lead_slot == 4   # paired on the moved range
    published = {t["slot"]: t for t in logger._build_assist_payload(NOW, targets)["targets"]}
    points = {4: SimpleNamespace(trackId=0, dRel=10.5, yRel=0.0, vRel=-1.0), 9: SimpleNamespace(trackId=1, dRel=20.0, yRel=3.4, vRel=-2.0)}
    for slot, point in points.items():
      assert _xcp_target_qualifies(published[slot], point, 20.0)[0], slot
    # the same two as the radar sent them: radard's gate refuses both
    assert not _xcp_target_qualifies(dict(published[4], flags=UNCLASSED), points[4], 20.0)[0]
    assert not _xcp_target_qualifies(dict(published[9], dRel_m=26.25, op_track_id=None), points[9], 20.0)[0]
  finally:
    built.close()


def test_a_failing_translation_hands_the_targets_back_as_read(monkeypatch):
  built = helpers._accord_recorder(monkeypatch)
  try:
    logger = built.logger
    monkeypatch.setattr(object_class, "translate", lambda how, target: 1 / 0)
    _object(logger, 9, NOW, x_m=26.25, flags=LONG)
    helpers._can(logger, NOW, (20.0, 0.0, -1.0))
    for _ in range(2):
      with logger._lock:
        _, _, targets, _ = logger._collect_targets_locked(NOW)
      assert targets[0]["dRel_m"] == 26.25 and "flags_wire" not in targets[0]
  finally:
    built.close()


def test_the_read_order_matches_a_slot_by_the_range_its_class_stands_for():
  def rec(x_m, flags):
    return dict(X_raw=int(x_m * 128), Vx_raw=0, Y_raw=0, kin_updated_ns=NOW, lat_updated_ns=NOW, flags_updated_ns=NOW, flags=flags)
  can = (20.0, 0.0, 0.0, True)
  long_vehicle = {5: rec(27.3, LONG)}                     # 1.5 m past its CAN track, given from 5.8 m further on
  assert order_mod.match(long_vehicle.items(), can, NOW) is None
  assert order_mod.match(long_vehicle.items(), can, NOW, how=ACCORD) == 5
  assert order_mod.match({5: rec(27.3, CAR)}.items(), can, NOW, how=ACCORD) is None      # a car that far on is another car
  pair = {1: rec(21.5, UNCLASSED), 2: rec(23.0, CAR)}     # the unclassed track sits on the CAN track, the other 1.5 m off
  assert order_mod.match(pair.items(), can, NOW) == 2     # read as a refused track it loses
  assert order_mod.match(pair.items(), can, NOW, how=ACCORD) == 1


def test_a_range_that_steps_has_its_flags_read_at_once(monkeypatch):
  lead = [(43.0, 0.0, 19.0, 0.95)]
  built = helpers._led_recorder(monkeypatch, lead)
  try:
    radar = {5: dict(x=41.5, v=-1.0, y=0.0, flags=CAR), 11: dict(x=30.0, v=-2.0, y=3.5, flags=CAR)}
    helpers._can(built.logger, NOW, (40.0, 0.0, -1.0))
    built.logger._op_tracks_ns = NOW + 2000 * helpers.STEP                 # the CAN track stays fresh for the run
    helpers._answer(built, radar, NOW, 240)
    assert built.order.leads == [5]
    start = NOW + 240 * helpers.STEP
    radar[5] = dict(x=47.3, v=-1.0, y=0.0, flags=LONG)                     # same car, now classed as a long vehicle
    asked = helpers._answer(built, radar, start, 48)
    reads = [(slot, offset) for _, slot, offset in asked]
    stepped = reads.index((5, xcp_mod.KIN_OFFSET))                         # the first range read after the change
    assert reads.index((5, 106)) - stepped <= 2                            # its flags follow, not 300 ms later
    assert built.order.leads == [5] and not built.order.failed            # and with them the slot is the lead again
    kin = [t for t, slot, offset in asked[12:] if (slot, offset) == (5, xcp_mod.KIN_OFFSET)]
    assert max(b - a for a, b in zip(kin, kin[1:])) <= 3 * helpers.STEP    # read every radar cycle, as before
  finally:
    built.close()


def test_a_steady_range_draws_no_extra_flag_reads():
  seen = (NOW, int(40.0 * 128), int(-2.0 * 256))
  def rec(x_m, after_s, v=-2.0):
    return dict(X_raw=int(x_m * 128), Vx_raw=int(v * 256), kin_updated_ns=NOW + int(after_s * 1e9))
  assert not order_mod.stepped(seen, rec(39.88, 0.06))                    # where its speed put it
  assert not order_mod.stepped(seen, rec(37.5, 0.5, v=-8.0))              # a hard closing change, still its own range
  assert order_mod.stepped(seen, rec(45.7, 0.06))
  assert order_mod.stepped(seen, rec(34.1, 0.06))
  assert not order_mod.stepped(seen, rec(45.7, 1.5))                      # too long ago to say
  assert not order_mod.stepped((0, 0, 0), rec(45.7, 0.06)) and not order_mod.stepped(seen, rec(0.0, 0.06))
