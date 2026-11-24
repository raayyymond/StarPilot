"""The fork gate around the Array B logger: which cars and firmware it runs on, the layout it applies, the candidate
rotation for a firmware whose address is not known yet, and when it gives up."""
from __future__ import annotations

import sys
from types import SimpleNamespace

import pytest

from opendbc.car.honda.values import CAR, HondaFlags
from openpilot.starpilot.car.honda import array_b_recorder as recorder
from openpilot.starpilot.car.honda import array_b_read_order as order_mod
import openpilot.starpilot.car.honda.xcp_radar_logger as xcp_mod


def _fw(version: bytes, *, ecu="fwdRadar", address=recorder.RADAR_ADDRESS, logging=False):
  return SimpleNamespace(ecu=ecu, address=address, fwVersion=version, logging=logging)


def _cp(firmware=(), *, fingerprint=CAR.HONDA_ACCORD, radar_unavailable=False, passive=False):
  return SimpleNamespace(brand="honda", carFingerprint=fingerprint, flags=int(HondaFlags.BOSCH),
                         radarUnavailable=radar_unavailable, passive=passive, carFw=list(firmware),
                         notCar=False, dashcamOnly=False, alternativeExperience=0,
                         openpilotLongitudinalControl=True, pcmCruise=False)


def _toggle_on(monkeypatch, on=True):
  """Pretend the master toggle is on/off. The real param -> enabled() path is covered by test_xcp_radar_assist."""
  monkeypatch.setattr(recorder, "_xcp_enabled", lambda CP, params: on)


@pytest.fixture(autouse=True)
def _restore_layout():
  saved = (xcp_mod.ARRAY_B_BASE, xcp_mod.REC0, xcp_mod.KIN_OFFSET, xcp_mod.LAT_OFFSET)
  yield
  (xcp_mod.ARRAY_B_BASE, xcp_mod.REC0, xcp_mod.KIN_OFFSET, xcp_mod.LAT_OFFSET) = saved


def test_radar_firmware_takes_the_non_logging_radar_entry():
  cp = _cp([_fw(b"junk", ecu="eps"), _fw(b"\x0e74522", logging=True), _fw(b"36802-TVA-A170\x00\x00")])
  assert recorder.radar_firmware(cp) == b"36802-TVA-A170"
  assert recorder.radar_firmware(_cp([])) is None


def test_unlisted_firmware_is_not_polled(monkeypatch):
  _toggle_on(monkeypatch)                                            # no base, so no poll even with the toggle on
  assert b"36802-ZZZ-Z999" not in recorder.ARRAY_B_BASES_BY_FIRMWARE
  assert recorder.build(_cp([_fw(b"36802-ZZZ-Z999\x00\x00")]), send_lock=None, pm=None, params=None) is None
  assert recorder.build(_cp([]), send_lock=None, pm=None, params=None) is None


def test_the_saved_toggle_reaches_the_gate(tmp_path):
  params = SimpleNamespace(get_param_path=lambda key: str(tmp_path / key))
  cp = _cp([_fw(b"36802-TVA-A170\x00\x00")], fingerprint=CAR.HONDA_ACCORD)
  assert recorder._xcp_enabled(cp, params) is False                  # never saved
  (tmp_path / "AlphaHondaXcpRadarAssist").write_bytes(b"1")
  assert recorder._xcp_enabled(cp, params) is True
  (tmp_path / "AlphaHondaXcpRadarAssist").write_bytes(b"0")
  assert recorder._xcp_enabled(cp, params) is False
  (tmp_path / "AlphaHondaXcpRadarAssist").write_bytes(b"1")
  (tmp_path / "IsReleaseBranch").write_bytes(b"1")                   # the gate is the assist's own: not on a release branch
  assert recorder._xcp_enabled(cp, params) is False


def test_the_master_toggle_controls_whether_we_poll_at_all(monkeypatch):
  monkeypatch.delenv("REPLAY", raising=False)
  cp = _cp([_fw(b"36802-TVA-A170\x00\x00")], fingerprint=CAR.HONDA_ACCORD)
  assert recorder._xcp_enabled(cp, None) is False                    # fail-closed: no params, no poll
  _toggle_on(monkeypatch, on=False)
  assert recorder.build(cp, send_lock=None, pm=None, params=None) is None            # off: dormant
  _toggle_on(monkeypatch, on=True)
  built = recorder.build(cp, send_lock=None, pm=None, params=None)
  try:
    assert built is not None and built.logger.enabled                                # on: polling
  finally:
    if built is not None:
      built.close()


@pytest.mark.parametrize("cp", [
  _cp([_fw(b"36802-TBA-A150")], fingerprint=CAR.HONDA_CIVIC_BOSCH, radar_unavailable=True),
  _cp([_fw(b"36802-TBA-A150")], fingerprint=CAR.HONDA_CIVIC_BOSCH, passive=True),
  _cp([_fw(b"36802-TBA-A150")], fingerprint=CAR.HONDA_CIVIC),  # not a verified Bosch-A car
])
def test_unsupported_cars_get_no_recorder(cp):
  assert recorder.build(cp, send_lock=None, pm=None, params=None) is None


@pytest.mark.parametrize("firmware, fingerprint, base, header, kin, lat, probe", [
  (b"36802-TBA-A150", CAR.HONDA_CIVIC_BOSCH, 0x400371A4, 0x10, 16, 22, True),
  (b"36802-TBA-A160", CAR.HONDA_CIVIC_BOSCH, 0x400371C4, 0x10, 16, 22, False),
  (b"36802-TVA-A170", CAR.HONDA_ACCORD, 0x40034F94, 0x08, 18, 24, False),
])
def test_listed_firmware_starts_on_its_layout(monkeypatch, firmware, fingerprint, base, header, kin, lat, probe):
  monkeypatch.delenv("REPLAY", raising=False)
  _toggle_on(monkeypatch)
  built = recorder.build(_cp([_fw(firmware + b"\x00\x00")], fingerprint=fingerprint), send_lock=None, pm=None, params=None)
  try:
    assert built is not None and built.logger.enabled
    assert xcp_mod.ARRAY_B_BASE == base
    assert xcp_mod.REC0 == base + header
    assert xcp_mod.KIN_OFFSET == kin
    assert xcp_mod.LAT_OFFSET == lat
    assert built.logger._wayb_probe_enabled is probe
  finally:
    if built is not None:
      built.close()


def test_a_failing_logger_never_stops_card(monkeypatch):
  _toggle_on(monkeypatch)
  def boom(*args, **kwargs):
    raise RuntimeError("logger broke")
  monkeypatch.setattr(xcp_mod, "XcpRadarLogger", boom)
  assert recorder.build(_cp([_fw(b"36802-TBA-A150")], fingerprint=CAR.HONDA_CIVIC_BOSCH),
                        send_lock=None, pm=None, params=None) is None


class _FakeLogger:
  def __init__(self, base=0):
    self.base = base
    self.enabled = True
    self.state = "NEED_CONNECT"
    self.closed = False
    self.sends = 0

  def observe_can(self, can_list, now_ns, v_ego=0.0):
    pass

  def next_can_sends(self, now_ns):
    self.sends += 1
    return []

  def observe_radar_tracks(self, RD, now_ns):
    pass

  def close(self):
    self.closed = True


def _recorder(bases=(1,)):
  made = []

  def make(base):
    made.append(_FakeLogger(base))
    return made[-1]
  return recorder.ArrayBRecorder(make, bases), made


def test_gives_up_after_repeated_lost_sessions():
  rec, made = _recorder()
  logger = made[0]
  now = 1_000_000_000
  rec.next_can_sends(now)
  for _ in range(recorder.MAX_SESSION_LOSSES):
    assert not rec.stopped
    logger.state = "READY"
    rec.observe_can([], now)
    logger.state = "NEED_CONNECT"
    rec.observe_can([], now)
  assert rec.stopped and logger.closed and not logger.enabled
  sends = logger.sends
  assert rec.next_can_sends(now) == [] and logger.sends == sends


def test_gives_up_when_the_radar_never_answers():
  rec, made = _recorder()
  rec.next_can_sends(1)
  rec.observe_can([], 1 + recorder.CONNECT_DEADLINE_NS)
  assert not rec.stopped
  rec.observe_can([], 2 + recorder.CONNECT_DEADLINE_NS)
  assert rec.stopped and made[0].closed


def test_a_single_known_layout_never_rotates():
  rec, made = _recorder()
  rec.next_can_sends(1)
  made[0].state = "READY"
  for t in range(2, 200):
    rec.observe_can([], t * recorder.SLICE_NS)
  assert not rec.stopped and not made[0].closed and len(made) == 1


def test_candidates_are_recorded_in_turn_and_a_change_is_not_a_lost_session():
  rec, made = _recorder(bases=(0xC4, 0xA4))
  rec.next_can_sends(1)
  made[0].state = "READY"
  rec.observe_can([], 1 + recorder.SLICE_NS)
  assert len(made) == 1                                   # not past the slice yet
  rec.observe_can([], 2 + recorder.SLICE_NS)
  assert [m.base for m in made] == [0xC4, 0xA4] and made[0].closed and not made[0].enabled
  assert rec.logger is made[1] and rec.session_losses == 0 and not rec.stopped
  made[1].state = "READY"
  rec.observe_can([], 3 + recorder.SLICE_NS)
  rec.observe_can([], 3 + 2 * recorder.SLICE_NS)          # second slice over: back to the first address
  assert [m.base for m in made] == [0xC4, 0xA4, 0xC4] and rec.session_losses == 0
  assert rec.next_can_sends(4 + 2 * recorder.SLICE_NS) == [] and made[2].sends == 1


def test_the_accord_firmware_pins_its_verified_base(monkeypatch):
  monkeypatch.delenv("REPLAY", raising=False)
  _toggle_on(monkeypatch)
  assert recorder.ARRAY_B_BASES_BY_FIRMWARE[b"36802-TVA-A170"] == (0x40034F94,)
  assert recorder.ARRAY_B_LAYOUTS_BY_FIRMWARE[b"36802-TVA-A170"] == (0x08, 18, 24)
  built = recorder.build(_cp([_fw(b"36802-TVA-A170\x00\x00")], fingerprint=CAR.HONDA_ACCORD), send_lock=None, pm=None, params=None)
  try:
    assert built is not None and built.logger.enabled
    assert xcp_mod.ARRAY_B_BASE == 0x40034F94
    assert xcp_mod.REC0 == 0x40034F9C
    assert xcp_mod.KIN_OFFSET == 18
    assert xcp_mod.LAT_OFFSET == 24
  finally:
    if built is not None:
      built.close()


class _Events:
  def __init__(self, monkeypatch):
    from openpilot.starpilot.car.honda import xcp_raw_window
    self.seen = []
    monkeypatch.setattr(xcp_raw_window.cloudlog, "event", lambda name, **kw: self.seen.append((name, kw)))
    self.cls = xcp_raw_window.RawWindowLogger


def _raw_logger(monkeypatch, window):
  monkeypatch.delenv("REPLAY", raising=False)
  events = _Events(monkeypatch)
  logger = events.cls(_cp([_fw(b"36802-TVA-A170")], fingerprint=CAR.HONDA_ACCORD), window=recorder.RawWindow(*window),
                      send_lock=None, pm=None, params=None)
  return logger, events.seen


def test_a_raw_window_is_read_front_to_back_and_logged_whole(monkeypatch):
  logger, seen = _raw_logger(monkeypatch, (0x40038800, 10, 4))
  try:
    reads = []
    for now in range(1, 7):                          # two sweeps of 4 + 4 + 2 bytes
      tag, addr, size = logger._next_wayb_task_locked(now)
      reads.append((addr - 0x40038800, size))
      logger._handle_wayb_reply_locked(tag, size, bytes([0xFF]) + bytes(range(addr & 0xFF, (addr & 0xFF) + size)), now)
    assert reads == [(0, 4), (4, 4), (8, 2)] * 2
    assert not logger._primary_lead_urgent_locked(7)
    assert [name for name, _ in seen] == ["xcp_raw_window"] * 2
    first = seen[0][1]
    assert (first["base"], first["offset"], first["start_ns"], first["end_ns"], first["missing"]) == (0x40038800, 0, 1, 3, [])
    assert bytes.fromhex(first["data"]) == bytes(range(10))
  finally:
    logger.close()


def test_an_unanswered_read_keeps_the_offsets_aligned(monkeypatch):
  logger, seen = _raw_logger(monkeypatch, (0x40035000, 8, 4))
  try:
    logger._next_wayb_task_locked(1)                 # no reply to the first read
    tag, addr, size = logger._next_wayb_task_locked(2)
    logger._handle_wayb_reply_locked(tag, size, b"\xff\x01\x02\x03\x04", 2)
    assert seen[0][1]["missing"] == [0] and bytes.fromhex(seen[0][1]["data"]) == bytes(4) + b"\x01\x02\x03\x04"
  finally:
    logger.close()


def test_a_block_is_logged_every_1024_bytes(monkeypatch):
  logger, seen = _raw_logger(monkeypatch, (0x40035000, 5440, 7))
  try:
    for now in range(1, 778 + 1):                    # 5440 B in 7 B reads
      tag, addr, size = logger._next_wayb_task_locked(now)
      logger._handle_wayb_reply_locked(tag, size, bytes(1 + size), now)
    assert [e["offset"] for _, e in seen] == [0, 1029, 2058, 3087, 4116, 5145]
    assert sum(len(e["data"]) // 2 for _, e in seen) == 5440 and logger._raw_offset == 0
  finally:
    logger.close()


def _slot_events(monkeypatch):
  seen = []
  monkeypatch.setattr(recorder.cloudlog, "event", lambda name, **kw: seen.append((name, kw)))
  return seen


def _accord_recorder(monkeypatch):
  monkeypatch.delenv("REPLAY", raising=False)
  _toggle_on(monkeypatch)
  built = recorder.build(_cp([_fw(b"36802-TVA-A170\x00\x00")], fingerprint=CAR.HONDA_ACCORD), send_lock=None, pm=None,
                         params=None)
  assert built is not None and built.slot_table is not None
  return built


def test_the_slot_table_records_every_slot_twice_a_second_once_the_session_is_up(monkeypatch):
  built = _accord_recorder(monkeypatch)
  try:
    seen = _slot_events(monkeypatch)
    logger = built.logger
    now = 10_000_000_000
    built.observe_can([], now)
    assert not [e for e in seen if e[0] == "xcp_array_b_slots"]        # no session yet
    logger.state = "READY"
    logger._handle_f4_reply_locked(7, xcp_mod.KIN_OFFSET, 6, b"\xff" + (1408).to_bytes(2, "big") + bytes(4), now, now)
    built.observe_can([], now)
    built.observe_can([], now + recorder.SLOT_TABLE_NS - 1)
    built.observe_can([], now + recorder.SLOT_TABLE_NS)
    tables = [kw for name, kw in seen if name == "xcp_array_b_slots"]
    assert len(tables) == 2
    slots = tables[0]["slots"]
    assert [row[0] for row in slots] == list(range(xcp_mod.NUM_SLOTS))
    # slot 7: active, X read just now, flags never read; it is in the table though the 20 Hz event would leave it out
    assert slots[7][:3] == [7, 1, 1408] and slots[7][11] == 0 and slots[7][14] == -1
    assert slots[8][1:3] == [0, 0] and slots[8][11] == -1
    assert tables[1]["slots"][7][11] == recorder.SLOT_TABLE_NS // 1_000_000
  finally:
    built.close()


def test_the_slot_table_counts_a_dropped_slot_and_keeps_the_flags_it_was_dropped_with(monkeypatch):
  built = _accord_recorder(monkeypatch)
  try:
    logger = built.logger
    now = 10_000_000_000
    logger._handle_f4_reply_locked(3, xcp_mod.KIN_OFFSET, 6, b"\xff" + (640).to_bytes(2, "big") + bytes(4), now, now)
    assert 3 in logger.active_slots
    logger._handle_f4_reply_locked(3, 106, 2, b"\xff\xc0\x40", now + 1, now + 1)   # flags without the valid bit
    assert 3 not in logger.active_slots
    row = built.slot_table.rows(now + 2)["slots"][3]
    assert row[1:3] == [0, 0] and row[15:] == [1, 0xC040, 640]
  finally:
    built.close()


def test_a_failing_slot_table_never_stops_the_recorder(monkeypatch):
  rec, made = _recorder()
  assert rec.slot_table is None                       # a logger without a slot table gets none
  rec.slot_table = SimpleNamespace(log=lambda now_ns, more: 1 / 0)
  rec.next_can_sends(1)
  rec.observe_can([], 2)
  assert rec.slot_table is None and not rec.stopped


def _kin(logger, slot, now, *, x_m, v_rel=0.0):
  words = int(x_m * 128).to_bytes(2, "big", signed=True) + int(v_rel * 256).to_bytes(2, "big", signed=True) + bytes(2)
  logger._handle_f4_reply_locked(slot, xcp_mod.KIN_OFFSET, 6, b"\xff" + words, now, now)


def _lat(logger, slot, now, *, y_m):
  logger._handle_f4_reply_locked(slot, xcp_mod.LAT_OFFSET, 6, b"\xff" + int(y_m * 128).to_bytes(2, "big", signed=True) + bytes(4), now, now)


def _car(logger, slot, now, *, x_m, y_m=0.0, v_rel=-1.0):
  _kin(logger, slot, now, x_m=x_m, v_rel=v_rel)
  _lat(logger, slot, now, y_m=y_m)


def _led_recorder(monkeypatch, lead):
  """An Accord recorder at 20 m/s whose vision lead is whatever `lead` holds: (d, y, v, prob) or None."""
  built = _accord_recorder(monkeypatch)
  built.lead_source = lambda: lead[0]
  assert built.order is not None
  return built


def _can(logger, now, *tracks):
  logger._op_tracks = [dict(tid=i, dRel=d, yRel=y, vRel=v) for i, (d, y, v) in enumerate(tracks)]
  logger._op_tracks_ns = now


NOW = 10_000_000_000
STEP = 20_800_000        # 48 reads a second


def test_the_slot_that_answers_to_the_vision_lead_is_the_lead(monkeypatch):
  lead = [None]
  built = _led_recorder(monkeypatch, lead)
  try:
    logger = built.logger
    built.observe_can([], NOW, v_ego=20.0)
    _car(logger, 3, NOW, x_m=20.0)
    _car(logger, 9, NOW, x_m=60.0, y_m=0.2)
    assert logger._select_priority_slots_locked() == [3, 9]        # the logger's own choice: nearest moving in path
    assert built.order.leads == [3]
    lead[0] = (61.5, 0.0, 19.0, 0.95)
    built.observe_can([], NOW, v_ego=20.0)
    assert logger._select_priority_slots_locked() == [9, 3]
    assert built.order.state()["lead_slots"] == [9] and built.order.unanswered == 0
    lead[0] = (61.5, 0.0, 19.0, 0.2)                               # vision no longer sure: the logger's choice again
    built.observe_can([], NOW, v_ego=20.0)
    assert logger._select_priority_slots_locked() == [3, 9]
    lead[0] = None                                                 # and a lead that stopped arriving runs out
    built.order.vision = (61.5, 0.0, 19.0, 0.95)
    built.observe_can([], NOW + order_mod.VISION_MAX_AGE_NS + 1, v_ego=20.0)
    assert logger._select_priority_slots_locked() == [3, 9]
  finally:
    built.close()


def test_with_a_vision_lead_and_no_slot_for_it_a_car_beside_the_lane_or_beyond_the_lead_loses_priority(monkeypatch):
  lead = [(80.0, 0.0, 19.0, 0.95)]
  built = _led_recorder(monkeypatch, lead)
  try:
    logger = built.logger
    built.observe_can([], NOW, v_ego=20.0)
    _car(logger, 8, NOW, x_m=20.0, y_m=3.4, v_rel=-9.0)      # the car that left the lane, far too slow to be the lead
    _car(logger, 4, NOW, x_m=138.0, v_rel=-9.0)              # a car well beyond the lead
    assert logger._select_priority_slots_locked() == [] and built.order.unanswered == 1
    _car(logger, 6, NOW, x_m=40.0, y_m=0.5, v_rel=-9.0)      # in lane and nearer than the lead: stays
    assert logger._select_priority_slots_locked() == [6]
  finally:
    built.close()


def test_the_lead_is_matched_through_its_can_track_and_a_nearer_car_in_path_is_a_lead_too(monkeypatch):
  lead = [(61.5, 0.0, 19.0, 0.95)]
  built = _led_recorder(monkeypatch, lead)
  try:
    logger = built.logger
    built.observe_can([], NOW, v_ego=20.0)
    _car(logger, 9, NOW, x_m=60.0, y_m=0.1)                  # the lead: Array B reads 1.5 m longer than its CAN track
    _car(logger, 12, NOW, x_m=66.0, y_m=0.1)                 # the radar's second track on the same car
    _car(logger, 15, NOW, x_m=60.5, y_m=3.3)                 # a car beside it
    _can(logger, NOW, (58.5, 0.1, -1.0), (58.0, 3.3, -1.0))
    logger._select_priority_slots_locked()
    assert built.order.leads == [9] and built.order.unanswered == 0
    _can(logger, NOW, (58.5, 0.1, -1.0), (20.0, 0.4, -3.0))  # a car cuts in close: radard may take it before vision does
    logger._select_priority_slots_locked()
    assert built.order.leads == [9] and built.order.unanswered == 1
    _car(logger, 2, NOW, x_m=21.5, y_m=0.4, v_rel=-3.0)
    logger._select_priority_slots_locked()
    assert built.order.leads == [2, 9] and built.order.unanswered == 0
    _can(logger, NOW, (58.5, 0.1, -1.0), (40.0, 0.2, -2.0), (20.0, 0.4, -3.0))   # two short of the lead: the nearest counts
    logger._select_priority_slots_locked()
    assert built.order.leads == [2, 9] and built.order.unanswered == 0
    lead[0] = (0.0, 0.0, 0.0, 0.0)                           # no vision lead: the nearest CAN track in path
    built.observe_can([], NOW, v_ego=20.0)
    logger._select_priority_slots_locked()
    assert built.order.leads == [2]
  finally:
    built.close()


def test_of_two_tracks_on_the_lead_the_nearer_takes_the_match():
  def rec(x_m, kin_ns=NOW, y_m=0.0, flags_ns=0, flags=0):
    return dict(X_raw=int(x_m * 128), Vx_raw=0, Y_raw=int(y_m * 128), kin_updated_ns=kin_ns, lat_updated_ns=kin_ns,
                flags_updated_ns=flags_ns, flags=flags)
  vision = (30.0, 0.0, 0.0, False)
  slots = {5: rec(35.0)}                              # the second track, 6.5 m beyond the usual one
  assert order_mod.match(slots.items(), vision, NOW) == 5
  slots[2] = rec(28.5)
  assert order_mod.match(slots.items(), vision, NOW, held=[5]) == 2
  slots[7] = rec(28.6)                                # an equal one does not take it from the held slot
  assert order_mod.match(slots.items(), vision, NOW, held=[2]) == 2
  assert order_mod.match(slots.items(), vision, NOW, held=[2], taken=[2]) == 7
  slots[2] = rec(28.5, flags_ns=NOW, flags=0xA240)    # a coasting track loses to one radard would take
  assert order_mod.match(slots.items(), vision, NOW, held=[2]) == 7
  slots[2] = rec(28.5, flags_ns=NOW, flags=0xE260)    # a ghost never answers
  slots[7] = rec(28.6, kin_ns=NOW - 4_000_000_000)    # nor a read too old to say where the car is
  assert order_mod.match(slots.items(), vision, NOW, held=[7]) == 5
  assert order_mod.match({1: rec(28.5, y_m=2.6)}.items(), vision, NOW) is None
  can = (28.0, 0.0, 0.0, True)                        # against a CAN track the second track is too far on
  assert order_mod.match({5: rec(35.0), 1: rec(29.5)}.items(), can, NOW) == 1
  assert order_mod.match({5: rec(35.0)}.items(), can, NOW) is None


def _answer(built, radar, start, reads):
  """Ask `reads` times and answer each from `radar` ({slot: dict(x, v, y, flags)}). Returns [(time, slot, offset)]."""
  logger, asked = built.logger, []
  for i in range(reads):
    now = start + i * STEP
    built.observe_can([], now, v_ego=20.0)
    slot, offset, size = logger._next_read_task_locked(now)
    asked.append((now, slot, offset))
    car = radar.get(slot)
    if car is None:
      data = bytes(size)
    elif offset == xcp_mod.KIN_OFFSET:
      data = int(car["x"] * 128).to_bytes(2, "big", signed=True) + int(car["v"] * 256).to_bytes(2, "big", signed=True) + bytes(2)
    elif offset == xcp_mod.LAT_OFFSET:
      data = int(car["y"] * 128).to_bytes(2, "big", signed=True) + bytes(4)
    elif offset == 2:
      data = (50).to_bytes(2, "big") + bytes(4)
    else:
      data = car["flags"].to_bytes(2, "big") if offset == 106 else bytes(2)
    logger._handle_f4_reply_locked(slot, offset, size, b"\xff" + data, now + STEP // 2, now)
  return asked


def test_the_lead_is_read_every_radar_cycle_and_the_whole_list_every_three_seconds(monkeypatch):
  lead = [(43.0, 0.0, 19.0, 0.95)]
  built = _led_recorder(monkeypatch, lead)
  try:
    radar = {5: dict(x=41.5, v=-1.0, y=0.0, flags=0xE240),      # the lead
             11: dict(x=30.0, v=-2.0, y=3.5, flags=0xE240),     # a car beside the lane, nearer than the lead
             20: dict(x=60.0, v=-20.0, y=8.0, flags=0xE240),    # roadside clutter
             30: dict(x=50.0, v=-20.0, y=0.0, flags=0x0000)}    # words the radar left in a slot it freed
    asked = _answer(built, radar, NOW, 480)                     # ten seconds
    assert built.order.leads == [5] and not built.order.failed
    kin = {s: [t for t, slot, off in asked if slot == s and off == xcp_mod.KIN_OFFSET] for s in range(xcp_mod.NUM_SLOTS)}
    settled = [t for t in kin[5] if t > NOW + 4_000_000_000]
    assert max(b - a for a, b in zip(settled, settled[1:])) == 3 * STEP          # 62 ms: every radar cycle, never late
    for slot, times in kin.items():
      assert times and max(b - a for a, b in zip([NOW] + times, times + [asked[-1][0]])) < 3_500_000_000, slot
    assert len(kin[11]) > 4 * len(kin[20])                      # what may become the lead is read far more often
    assert len(kin[30]) <= 5                                    # a freed slot does not draw reads by being dropped
    lat = [t for t, slot, off in asked if slot == 5 and off == xcp_mod.LAT_OFFSET and t > NOW + 4_000_000_000]
    assert max(b - a for a, b in zip(lat, lat[1:])) < 400_000_000
    assert 30 not in built.logger.active_slots and not built.logger._init_queue
  finally:
    built.close()


def test_two_leads_on_a_slow_radar_leave_every_other_read_to_the_rest_of_the_list(monkeypatch):
  lead = [(43.0, 0.0, 19.0, 0.95)]
  built = _led_recorder(monkeypatch, lead)
  monkeypatch.setattr(sys.modules[__name__], "STEP", 31_000_000)       # 32 reads a second: two leads alone would take them all
  try:
    radar = {5: dict(x=41.5, v=-1.0, y=0.0, flags=0xE240), 7: dict(x=21.5, v=-3.0, y=0.3, flags=0xE240),
             11: dict(x=30.0, v=-2.0, y=3.5, flags=0xE240)}
    _answer(built, radar, NOW, 100)
    _can(built.logger, NOW + 100 * STEP, (40.0, 0.0, -1.0), (20.0, 0.3, -3.0))
    start = NOW + 100 * STEP
    built.logger._op_tracks_ns = start + 400 * STEP                    # the CAN tracks stay fresh for the run
    asked = _answer(built, radar, start, 320)                          # ten seconds
    assert built.order.leads == [7, 5]
    lead_kin = sum(1 for _, slot, off in asked if slot in (5, 7) and off == xcp_mod.KIN_OFFSET)
    assert 0.4 * len(asked) < lead_kin <= 0.5 * len(asked) + 1
    others = {slot for _, slot, off in asked if off == xcp_mod.KIN_OFFSET} - {5, 7}
    assert len(others) == xcp_mod.NUM_SLOTS - 2                        # the whole list was still read
    radar.pop(11)                                                      # the car beside leaves: the logger drops its slot
    _answer(built, radar, start + 320 * STEP, 320)
    assert 11 not in built.logger.active_slots
  finally:
    built.close()


def test_a_failing_order_hands_the_reading_back_to_the_logger(monkeypatch):
  lead = [(61.5, 0.0, 19.0, 0.95)]
  built = _led_recorder(monkeypatch, lead)
  try:
    logger, order = built.logger, built.order
    built.observe_can([], NOW, v_ego=20.0)
    monkeypatch.setattr(order_mod, "match", lambda *a, **k: 1 / 0)
    assert logger._select_priority_slots_locked() == []            # the logger's own answer, not an exception
    assert order.failed and logger._select_priority_slots_locked == order.own_priority
    assert logger._next_read_task_locked == order.own_task
    built.observe_can([], NOW + 1, v_ego=20.0)
    assert not built.stopped
  finally:
    built.close()
  monkeypatch.undo()

  built = _led_recorder(monkeypatch, lead)
  try:
    monkeypatch.setattr(order_mod, "references", lambda *a, **k: 1 / 0)
    assert built.logger._next_read_task_locked(NOW) is not None    # the read loop goes on in the logger's order
    assert built.order.failed
  finally:
    built.close()
  monkeypatch.undo()

  built = _led_recorder(monkeypatch, lead)
  try:
    built.lead_source = lambda: 1 / 0                              # the model socket failing: on from the CAN tracks
    built.observe_can([], NOW, v_ego=20.0)
    assert built.lead_source is None and not built.order.failed and not built.stopped
    assert built.logger._next_read_task_locked == built.order.task
  finally:
    built.close()


def test_closing_the_recorder_closes_the_model_socket(monkeypatch):
  closed = []
  built = _accord_recorder(monkeypatch)
  built.lead_source = SimpleNamespace(close=lambda: closed.append(True))
  built.close()
  assert closed == [True]


def test_the_table_carries_the_vision_lead_and_the_connect_reply_is_logged_once(monkeypatch):
  lead = [(61.5, 0.25, 19.0, 0.95)]
  built = _led_recorder(monkeypatch, lead)
  try:
    seen = _slot_events(monkeypatch)
    logger = built.logger
    logger.in_flight = ("CONNECT", -1, 0, 0, NOW)
    logger._process_dto_locked(b"\xff\x05\x80\x08\x08\x00\x01\x01", NOW)
    assert logger.state == "NEED_STATUS"                            # the logger still got its reply
    logger.in_flight = ("CONNECT", -1, 0, 0, NOW)
    logger._process_dto_locked(b"\xff\x01\x80\x08\x08\x00\x01\x01", NOW)
    connects = [kw for name, kw in seen if name == "array_b_recorder_connect"]
    assert connects == [dict(reply="ff05800808000101", daq=True)]
    logger.state = "READY"
    built.observe_can([], NOW, v_ego=20.0)
    table = [kw for name, kw in seen if name == "xcp_array_b_slots"][0]
    assert table["vision"] == [61.5, 0.25, 19.0, 0.95] and table["vision_age_ms"] == 0
    assert table["lead_slots"] == [] and table["unanswered"] == 1 and table["order_failed"] is False
  finally:
    built.close()
