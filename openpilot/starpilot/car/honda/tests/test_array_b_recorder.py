"""The fork gate around the Array B logger: which cars and firmware it runs on, the layout it applies, the candidate
rotation for a firmware whose address is not known yet, and when it gives up."""
from __future__ import annotations

from types import SimpleNamespace

import pytest

from opendbc.car.honda.values import CAR, HondaFlags
from openpilot.starpilot.car.honda import array_b_recorder as recorder
import openpilot.starpilot.car.honda.xcp_radar_logger as xcp_mod


def _fw(version: bytes, *, ecu="fwdRadar", address=recorder.RADAR_ADDRESS, logging=False):
  return SimpleNamespace(ecu=ecu, address=address, fwVersion=version, logging=logging)


def _cp(firmware=(), *, fingerprint=CAR.HONDA_ACCORD, radar_unavailable=False, passive=False):
  return SimpleNamespace(brand="honda", carFingerprint=fingerprint, flags=int(HondaFlags.BOSCH),
                         radarUnavailable=radar_unavailable, passive=passive, carFw=list(firmware))


@pytest.fixture(autouse=True)
def _restore_layout():
  saved = (xcp_mod.ARRAY_B_BASE, xcp_mod.REC0, xcp_mod.KIN_OFFSET, xcp_mod.LAT_OFFSET)
  yield
  (xcp_mod.ARRAY_B_BASE, xcp_mod.REC0, xcp_mod.KIN_OFFSET, xcp_mod.LAT_OFFSET) = saved


def test_radar_firmware_takes_the_non_logging_radar_entry():
  cp = _cp([_fw(b"junk", ecu="eps"), _fw(b"\x0e74522", logging=True), _fw(b"36802-TVA-A170\x00\x00")])
  assert recorder.radar_firmware(cp) == b"36802-TVA-A170"
  assert recorder.radar_firmware(_cp([])) is None


def test_unlisted_firmware_is_not_polled():
  assert b"36802-ZZZ-Z999" not in recorder.ARRAY_B_BASES_BY_FIRMWARE
  assert recorder.build(_cp([_fw(b"36802-ZZZ-Z999\x00\x00")]), send_lock=None, pm=None, params=None) is None
  assert recorder.build(_cp([]), send_lock=None, pm=None, params=None) is None


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
