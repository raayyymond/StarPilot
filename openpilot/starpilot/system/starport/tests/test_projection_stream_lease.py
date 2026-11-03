import os
import tempfile
import threading
import time
from types import SimpleNamespace as NS

from openpilot.starpilot.system.starport import supervisor
from openpilot.starpilot.system.starport.projection_stream_lease import (MAX_AGE_S, StreamLeaseReader, StreamLeaseWriter,
                                                                         projection_stream_active)


def test_lease_is_active_only_while_fresh_and_ends_on_release():
  with tempfile.TemporaryDirectory() as directory:
    path = os.path.join(directory, 'lease')
    writer = StreamLeaseWriter(path)
    assert not projection_stream_active(path)
    writer.touch(now=100.0)
    assert projection_stream_active(path)
    mtime = os.stat(path).st_mtime
    assert projection_stream_active(path, now=mtime + MAX_AGE_S)
    assert not projection_stream_active(path, now=mtime + MAX_AGE_S + 0.5)
    writer.release()
    assert not projection_stream_active(path)
    writer.release()


def test_reader_stats_at_most_once_a_second():
  with tempfile.TemporaryDirectory() as directory:
    path = os.path.join(directory, 'lease')
    clock = [10.0]
    reader = StreamLeaseReader(path, clock=lambda: clock[0])
    assert reader() is False
    StreamLeaseWriter(path).touch()
    clock[0] = 10.9
    assert reader() is False  # cached
    clock[0] = 11.0
    assert reader() is True
    os.unlink(path)
    clock[0] = 11.5
    assert reader() is True
    clock[0] = 12.0
    assert reader() is False


class SendingSession:
  """The fakes follow test_stream_focus.py; this car keeps accepting frames."""

  def __init__(self, focused):
    self.focused, self.focus_epoch = focused, 1
    self.touch_events: list = []
    self.unacked = 0
    self.needs_keyframe = False
    self.sent = 0

  def pump(self, timeout):
    time.sleep(0.002)
    return False

  def check_progress(self):
    pass

  def stats(self):
    return {}

  def can_send(self):
    return True

  def send_frame(self, data, pts, keyframe):
    self.sent += 1


class CarSource:
  label = view = "car"
  frames = 0

  def __init__(self):
    self.source = NS(mark_sent=lambda _ns: None)

  def demand(self, _):
    pass

  def release_demand(self):
    pass

  def drain_controls(self):
    return []

  def check(self, now, focused):
    pass

  def latest(self):
    return NS(captured_ns=time.monotonic_ns(), pixel_format=None, data=b'x')


def run_stream(monkeypatch, session, lease):
  monkeypatch.setattr(supervisor.device_health, "sample", lambda interface: {})
  sup = supervisor.Supervisor.__new__(supervisor.Supervisor)
  sup.log = lambda name, **values: None
  sup._stop, sup._status = threading.Event(), {"state": ""}
  sup._stage = lambda *a, **k: None
  sup._set = lambda **values: None
  sup._native_reconnects, sup._native_reconnect_limit = 0, 0
  sup._stream_lease = lease
  sup._companion_keeper = None  # dev feeb87e45: the loop checks the hands-free keeper once a second
  encoder = NS(last_encode_ms=0.0, encode_rgba=lambda data, keyframe: (b'h264', keyframe))
  timer = threading.Timer(0.2, sup._stop.set)
  timer.start()
  try:
    sup._stream(session, encoder, CarSource(), NS(lost=None, wifi_dbm=None, lease=None), 1 / 30)
  finally:
    sup._stop.set()
    timer.cancel()


def test_supervisor_touches_the_lease_only_while_focused_frames_reach_the_car(monkeypatch):
  for focused in (True, False):
    with tempfile.TemporaryDirectory() as directory:
      path = os.path.join(directory, 'lease')
      session = SendingSession(focused)
      run_stream(monkeypatch, session, StreamLeaseWriter(path))
      assert session.sent > 0
      assert projection_stream_active(path) is focused
