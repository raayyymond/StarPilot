"""Lets the comma's own screen go black (alerts only) while Android Auto is streaming the car view.

The supervisor touches the lease while frames are actually reaching the car; the native UI stats it
at most once a second. A crashed supervisor stops touching it, so the normal UI returns on its own.
"""
import time

from openpilot.starpilot.system.starport.projection_map_lease import LeaseWriter, lease_active

DEFAULT_PATH = '/tmp/starpilot-projection-stream.lease'
MAX_AGE_S = 2.0  # touched every second; a stopped stream restores the normal UI within ~3 s even without release()
CHECK_EVERY_S = 1.0


class StreamLeaseWriter(LeaseWriter):
  def __init__(self, path=DEFAULT_PATH):
    super().__init__(path)


def projection_stream_active(path=DEFAULT_PATH, now=None):
  return lease_active(path, MAX_AGE_S, now)


class StreamLeaseReader:
  """Callable answering projection_stream_active(), cached so the render loop stats the file at most once a second."""

  def __init__(self, path=DEFAULT_PATH, clock=time.monotonic):
    self.path, self._clock = str(path), clock
    self._checked = None
    self._active = False

  def __call__(self) -> bool:
    now = self._clock()
    if self._checked is None or now - self._checked >= CHECK_EVERY_S:
      self._checked = now
      self._active = projection_stream_active(self.path)
    return self._active
