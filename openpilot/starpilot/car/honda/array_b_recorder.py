"""Fork gate around the verbatim Bosch-A Array B logger (xcp_radar_logger.py, from bob-unify-car).

The logger reads the radar's internal track list at 20 Hz and only records it (rlog event xcp_array_b_20hz).
Nothing here reaches control. This module decides whether it runs at all, and with which layout:

  - only on a verified Bosch-A car (the logger itself would start on every Bosch Honda with a radar);
  - only for a radar firmware with an Array B address to read. The address moves between builds (A150 0x400371A4,
    A160 0x400371C4), so a firmware with no entry is logged and not polled;
  - a firmware whose address is not known yet lists the candidates, and the recorder takes them in turn, one slice
    each, with an array_b_recorder_layout event at every change. The slice whose targets line up with the CAN
    tracks names the address; it is then pinned here as a single entry;
  - the 1 Hz internal-fusion probe only on the firmware its addresses were read from;
  - it stops for the rest of the drive after repeated lost sessions, or when the radar never answers.
"""
from __future__ import annotations

from openpilot.common.swaglog import cloudlog

RADAR_ADDRESS = 0x18DAB0F1
RECORD0_OFFSET = 0x10  # first record sits 16 bytes after the array base

A150_BASE = 0x400371A4  # live on the Civic (Peter)
A160_BASE = 0x400371C4  # static, from the Civic A160 image

# radar firmware -> Array B base(s). More than one: not known yet, recorded in turn.
ARRAY_B_BASES_BY_FIRMWARE: dict[bytes, tuple[int, ...]] = {
  b"36802-TBA-A150": (A150_BASE,),
  b"36802-TBA-A160": (A160_BASE,),
  b"36802-TVA-A170": (A160_BASE, A150_BASE),  # the operator's Accord; neither is confirmed for this build
}
INTERNAL_FUSION_PROBE_FIRMWARE = frozenset({b"36802-TBA-A150"})

SLICE_NS = 120_000_000_000              # time on one candidate address before the next
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


class ArrayBRecorder:
  """The logger's card-facing surface, plus the candidate rotation and the give-up rule."""

  def __init__(self, make_logger, bases: tuple[int, ...]):
    self.make_logger = make_logger   # base -> a started logger
    self.bases = bases
    self.index = 0
    self.logger = make_logger(bases[0])
    self.slice_start_ns = 0
    self.armed_ns = 0
    self.was_ready = False
    self.ever_ready = False
    self.session_losses = 0
    self.stopped = False

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
    self.slice_start_ns = int(now_ns)
    self.was_ready = False
    cloudlog.event("array_b_recorder_layout", base=self.bases[self.index], mono_ns=int(now_ns))

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
    self.logger.observe_can(can_list, now_ns, v_ego=v_ego)
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


def build(CP, *, send_lock, pm, params) -> ArrayBRecorder | None:
  """The recorder for this car, or None. Never raises: card must start without it."""
  try:
    if not supported(CP):
      return None
    firmware = radar_firmware(CP)
    bases = ARRAY_B_BASES_BY_FIRMWARE.get(firmware, ()) if firmware is not None else ()
    cloudlog.event("array_b_recorder", firmware=None if firmware is None else firmware.decode(errors="replace"),
                   bases=list(bases), polling=bool(bases))
    if not bases:
      return None

    from openpilot.starpilot.car.honda import xcp_radar_logger

    def make_logger(base: int):
      xcp_radar_logger.ARRAY_B_BASE = base
      xcp_radar_logger.REC0 = base + RECORD0_OFFSET
      return xcp_radar_logger.XcpRadarLogger(CP, send_lock=send_lock, pm=pm, params=params,
                                             probe_wayb=firmware in INTERNAL_FUSION_PROBE_FIRMWARE)

    recorder = ArrayBRecorder(make_logger, bases)
    if not recorder.logger.enabled:
      recorder.close()
      return None
    cloudlog.event("array_b_recorder_layout", base=bases[0], mono_ns=0)
    return recorder
  except Exception:
    cloudlog.exception("array_b_recorder: not started")
    return None
