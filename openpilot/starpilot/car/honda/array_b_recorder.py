"""Fork gate around the verbatim Bosch-A Array B logger (xcp_radar_logger.py, from bob-unify-car).

The logger reads the radar's internal track list at 20 Hz and only records it (rlog event xcp_array_b_20hz and the
cyan overlay). Nothing here reaches control. This module decides whether it runs at all, and with which layout:

  - only on a verified Bosch-A car (the logger itself would start on every Bosch Honda with a radar);
  - only for a radar firmware whose Array B address is known. The address moves between builds (A150 0x400371A4,
    A160 0x400371C4), so a firmware with no entry is logged and not polled;
  - the 1 Hz internal-fusion probe only on the firmware its addresses were read from;
  - it stops for the rest of the drive after repeated lost sessions, or when the radar never answers.
"""
from __future__ import annotations

from openpilot.common.swaglog import cloudlog

RADAR_ADDRESS = 0x18DAB0F1
RECORD0_OFFSET = 0x10  # first record sits 16 bytes after the array base

# radar firmware -> Array B base. A150: live on the Civic (Peter). A160: static, from the Civic image.
ARRAY_B_BASE_BY_FIRMWARE: dict[bytes, int] = {
  b"36802-TBA-A150": 0x400371A4,
  b"36802-TBA-A160": 0x400371C4,
}
INTERNAL_FUSION_PROBE_FIRMWARE = frozenset({b"36802-TBA-A150"})

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
  """The logger's card-facing surface, plus the give-up rule."""

  def __init__(self, logger):
    self.logger = logger
    self.armed_ns = 0
    self.was_ready = False
    self.ever_ready = False
    self.session_losses = 0
    self.stopped = False

  def _watch(self, now_ns: int) -> None:
    if self.stopped or not self.logger.enabled:
      return
    ready = self.logger.state == "READY"
    if ready:
      self.ever_ready = True
    elif self.was_ready:
      self.session_losses += 1
    self.was_ready = ready
    never_answered = self.armed_ns and not self.ever_ready and now_ns - self.armed_ns > CONNECT_DEADLINE_NS
    if self.session_losses >= MAX_SESSION_LOSSES or never_answered:
      self.stopped = True
      self.logger.enabled = False
      self.logger.close()
      cloudlog.event("array_b_recorder_stopped", session_losses=self.session_losses, ever_ready=self.ever_ready)

  def observe_can(self, can_list, now_ns: int, v_ego: float = 0.0) -> None:
    self.logger.observe_can(can_list, now_ns, v_ego=v_ego)
    self._watch(now_ns)

  def next_can_sends(self, now_ns: int):
    if self.stopped:
      return []
    if not self.armed_ns:
      self.armed_ns = int(now_ns)
    return self.logger.next_can_sends(now_ns)

  def observe_radar_tracks(self, RD, now_ns: int) -> None:
    self.logger.observe_radar_tracks(RD, now_ns)

  def close(self) -> None:
    self.logger.close()


def build(CP, *, send_lock, pm, params) -> ArrayBRecorder | None:
  """The recorder for this car, or None. Never raises: card must start without it."""
  try:
    if not supported(CP):
      return None
    firmware = radar_firmware(CP)
    base = ARRAY_B_BASE_BY_FIRMWARE.get(firmware) if firmware is not None else None
    cloudlog.event("array_b_recorder", firmware=None if firmware is None else firmware.decode(errors="replace"),
                   base=base, polling=base is not None)
    if base is None:
      return None

    from openpilot.starpilot.car.honda import xcp_radar_logger
    xcp_radar_logger.ARRAY_B_BASE = base
    xcp_radar_logger.REC0 = base + RECORD0_OFFSET
    logger = xcp_radar_logger.XcpRadarLogger(CP, send_lock=send_lock, pm=pm, params=params,
                                             probe_wayb=firmware in INTERNAL_FUSION_PROBE_FIRMWARE)
    return ArrayBRecorder(logger) if logger.enabled else None
  except Exception:
    cloudlog.exception("array_b_recorder: not started")
    return None
