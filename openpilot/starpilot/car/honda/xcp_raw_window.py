"""Fork: record a raw window of the Bosch-A radar's memory, for a firmware whose track list has not been located.

Rides the verbatim logger's session and its one-request-in-flight probe hook (xcp_radar_logger.py stays untouched):
every read goes to the window, front to back, over and over, and the bytes land in the rlog as xcp_raw_window events
of up to BLOCK bytes. Nothing is decoded here and nothing reaches control. A chunk's time lies between its block's
start_ns and end_ns, in order.
"""
from __future__ import annotations

from typing import NamedTuple

from openpilot.common.swaglog import cloudlog
from openpilot.starpilot.car.honda.xcp_radar_logger import XcpRadarLogger

BLOCK = 1024      # bytes per rlog event
MAX_CHUNK = 7     # one SHORT_UPLOAD carries at most 7 bytes


class RawWindow(NamedTuple):
  address: int
  length: int
  chunk: int      # bytes per read; 4 keeps a 32-bit value in one read


class RawWindowLogger(XcpRadarLogger):
  def __init__(self, CP, *, window: RawWindow, **kwargs):
    kwargs["probe_wayb"] = True   # the hook the window reads ride on
    super().__init__(CP, **kwargs)
    if not 1 <= window.chunk <= MAX_CHUNK or window.length <= 0:
      raise ValueError(f"bad raw window {window}")
    self.window = window
    self._raw_offset = 0
    self._raw_pending: tuple[int, int] | None = None
    self._raw_block = bytearray()
    self._raw_block_offset = 0
    self._raw_block_ns = 0
    self._raw_missing: list[int] = []

  def _primary_lead_urgent_locked(self, now_ns: int) -> bool:
    return False   # no track list is being read, so nothing outranks the window

  def _next_wayb_task_locked(self, now_ns: int) -> tuple[str, int, int]:
    if self._raw_pending is not None:   # the last read got an error or no answer: keep the offsets aligned
      offset, size = self._raw_pending
      self._raw_missing.append(offset)
      self._raw_store(offset, bytes(size), now_ns)
    offset = self._raw_offset
    size = min(self.window.chunk, self.window.length - offset)
    self._raw_offset = offset + size if offset + size < self.window.length else 0
    self._raw_pending = (offset, size)
    return (f"raw:{offset}", self.window.address + offset, size)

  def _handle_wayb_reply_locked(self, tag: str, size: int, payload: bytes, now_ns: int) -> None:
    if self._raw_pending is None or len(payload) < 1 + size:
      return
    self._raw_store(self._raw_pending[0], bytes(payload[1:1 + size]), now_ns)

  def _raw_store(self, offset: int, data: bytes, now_ns: int) -> None:
    self._raw_pending = None
    if not self._raw_block:
      self._raw_block_offset, self._raw_block_ns = offset, int(now_ns)
    self._raw_block += data
    if len(self._raw_block) >= BLOCK or offset + len(data) >= self.window.length:
      cloudlog.event("xcp_raw_window", base=self.window.address, offset=self._raw_block_offset,
                     start_ns=self._raw_block_ns, end_ns=int(now_ns), chunk=self.window.chunk,
                     missing=self._raw_missing, data=bytes(self._raw_block).hex())
      self._raw_block = bytearray()
      self._raw_missing = []
