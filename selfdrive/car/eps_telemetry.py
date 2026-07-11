"""eps_telemetry.py -- poll the 2020 Accord EPS gentle-EME RAM telemetry over UDS.

Sends a UDS ReadDataByIdentifier (DID 0x4801) to the EPS diagnostic tester address
0x18DA30F1 on bus 1 and reassembles the ISO-TP response from 0x18DAF130, which the
V31U EPS firmware answers with 8 data bytes = 4x little-endian u16:

    [voter-MAX torque, voter-AVG torque, |column torque|, steering angle]

These are the gentle-EME globals (gp-0x6a62 / gp-0x6a5e / gp-0x4f68 / gp-0x6cc4).
Logged into the rlog (service "epsTelemetry") they capture the ~90 ms torque
disengage with the LKAS voters live, time-aligned with the 399/427 CAN messages
already in the log.

This runs INSIDE card (selfdrive/car/card.py) so it reuses card's sole `sendcan`
publisher and the `can` stream card already drains every tick -- no second panda,
no Y-splitter, no extra process racing on sendcan.

Safety:
  * Every transmitted frame is a UDS ReadDataByIdentifier (SID 0x22), TesterPresent
    (SID 0x3E), or ISO-TP flow control (0x30) -- all read-only, zero ECU state change.
  * The panda Honda safety model INDEPENDENTLY enforces that only these frames are
    allowed on 0x18DA30F1 (opendbc/safety/modes/honda.h). A bug here cannot bypass
    that on-panda guard; worst case is a dropped/blocked frame.
  * Polling is ON by default (the `EpsTelemetryEnabled` param defaults to 1); set that
    param to 0 to stop all diagnostic CAN TX. Honda cars only (the poller is not even
    constructed on other brands).

Read logic is ported from the bench tool tools/bench_uds_telem_read.py in the
accord-eps-torque-mod kit (the proven working transport for this ECU).
"""
import struct
import time

from opendbc.car.can_definitions import CanData

EPS_TX_ADDR = 0x18DA30F1   # UDS request (tester -> A160 EPS)
EPS_RX_ADDR = 0x18DAF130   # UDS response (EPS -> tester)
EPS_BUS = 1                # comma bus 1 = the camera/EPS diagnostic bus
DEFAULT_DID = 0x4801

# ISO-TP PCI type nibbles (high nibble of byte 0)
_PCI_SF = 0x0  # single frame
_PCI_FF = 0x1  # first frame
_PCI_CF = 0x2  # consecutive frame
# (0x3 = flow control, which we transmit)

_REQUEST_TIMEOUT = 0.10        # s -- safety net: re-request if no complete response
_TESTER_PRESENT_PERIOD = 1.0   # s -- keep the diagnostic session warm


class EpsTelemetrySample:
  __slots__ = ("valid", "voter_max", "voter_avg", "col_torque", "angle", "did",
               "request_mono_ns", "response_mono_ns", "raw")

  def __init__(self, valid, voter_max, voter_avg, col_torque, angle, did,
               request_mono_ns, response_mono_ns, raw):
    self.valid = valid
    self.voter_max = voter_max
    self.voter_avg = voter_avg
    self.col_torque = col_torque
    self.angle = angle
    self.did = did
    self.request_mono_ns = request_mono_ns
    self.response_mono_ns = response_mono_ns
    self.raw = raw


class EpsTelemetryPoller:
  """Single-outstanding-request UDS poller driven one card tick at a time.

  Call update(can_list, now_ns, enabled) every card step. It scans the tick's CAN
  frames for the ECU response, drives the ISO-TP reassembly state machine, and
  issues the next request when idle. Returns a list of EpsTelemetrySample (0 or 1
  in practice) for card to publish as `epsTelemetry`.
  """

  def __init__(self, can_send, did=DEFAULT_DID, bus=EPS_BUS,
               tx_addr=EPS_TX_ADDR, rx_addr=EPS_RX_ADDR):
    self._can_send = can_send
    self.did = did
    self.bus = bus
    self.tx_addr = tx_addr
    self.rx_addr = rx_addr

    self._awaiting = False          # a request is outstanding
    self._request_mono = 0.0        # time.monotonic() when it was sent (for timeout)
    self._request_mono_ns = 0       # matching monotonic-ns for logging
    self._rx_buf = bytearray()      # ISO-TP reassembly buffer (UDS payload bytes)
    self._rx_expected = 0           # declared ISO-TP length
    self._last_tester_present = 0.0

  # --- transmit helpers ------------------------------------------------------
  def _send_isotp(self, payload):
    """Send a single-frame ISO-TP request (len prefix + payload, zero padded)."""
    frame = (bytes([len(payload)]) + bytes(payload)).ljust(8, b"\x00")
    self._can_send([CanData(self.tx_addr, frame, self.bus)])

  def _send_flow_control(self):
    """Clear-to-send flow control (0x30, BS=0, STmin=0)."""
    self._can_send([CanData(self.tx_addr, b"\x30\x00\x00\x00\x00\x00\x00\x00", self.bus)])

  # --- state ----------------------------------------------------------------
  def _reset(self):
    self._rx_buf = bytearray()
    self._rx_expected = 0
    self._awaiting = False

  def _decode(self, uds_payload, now_ns):
    """uds_payload = 62 <did_hi> <did_lo> <8 data bytes>. Return a sample."""
    if len(uds_payload) >= 3 and uds_payload[0] == 0x62:
      data = uds_payload[3:]
      if len(data) >= 8:
        mx, av, tq, an = struct.unpack_from("<HHHH", data, 0)
        return EpsTelemetrySample(True, mx, av, tq, an, self.did,
                                  self._request_mono_ns, now_ns, bytes(uds_payload))
    # negative response (7F ..) or short/unexpected payload -> still log it invalid
    return EpsTelemetrySample(False, 0, 0, 0, 0, self.did,
                              self._request_mono_ns, now_ns, bytes(uds_payload))

  # --- main tick ------------------------------------------------------------
  def update(self, can_list, now_ns, enabled):
    """can_list: [(nanos, [(addr, dat, src), ...]), ...] from can_capnp_to_list."""
    if not enabled:
      if self._awaiting or self._rx_buf:
        self._reset()
      return []

    now = time.monotonic()
    samples = []

    # 1) consume any response frames received this tick
    for _nanos, frames in can_list:
      for addr, dat, src in frames:
        if src != self.bus or addr != self.rx_addr or len(dat) == 0:
          continue
        pci = dat[0] >> 4
        if pci == _PCI_SF:                       # single frame (negative/short resp)
          n = dat[0] & 0x0F
          samples.append(self._decode(bytes(dat[1:1 + n]), now_ns))
          self._reset()
        elif pci == _PCI_FF:                     # first frame -> ack with flow control
          self._rx_expected = ((dat[0] & 0x0F) << 8) | dat[1]
          self._rx_buf = bytearray(dat[2:8])
          self._send_flow_control()
        elif pci == _PCI_CF:                      # consecutive frame
          if self._rx_expected:
            self._rx_buf += bytes(dat[1:8])
            if len(self._rx_buf) >= self._rx_expected:
              samples.append(self._decode(bytes(self._rx_buf[:self._rx_expected]), now_ns))
              self._reset()

    # 2) periodic TesterPresent keepalive (suppress positive response: 3E 80 -> no reply)
    if (now - self._last_tester_present) >= _TESTER_PRESENT_PERIOD:
      self._send_isotp([0x3E, 0x80])
      self._last_tester_present = now

    # 3) drop a stalled request, then issue the next one when idle (max round-trip rate)
    if self._awaiting and (now - self._request_mono) >= _REQUEST_TIMEOUT:
      self._reset()
    if not self._awaiting:
      self._awaiting = True
      self._request_mono = now
      self._request_mono_ns = now_ns
      self._rx_buf = bytearray()
      self._rx_expected = 0
      self._send_isotp([0x22, (self.did >> 8) & 0xFF, self.did & 0xFF])

    return samples
