"""A car-state source gap is announced only once it has lasted.

The always-on-lateral runtime treats a carState older than 30 ms as lost, withdraws both commands for that
frame and takes them back with the next fresh one. That handling stays as it is. What changes is the alert:
one late CAN batch used to sound "TAKE CONTROL IMMEDIATELY / Communication Issue" (2026-11-23 route 00000074,
t=359.97 on a 32 ms gap and t=866.93 on a 29 ms gap; the same on 2026-11-09 routes 0000002b and 0000002c).
Stock openpilot calls a 100 Hz service lost after 100 ms. A gap that outlasts GRACE_NS is announced exactly
as before, GRACE_NS late.
"""
GRACE_NS = 100_000_000
AVAILABILITY_WARNING = 'aolAvailability/warning'


def source_gap_alert(alert_type: str) -> bool:
  return alert_type == AVAILABILITY_WARNING or alert_type.startswith('commIssue/')


class SourceGapAlert:
  def __init__(self):
    self.since_ns = 0
    self.healed_ns = 0  # length of the last gap that ended unannounced, for the log

  def hold(self, gap: bool, now_ns: int) -> bool:
    """True while a source gap is still too young to announce."""
    self.healed_ns = 0
    if not gap or now_ns < self.since_ns:
      if self.since_ns and 0 <= now_ns - self.since_ns < GRACE_NS:
        self.healed_ns = max(1, now_ns - self.since_ns)
      self.since_ns = 0
      return False
    if not self.since_ns:
      self.since_ns = now_ns
    return now_ns - self.since_ns < GRACE_NS
