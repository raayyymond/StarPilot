"""2026-11-23 route 00000074: a 32 ms and a 29 ms carState gap each sounded a Communication Issue alert."""
from openpilot.starpilot.aol.source_gap_alert import GRACE_NS, SourceGapAlert, source_gap_alert

T = 1_000_000_000
FRAME = 10_000_000


def test_a_gap_of_a_few_frames_is_not_announced():
  gate = SourceGapAlert()
  assert not gate.hold(False, T)
  assert gate.hold(True, T + FRAME), 'the frame of t=359.97'
  assert gate.hold(True, T + 2 * FRAME)
  assert not gate.hold(False, T + 3 * FRAME) and gate.healed_ns == 2 * FRAME, 'healed, and logged once'
  assert not gate.hold(False, T + 4 * FRAME) and not gate.healed_ns


def test_a_gap_that_lasts_is_announced_one_grace_late_and_stays_announced():
  gate = SourceGapAlert()
  assert gate.hold(True, T)
  assert gate.hold(True, T + GRACE_NS - 1)
  for later in (T + GRACE_NS, T + GRACE_NS + FRAME, T + 3_000_000_000):
    assert not gate.hold(True, later)
  assert not gate.hold(False, T + 3_000_000_000 + FRAME) and not gate.healed_ns, 'an announced gap is not logged as silent'


def test_each_gap_gets_its_own_grace_and_a_clock_step_back_starts_over():
  gate = SourceGapAlert()
  assert gate.hold(True, T) and not gate.hold(False, T + FRAME)
  assert gate.hold(True, T + 2 * FRAME) and gate.since_ns == T + 2 * FRAME
  assert not gate.hold(True, T) and not gate.since_ns, 'never held on a clock that went backwards'


def test_only_the_source_gap_presentations_are_held():
  assert source_gap_alert('commIssue/softDisable') and source_gap_alert('commIssue/noEntry')
  assert source_gap_alert('aolAvailability/warning')
  for other in ('commIssueAvgFreq/softDisable', 'controlsMismatch/immediateDisable', 'selfdrivedLagging/softDisable',
                'steerTempUnavailable/softDisable', 'canError/immediateDisable'):
    assert not source_gap_alert(other)


def lost_source(cleanup):
  from openpilot.starpilot.aol.tests.test_shared_source_availability import BASE, callers, intent
  sd, _, drive = callers(cleanup, 'honda_classic_bosch_long')
  drive(2, state_stamp=BASE, offset_ns=4_497_535, intent_message=intent(BASE, 3, latched=False))
  return sd, lambda tick: drive(tick, timeout=True, offset_ns=5_399_000, intent_message=intent(BASE, 3, latched=False))


def test_selfdrived_withdraws_on_the_gap_at_once_and_announces_it_only_after_the_grace():
  from contextlib import ExitStack
  with ExitStack() as cleanup:
    sd, gap = lost_source(cleanup)
    for tick in range(5, 15):
      axis, state, command = gap(tick)
      assert str(axis.faultReason) == 'availabilityPause', 'the handling is untouched'
      assert not command.latActive and not command.longActive
      assert state.alertType == '', 'ten frames, 100 ms: silent'
    for tick in range(15, 19):
      axis, state, command = gap(tick)
      assert state.alertType == 'commIssue/softDisable', 'a gap that lasts is announced as before'
      assert not command.latActive and not command.longActive


def test_selfdrived_without_the_grace_announces_the_first_frame():
  from contextlib import ExitStack
  from unittest import mock
  with ExitStack() as cleanup:
    cleanup.enter_context(mock.patch('openpilot.starpilot.car.honda.aol.source_gap_alert_grace', return_value=False))
    sd, gap = lost_source(cleanup)
    assert gap(5)[1].alertType == 'commIssue/softDisable', 'what the car did at t=359.97'
