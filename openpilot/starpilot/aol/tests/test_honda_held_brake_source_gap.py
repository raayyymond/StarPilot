"""One late carState while the Accord's brake is held in lateral-only must not end always-on lateral.

2026-11-11 route 00000038 t=476.297: cruise off, brake held, 3 m/s. CAN and carState arrived 28.1 ms after the
previous sample and selfdrived published 30.9 ms after it, past the 30 ms output lease. The fresh carState could not
renew the lease because the held brake is a user disable, and the car raised controlsMismatch. Route 00000039
t=261.170 had a 26.4 ms gap with no pedal and renewed.
"""
from contextlib import ExitStack
from unittest import mock

import pytest

from openpilot.cereal import log
from openpilot.common.prefix import OpenpilotPrefix
from openpilot.starpilot.aol.tests import test_ioniq_availability as fixture
from openpilot.starpilot.aol.tests.test_hkg_early_long_withdrawal import BASE, wire
from openpilot.starpilot.aol.tests.test_honda_early_long_withdrawal import engaged

Mismatch = log.OnroadEvent.EventName.controlsMismatch
HELD = dict(brake=True, native_request_lat=True, native_request_long=False, panda_controls_allowed=False)
GAP = dict(offset_ns=900_000)  # with tick 33 after tick 30: 30.9 ms past the last source, as selfdrived published it


def braking_in_lateral_only(case):
  sd, drive = engaged(case)
  drive(10, native_request_lat=True, native_request_long=True, long_allowed=False, panda_controls_allowed=False,
        safety_message=wire(sd, BASE + 100_000_000, health_controls=False))
  for tick in range(11, 31):
    axis, state, command = drive(tick, **HELD)
    assert Mismatch not in sd.events.names
    assert not state.enabled and axis.lateralActive and command.latActive and not command.longActive
  return sd, drive


def test_a_late_sample_under_a_held_brake_renews_lateral():
  case = fixture.IoniqAvailabilityTests(methodName='runTest')
  with ExitStack() as cleanup:
    cleanup.callback(case.doCleanups)
    cleanup.enter_context(OpenpilotPrefix())
    sd, drive = braking_in_lateral_only(case)
    axis, state, command = drive(33, **GAP, **HELD)
    assert Mismatch not in sd.events.names, 'what the car did at t=476.300'
    assert str(axis.faultReason) == 'none' and axis.lateralActive
    for tick in range(34, 40):
      axis, state, command = drive(tick, **HELD)
      assert Mismatch not in sd.events.names and axis.lateralActive and command.latActive
      assert not state.enabled and not command.longActive


def test_without_the_honda_renewal_the_same_gap_is_the_recorded_fault():
  case = fixture.IoniqAvailabilityTests(methodName='runTest')
  with ExitStack() as cleanup:
    cleanup.callback(case.doCleanups)
    cleanup.enter_context(OpenpilotPrefix())
    sd, drive = braking_in_lateral_only(case)
    cleanup.enter_context(mock.patch(
      'openpilot.selfdrive.selfdrived.selfdrived.honda_held_brake_renews_lateral', return_value=False))
    axis, state, command = drive(33, **GAP, **HELD)
    assert Mismatch in sd.events.names and str(axis.faultReason) == 'critical' and not axis.lateralActive


@pytest.mark.parametrize('denied', ('lateral', 'request'))
def test_the_renewal_still_needs_the_panda_to_grant_lateral(denied):
  case = fixture.IoniqAvailabilityTests(methodName='runTest')
  with ExitStack() as cleanup:
    cleanup.callback(case.doCleanups)
    cleanup.enter_context(OpenpilotPrefix())
    sd, drive = braking_in_lateral_only(case)
    kwargs = dict(HELD, native_request_lat=False) if denied == 'request' else dict(HELD, allowed=False)
    axis, state, command = drive(33, **GAP, **kwargs)
    assert not axis.lateralActive and not axis.longitudinalActive


def test_a_brake_release_inside_the_gap_does_not_renew():
  case = fixture.IoniqAvailabilityTests(methodName='runTest')
  with ExitStack() as cleanup:
    cleanup.callback(case.doCleanups)
    cleanup.enter_context(OpenpilotPrefix())
    sd, drive = braking_in_lateral_only(case)
    axis, state, command = drive(33, **GAP, **dict(HELD, brake=False, gas=True))
    assert not axis.longitudinalActive and not command.longActive and not state.enabled
