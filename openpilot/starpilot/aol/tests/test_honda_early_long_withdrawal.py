"""The Accord's panda drops longitudinal on brake or CANCEL before selfdrived sees why.

2026-11-11 route 00000036: at t=231.655 (brake) and t=520.857 (CANCEL) selfdrived read the panda's mask, lateral
granted and longitudinal denied, 5 ms before the carState that carried the cause. pandaStates is 10 Hz and still
said controls allowed. It raised controlsMismatch. Nine other disengagements in the two routes of that day, where
the carState came first, were clean.
"""
from contextlib import ExitStack

import pytest

from openpilot.cereal import log
from openpilot.common.prefix import OpenpilotPrefix
from openpilot.starpilot.aol.intent import AolSettings
from openpilot.starpilot.aol.tests import test_ioniq_availability as fixture
from openpilot.starpilot.aol.tests.test_hkg_early_long_withdrawal import BASE, wire
from openpilot.starpilot.car.honda.tests.test_honda_cancel_lateral import stock_proof

Mismatch = log.OnroadEvent.EventName.controlsMismatch


def engaged(case):
  sd, _, drive = case._authority_callers(True, cp=stock_proof()[0], genuine_events=True)
  sd.aol_settings = AolSettings(True, 0., 0, 0, (0, 0, 0), (0, 0, 0))  # CANCEL unmapped, as on the car
  for tick in range(10):
    axis, state, command = drive(tick)
    assert state.enabled and axis.lateralActive and axis.longitudinalActive
    assert command.latActive and command.longActive
  return sd, drive


@pytest.mark.parametrize('health_caught_up', (False, True))
@pytest.mark.parametrize('cause', ('brake', 'cancel'))
def test_the_mask_ahead_of_its_cause_withdraws_longitudinal_and_is_not_a_fault(cause, health_caught_up):
  case = fixture.IoniqAvailabilityTests(methodName='runTest')
  with ExitStack() as cleanup:
    cleanup.callback(case.doCleanups)
    cleanup.enter_context(OpenpilotPrefix())
    sd, drive = engaged(case)
    now = BASE + 100_000_000
    axis, state, command = drive(10, native_request_lat=True, native_request_long=True, long_allowed=False,
      panda_controls_allowed=not health_caught_up, safety_message=wire(sd, now, health_controls=not health_caught_up))
    assert Mismatch not in sd.events.names, 'what the car did at t=231.655 and t=520.857'
    assert state.enabled and axis.lateralActive and command.latActive
    assert not axis.longitudinalActive and not command.longActive
    kwargs = dict(native_request_lat=True, native_request_long=True, long_allowed=False, panda_controls_allowed=False)
    if cause == 'brake':
      axis, state, command = drive(11, brake=True, **kwargs)
    else:
      cs = fixture.car_state()
      cs.buttonEvents = [{'type': 'cancel', 'pressed': True}]
      axis, state, command = drive(11, observed_state=cs, **kwargs)
    assert Mismatch not in sd.events.names
    assert not state.enabled and not axis.longitudinalActive and not command.longActive
    assert axis.lateralActive and command.latActive, 'always-on lateral stays'


def test_a_denied_lateral_grant_is_still_a_fault():
  case = fixture.IoniqAvailabilityTests(methodName='runTest')
  with ExitStack() as cleanup:
    cleanup.callback(case.doCleanups)
    cleanup.enter_context(OpenpilotPrefix())
    sd, drive = engaged(case)
    now = BASE + 100_000_000
    axis, state, command = drive(10, native_request_lat=True, native_request_long=True, long_allowed=False,
      panda_controls_allowed=False, safety_message=wire(sd, now, lateral=False))
    assert Mismatch in sd.events.names
    assert not state.enabled and not axis.lateralActive and not command.latActive


def test_a_denial_with_no_cause_ends_on_the_ordinary_two_second_mismatch():
  case = fixture.IoniqAvailabilityTests(methodName='runTest')
  with ExitStack() as cleanup:
    cleanup.callback(case.doCleanups)
    cleanup.enter_context(OpenpilotPrefix())
    sd, drive = engaged(case)
    for tick in range(10, 214):
      axis, state, command = drive(tick, native_request_lat=True, native_request_long=True,
        long_allowed=False, panda_controls_allowed=False)
      assert not axis.longitudinalActive and not command.longActive
      if Mismatch in sd.events.names:
        break
    assert sd.mismatch_counter >= 200 and not state.enabled
