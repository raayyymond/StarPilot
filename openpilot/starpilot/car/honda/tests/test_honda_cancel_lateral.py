from dataclasses import replace

import pytest
from opendbc.car import gen_empty_fingerprint, structs
from opendbc.car.honda.interface import CarInterface
from opendbc.car.honda.values import CAR
from openpilot.starpilot.aol.intent import AolSettings
from openpilot.starpilot.aol.runtime import AxisDecision
from openpilot.starpilot.aol.wire import IntentState, PandaSlot, SafetyState
from openpilot.starpilot.car.honda.aol import physical_cancel_lateral_allowed

ANGLE_EPS_FW = [structs.CarParams.CarFw(ecu=structs.CarParams.Ecu.eps, fwVersion=b"39990-TVA,A16C\x00\x00",
                                        address=0x18da30f1, subAddress=0, brand='honda')]
BOSCH_LONG_AOL = 34  # BOSCH_LONG | the AOL marker selfdrived adds for the classic alpha-long owners


def proof(identity, *, alpha_long=True, car_fw=()):
  cp = CarInterface.get_params(identity, gen_empty_fingerprint(), list(car_fw), alpha_long, False, False)
  cp.safetyConfigs[0].safetyParam = BOSCH_LONG_AOL
  cs = structs.CarState(canValid=True, vEgo=20., gearShifter=structs.CarState.GearShifter.drive)
  cs.cruiseState.available = True
  cs.buttonEvents = [structs.CarState.ButtonEvent(type=structs.CarState.ButtonEvent.Type.cancel, pressed=True)]
  settings = AolSettings(True, 0., 0, 0, (0, 0, 0), (0, 0, 0))
  intent = IntentState('card-session', 1, 1_000_000_000, 1_000_000_000, 1_200_000_000,
                       True, False, False, True, True)
  native = SafetyState(1, True, 1_000_000_000, 1_200_000_000, int(structs.CarParams.SafetyModel.hondaBosch),
                       BOSCH_LONG_AOL, True, False, True, False, 'honda-panda', 'axis-session', 1_000_000_000,
                       (PandaSlot(0, 'honda-panda', int(structs.CarParams.SafetyModel.hondaBosch), BOSCH_LONG_AOL, 0),))
  context = {'settings': settings, 'previous': AxisDecision(True, False, True, False, True),
             'source_ns': 1_000_000_000, 'now_ns': 1_010_000_000, 'axis_session_id': 'axis-session'}
  return cp, cs, intent, native, context


def test_accord_angle_cancel_keeps_existing_native_lateral_and_handles_release():
  cp, cs, intent, native, context = proof(CAR.HONDA_ACCORD, car_fw=ANGLE_EPS_FW)
  assert cp.steerControlType == structs.CarParams.SteerControlType.angle
  assert physical_cancel_lateral_allowed(cp, cs, intent, native, **context)
  cs.buttonEvents[0].pressed = False
  assert physical_cancel_lateral_allowed(cp, cs, intent, native, **context)


@pytest.mark.parametrize('identity', (CAR.HONDA_ACCORD, CAR.HONDA_CIVIC_BOSCH, CAR.HONDA_CRV_5G))
def test_classic_bosch_alpha_long_cancel_keeps_lateral(identity):
  cp, cs, intent, native, context = proof(identity)
  assert physical_cancel_lateral_allowed(cp, cs, intent, native, **context)


def test_stock_cruise_accord_is_not_covered():
  cp, cs, intent, native, context = proof(CAR.HONDA_ACCORD, alpha_long=False, car_fw=ANGLE_EPS_FW)
  assert cp.pcmCruise
  assert not physical_cancel_lateral_allowed(cp, cs, intent, native, **context)


@pytest.mark.parametrize('failure', (
  'disabled', 'unarmed', 'paused',
  'intent_stale', 'source_mismatch', 'main_off', 'invalid_can', 'timeout', 'temporary_eps', 'permanent_eps',
  'acc_fault', 'door', 'no_cancel', 'native_missing', 'native_denied', 'native_unrequested', 'native_stale',
  'native_session', 'native_model', 'native_param', 'native_rx_fault', 'native_fault', 'no_inventory',
  'no_previous', 'previous_inactive', 'dashcam', 'nidec',
))
def test_cancel_exception_never_invents_intent_or_permission(failure):
  cp, cs, intent, native, context = proof(CAR.HONDA_ACCORD, car_fw=ANGLE_EPS_FW)
  if failure == 'disabled':
    context['settings'] = replace(context['settings'], enabled=False)
  elif failure == 'unarmed':
    intent = replace(intent, allowedLatch=False, lateralArmed=False)
  elif failure == 'paused':
    intent = replace(intent, pauseLateral=True)
  elif failure == 'intent_stale':
    intent = replace(intent, observedMonoTime=700_000_000, carStateLogMonoTime=700_000_000)
  elif failure == 'source_mismatch':
    context['source_ns'] -= 1
  elif failure == 'main_off':
    cs.cruiseState.available = False
  elif failure == 'invalid_can':
    cs.canValid = False
  elif failure == 'timeout':
    cs.canTimeout = True
  elif failure == 'temporary_eps':
    cs.steerFaultTemporary = True
  elif failure == 'permanent_eps':
    cs.steerFaultPermanent = True
  elif failure == 'acc_fault':
    cs.accFaulted = True
  elif failure == 'door':
    cs.doorOpen = True
  elif failure == 'no_cancel':
    cs.buttonEvents = []
  elif failure == 'native_missing':
    native = None
  elif failure == 'native_denied':
    native = replace(native, lateralAllowed=False)
  elif failure == 'native_unrequested':
    native = replace(native, requestedLateral=False)
  elif failure == 'native_stale':
    native = replace(native, observedMonoTime=700_000_000, validUntilMonoTime=900_000_000)
  elif failure == 'native_session':
    native = replace(native, axisSessionId='other-axis')
  elif failure == 'native_model':
    native = replace(native, safetyModel=int(structs.CarParams.SafetyModel.hondaNidec))
  elif failure == 'native_param':
    native = replace(native, safetyParam=2)
  elif failure == 'native_rx_fault':
    native = replace(native, pandaInventory=(replace(native.pandaInventory[0], safetyRxChecksInvalid=True),))
  elif failure == 'native_fault':
    native = replace(native, pandaInventory=(replace(native.pandaInventory[0], faults=1),))
  elif failure == 'no_inventory':
    native = replace(native, pandaInventory=())
  elif failure == 'no_previous':
    context['previous'] = None
  elif failure == 'previous_inactive':
    context['previous'] = AxisDecision()
  elif failure == 'dashcam':
    cp.dashcamOnly = True
  elif failure == 'nidec':
    cp, cs, intent, native, context = proof(CAR.HONDA_CIVIC)
  assert not physical_cancel_lateral_allowed(cp, cs, intent, native, **context)


STOCK_CLASSIC_LONG = 2083  # what the Accord really runs (route 0000002d carParams), with alternativeExperience 32


def stock_proof(identity=CAR.HONDA_ACCORD):
  cp, cs, intent, native, context = proof(identity, car_fw=ANGLE_EPS_FW)
  cp.safetyConfigs[0].safetyParam = STOCK_CLASSIC_LONG
  cp.alternativeExperience, cp.flags = 32, 32774  # the real car's flags: Bosch, alt brake, angle-loop EPS firmware
  cp.transmissionType = structs.CarParams.TransmissionType.automatic
  native = replace(native, safetyParam=STOCK_CLASSIC_LONG,
                   pandaInventory=(replace(native.pandaInventory[0], safetyParam=STOCK_CLASSIC_LONG, alternativeExperience=32),))
  return cp, cs, intent, native, context


def test_the_accord_as_it_actually_runs_keeps_lateral_through_cancel():
  from opendbc.car.honda.stock_aol import qualified as qualified_stock
  from openpilot.starpilot.car.honda.aol import qualified_honda
  cp, cs, intent, native, context = stock_proof()
  assert qualified_stock(cp, marked_only=True) and not qualified_honda(cp), 'the stock-owned contract, not the host latch'
  assert physical_cancel_lateral_allowed(cp, cs, intent, native, **context)
  cs.buttonEvents[0].pressed = False
  assert physical_cancel_lateral_allowed(cp, cs, intent, native, **context), 'the release frame carries buttonCancel too'


def test_a_mapped_cancel_action_keeps_its_own_handling():
  cp, cs, intent, native, context = stock_proof()
  context['settings'] = replace(context['settings'], cancel_actions=(9, 0, 0))
  assert not physical_cancel_lateral_allowed(cp, cs, intent, native, **context)


def test_the_stock_owner_still_needs_the_latch_and_the_panda():
  cp, cs, intent, native, context = stock_proof()
  assert not physical_cancel_lateral_allowed(cp, cs, replace(intent, allowedLatch=False, lateralArmed=False), native, **context)
  assert not physical_cancel_lateral_allowed(cp, cs, intent, replace(native, lateralAllowed=False), **context)
  cs.buttonEvents = []
  assert not physical_cancel_lateral_allowed(cp, cs, intent, native, **context)
