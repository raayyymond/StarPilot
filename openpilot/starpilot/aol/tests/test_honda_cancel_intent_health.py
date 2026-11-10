"""CANCEL on the Accord ends cruise and keeps always-on lateral, whenever the panda status happens to land.

2026-11-09 route 0000002f: three CANCEL presses with lateral on. Two kept lateral (t=296.48, t=738.43), one lost
it (t=790.87). In the lost one the panda status was sampled 5 ms after the press: longitudinal already dropped by
the panda, the host's longitudinal request not yet withdrawn. Card read that as a transport fault and cleared the
latch 9 ms after the press. This file rebuilds that sample.
"""
from typing import Any

from opendbc.car import structs
from openpilot.cereal import messaging
from openpilot.selfdrive.car.card import Car
from openpilot.starpilot.aol.intent import AolSettings
from openpilot.starpilot.aol.transport_pause import TransportPauseFeedback
from openpilot.starpilot.aol.wire import PandaSlot, SafetyState, encode_safety
from openpilot.starpilot.car.honda.tests.test_honda_cancel_lateral import STOCK_CLASSIC_LONG, stock_proof

NOW = 10_000_000_000
PRESS = NOW - 5_000_000
Cancel = structs.CarState.ButtonEvent.Type.cancel


class Peers:
  def __init__(self, messages):
    self.messages = {m.which(): m for m in messages}
    self.logMonoTime = {n: int(m.logMonoTime) for n, m in self.messages.items()}
    self.seen = self.valid = self.alive = dict.fromkeys(self.messages, True)

  def __getitem__(self, name):
    return getattr(self.messages[name], name)


def sample(*, pressed=True, button=Cancel):
  """The panda sample of t=790.873: longitudinal dropped, the host still asking for it, lateral granted."""
  cp, _, _, _, _ = stock_proof()
  config = cp.safetyConfigs[0]
  slots = (PandaSlot(0, 'accord-panda', config.safetyModel.raw, STOCK_CLASSIC_LONG, 32, False),)
  native = SafetyState(1, True, NOW - 1_000_000, NOW + 199_000_000, config.safetyModel.raw, STOCK_CLASSIC_LONG,
                       True, False, True, True, 'accord-panda', 'axis-session',
                       sourcePandaStatesMonoTime=NOW + 9_000_000_000, pandaInventory=slots)
  safety = messaging.new_message('aolSafetyWire', 0, valid=True, logMonoTime=native.observedMonoTime)
  safety.aolSafetyWire = encode_safety(native)
  pandas = messaging.new_message('pandaStates', 1, valid=True, logMonoTime=NOW)
  p = pandas.pandaStates[0]
  p.safetyModel, p.safetyParam, p.alternativeExperience, p.controlsAllowed = config.safetyModel, STOCK_CLASSIC_LONG, 32, False
  axis = messaging.new_message('aolAxisState', valid=True, logMonoTime=NOW - 1_000_000)
  a = axis.aolAxisState
  a.sessionId = a.faultSessionId = 'axis-session'
  a.qualified, a.sequence, a.faultReason = True, 2, 'none'
  a.observedMonoTime, a.validUntilMonoTime = axis.logMonoTime, axis.logMonoTime + 30_000_000
  a.sourceCarStateMonoTime = axis.logMonoTime - 1000
  a.desiredLateral = a.lateralActive = a.nativeAcknowledged = True
  events = messaging.new_message('onroadEvents', 1, valid=True, logMonoTime=NOW - 1_000_000)
  events.onroadEvents[0].name = 'buttonCancel'
  events.onroadEvents[0].userDisable = events.onroadEvents[0].noEntry = True
  sm = Peers([safety.as_reader(), pandas.as_reader(), axis.as_reader(), events.as_reader()])
  cs = structs.CarState(canValid=True, vEgo=20.7, gearShifter=structs.CarState.GearShifter.drive)
  cs.cruiseState.available = True
  cs.buttonEvents = [structs.CarState.ButtonEvent(type=button, pressed=pressed)]
  feedback = TransportPauseFeedback(cp)
  feedback.session = native.axisSessionId
  feedback.healthy_feedback_ns = NOW - 10_000_000
  feedback.healthy_native_ns = native.observedMonoTime
  feedback.healthy_inventory = (native.pandaSerial, tuple(slot.hardwareSerial for slot in slots))
  return cp, cs, sm, feedback


def card_for(cp, cs, *, cancel_actions=(0, 0, 0), enabled=True, received_ns=PRESS):
  card: Any = Car.__new__(Car)
  card.CP = cp
  card.aol_settings = AolSettings(enabled, 0., 0, 0, (0, 0, 0), cancel_actions)
  card._aol_can_observation = (cs, received_ns, NOW + 9_000_000_000, NOW + 9_000_001_000)
  return card


def test_without_the_hold_the_route_sample_clears_lateral():
  cp, cs, sm, feedback = sample()
  assert feedback.observe(sm, cp, cs, NOW, latched=True), 'what the car did at t=790.877: a transport fault'


def test_a_cancel_press_holds_lateral_intent_through_the_route_sample():
  cp, cs, sm, feedback = sample()
  source = card_for(cp, cs).aol_honda_brake_source(cs, NOW)
  assert source == PRESS
  assert not feedback.observe(sm, cp, cs, NOW, latched=True, brake_source_ns=source), 'the latch stays'
  cs.buttonEvents = []
  assert not feedback.observe(sm, cp, cs, NOW + 10_000_000 - 9_000_000, latched=True), 'and through the next frames'
  assert feedback.honda_brake_ns == PRESS, 'on the one original deadline'


def test_the_hold_ends_200_ms_after_the_press():
  cp, cs, sm, feedback = sample()
  assert not feedback.observe(sm, cp, cs, NOW, latched=True, brake_source_ns=PRESS)
  cs.buttonEvents = []
  axis = sm['aolAxisState']
  assert feedback._honda_brake_lateral_current(sm, cp, cs, axis, PRESS + 200_000_001, latched=True, brake_source_ns=0) is None


def test_only_a_fresh_unmapped_cancel_press_on_this_frame_counts():
  cp, cs, sm, feedback = sample()
  assert card_for(cp, cs, cancel_actions=(9, 0, 0)).aol_honda_brake_source(cs, NOW) == 0, 'a mapped CANCEL keeps its action'
  assert card_for(cp, cs, enabled=False).aol_honda_brake_source(cs, NOW) == 0, 'always-on lateral off'
  assert card_for(cp, cs, received_ns=NOW - 30_000_001).aol_honda_brake_source(cs, NOW) == 0, 'a stale CAN batch'
  assert card_for(cp, structs.CarState(**cs.to_dict())).aol_honda_brake_source(cs, NOW) == 0, 'another frame'
  cp, cs, sm, feedback = sample(pressed=False)
  assert card_for(cp, cs).aol_honda_brake_source(cs, NOW) == 0, 'a release'
  assert feedback.observe(sm, cp, cs, NOW, latched=True, brake_source_ns=PRESS), 'a release with a borrowed source'
  cp, cs, sm, feedback = sample(button=structs.CarState.ButtonEvent.Type.decelCruise)
  assert card_for(cp, cs).aol_honda_brake_source(cs, NOW) == 0, 'another button'
  assert feedback.observe(sm, cp, cs, NOW, latched=True, brake_source_ns=PRESS)


def test_an_unlatched_driver_gets_nothing_from_cancel():
  cp, cs, sm, feedback = sample()
  axis = sm['aolAxisState']
  assert feedback._honda_brake_lateral_current(sm, cp, cs, axis, NOW, latched=False, brake_source_ns=PRESS) is None
