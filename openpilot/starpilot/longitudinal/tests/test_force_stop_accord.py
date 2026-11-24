"""Where the Accord comes to rest at a light with no lead, under the default hand-off and under distance braking.

2026-11-23 routes 00000073/74, Force Stops on, offset 0. Four stops with no lead:
  r73 t=104.3 and t=425.8: rest 4 m short of the model's stop point (the driver crept 4.0 m after the second)
  r74 t=909.0: 1-2 m short, the car was still at 3.9 m/s at the hand-off
  r74 t=524.3: 5-10 m short, the model moved its point out while the car slowed (the driver crept 6.6 m)

The plant here is a stand-in, not the car: commanded acceleration through a 0.3 s lag. Under the default law the
cruise ceiling is followed no harder than the cruise floor the routes show (-1.2 m/s^2 at the end, to -2.2 at speed)
and the native stop ramps at the logged 0.6 m/s^3. It reproduces the 4 m and the 5-10 m stops, which is what it is
for; it does not reproduce the 1-2 m one, where the real car carried more speed into the hand-off.
"""
import math

import pytest

from openpilot.common.realtime import DT_MDL
from openpilot.starpilot.longitudinal.force_stop import ForceStop, StopFrame, StopTune
from openpilot.starpilot.longitudinal.force_stop_braking import STOP_RESERVE_M, distance_target, eligible

DELAY = .5
LAG = .3
STEP_NS = int(DT_MDL * 1e9)


def cruise_floor(v):
  return -1.2 - min(max(v - 8., 0.), 10.) * .1


def rest_short_of_the_models_point(tune, entry_speed, *, start_m=150., receding=0.):
  """Metres between the car at rest and the point the model's path ended on when the car had 6 m left."""
  stop, x, v, a, now = ForceStop(), 0., entry_speed, 0., 1_000_000_000
  command, native, forced = 0., None, False
  point = start_m
  for _ in range(int(60 / DT_MDL)):
    # A model still previewing its path keeps the end ahead of the car at low speed (r74 t=524.3).
    preview = receding * min(1., max(0., (8. - v) / 6.)) if point - x > 6. else 0.
    if preview:
      point = max(point, start_m + preview)
    horizon = max(0., point - x)
    plan = stop.step(StopFrame(
      model_ns=now, speed_mps=v, horizon_m=horizon, light=True, raw_model_stopped=False, model_should_stop=False,
      standstill=v < .02, enabled=True, pedal_override=False, tracking_lead=False, lead_present=False,
      lead_distance_m=0., lead_speed_mps=0., driving_in_curve=False, road_curvature=0., blinker=False,
      steering_angle_deg=0., moving_preview=True, acceleration_mps2=a, actuator_delay_s=DELAY,
      acceleration_ns=now, sample_ns=now), tune)
    ceiling = plan.speed_ceiling_mps
    cruise = 0. if ceiling is None else max(cruise_floor(v), min(0., (ceiling - v) / 1.5))
    if tune.distance_control and plan.forcing and plan.obstacle_m is not None:
      command = min(cruise, distance_target(v, a, plan.obstacle_m, DELAY))
    elif plan.should_stop:
      native = min(command, 0.) if native is None else native
      native = max(-2., native - .6 * DT_MDL)
      command = native
    else:
      command = cruise
    a += (command - a) * DT_MDL / LAG
    v = max(0., v + a * DT_MDL)
    x += v * DT_MDL
    now += STEP_NS
    forced |= plan.forcing
    if forced and v < .02:
      return point - x
  raise AssertionError('never stopped')


@pytest.mark.parametrize('entry_speed', (9., 14., 19., 22.))
def test_the_default_hand_off_rests_metres_short_and_distance_braking_rests_at_the_reserve(entry_speed):
  default = rest_short_of_the_models_point(StopTune(), entry_speed)
  assert 3. < default < 5., 'r73 t=104.3 and t=425.8: 4 m short of the model point'
  braked = rest_short_of_the_models_point(StopTune(distance_control=True), entry_speed)
  assert 0. < braked < STOP_RESERVE_M + 1., 'never past the point, and within a metre of the reserve'


def test_distance_braking_rests_at_the_same_place_from_any_speed_and_brakes_no_harder_than_it_must():
  braked = [rest_short_of_the_models_point(StopTune(distance_control=True), v) for v in (6., 9., 14., 19., 22., 26.)]
  assert max(braked) - min(braked) < .2 and min(braked) > 0.
  assert distance_target(9., 0., 87. * .93, DELAY) > -.65, 'from 87 m at 9 m/s: gentler than the 0.65 m/s^2 approach it replaces'


def test_a_model_point_that_moves_out_while_the_car_slows_is_followed_not_held():
  default = rest_short_of_the_models_point(StopTune(), 22., receding=6.)
  braked = rest_short_of_the_models_point(StopTune(distance_control=True), 22., receding=6.)
  assert 5. < default < 11., 'r74 t=524.3: the first, shorter distance was kept, 5-10 m short'
  assert 0. < braked < STOP_RESERVE_M + 1.


def test_only_the_accord_on_bosch_alpha_long_is_admitted():
  from opendbc.car.honda.values import CAR
  from openpilot.starpilot.car.honda.tests.test_honda_cancel_lateral import stock_proof
  cp = stock_proof()[0]
  assert cp.carFingerprint == CAR.HONDA_ACCORD and eligible(cp)
  for field, value in (('pcmCruise', True), ('openpilotLongitudinalControl', False), ('passive', True),
                       ('dashcamOnly', True), ('notCar', True), ('carFingerprint', CAR.HONDA_CIVIC_BOSCH)):
    other = stock_proof()[0]
    setattr(other, field, value)
    assert not eligible(other), field


def test_the_accord_long_control_is_unchanged_outside_a_force_stop_and_takes_the_distance_target_inside_one():
  from unittest.mock import patch
  from opendbc.car import structs
  from openpilot.selfdrive.controls.lib.longcontrol import LongControl, LongCtrlState
  from openpilot.starpilot.car.honda.tests.test_honda_cancel_lateral import stock_proof
  from openpilot.starpilot.longitudinal.extension import LongitudinalContext
  cp = stock_proof()[0]
  with patch('openpilot.starpilot.longitudinal.extension.distance_braking_eligible', return_value=False):
    before = LongControl(cp)
  after = LongControl(cp)
  car = structs.CarState(vEgo=4., aEgo=-1., canValid=True)
  for target, stopping in ((-.8, False), (.5, False), (-1.6, False), (-.8, True), (-.8, True), (-3., True)):
    assert before.update(True, car, target, stopping, (-3.5, 2.)) == after.update(True, car, target, stopping, (-3.5, 2.))
  assert after.long_control_state == LongCtrlState.stopping
  inside = LongitudinalContext(force_stop_braking=True)
  assert after.update(True, car, -.8, True, (-3.5, 2.), context=inside) == pytest.approx(-.8)
  assert after.update(True, car, -3., True, (-3.5, 2.), context=inside) == pytest.approx(-3.)
  assert math.isfinite(after.update(True, car, math.nan, True, (-3.5, 2.), context=inside))


def test_controlsd_reads_the_stop_intent_for_the_accord_and_drops_it_when_stale():
  import tempfile
  from unittest.mock import patch
  from openpilot.cereal import messaging
  from openpilot.common.params import Params
  from openpilot.selfdrive.controls.lib.longcontrol import LongControl, LongCtrlState
  from openpilot.starpilot.car.honda.tests.test_honda_cancel_lateral import stock_proof
  from openpilot.starpilot.longitudinal.inputs import LongitudinalInputs
  from openpilot.starpilot.longitudinal.tests.test_force_stop_braking import NOW, inputs
  from openpilot.starpilot.longitudinal.tests.test_ioniq6_start import car_state
  cp = stock_proof()[0]
  sm = inputs()
  sm.all_checks = lambda names: all(sm.valid.get(name, False) for name in names)
  stop = messaging.new_message('starpilotLongitudinalPlan', valid=True)
  stop.logMonoTime = NOW - 10_000_000
  for key, value in vars(sm['starpilotLongitudinalPlan']).items():
    setattr(stop.starpilotLongitudinalPlan, key, value)
  plan = messaging.new_message('longitudinalPlan', valid=True)
  plan.logMonoTime = NOW - 10_000_000
  plan.longitudinalPlan.modelMonoTime = NOW - 20_000_000
  plan.longitudinalPlan.shouldStop = True
  sm['starpilotLongitudinalPlan'] = messaging.log_from_bytes(stop.to_bytes()).starpilotLongitudinalPlan
  sm['longitudinalPlan'] = messaging.log_from_bytes(plan.to_bytes()).longitudinalPlan
  car = car_state(speed=2.5, standstill=False)
  sm['carState'] = car
  with tempfile.TemporaryDirectory() as directory:
    reader = LongitudinalInputs(cp, Params(directory), lambda: sm)
    assert 'starpilotLongitudinalPlan' in reader.optional_services
    with patch('openpilot.starpilot.longitudinal.inputs.time.monotonic_ns', return_value=NOW):
      context = reader.context(True)
    assert context.force_stop_braking
    control = LongControl(cp)
    control.long_control_state = LongCtrlState.pid
    control.last_output_accel = -1.2
    assert control.update(True, car, -.6, True, (-3.5, 2.), context=context) == pytest.approx(-.6), 'the distance target'
    assert control.long_control_state == LongCtrlState.stopping
    with patch('openpilot.starpilot.longitudinal.inputs.time.monotonic_ns', return_value=NOW + 200_000_000):
      stale = reader.context(True)
    assert not stale.force_stop_braking
    control.last_output_accel = -.6
    assert control.update(True, car, -.6, True, (-3.5, 2.), context=stale) < -.6, 'a stale intent falls back to the native ramp'
