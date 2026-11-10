import math
import time
from types import SimpleNamespace

import pytest

from opendbc.car import ACCELERATION_DUE_TO_GRAVITY, gen_empty_fingerprint, structs
from opendbc.car.honda.bosch_longitudinal import (ACCORD_BRAKE_RATIO_V, ACCORD_GAS_FACTOR_BP, ACCORD_GAS_FACTOR_V,
                                                 BRAKE_RATIO_LIMITS, GAS_FACTOR_LIMITS, STEADY_FRAMES, TABLE_PRIOR_WEIGHT,
                                                 BoschLongitudinal, SpeedTable, brake_command, qualified)
from opendbc.car.honda.interface import CarInterface
from opendbc.car.honda.values import CAR, CarControllerParams


def owner(identity=CAR.HONDA_ACCORD):
  cp = CarInterface.get_params(identity, gen_empty_fingerprint(), [], True, False, False)
  assert qualified(cp)
  return BoschLongitudinal(cp, CarControllerParams(cp))


def inputs(speed=0.0, measured=0.0, active=True, state="pid", gas=False, brake=False):
  cc = structs.CarControl(longActive=active)
  cc.actuators.longControlState = state
  cs = SimpleNamespace(out=SimpleNamespace(vEgo=speed, aEgo=measured, gasPressed=gas, brakePressed=brake))
  return cc, cs


def test_pitch_observation_retains_odd_frame_sample_for_next_command():
  controller = owner()
  controller.observe_orientation((0.0, 0.1, 0.0))
  controller.observe_orientation(())
  cc, cs = inputs(state="off")
  result = controller.update(cc, cs, 0.0)
  assert result[2] == pytest.approx(math.sin(0.1) * ACCELERATION_DUE_TO_GRAVITY)
  controller.observe_orientation((0.0, -0.1, 0.0))
  assert controller.update(cc, cs, 0.0)[2] < 0


def test_disabled_update_preserves_original_gas_ramp_without_requesting_brakes():
  controller = owner()
  cc, cs = inputs(active=False, state="off")
  controller.observe_orientation((0.0, 0.2, 0.0))
  first = controller.update(cc, cs, 0.0)
  second = controller.update(cc, cs, 0.0)
  assert first[1] == 60.0
  assert second[1] == 120.0
  assert first[3] is second[3] is False
  # The Accord's gas factor is its per-speed table, read at the current speed; the wind factor is still the scalar.
  assert controller.bosch_gas_factor == pytest.approx(ACCORD_GAS_FACTOR_V[0])
  assert controller.bosch_wind_factor == 1.0


@pytest.mark.parametrize("identity,rate", ((CAR.HONDA_CIVIC_BOSCH, 50.0), (CAR.HONDA_INSIGHT, 150.0),
                                         (CAR.ACURA_RDX_3G, 300.0), (CAR.ACURA_RDX_3G_MMR, 300.0)))
def test_learning_uses_actual_vehicle_rate_and_driver_gas_override(identity, rate):
  controller = owner(identity)
  cc, cs = inputs(measured=0.5)
  controller.update(cc, cs, 1.0)
  assert controller.bosch_gas_factor == pytest.approx(1.0 + 0.5 / rate)
  previous = controller.bosch_gas_factor
  cs.out.gasPressed = True
  controller.update(cc, cs, 1.0)
  assert controller.bosch_gas_factor == previous


def test_braking_hysteresis_stopping_and_inactive_release():
  # the shared -0.12 / -0.02 thresholds; the Accord has its own (test_accord_brake_mode_arms_at_its_floor_...)
  controller = owner(CAR.HONDA_CIVIC_BOSCH)
  cc, cs = inputs(state="off")
  assert controller.update(cc, cs, -0.13)[3]
  assert controller.update(cc, cs, -0.03)[3]
  assert not controller.update(cc, cs, -0.01)[3]
  cc.actuators.longControlState = "stopping"
  assert controller.update(cc, cs, 0.1)[3]
  cc.longActive = False
  assert not controller.update(cc, cs, 0.0)[3]


def test_saved_factors_keep_shared_learner_range_and_reject_invalid_values():
  controller = owner()
  controller.set_factors(0.01, 5.0)
  assert (controller.bosch_gas_factor, controller.bosch_wind_factor) == (0.01, 5.0)
  controller.set_factors(float("nan"), float("inf"))
  assert (controller.bosch_gas_factor, controller.bosch_wind_factor) == (1.0, 1.0)


def test_actual_controller_captures_pitch_on_non_transmit_frame():
  cp = CarInterface.get_params(CAR.HONDA_ACCORD, gen_empty_fingerprint(), [], True, False, False)
  ci = CarInterface(cp)
  ci.update([])
  command = structs.CarControl()
  command.actuators.longControlState = "off"
  ci.apply(command.as_reader(), 1_000_000_000)
  command.orientationNED = [0.0, 0.1, 0.0]
  ci.apply(command.as_reader(), 1_010_000_000)
  command.orientationNED = []
  ci.apply(command.as_reader(), 1_020_000_000)
  assert ci.CC.bosch_longitudinal.pitch == pytest.approx(0.1)
  assert ci.CC.bosch_longitudinal.bosch_last_gas > 0.0


def test_actual_configure_loads_typed_factors_without_startup_rewrite():
  from openpilot.common.params import Params
  from openpilot.common.prefix import OpenpilotPrefix
  from openpilot.starpilot.controller_extensions import configure_controller

  with OpenpilotPrefix():
    saved = Params()
    saved.put("HondaGasFactorParams", 1.25, block=True)
    saved.put("HondaWindFactorParams", 0.85, block=True)
    cp = CarInterface.get_params(CAR.HONDA_ACCORD, gen_empty_fingerprint(), [], True, False, False)
    ci = CarInterface(cp)
    configure_controller(ci, saved)
    learning = ci.CC.bosch_learning_params
    assert learning.owner is ci.CC.bosch_longitudinal
    assert (learning.owner.bosch_gas_factor, learning.owner.bosch_wind_factor) == (1.25, 0.85)
    learning.owner.set_factors(1.1, 0.9)
    learning.persist(0)
    learning.persist(5999)
    assert saved.get("HondaGasFactorParams") == 1.25
    assert saved.get("HondaWindFactorParams") == 0.85
    learning.persist(6000)
    deadline = time.monotonic() + 2.0
    while saved.get("HondaWindFactorParams") != 0.9 and time.monotonic() < deadline:
      time.sleep(0.01)
    assert saved.get("HondaGasFactorParams") == pytest.approx(1.1)
    assert saved.get("HondaWindFactorParams") == pytest.approx(0.9)
    writes = []
    learning.params = SimpleNamespace(put=lambda *args: writes.append(args))
    learning.persist(12000)
    assert writes == []
    learning.owner.set_factors(1.2, 0.9)
    learning.persist(18000)
    assert writes == [("HondaGasFactorParams", 1.2), ("HondaWindFactorParams", 0.9)]


def test_accord_brake_mode_arms_at_its_floor_in_the_owner():
  # route bb (2026-10-21): the request band that armed the brake (-0.12..-0.23) now coasts on the Accord
  controller = owner()
  cc, cs = inputs(state="off")
  assert not controller.update(cc, cs, -0.13)[3]
  assert not controller.update(cc, cs, -0.44)[3]
  assert controller.update(cc, cs, -0.46)[3]
  assert controller.update(cc, cs, -0.21)[3]
  assert not controller.update(cc, cs, -0.19)[3]


def test_gas_learner_frozen_while_the_last_gas_sat_on_the_rail():
  controller = owner(CAR.HONDA_CIVIC_BOSCH)
  rail = controller.params.BOSCH_GAS_LOOKUP_V[-1]
  cc, cs = inputs(measured=0.5)
  controller.bosch_last_gas = rail
  controller.update(cc, cs, 1.0)
  assert controller.bosch_gas_factor == 1.0
  controller.bosch_last_gas = rail - 1.0
  controller.update(cc, cs, 1.0)
  assert controller.bosch_gas_factor == pytest.approx(1.0 + 0.5 / 50.0)


def test_speed_table_starts_on_the_logged_default_and_moves_at_the_pace_of_evidence():
  table = SpeedTable([0.0, 10.0, 20.0], [0.6, 0.5, 0.4], GAS_FACTOR_LIMITS, horizon=1000.0, prior_weight=100.0)
  assert table.value(5.0) == pytest.approx(0.55)
  for _ in range(100):
    table.learn(10.0, 1.0)
  # 100 samples of 1.0 against a prior worth 100 samples of 0.5: halfway
  assert table.value(10.0) == pytest.approx(0.75)
  assert table.value(0.0) == pytest.approx(0.6) and table.value(20.0) == pytest.approx(0.4)
  table.learn(10.0, float("nan"))
  table.learn(10.0, 50.0)
  assert table.value(10.0) == pytest.approx(0.75)


def test_speed_table_forgets_by_driving_time_near_the_knot_not_by_wall_time():
  table = SpeedTable([0.0, 10.0, 20.0], [0.5, 0.5, 0.5], GAS_FACTOR_LIMITS, horizon=1000.0, prior_weight=100.0)
  for _ in range(100):
    table.learn(10.0, 1.0)
  learned = table.value(10.0)
  for _ in range(5000):
    table.observe_driving(0.0)  # driving far from the 10 m/s knot leaves it alone
  assert table.value(10.0) == pytest.approx(learned)
  for _ in range(1000):
    table.observe_driving(10.0)  # one horizon of driving near the knot decays its evidence to 1/e
  assert table.sums[1] == pytest.approx(100.0 * math.exp(-1.0), rel=0.01)
  assert 0.5 < table.value(10.0) < learned


def test_speed_table_state_round_trips_and_rejects_foreign_knots():
  table = SpeedTable([0.0, 10.0], [0.5, 0.5], GAS_FACTOR_LIMITS)
  table.learn(5.0, 0.8)
  copy = SpeedTable([0.0, 10.0], [0.5, 0.5], GAS_FACTOR_LIMITS)
  assert copy.load(table.state()) and copy.values() == table.values()
  assert not SpeedTable([0.0, 15.0], [0.5, 0.5], GAS_FACTOR_LIMITS).load(table.state())
  bad = table.state()
  bad["weights"] = [float("inf"), 0.0]
  assert not copy.load(bad) and copy.values() == table.values()


def test_brake_command_divides_by_the_ratio_and_passes_hard_braking_through():
  assert brake_command(-1.0, 1.1) == pytest.approx(-1.0 / 1.1)
  assert brake_command(-2.0, 1.25) == pytest.approx(-1.6)
  assert brake_command(-2.5, 1.25) == pytest.approx(-2.5 * (0.5 / 1.25 + 0.5))
  assert brake_command(-3.5, 1.25) == -3.5
  assert brake_command(0.5, 1.25) == 0.5


def steady_run(controller, accel, speed, measured, frames=STEADY_FRAMES + 2, pitch=0.0):
  controller.observe_orientation((0.0, pitch, 0.0))
  cc, cs = inputs(speed=speed, measured=measured)
  for _ in range(frames):
    result = controller.update(cc, cs, accel)
  return result


def test_accord_gas_table_learns_only_from_steady_commands_and_at_its_speed():
  controller = owner()
  before = controller.gas_table.values()
  # a ramping command (the launch case) teaches nothing
  controller.observe_orientation((0.0, 0.0, 0.0))
  cc, cs = inputs(speed=20.0, measured=0.5)
  for i in range(STEADY_FRAMES + 2):
    controller.update(cc, cs, 0.5 + 0.02 * i)
  assert controller.gas_table.values() == before
  # the car produces more than the table predicted for the gas sent: the knot at 20 m/s moves down
  steady_run(controller, 1.0, 20.0, 1.5, frames=STEADY_FRAMES + 200)
  after = controller.gas_table.values()
  knot = ACCORD_GAS_FACTOR_BP.index(20.0)
  assert after[knot] < before[knot] and after[knot - 1] == before[knot - 1] and after[knot + 1] == before[knot + 1]
  # and the gas it sends now follows the table at that speed
  assert controller.bosch_gas_factor == pytest.approx(controller.gas_table.value(20.0), rel=1e-3)  # read before this frame taught
  assert controller.gas_table.weights[knot] > 100


def test_accord_brake_request_is_divided_by_the_learned_ratio_while_the_bosch_brake_is_in_charge():
  controller = owner()
  cc, cs = inputs(speed=15.0, measured=-1.0)
  command, gas, force, braking = controller.update(cc, cs, -1.0)
  assert braking and gas == 0.0
  knot = 2  # 15 m/s
  assert command == pytest.approx(-1.0 / ACCORD_BRAKE_RATIO_V[knot])
  assert controller.accel == -1.0  # the target the learner judges itself against is unscaled
  # the Civic keeps the request as is
  civic = owner(CAR.HONDA_CIVIC_BOSCH)
  assert civic.update(cc, cs, -1.0)[0] == -1.0
  # a steady over-delivering brake teaches a larger ratio
  before = controller.brake_table.values()[knot]
  steady_run(controller, -1.0, 15.0, -1.3, frames=STEADY_FRAMES + 200)
  assert BRAKE_RATIO_LIMITS[0] < before < controller.brake_table.values()[knot] < BRAKE_RATIO_LIMITS[1]


def test_accord_tables_round_trip_through_the_learning_params_file(tmp_path):
  from openpilot.starpilot.car.honda.longitudinal import BoschLearningParams

  class FakeParams:
    def __init__(self):
      self.store = {"HondaGasFactorParams": 1.0, "HondaWindFactorParams": 1.0}
    def get(self, key, return_default=False):
      return self.store[key]
    def put(self, key, value, block=False):
      self.store[key] = value

  path = tmp_path / "tables.json"
  controller = owner()
  learning = BoschLearningParams(FakeParams(), controller, path=path)
  steady_run(controller, 1.0, 20.0, 1.5, frames=STEADY_FRAMES + 200)
  learning.persist(5999)
  assert not path.exists()
  learning.persist(6000)
  assert path.exists()
  fresh = owner()
  BoschLearningParams(FakeParams(), fresh, path=path)
  assert fresh.gas_table.values() == controller.gas_table.values()
  assert fresh.brake_table.values() == controller.brake_table.values()
  # a Civic has no tables and never touches the file
  path.unlink()
  BoschLearningParams(FakeParams(), owner(CAR.HONDA_CIVIC_BOSCH), path=path).persist(6000)
  assert not path.exists()
  assert TABLE_PRIOR_WEIGHT > 0


def test_a_saved_table_keeps_what_it_learned_when_a_knot_is_added_between_its_knots():
  # 2026-11-09: the 1.5 m/s gas knot was added on a car with a learned 7-knot table.
  old_bp = [x for x in ACCORD_GAS_FACTOR_BP if x != 1.5]
  old = SpeedTable(old_bp, [v for x, v in zip(ACCORD_GAS_FACTOR_BP, ACCORD_GAS_FACTOR_V, strict=True) if x != 1.5], GAS_FACTOR_LIMITS)
  for _ in range(4000):
    old.learn(3.0, 0.8)
    old.learn(20.0, 0.4)
  new = SpeedTable(ACCORD_GAS_FACTOR_BP, ACCORD_GAS_FACTOR_V, GAS_FACTOR_LIMITS)
  assert new.load(old.state())
  learned, now = dict(zip(old_bp, old.values(), strict=True)), dict(zip(ACCORD_GAS_FACTOR_BP, new.values(), strict=True))
  assert {knot: now[knot] for knot in old_bp} == learned, 'every old knot is exactly what the car had'
  assert now[1.5] == 1.3 and new.weights[1] == 0.0, 'the new knot starts on its logged default'
  assert new.value(1.5) == 1.3 and learned[0.0] < new.value(0.75) < 1.3
  assert SpeedTable(ACCORD_GAS_FACTOR_BP, ACCORD_GAS_FACTOR_V, GAS_FACTOR_LIMITS).load(new.state()), 'and round-trips after'
  duplicate = old.state()
  duplicate['bp'][1] = duplicate['bp'][0]
  assert not new.load(duplicate)
