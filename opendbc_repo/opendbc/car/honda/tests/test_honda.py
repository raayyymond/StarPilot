import unittest

import pytest

from opendbc.car.honda.bosch_longitudinal import (
  BOSCH_BRAKE_FORCE_ON,
  BOSCH_BRAKE_FORCE_RELEASE,
  bosch_brake_force_thresholds,
  bosch_gas_learn_speed,
  bosch_gas_learning_allowed,
  update_honda_bosch_braking,
)
from opendbc.car.honda.values import CAR, CarControllerParams, HondaFlags


class TestHondaFingerprint(unittest.TestCase):
  def test_tja_bosch_only(self):
    for car_model in CAR:
      if car_model.config.flags & HondaFlags.BOSCH_TJA_CONTROL:
        assert car_model.config.flags & HondaFlags.BOSCH, "Nidec car found with TJA control"


def test_accord_brake_mode_arms_at_its_floor_and_releases_to_a_coast():
  # route bb (2026-10-21): brake mode delivers -0.3..-0.4 for any request above -0.25, so a request there is a coast
  on, release = bosch_brake_force_thresholds(CAR.HONDA_ACCORD)
  assert (on, release) == (-0.45, -0.20)
  assert bosch_brake_force_thresholds(CAR.HONDA_CIVIC_BOSCH) == (BOSCH_BRAKE_FORCE_ON, BOSCH_BRAKE_FORCE_RELEASE)
  assert (BOSCH_BRAKE_FORCE_ON, BOSCH_BRAKE_FORCE_RELEASE) == (-0.12, -0.02)

  # the forces that armed the brake 21 times in 5 min on route bb (-0.12..-0.23) now coast
  for force in (-0.12, -0.19, -0.23, -0.44):
    assert not update_honda_bosch_braking(False, force, False, True, on, release)
  braking = update_honda_bosch_braking(False, -0.46, False, True, on, release)
  assert braking
  # once armed it stays through the floor band and lets go when the request is back at a coast
  assert update_honda_bosch_braking(braking, -0.30, False, True, on, release)
  assert update_honda_bosch_braking(braking, -0.21, False, True, on, release)
  assert not update_honda_bosch_braking(braking, -0.19, False, True, on, release)
  # stopping and inactive are unchanged
  assert update_honda_bosch_braking(False, 0.5, True, True, on, release)
  assert not update_honda_bosch_braking(True, -1.0, False, False, on, release)


def test_bosch_braking_default_thresholds_unchanged():
  braking = update_honda_bosch_braking(False, BOSCH_BRAKE_FORCE_ON - 0.01, False, True)
  assert braking
  assert update_honda_bosch_braking(braking, BOSCH_BRAKE_FORCE_RELEASE, False, True)
  assert not update_honda_bosch_braking(braking, BOSCH_BRAKE_FORCE_RELEASE + 0.01, False, True)
  assert not update_honda_bosch_braking(False, BOSCH_BRAKE_FORCE_ON, False, True)


def test_accord_bosch_gas_learner_is_slow_and_frozen_at_the_rail():
  # 2020 Accord 2.0T/10AT: the launch accel error is turbo + shift lag, not gain (routes b5-b9, 2026-10-21)
  assert bosch_gas_learn_speed(CAR.HONDA_ACCORD) == 2000.0
  assert bosch_gas_learn_speed(CAR.HONDA_CIVIC_BOSCH) == 50.0
  assert bosch_gas_learn_speed(CAR.HONDA_INSIGHT) == 150.0
  assert bosch_gas_learn_speed(CAR.ACURA_RDX_3G) == 300.0
  assert bosch_gas_learn_speed(CAR.ACURA_RDX_3G_MMR) == 300.0
  # one 20 ms frame at a 1 m/s^2 error and 2 m/s^2 force moves the factor by 1e-3 (was 0.04)
  assert 1.0 / bosch_gas_learn_speed(CAR.HONDA_ACCORD) * 2.0 == pytest.approx(1e-3)
  # no learning while the last gas command sat on the 1600 rail
  rail = CarControllerParams.BOSCH_GAS_LOOKUP_V[-1]
  assert rail == 1600
  assert bosch_gas_learning_allowed(rail - 1.0, rail)
  assert not bosch_gas_learning_allowed(rail, rail)
