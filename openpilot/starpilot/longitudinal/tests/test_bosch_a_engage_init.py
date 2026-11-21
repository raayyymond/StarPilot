"""Stock Brake Feel engage init: on reset_state, a_cruise = min(aEgo-seeded value, cruise target), both planners.

Static tests with the recording solver stand-in from test_bosch_a_toggles; no replay or road evidence.
"""
import unittest
from unittest.mock import patch

from openpilot.starpilot.longitudinal.tests.test_bosch_a_toggles import BOSCH_A, NOT_BOSCH_A, RecordingSolver, submaster

V_EGO = 22.0


def engage_sm(a_ego, v_cruise_kph, frame=0):
  sm = submaster(frame)
  sm['carState'].vEgo, sm['carState'].aEgo, sm['carState'].vCruise = V_EGO, a_ego, v_cruise_kph
  return sm


def starpilot(cp):
  from openpilot.selfdrive.controls.lib import longitudinal_planner
  from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib import long_mpc
  with patch.object(long_mpc, 'AcadosOcpSolverCython', RecordingSolver):
    return longitudinal_planner.LongitudinalPlanner(cp)


class EngageInit:
  build = None
  floor = -1.2  # A_CRUISE_MIN / the default cruise_min

  def make(self, cp, law):
    """Stock Brake Feel is permanent now; law=False builds the same planner with the law switched off as the reference."""
    p = self.build(cp)  # ty: ignore[call-non-callable]
    if not law and p.bosch_a_long is not None:
      p.bosch_a_long.stock_feel = False
    return p

  def reset_a_cruise(self, cp, law, a_ego, v_cruise_kph):
    p = self.make(cp, law)
    p.update(engage_sm(a_ego, v_cruise_kph))
    return p.a_cruise

  def test_above_set_speed_seeds_cruise_target(self):
    # 22 m/s against a 72 km/h (20 m/s) set speed: cruise target is the floor, aEgo is +1.5.
    on = self.reset_a_cruise(BOSCH_A, True, 1.5, 72.0)
    off = self.reset_a_cruise(BOSCH_A, False, 1.5, 72.0)
    self.assertGreater(off, 1.0)  # stock: jerk-limited around aEgo
    self.assertAlmostEqual(on, self.floor, places=6)

  def test_never_below_cruise_floor(self):
    on = self.reset_a_cruise(BOSCH_A, True, 1.5, 30.0)
    self.assertGreaterEqual(on, self.floor - 1e-9)

  def test_target_above_aego_is_stock(self):
    # Set speed well above vEgo: min() keeps the stock value bit-for-bit.
    self.assertEqual(self.reset_a_cruise(BOSCH_A, True, -0.5, 120.0), self.reset_a_cruise(BOSCH_A, False, -0.5, 120.0))

  def test_other_cars_untouched(self):
    for cp in NOT_BOSCH_A:
      self.assertEqual(self.reset_a_cruise(cp, True, 1.5, 72.0), self.reset_a_cruise(cp, False, 1.5, 72.0), cp)

  def test_engaged_frames_do_not_reseed(self):
    on, off = self.make(BOSCH_A, True), self.make(BOSCH_A, False)
    for p in (on, off):
      p.update(engage_sm(-0.3, 120.0, frame=0))
      p.update(engage_sm(1.5, 72.0, frame=1))  # engaged (pid): only the normal jerk-limited path runs
    self.assertEqual(on.a_cruise, off.a_cruise)


class TestStarPilotPlannerEngageInit(EngageInit, unittest.TestCase):
  build = staticmethod(starpilot)
  floor = -0.5  # StarPilot's cruise_min in this harness (global braking floor with default params), not A_CRUISE_MIN


if __name__ == '__main__':
  unittest.main()
