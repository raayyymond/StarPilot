"""rev 6.2: lane-change smoothing is kept, but it is not applied to a corner.

Route 76 (2026-09-16): a nudgeless lane change was auto-started by the turn blinker at 18.3 m/s about six
seconds before a right turn.  The smoothing clamp then did exactly what it is designed to do -- and held the
controls setpoint 1.2-1.3 m/s^2 behind the model for ~1.5 s through the corner entry.  That was the only
diagnosable understeer event on the drive.  The clamp is not the fault; applying it to a turn is.
"""

from openpilot.selfdrive.controls.controlsd import LANE_CHANGE_TURN_LAT_ACCEL, lane_change_turn_latch


def _curv(lat_accel: float, v_ego: float) -> float:
  return lat_accel / v_ego ** 2


def test_a_real_lane_change_never_arms_the_gate():
  # the fork's own lane-change profile at pace <= 9 peaks around 0.5-0.7 m/s^2 on route 76
  v = 18.3
  latched = False
  for lat_accel in (0.0, 0.2, 0.45, 0.7, 0.55, 0.3, 0.0):
    latched = lane_change_turn_latch(latched, True, _curv(lat_accel, v), v)
    assert not latched, lat_accel


def test_a_corner_inside_a_lane_change_state_arms_the_gate_and_it_stays_armed():
  v = 18.3
  latched = lane_change_turn_latch(False, True, _curv(2.1, v), v)
  assert latched
  # the demand dipping back under the threshold mid-corner must not un-latch: an instantaneous gate would
  # chatter the clamp on and off through the corner, which is worse than either state.
  for lat_accel in (1.4, 0.9, 1.8, 0.2):
    latched = lane_change_turn_latch(latched, True, _curv(lat_accel, v), v)
    assert latched, lat_accel


def test_the_latch_clears_when_the_lane_change_state_clears():
  v = 18.3
  latched = lane_change_turn_latch(False, True, _curv(3.0, v), v)
  assert latched
  assert not lane_change_turn_latch(latched, False, _curv(3.0, v), v)
  # and the next lane-change state starts unarmed
  assert not lane_change_turn_latch(False, True, _curv(0.5, v), v)


def test_the_threshold_is_lateral_acceleration_so_it_cannot_fire_at_a_crawl():
  # the same curvature that is 1.5 m/s^2 at 18.3 m/s is 0.14 m/s^2 at 5.6 m/s -- a parking-lot-speed
  # lane change bends hard in curvature and is nowhere near a corner in what the car feels.
  k = _curv(LANE_CHANGE_TURN_LAT_ACCEL, 18.3)
  assert lane_change_turn_latch(False, True, k, 18.3)
  assert not lane_change_turn_latch(False, True, k, 5.6)


def test_the_gate_sits_between_the_lane_change_peak_and_the_corner_that_was_clamped():
  assert 0.7 < LANE_CHANGE_TURN_LAT_ACCEL < 2.1
