import math
import time

import numpy as np
import pytest

from openpilot.common.transformations.camera import DEVICE_CAMERAS
from openpilot.system.camerad.cameras.nv12_info import get_nv12_info
from openpilot.tools.simhil import publisher as pub
from openpilot.tools.simhil.camera import RoadCameraRenderer, device_camera
from openpilot.tools.simhil.run import parse_args
from openpilot.tools.simhil.scenario import (CAMERA_HEIGHT, LANE_WIDTH, RADAR_TO_CAMERA, SCENARIOS, X_IDXS, T_IDXS,
                                            Simulation)


def test_constants_match_openpilot():
  from openpilot.selfdrive.controls.radard import RADAR_TO_CAMERA as radard_value
  from openpilot.selfdrive.locationd.calibrationd import HEIGHT_INIT
  assert RADAR_TO_CAMERA == radard_value
  assert CAMERA_HEIGHT == pytest.approx(HEIGHT_INIT[0])
  assert len(X_IDXS) == len(T_IDXS) == 33


@pytest.mark.parametrize("name", SCENARIOS)
def test_frames_are_finite_and_shaped(name):
  sim = Simulation(name, seed=3, duration=30.0)
  for t in (0.0, 7.3, 15.0, 29.9):
    f = sim.frame(t)
    assert f.lane_lines.shape == (4, 3, 33) and f.road_edges.shape == (2, 3, 33) and f.position.shape == (3, 33)
    for array in (f.lane_lines, f.road_edges, f.position, f.velocity, f.orientation, f.orientation_rate):
      assert np.isfinite(array).all()
    np.testing.assert_allclose(f.lane_lines[:, 0], np.broadcast_to(X_IDXS, (4, 33)))
    np.testing.assert_allclose(f.lane_lines[:, 2], CAMERA_HEIGHT)
    assert np.all(np.diff(f.lane_lines[:, 1, :5], axis=0) > 0)  # left to right in the model's y-right frame
    assert np.all(np.diff(f.position[0]) >= -1e-6)
    assert f.v_ego >= 0


def test_deterministic_by_seed():
  a, b, c = (Simulation("s_curve", seed=s) for s in (1, 1, 2))
  np.testing.assert_array_equal(a.frame(12.0).lane_lines, b.frame(12.0).lane_lines)
  assert not np.allclose(a.frame(12.0).lane_lines, c.frame(12.0).lane_lines)


def test_straight_lanes_without_noise():
  f = Simulation("straight", noise=0.0).frame(5.0)
  np.testing.assert_allclose(f.lane_lines[1, 1], -LANE_WIDTH / 2, atol=1e-6)
  np.testing.assert_allclose(f.lane_lines[2, 1], LANE_WIDTH / 2, atol=1e-6)
  np.testing.assert_allclose(f.position[1], 0.0, atol=1e-6)
  assert f.leads == () and f.curvature == pytest.approx(0.0)


def test_s_curve_steers_with_the_road():
  sim = Simulation("s_curve", noise=0.0)
  signs = set()
  for t in np.arange(5.0, 60.0, 1.0):
    f = sim.frame(t)
    if abs(f.curvature) > 1e-3:
      assert math.copysign(1, f.steering_angle_deg) == math.copysign(1, f.curvature)
      # a left curve bends the near lane lines left (negative y in the model frame)
      assert math.copysign(1, f.lane_lines[1, 1, 0] - f.lane_lines[1, 1, 10]) == math.copysign(1, f.curvature)
      signs.add(f.curvature > 0)
  assert signs == {True, False}


def test_lead_stop_comes_to_rest_behind_lead():
  sim = Simulation("lead_stop", duration=40.0)
  assert sim.frame(1.0).lead_one is not None
  end = sim.frame(39.0)
  assert end.v_ego < 0.1
  assert end.lead_one is not None and 3.0 < end.lead_one.d_rel < 12.0
  assert np.all(sim.lead_s - sim.s > 2.0)


def test_lane_change_moves_one_lane_left():
  sim = Simulation("lane_change", noise=0.0)
  assert sim.frame(7.0).lane_change_state == "laneChangeStarting"
  assert sim.frame(7.0).left_blinker
  assert sim.frame(20.0).lane_change_state == "off"
  assert np.interp(20.0, sim.t, sim.d) == pytest.approx(LANE_WIDTH)
  after = sim.frame(20.0)
  assert after.lane_lines[0, 1, 0] < -LANE_WIDTH  # far-left line now beyond the road
  assert after.lane_line_probs[0] < 0.1 and after.lane_line_probs[1] > 0.9


def test_merge_lead_enters_ego_lane():
  sim = Simulation("merge", noise=0.0)
  assert sim.frame(1.0).lead_one is None and sim.frame(1.0).lead_two is not None
  assert sim.frame(9.0).lead_one is not None


@pytest.mark.parametrize("device", ["mici", "tizi"])
def test_camera_nv12_and_projection(device):
  config = device_camera(device)
  assert config is DEVICE_CAMERAS[(device, "os04c10" if device == "mici" else "ox03c10")].narrow_road
  renderer = RoadCameraRenderer(config, CAMERA_HEIGHT)
  frame = Simulation("straight", noise=0.0).frame(1.0)
  rgb = renderer.render_rgb(frame)
  assert rgb.shape == (config.height // 2, config.width // 2, 3)
  assert rgb[2].mean(axis=0)[2] > rgb[2].mean(axis=0)[0]  # sky is blue at the top
  asphalt = rgb[-1, rgb.shape[1] // 2]
  assert np.all(np.abs(asphalt - asphalt.mean()) < 6) and asphalt.mean() < 90
  # The solid right road line (white) projects where the pinhole model puts it.
  depth, lateral = 30.0, -1.5 * LANE_WIDTH
  u = config.width / 2 - config.focal_length * lateral / depth
  v = config.height / 2 + config.focal_length * CAMERA_HEIGHT / depth
  assert rgb[int(v // 2), int(u // 2)].min() > 150
  nv12 = renderer.render_nv12(frame)
  assert len(nv12) == get_nv12_info(config.width, config.height)[3]


def test_message_builders_serialize():
  publisher = pub.SimPublisher.__new__(pub.SimPublisher)
  publisher.config = pub.PublisherConfig(scenario="lead_stop")
  publisher.start = time.monotonic()
  frame = Simulation("lead_stop").frame(8.0)
  for name in pub.RATES:
    if name in ("modelV2", "narrowRoadCameraState"):
      continue
    msg = getattr(publisher, "_" + name)(frame)
    assert msg.which() == name and msg.valid
    msg.to_bytes()
  model = pub.build_model(frame, 7, 123).as_reader().modelV2
  assert model.frameId == 7 and len(model.laneLines) == 4 and len(model.leadsV3) == 3
  assert model.leadsV3[0].prob > 0.5 and model.leadsV3[0].x[0] == pytest.approx(frame.lead_one.x)
  radar = publisher._radarState(frame).as_reader().radarState
  assert radar.leadOne.present and radar.leadOne.dRel == pytest.approx(frame.lead_one.x - RADAR_TO_CAMERA, abs=1e-4)
  device = publisher._deviceState(frame).as_reader().deviceState
  assert device.started and str(device.deviceType) == "mici"


def test_run_args():
  args = parse_args(["--c3", "--scenario", "merge", "--headless"])
  assert args.ui == "c3" and args.scenario == "merge" and args.seconds > 0
  assert parse_args([]).ui == "c4"
  with pytest.raises(SystemExit):
    parse_args(["--scenario", "nope"])
