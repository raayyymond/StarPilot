"""Birds-eye projection math and scene building, without a GL context."""
import math
from types import SimpleNamespace

import numpy as np
import pytest

from openpilot.selfdrive.ui.onroad import birds_eye_math as bem

RECT = SimpleNamespace(x=100.0, y=50.0, width=520.0, height=650.0)


def test_ego_bottom_centre_and_far_edge_top():
  cfg = bem.BirdsEyeConfig()
  pts = bem.car_to_widget(np.array([[cfg.x_min, 0.0], [cfg.x_max, 0.0]]), RECT, cfg)
  assert pts[0, 0] == pytest.approx(RECT.x + RECT.width / 2)
  assert pts[0, 1] == pytest.approx(RECT.y + RECT.height * (1 - cfg.bottom_pad))
  assert pts[1, 1] == pytest.approx(RECT.y + RECT.height * cfg.top_pad, abs=1e-3)


def test_orthographic_is_linear_and_uniform():
  cfg = bem.BirdsEyeConfig(perspective=0.0)
  xs = np.array([0.0, 10.0, 20.0, 40.0])
  pts = bem.car_to_widget(np.stack((xs, np.full_like(xs, 2.0)), axis=1), RECT, cfg)
  dy = np.diff(pts[:, 1]) / np.diff(xs)
  assert np.allclose(dy, dy[0])
  assert np.allclose(pts[:, 0], pts[0, 0]), 'a straight lane stays vertical with no perspective'
  view = bem.view_params(RECT, cfg)
  assert view.lateral_ppm == pytest.approx(RECT.width / (2 * cfg.lateral_range))


def test_perspective_narrows_distance_and_keeps_lines_straight():
  cfg = bem.BirdsEyeConfig(perspective=1.0)
  view = bem.view_params(RECT, cfg)
  assert view.k > 0
  near, far = bem.scale_at(np.array([0.0, cfg.x_max]), view)
  assert far < near
  u0 = (0 - cfg.x_min) / (cfg.x_max - cfg.x_min)
  assert near / far == pytest.approx((1 + view.k) / (1 + view.k * u0))
  # A straight diagonal on the ground stays collinear on screen (homography).
  xs = np.linspace(0, 70, 8)
  pts = bem.car_to_widget(np.stack((xs, 0.1 * xs - 3), axis=1), RECT, cfg).astype(np.float64)
  a, b = pts[0], pts[-1]
  cross = (b[0] - a[0]) * (pts[:, 1] - a[1]) - (b[1] - a[1]) * (pts[:, 0] - a[0])
  assert np.abs(cross).max() / np.linalg.norm(b - a) < 0.5
  # Monotonic: farther is always higher on screen.
  ys = bem.car_to_widget(np.stack((np.linspace(-6, 90, 50), np.zeros(50)), axis=1), RECT, cfg)[:, 1]
  assert np.all(np.diff(ys) < 0)


def test_isotropic_at_ego_with_full_perspective():
  cfg = bem.BirdsEyeConfig(perspective=1.0)
  view = bem.view_params(RECT, cfg)
  x0 = cfg.x_min
  assert float(bem.depth_scale_at(x0, view)) == pytest.approx(float(bem.scale_at(x0, view)), rel=1e-6)


def test_right_is_positive_and_scale_follows_rect():
  cfg = bem.BirdsEyeConfig()
  pts = bem.car_to_widget(np.array([[0.0, -2.0], [0.0, 2.0]]), RECT, cfg)
  assert pts[0, 0] < RECT.x + RECT.width / 2 < pts[1, 0]
  wide = SimpleNamespace(x=0, y=0, width=1800, height=1020)
  view = bem.view_params(wide, cfg)
  assert view.lateral_ppm == pytest.approx(1020 * cfg.max_ppm_fraction), 'a wide frame caps the scale by its height'


def test_far_behind_points_stay_finite():
  cfg = bem.BirdsEyeConfig(perspective=1.0)
  pts = bem.car_to_widget(np.array([[-500.0, 1.0]]), RECT, cfg)
  assert np.isfinite(pts).all()


def test_resample_line_extends_back_and_marks_missing():
  grid = bem.sample_grid()
  ys = bem.resample_line([0, 10, 20], [1, 2, 3], grid)
  assert ys[grid < 0] == pytest.approx(1.0)
  assert np.isnan(ys[grid > 20]).all()
  assert bem.lateral_at(grid, ys, 15.0) == pytest.approx(2.5)
  stopped = bem.resample_line([0, 0, 0], [0, 0, 0], grid)
  assert np.isnan(stopped).all()


def test_sample_grid_is_increasing_and_spans_the_range():
  cfg = bem.BirdsEyeConfig()
  grid = bem.sample_grid(cfg)
  assert grid[0] < cfg.x_min and grid[-1] == cfg.x_max
  assert np.all(np.diff(grid) > 0)
  assert np.diff(grid)[0] < np.diff(grid)[-2], 'coarser far away'


def test_ribbon_shape_and_left_first():
  centre = np.array([[100.0, 500.0], [100.0, 400.0], [100.0, 300.0]])
  poly = bem.ribbon(centre, 5.0)
  assert poly.shape == (6, 2)
  assert np.allclose(poly[:3, 0], 95.0), 'first chain is screen-left for an upward line'
  assert np.allclose(poly[3:, 0], 105.0)
  assert np.allclose(poly[3:, 1], [300, 400, 500])


def test_valid_runs():
  assert bem.valid_runs(np.array([False, True, True, False, True])) == [(1, 3), (4, 5)]
  assert bem.valid_runs(np.array([], dtype=bool)) == []


def test_lead_colour_shifts_to_red_with_ttc():
  assert bem.lead_color(40, 1.0) == bem.NEUTRAL_LEAD
  assert bem.lead_color(40, -5.0) == bem.NEUTRAL_LEAD  # 8 s
  amber = bem.lead_color(30, -10.0)  # 3 s
  assert amber == bem.CAUTION_LEAD
  assert bem.lead_color(10, -10.0) == bem.ALERT_LEAD  # 1 s
  assert bem.lead_color(3.0, 0.0) == bem.ALERT_LEAD, 'very close is red even when not closing'
  mid = bem.lead_color(30, -6.0)  # 5 s, between neutral and amber
  assert mid not in (bem.NEUTRAL_LEAD, bem.CAUTION_LEAD)
  assert bem.CAUTION_LEAD[2] < mid[2] < bem.NEUTRAL_LEAD[2]
  assert bem.time_to_collision(20, 0.0) == math.inf
  assert bem.time_to_collision(20, -4.0) == pytest.approx(5.0)


def test_labels_and_wheel_angle():
  assert bem.distance_label(32.4) == '32 m'
  assert bem.distance_label(10, metric=False) == '33 ft'
  assert bem.ego_wheel_angle(90.0) < 0 < bem.ego_wheel_angle(-90.0), 'left steering turns the glyph wheels left'
  assert abs(bem.ego_wheel_angle(1e4)) == pytest.approx(math.radians(35))
  assert bem.ego_wheel_angle(float('nan')) == 0.0


def test_car_outline_is_convex_and_rotates():
  hull = bem.car_outline(10.0, 0.0, 0.0)
  assert hull[:, 0].max() == pytest.approx(12.3) and hull[:, 1].max() == pytest.approx(0.95)
  turned = bem.car_outline(10.0, 0.0, math.pi / 2)
  assert turned[:, 1].max() == pytest.approx(2.3)


def test_smooth_blends_and_accepts_new_shapes():
  a = np.array([0.0, 0.0, np.nan])
  b = np.array([1.0, 2.0, 3.0])
  out = bem.smooth(a, b, 0.5)
  assert out == pytest.approx([0.5, 1.0, 3.0])
  assert bem.smooth(None, b, 0.5) == pytest.approx(b)
  assert bem.smooth(np.zeros(2), b, 0.5) == pytest.approx(b)


def _xyz(xs, ys):
  return SimpleNamespace(x=list(xs), y=list(ys), z=[0.0] * len(xs))


def _model(lead_prob=0.0, path_y=0.0):
  xs = np.linspace(0, 120, 33)
  return SimpleNamespace(position=_xyz(xs, np.full(33, path_y)), acceleration=SimpleNamespace(x=[0.0] * 33),
                         laneLines=[_xyz(xs, np.full(33, off)) for off in (-5.4, -1.8, 1.8, 5.4)], laneLineProbs=[.5, .9, .9, .5],
                         roadEdges=[_xyz(xs, np.full(33, off)) for off in (-7, 7)], roadEdgeStds=[.2, .3],
                         leadsV3=[SimpleNamespace(prob=lead_prob, x=[31.5], y=[0.2], v=[20.0])])


def test_scene_keeps_last_good_frame_and_merges_leads():
  # The renderer module imports pyray but needs no window for ingest().
  from openpilot.selfdrive.ui.onroad.birds_eye_renderer import BirdsEyeRenderer, scene_leads
  radar = SimpleNamespace(leadOne=SimpleNamespace(present=True, dRel=30.0, yRel=-0.3, vRel=-2.0),
                          leadTwo=SimpleNamespace(present=False, dRel=0.0, yRel=0.0, vRel=0.0))
  leads = scene_leads(radar, _model(lead_prob=0.9))
  assert len(leads) == 1 and leads[0].radar and leads[0].y == pytest.approx(0.3), 'radar yRel is left-positive'
  leads = scene_leads(None, _model(lead_prob=0.9))
  assert len(leads) == 1 and not leads[0].radar and leads[0].d_rel == pytest.approx(30.0)
  assert scene_leads(None, _model(lead_prob=0.2)) == []

  view = BirdsEyeRenderer(text=object())
  view.ingest(_model(), radar, SimpleNamespace(steeringAngleDeg=5.0, leftBlinker=True, rightBlinker=False), engaged=True)
  assert view.scene.valid and view.scene.engaged and view.scene.left_blinker
  before = view.scene.lanes[1].copy()
  view.ingest(None, None, None)
  assert np.array_equal(view.scene.lanes[1], before, equal_nan=True), 'no model keeps the last good road'
  view.ingest(_model(path_y=1.0), radar, None)
  path = view.scene.path[np.isfinite(view.scene.path)]
  assert 0.0 < path[0] < 1.0, 'a new frame is blended in, not jumped to'


class _SM(dict):
  def __init__(self, services, frame, updated):
    super().__init__(services)
    self.valid = dict.fromkeys(services, True)
    self.recv_frame = dict.fromkeys(services, frame)
    self.updated = dict.fromkeys(services, updated)


def test_poll_accepts_only_fresh_model():
  from openpilot.selfdrive.ui.onroad.birds_eye_renderer import BirdsEyeRenderer
  radar = SimpleNamespace(leadOne=SimpleNamespace(present=False, dRel=0, yRel=0, vRel=0), leadTwo=SimpleNamespace(present=False, dRel=0, yRel=0, vRel=0))
  services = {'modelV2': _model(), 'radarState': radar, 'carState': SimpleNamespace(steeringAngleDeg=0.0, leftBlinker=False, rightBlinker=False),
              'selfdriveState': SimpleNamespace(enabled=True, experimentalMode=False), 'longitudinalPlan': SimpleNamespace(allowThrottle=False)}
  view = BirdsEyeRenderer(text=object())
  view.poll(_SM(services, frame=5, updated=True), started_frame=10)
  assert not view.scene.valid, 'data from before this drive is ignored'
  view.poll(_SM(services, frame=12, updated=True), started_frame=10)
  assert view.scene.valid and view.scene.engaged and not view.scene.allow_throttle


def test_ribbon_sides_matches_ribbon_on_each_run():
  rng = np.random.default_rng(0)
  centre = np.cumsum(rng.normal(size=(12, 2)), axis=0) * 10
  half = rng.uniform(1, 4, size=12)
  valid = np.array([True] * 5 + [False] + [True] * 2 + [False] + [True] * 3)
  left, right = bem.ribbon_sides(centre, half, valid)
  for start, end in bem.valid_runs(valid):
    poly = bem.ribbon(centre[start:end], half[start:end])
    n = end - start
    assert left[start:end] == pytest.approx(poly[:n], abs=1e-3)
    assert right[start:end] == pytest.approx(poly[n:][::-1], abs=1e-3)
  stacked_left, _ = bem.ribbon_sides(np.stack((centre, centre)), np.stack((half, half)), np.stack((valid, valid)))
  assert stacked_left[1] == pytest.approx(left), 'leading axes are independent ribbons'


def test_hls_to_rgb_matches_colorsys():
  import colorsys
  rng = np.random.default_rng(1)
  hue, light, sat = rng.uniform(0, 1, 50), rng.uniform(0, 1, 50), np.concatenate((rng.uniform(0, 1, 49), [0.0]))
  expected = np.array([colorsys.hls_to_rgb(*v) for v in zip(hue, light, sat, strict=True)])
  assert bem.hls_to_rgb(hue, light, sat) == pytest.approx(expected)


def _renderer_with_scene():
  from openpilot.selfdrive.ui.onroad.birds_eye_renderer import BirdsEyeRenderer
  view = BirdsEyeRenderer(text=SimpleNamespace(measure=lambda text, size: (10.0, 10.0), draw=lambda *args: None))
  radar = SimpleNamespace(leadOne=SimpleNamespace(present=True, dRel=30.0, yRel=0.0, vRel=-2.0),
                          leadTwo=SimpleNamespace(present=False, dRel=0.0, yRel=0.0, vRel=0.0))
  view.ingest(_model(), radar, SimpleNamespace(steeringAngleDeg=5.0, leftBlinker=False, rightBlinker=False), engaged=True)
  return view, radar


def _layers(view, rect=RECT):
  key = (float(rect.x), float(rect.y), float(rect.width), float(rect.height), view.config)
  return view._scene_layers(view.scene, rect, key, bem.view_params(rect, view.config))


def test_scene_geometry_is_cached_until_an_input_changes():
  view, radar = _renderer_with_scene()
  road, lines, path = first = _layers(view)
  assert road and lines and path, 'road, lines and path all produce quads'
  assert len(road) % 6 == len(lines) % 6 == len(path) % 6 == 0, 'six raw calls per quad'
  assert _layers(view) is first, 'an unchanged scene replays the cached call lists'
  view.ingest(None, radar, None)
  assert _layers(view) is first, 'leads and car state alone keep the road'
  view.opacity = 0.5
  assert _layers(view) is not first, 'opacity is part of the key'
  again = _layers(view)
  assert _layers(view, SimpleNamespace(x=0.0, y=0.0, width=300.0, height=400.0)) is not again, 'so is the rect'
  again = _layers(view)
  view.ingest(_model(path_y=0.5), radar, None)
  assert _layers(view) is not again, 'a new model frame rebuilds'
  again = _layers(view)
  view.road_style = {'laneLines': '#FF8800C0'}
  assert _layers(view) is not again, 'so does a lane color setting'


def test_ego_glyph_cache_ignores_steering_jitter_below_its_resolution(monkeypatch):
  import openpilot.selfdrive.ui.onroad.birds_eye_renderer as ber
  view, _ = _renderer_with_scene()
  built = []
  monkeypatch.setattr(view, '_ego_glyph', lambda rect, v, wheel_angle: built.append(wheel_angle) or ([], ()))
  monkeypatch.setattr(ber.rl, 'draw_circle_gradient', lambda *args: None)
  rect_key = (RECT.x, RECT.y, RECT.width, RECT.height, view.config)
  for steering in (5.0, 5.05, 5.1, 9.0):
    view.scene.steering_deg = steering
    view._ego(view.scene, RECT, rect_key, None)
  assert len(built) == 2, 'sub-0.25 degree wheel changes reuse the glyph; a real turn rebuilds it'
