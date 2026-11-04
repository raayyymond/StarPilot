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


def test_default_is_isotropic_true_to_scale():
  cfg = bem.DEFAULT_CONFIG
  stream = SimpleNamespace(x=0.0, y=0.0, width=1280.0, height=720.0)
  view = bem.view_params(stream, cfg)
  assert view.k == 0.0
  assert float(bem.depth_scale_at(30.0, view)) == pytest.approx(view.lateral_ppm), 'one px/m forward and sideways'
  assert view.lateral_ppm == pytest.approx(720 * (1 - cfg.top_pad - cfg.bottom_pad) / (cfg.x_max - cfg.x_min))
  narrow = SimpleNamespace(x=0.0, y=0.0, width=200.0, height=720.0)
  assert bem.view_params(narrow, cfg).lateral_ppm == pytest.approx(200 / (2 * cfg.lateral_range)), 'a narrow rect still fits the lanes'
  # 3.7 m sideways is the same pixel count as 3.7 m forward.
  pts = bem.car_to_widget(np.array([[10.0, -1.85], [10.0, 1.85], [13.7, -1.85]]), stream, cfg)
  assert pts[1, 0] - pts[0, 0] == pytest.approx(pts[0, 1] - pts[2, 1], rel=1e-4)


def test_orthographic_is_linear_and_uniform():
  cfg = bem.BirdsEyeConfig(perspective=0.0, isotropic=False)
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
  cfg = bem.BirdsEyeConfig(isotropic=False)
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
  assert bem.distance_label(10, metric=False) == '11 yd'
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
  """(road, path) draws for the current scene, as draw() would fetch them."""
  key = (float(rect.x), float(rect.y), float(rect.width), float(rect.height), view.config)
  pv = bem.view_params(rect, view.config)
  return view._road_draw(view.scene, rect, key, pv), view._path_draw(view.scene, rect, key, pv)


def _op_points(op) -> np.ndarray:
  """The (n, 2) points a DrawTriangleStrip/Fan op passes to C."""
  return np.array([(op[1][i].x, op[1][i].y) for i in range(op[2])])


def _draw_ops(ops):
  """The C draw ops of an op list (without the trailing buffer keep-alive entry)."""
  from openpilot.selfdrive.ui.onroad import birds_eye_renderer as ber
  return [op for op in ops if op[0] is not ber._keep]


def test_scene_geometry_is_cached_until_an_input_changes():
  from openpilot.selfdrive.ui.onroad import birds_eye_renderer as ber
  view, radar = _renderer_with_scene()
  (road, path) = first = _layers(view)
  assert road.road and road.lines and road.overlay and path.strips, 'road, lines, fade overlay and path all produce ops'
  lines = _draw_ops(road.lines)
  assert all(op[0] is ber._raw.DrawTriangleStrip for op in lines) and len(lines) == 2 + 4, 'one strip per edge and lane line'
  assert len(_draw_ops(road.road)) == 1, 'the road fill is one strip'
  assert all(op[0] is ber._raw.DrawRectangleGradientV for op in road.overlay)
  assert _layers(view)[0] is first[0] and _layers(view)[1] is first[1], 'an unchanged scene replays the cached ops'
  view.ingest(None, radar, None)
  assert _layers(view)[0] is first[0], 'leads and car state alone keep the road'
  view.opacity = 0.5
  assert _layers(view)[0] is not first[0], 'opacity is part of the key'
  again = _layers(view)[0]
  assert _layers(view, SimpleNamespace(x=0.0, y=0.0, width=300.0, height=400.0))[0] is not again, 'so is the rect'
  again = _layers(view)
  view.ingest(_model(path_y=0.5), radar, None)
  assert _layers(view)[0] is not again[0] and _layers(view)[1] is not again[1], 'a new model frame rebuilds'
  again = _layers(view)[0]
  view.road_style = {'laneLines': '#FF8800C0'}
  assert _layers(view)[0] is not again, 'so does a lane color setting'


def test_one_strip_per_ribbon_run_and_per_vertex_only_for_row_colour_modes():
  view, _ = _renderer_with_scene()
  lane = view.scene.lanes[0].copy()
  lane[(view.scene.grid > 20) & (view.scene.grid < 30)] = np.nan  # a gap splits the line
  view.scene.lanes = [lane, *view.scene.lanes[1:]]
  road, path = _layers(view)
  assert len(_draw_ops(road.lines)) == 2 + 4 + 1, 'the line with a gap is two strips'
  assert path.strips and not path.quads, 'a single-colour path is strips'
  assert len(_draw_ops(path.strips)) <= 30, 'the stepped fade stays a few dozen calls'
  view.ingest(None, None, None, experimental=True)
  view.scene.path_accel = np.full(view.scene.grid.size, 1.0)
  _, path = _layers(view)
  assert path.quads and not path.strips, 'the experimental accel tint keeps per-vertex colours'
  assert len(path.quads) % 6 == 0


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


def _text_recorder():
  drawn = []
  return SimpleNamespace(measure=lambda text, size: (0.5 * size * len(text), size), draw=lambda text, size, *a: drawn.append((text, size))), drawn


def test_label_font_is_about_twice_the_first_version():
  from openpilot.selfdrive.ui.onroad.birds_eye_renderer import BirdsEyeRenderer
  view = BirdsEyeRenderer(text=object())
  stream = SimpleNamespace(x=0.0, y=0.0, width=1280.0, height=720.0)
  old = max(15.0, min(26.0, stream.height * 0.026))
  assert view._font_size(stream) == pytest.approx(2 * old, rel=0.05)
  assert view._font_size(SimpleNamespace(height=200.0)) >= 30.0
  assert view._font_size(SimpleNamespace(height=4000.0)) <= 54.0
  assert view._font_size(stream) == bem.label_font_size(stream.height)


def test_ego_lane_lines_follow_the_lateral_state():
  from openpilot.selfdrive.ui.onroad import birds_eye_renderer as ber
  from openpilot.starpilot.lateral.lane_feedback import BLUE
  assert ber.LANE_CENTERING_LANE == BLUE
  view, radar = _renderer_with_scene()

  def colors():
    lanes = view._line_styles(view.scene)[len(view.scene.edges):]
    return [line[3] for line in lanes]

  idle = colors()
  assert ber.LAT_ACTIVE_LANE not in idle and BLUE not in idle
  first = _layers(view)[0]
  view.ingest(None, radar, None, lat_active=True)
  assert colors()[1:3] == [ber.LAT_ACTIVE_LANE] * 2, 'the ego lane own lines turn green'
  assert colors()[0] == idle[0] and colors()[3] == idle[3], 'the outer lines keep their color'
  green = _layers(view)[0]
  assert green is not first, 'the lateral state is part of the scene cache key'
  assert _layers(view)[0] is green, 'and an unchanged state still replays the cache'
  view.ingest(None, radar, None, lane_centering_direction=-1)
  assert colors()[1:3] == [BLUE, ber.LAT_ACTIVE_LANE], 'only the left line, the side being corrected toward, is blue'
  left = _layers(view)[0]
  assert left is not green, 'the direction is part of the scene cache key'
  view.ingest(None, radar, None, lane_centering_direction=1)
  assert colors()[1:3] == [ber.LAT_ACTIVE_LANE, BLUE], 'then only the right line'
  assert _layers(view)[0] is not left
  view.ingest(None, radar, None, lat_active=False)
  assert colors() == idle, 'centering without lateral control is not shown'


def _lateral_sm(frame, lat_active, applied, now_ns, fresh_ns=None):
  services = {'modelV2': _model(), 'carControl': SimpleNamespace(latActive=lat_active),
              'starpilotLateralState': SimpleNamespace(laneCentering=SimpleNamespace(
                version=1, lateralActive=lat_active, requestedCorrection=applied, appliedCorrection=applied,
                modelMonoTime=now_ns - 1000, carControlMonoTime=now_ns - 1000, reason='ok'))}
  sm = _SM(services, frame=frame, updated=True)
  sm.alive = dict.fromkeys(services, True)
  sm.logMonoTime = dict.fromkeys(services, now_ns - 1000 if fresh_ns is None else fresh_ns)
  return sm


def test_poll_reads_lateral_active_and_lane_centering():
  from openpilot.selfdrive.ui.onroad.birds_eye_renderer import BirdsEyeRenderer
  now = 10_000_000_000
  view = BirdsEyeRenderer(text=object())
  view.clock_ns = lambda: now
  view.poll(_lateral_sm(12, True, 0.0, now), started_frame=10)
  assert view.scene.lat_active and view.scene.lane_centering_direction == 0, 'active with no applied correction is green'
  view.poll(_lateral_sm(12, True, 0.001, now), started_frame=10)
  assert view.scene.lat_active and view.scene.lane_centering_direction == 1
  view.poll(_lateral_sm(12, True, -0.001, now), started_frame=10)
  assert view.scene.lat_active and view.scene.lane_centering_direction == -1, 'the sign is carried, not just on/off'
  view.poll(_lateral_sm(12, True, 0.001, now, fresh_ns=now - 1_000_000_000), started_frame=10)
  assert view.scene.lat_active and view.scene.lane_centering_direction == 0, 'a stale lane-centering message is not blue'
  view.poll(_lateral_sm(12, False, 0.0, now), started_frame=10)
  assert not view.scene.lat_active and view.scene.lane_centering_direction == 0
  view.poll(_SM({'carControl': SimpleNamespace(latActive=True)}, frame=12, updated=True), started_frame=10)
  assert view.scene.lat_active and view.scene.lane_centering_direction == 0, 'a SubMaster without the fork service is green only'


def test_distance_rungs_are_gone():
  from openpilot.selfdrive.ui.onroad.birds_eye_renderer import BirdsEyeRenderer
  assert not hasattr(BirdsEyeRenderer, '_ticks')


def test_default_forward_scale_is_linear_and_cars_are_true_to_scale():
  from openpilot.selfdrive.ui.onroad import birds_eye_renderer as ber
  cfg = bem.DEFAULT_CONFIG
  stream = SimpleNamespace(x=0.0, y=0.0, width=1280.0, height=720.0)
  view = bem.view_params(stream, cfg)
  assert cfg.perspective == 0.0 and view.k == 0.0
  front = ber.EGO_FRONT
  xs = np.array([front, front + 15.0, front + 30.0, front + 60.0])
  ys = bem.car_to_widget(np.stack((xs, np.zeros(4)), axis=1), stream, cfg)[:, 1]
  gaps = ys[0] - ys[1:]
  assert gaps / gaps[0] == pytest.approx([1.0, 2.0, 4.0], rel=1e-5), 'twice the distance is twice the pixels'
  ppm = view.lateral_ppm
  assert gaps[1] / ppm == pytest.approx(30.0, rel=1e-4), 'the drawn gap is the printed gap'
  # The printed number is radar dRel unchanged.
  radar = SimpleNamespace(leadOne=SimpleNamespace(present=True, dRel=30.0, yRel=0.0, vRel=0.0),
                          leadTwo=SimpleNamespace(present=False, dRel=0.0, yRel=0.0, vRel=0.0))
  lead = ber.scene_leads(radar, None)[0]
  assert lead.d_rel == 30.0
  assert bem.distance_label(lead.d_rel) == '30 m' and bem.distance_label(lead.d_rel, metric=False) == '33 yd'
  assert bem.distance_label(91.44, metric=False) == '100 yd'
  # Lead and ego outlines: 4.6 x 1.9 m at the lane lines' px/m, the lead's rear bumper at dRel.
  assert (ber.EGO_LENGTH, ber.EGO_WIDTH, ber.LEAD_LENGTH, ber.LEAD_WIDTH) == (4.6, 1.9, 4.6, 1.9)
  renderer = ber.BirdsEyeRenderer(text=object())
  body = ber._car_body(ber.LEAD_LENGTH, ber.LEAD_WIDTH)
  cx = lead.d_rel + front + ber.LEAD_LENGTH / 2
  _, hulls = renderer._cars(view, stream, [(cx, 0.0, 0.0)], body, ber.CAR_LAYOUT, 0, [[(1, 1, 1, 1)] * 3], [(1, 1, 1, 1)])
  hull = hulls[0]
  assert np.ptp(hull[:, 0]) / ppm == pytest.approx(1.9, rel=1e-3)
  assert np.ptp(hull[:, 1]) / ppm == pytest.approx(4.6, rel=1e-3)
  assert (ys[0] - hull[:, 1].max()) / ppm == pytest.approx(30.0, abs=0.01)
  assert 3.7 * ppm / np.ptp(hull[:, 0]) == pytest.approx(3.7 / 1.9, rel=1e-3), 'a lane is ~2 car widths'
  # A 60 m lead's whole outline and the whole ego fit on the stream frame.
  far = bem.car_to_widget(np.array([[60.0 + front + ber.LEAD_LENGTH, 0.0], [front - ber.EGO_LENGTH, 0.0]]), stream, cfg)
  assert far[0, 1] > 0 and far[1, 1] < stream.height


def test_lead_heading_follows_the_lanes_not_the_planned_path():
  from openpilot.selfdrive.ui.onroad.birds_eye_renderer import BirdsEyeRenderer
  xs = np.linspace(0, 120, 33)
  model = _model()
  model.position = _xyz(xs, 0.004 * xs ** 2)  # the plan curves right (steering), the road is straight
  view = BirdsEyeRenderer(text=object())
  view.ingest(model, None, None)
  assert view._heading_at(view.scene, 30.0) > 0.1, 'the path heading bends with the plan'
  assert view._road_heading(view.scene, 30.0) == pytest.approx(0.0, abs=1e-9), 'the lead follows the road'
  bent = _model()
  bent.laneLines = [_xyz(xs, 0.1 * xs + off) for off in (-5.4, -1.8, 1.8, 5.4)]
  bent.roadEdges = [_xyz(xs, np.full(33, off)) for off in (-7, 7)]
  view = BirdsEyeRenderer(text=object())
  view.ingest(bent, None, None)
  assert view._road_heading(view.scene, 30.0) == pytest.approx(math.atan(0.1))
  profiles = view._road_profiles(view.scene)
  assert view._road_profiles(view.scene) is profiles, 'cached per model frame'
  # Unconfident ego lane lines fall back to the road edges, then to the path.
  bent.laneLineProbs = [0.9, 0.1, 0.1, 0.9]
  view = BirdsEyeRenderer(text=object())
  view.ingest(bent, None, None)
  assert view._road_heading(view.scene, 30.0) == pytest.approx(0.0, abs=1e-9), 'edges are straight here'
  bent.roadEdges = []
  model.laneLineProbs, model.roadEdges, model.position = [0.0] * 4, [], _xyz(xs, 0.1 * xs)
  view = BirdsEyeRenderer(text=object())
  view.ingest(model, None, None)
  assert view._road_heading(view.scene, 30.0) == pytest.approx(math.atan(0.1), abs=1e-6), 'path only without lanes/edges'


def test_zoom_follows_speed_and_keeps_the_view_true_to_scale():
  cfg = bem.DEFAULT_CONFIG
  assert bem.zoom_x_max(0.0) == bem.zoom_x_max(10.0) == bem.ZOOM_NEAR_X_MAX
  assert bem.zoom_x_max(30.0) == bem.zoom_x_max(40.0) == cfg.x_max
  assert bem.zoom_x_max(20.0) == pytest.approx((bem.ZOOM_NEAR_X_MAX + cfg.x_max) / 2)
  assert bem.zoomed_config(cfg, cfg.x_max) is cfg
  stream = SimpleNamespace(x=0.0, y=0.0, width=1280.0, height=720.0)
  near = bem.zoomed_config(cfg, bem.ZOOM_NEAR_X_MAX)
  far_view, near_view = bem.view_params(stream, cfg), bem.view_params(stream, near)
  assert far_view.lateral_ppm == pytest.approx(9.3, abs=0.1) and near_view.lateral_ppm == pytest.approx(19.4, abs=0.2)
  assert float(bem.depth_scale_at(10.0, near_view)) == pytest.approx(near_view.lateral_ppm), 'still isotropic'
  assert near.x_min == cfg.x_min and near.lateral_range / cfg.lateral_range == pytest.approx(35 / 73)
  ego = np.array([[cfg.x_min, 0.0]])
  assert bem.car_to_widget(ego, stream, near)[0] == pytest.approx(bem.car_to_widget(ego, stream, cfg)[0]), 'the view is anchored at x_min'


def test_zoom_filter_is_smooth_and_quantized():
  zoom = bem.ZoomFilter()
  assert zoom.update(None, 0.0) == bem.DEFAULT_CONFIG.x_max, 'no speed yet: the default view'
  assert zoom.update(5.0, 0.0) == bem.ZOOM_NEAR_X_MAX, 'the first speed is taken directly'
  values = [zoom.update(30.0, 1 / 30) for _ in range(30)]
  assert values[0] < bem.ZOOM_NEAR_X_MAX + 2.0, 'a speed step does not jump the view'
  assert 50.0 < values[-1] < 60.0, '~1 s time constant: ~63 % of the way after 1 s'
  assert all(v % bem.ZOOM_STEP == 0 for v in values) and values == sorted(values)
  # A real acceleration (1.5 m/s^2 from 10 to 30 m/s) changes the view on a minority of frames.
  zoom = bem.ZoomFilter()
  ramp = [zoom.update(min(30.0, 10.0 + 1.5 * i / 30), 1 / 30) for i in range(600)]
  changes = sum(a != b for a, b in zip(ramp, ramp[1:], strict=False))
  assert ramp[-1] == bem.DEFAULT_CONFIG.x_max and changes < 0.3 * len(ramp)
  for i in range(300):
    zoom.update(29.8 if i % 2 else 30.2, 1 / 30)
  assert zoom.x_max == bem.DEFAULT_CONFIG.x_max, 'speed noise under a step does not move the view'


def test_zoom_tightens_to_a_radar_lead():
  cfg = bem.DEFAULT_CONFIG
  from openpilot.selfdrive.ui.onroad.birds_eye_renderer import EGO_FRONT, LEAD_LENGTH
  def front(d_rel):
    return d_rel + EGO_FRONT + LEAD_LENGTH
  assert bem.lead_zoom_x_max(30.0, None) == cfg.x_max, 'no lead: the speed preset'
  assert bem.lead_zoom_x_max(30.0, front(20.0)) == pytest.approx(20.0 + EGO_FRONT + LEAD_LENGTH + bem.ZOOM_LEAD_BUFFER)
  assert bem.lead_zoom_x_max(30.0, front(55.0)) == pytest.approx(67.1), 'a far lead still fits under the preset'
  assert bem.lead_zoom_x_max(30.0, front(80.0)) == cfg.x_max, 'never wider than the preset'
  assert bem.lead_zoom_x_max(30.0, front(5.0)) == bem.ZOOM_NEAR_X_MAX, 'never tighter than the city view'
  assert bem.lead_zoom_x_max(5.0, front(20.0)) == bem.ZOOM_NEAR_X_MAX
  zoom = bem.ZoomFilter()
  assert zoom.update(30.0, 0.0, front(20.0)) == 32.0, 'quantized to ZOOM_STEP'
  assert zoom.update(30.0, 0.0, None) == 32.0, 'the change is filtered, not a jump'
  for _ in range(300):
    zoom.update(30.0, 1 / 30, None)
  assert zoom.x_max == cfg.x_max, 'the lead gone: back to the preset'


def test_renderer_zoom_follows_the_radar_lead_one():
  from openpilot.selfdrive.ui.onroad.birds_eye_renderer import BirdsEyeRenderer
  view = BirdsEyeRenderer(text=object())
  view.clock = lambda: 0.0
  view.ingest(None, None, SimpleNamespace(steeringAngleDeg=0.0, leftBlinker=False, rightBlinker=False, vEgo=30.0))
  view.scene.lead_one_d = 20.0
  assert view.view_config().x_max == 32.0
  view = BirdsEyeRenderer(text=object())
  view.clock = lambda: 0.0
  view.ingest(None, None, SimpleNamespace(steeringAngleDeg=0.0, leftBlinker=False, rightBlinker=False, vEgo=30.0))
  assert view.view_config().x_max == view.config.x_max, 'no lead: the preset'


def test_renderer_zoom_rebuilds_the_view_only_on_a_step():
  from openpilot.selfdrive.ui.onroad.birds_eye_renderer import BirdsEyeRenderer
  view = BirdsEyeRenderer(text=object())
  now = [0.0]
  view.clock = lambda: now[0]
  assert view.view_config() == view.config, 'no carState yet'
  view.ingest(None, None, SimpleNamespace(steeringAngleDeg=0.0, leftBlinker=False, rightBlinker=False, vEgo=3.0))
  first = view.view_config()
  assert first.x_max == bem.ZOOM_NEAR_X_MAX and first.x_min == view.config.x_min
  now[0] += 1 / 30
  assert view.view_config() == first
  assert view.scene.grid[-1] == view.config.x_max, 'the scene grid spans the widest zoom'


def test_lines_stop_at_the_camera_draw_distance_and_the_path_at_the_camera_lead_cut():
  from openpilot.selfdrive.ui.onroad import birds_eye_renderer as ber
  model = _model()
  model.position = _xyz(np.linspace(0, 40, 33), np.zeros(33))  # a slow plan: the camera draws 40 m
  assert ber.camera_draw_distance(model) == 40.0
  model.position = _xyz(np.linspace(0, 3, 33), np.zeros(33))
  assert ber.camera_draw_distance(model) == ber.MIN_DRAW_DISTANCE
  assert ber.camera_path_end(None, 80.0) == 80.0
  assert ber.camera_path_end(30.0, 80.0) == pytest.approx(50.0), 'lead_d = 60, minus min(21, 10)'
  assert ber.camera_path_end(10.0, 80.0) == pytest.approx(13.0), 'lead_d = 20, minus 7'
  assert ber.camera_path_end(60.0, 80.0) == 80.0
  view, radar = _renderer_with_scene()  # the plan reaches 120 m, a radar lead at 30 m
  assert view.scene.draw_distance == ber.MAX_DRAW_DISTANCE and view.scene.lead_one_d == 30.0
  short = _model()
  short.position = _xyz(np.linspace(0, 40, 33), np.zeros(33))
  view.ingest(short, radar, None)
  assert view.scene.draw_distance == 40.0
  captured = {}
  original = view._build_road

  def spy(scene, rect, v, reach):
    captured.update(reach=reach)
    return original(scene, rect, v, reach)
  view._build_road = spy
  _, path = _layers(view)
  assert captured['reach'] <= 40.0 and captured['reach'] > 38.0
  assert path.end is not None and path.end[0] <= 40.0, 'the lead cut (50 m) is clipped to the draw distance'
  stream = SimpleNamespace(x=0.0, y=0.0, width=1280.0, height=720.0)
  pv = bem.view_params(stream, view.config)
  road = original(view.scene, stream, pv, 40.0)
  top = bem.car_to_widget(np.array([[40.0, 0.0]]), stream, view.config, pv)[0, 1]
  for op in _draw_ops(road.road + road.lines):
    assert _op_points(op)[:, 1].min() >= top - 4.0, 'nothing beyond the draw distance'


def test_ego_lane_lines_are_at_least_three_pixels_zoomed_out():
  from openpilot.selfdrive.ui.onroad import birds_eye_renderer as ber
  view, _ = _renderer_with_scene()
  styles = view._line_styles(view.scene)
  edges, lanes = styles[:len(view.scene.edges)], styles[len(view.scene.edges):]
  assert all(line[2] == ber.EDGE_MIN_PX == 3.5 for line in edges)
  assert all(line[2] == ber.LANE_MIN_PX == 3.0 for line in lanes)
  assert not hasattr(view, '_blinkers') and not hasattr(ber, 'TRIANGLE_LAYOUT'), 'the HUD owns the blinker arrow'


def test_radar_lead_lateral_is_radard_yrel_only():
  from openpilot.selfdrive.ui.onroad.birds_eye_renderer import scene_leads
  radar = SimpleNamespace(leadOne=SimpleNamespace(present=True, dRel=30.0, yRel=0.0, vRel=0.0),
                          leadTwo=SimpleNamespace(present=False, dRel=0.0, yRel=0.0, vRel=0.0))
  model = _model(lead_prob=0.9)
  model.leadsV3[0].y = [2.5]
  assert scene_leads(radar, model)[0].y == 0.0, 'the model never overrides the fused lead'
  radar.leadOne.yRel = -6.0
  assert scene_leads(radar, model)[0].y == 6.0, 'a lead two lanes over is not clamped'


def test_lead_smoothing_follows_a_lane_change():
  from openpilot.selfdrive.ui.onroad.birds_eye_renderer import BirdsEyeLead, BirdsEyeRenderer
  leads = [BirdsEyeLead(30.0, 0.0, 0.0)]
  for _ in range(10):  # 0.5 s at 20 Hz after a 1.2 m step
    leads = BirdsEyeRenderer._smooth_leads(leads, [BirdsEyeLead(30.0, 1.2, 0.0)])
  assert leads[0].y == pytest.approx(1.2, abs=0.01), 'lateral motion is tracked, nothing clamps y'
  jumped = BirdsEyeRenderer._smooth_leads(leads, [BirdsEyeLead(30.0, 4.9, 0.0)])
  assert jumped[0].y == 4.9, 'a whole-lane jump is taken directly, not held back'


def test_lead_glyph_is_drawn_at_its_own_lateral_position(monkeypatch):
  from openpilot.selfdrive.ui.onroad import birds_eye_renderer as ber
  view, radar = _renderer_with_scene()
  poses = []
  monkeypatch.setattr(view, '_cars', lambda v, rect, p, *a: poses.extend(p) or ([[]] * len(p), np.zeros((len(p), 8, 2))))
  monkeypatch.setattr(ber, '_run', lambda *a: None)
  monkeypatch.setattr(ber.rl, 'draw_rectangle_rounded', lambda *a: None)
  for y_rel in (0.0, -3.0):  # the label filter follows the glyph; it never moves the glyph
    radar.leadOne.yRel = y_rel
    view.scene.leads = ber.scene_leads(radar, None)
    view._leads(view.scene, RECT, bem.view_params(RECT, view.config))
  assert [p[1] for p in poses] == [0.0, 3.0]


def test_label_position_is_filtered_and_whole_pixels():
  from openpilot.selfdrive.ui.onroad.birds_eye_renderer import BirdsEyeRenderer
  view = BirdsEyeRenderer(text=object())
  x, y = view._label_position([], 100.4, 200.6)
  assert (x, y) == (100.0, 201.0)
  prev, view._labels = view._labels, []
  x, y = view._label_position(prev, 110.4, 200.6)
  assert (x, y) == (105.0, 201.0), 'half way toward the glyph'
  prev, view._labels = view._labels, []
  assert view._label_position(prev, 400.0, 200.0) == (400.0, 200.0), 'a side flip snaps'
