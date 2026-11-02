"""Tesla-style birds-eye (top-down chase view) of the model's road, plan and leads.

Pure 2D: everything is mapped through birds_eye_math.car_to_widget, no camera matrix.
Geometry is resampled onto one forward-distance grid so every line has the same sample
count, which keeps the ribbons simple and lets frames be blended without jitter.
"""
from collections.abc import Callable
from dataclasses import dataclass, field
import math
import time

import numpy as np
import pyray as rl

from openpilot.selfdrive.ui.onroad import birds_eye_math as bem
from openpilot.system.ui.lib.shader_polygon import Gradient
from openpilot.system.ui.widgets import Widget

EGO_FRONT = 1.5      # meters from the camera to the ego front bumper
EGO_LENGTH = 4.8
EGO_WIDTH = 1.9
LEAD_LENGTH = 4.6
LEAD_WIDTH = 1.9
PATH_WIDTH = 1.7
LANE_WIDTH = 0.14
EDGE_WIDTH = 0.28
MODEL_LEAD_PROB = 0.5
SMOOTHING = 0.55     # blend toward each new model frame
LEAD_SMOOTHING = 0.5

THROTTLE = ((13, 248, 122), (114, 255, 92))
NO_THROTTLE = ((242, 242, 242), (242, 242, 242))


@dataclass
class BirdsEyeLead:
  d_rel: float
  y: float            # right-positive, meters
  v_rel: float
  radar: bool = True


@dataclass
class BirdsEyeScene:
  grid: np.ndarray = field(default_factory=lambda: bem.sample_grid())
  path: np.ndarray | None = None          # lateral offset per grid point, NaN where absent
  path_accel: np.ndarray | None = None
  lanes: list[np.ndarray] = field(default_factory=list)
  lane_probs: np.ndarray = field(default_factory=lambda: np.zeros(4))
  edges: list[np.ndarray] = field(default_factory=list)
  edge_stds: np.ndarray = field(default_factory=lambda: np.ones(2))
  leads: list[BirdsEyeLead] = field(default_factory=list)
  steering_deg: float = 0.0
  left_blinker: bool = False
  right_blinker: bool = False
  engaged: bool = False
  experimental: bool = False
  allow_throttle: bool = True
  valid: bool = False


def _floats(value) -> np.ndarray:
  return np.asarray(list(value), dtype=np.float64)


def model_lines(model, grid):
  """Resampled path, lane lines and road edges from a modelV2 message (or a duck-typed stand-in)."""
  pos = model.position
  path = bem.resample_line(_floats(pos.x), _floats(pos.y), grid, extend_back=False)
  path[grid < 0] = np.nan
  accel_src = _floats(model.acceleration.x) if hasattr(model, 'acceleration') else np.empty(0)
  path_accel = None
  px = _floats(pos.x)
  if accel_src.size and px.size:
    n = min(accel_src.size, px.size)
    path_accel = bem.resample_line(px[:n], accel_src[:n], grid, extend_back=False)
  lanes = [bem.resample_line(_floats(line.x), _floats(line.y), grid) for line in list(model.laneLines)[:4]]
  edges = [bem.resample_line(_floats(edge.x), _floats(edge.y), grid) for edge in list(model.roadEdges)[:2]]
  probs = _floats(model.laneLineProbs)[:4]
  stds = _floats(model.roadEdgeStds)[:2]
  return path, path_accel, lanes, np.pad(probs, (0, 4 - probs.size)), edges, np.pad(stds, (0, 2 - stds.size), constant_values=1.0)


def scene_leads(radar_state, model) -> list[BirdsEyeLead]:
  """Radar leads first; a confident model lead is added only where radar has none."""
  leads: list[BirdsEyeLead] = []
  if radar_state is not None:
    for lead in (radar_state.leadOne, radar_state.leadTwo):
      if lead.present and math.isfinite(lead.dRel) and lead.dRel > 0:
        # radarState yRel is left-positive.
        leads.append(BirdsEyeLead(float(lead.dRel), -float(lead.yRel), float(lead.vRel), radar=True))
  if model is not None:
    for lead in list(getattr(model, 'leadsV3', []))[:1]:
      xs, ys = list(lead.x), list(lead.y)
      if lead.prob < MODEL_LEAD_PROB or not xs or not ys:
        continue
      d, y = float(xs[0]) - EGO_FRONT, float(ys[0])
      if d <= 0 or any(abs(o.d_rel - d) < 6.0 and abs(o.y - y) < 2.0 for o in leads):
        continue
      # leadsV3 speed is absolute; without ego speed here the closing rate is unknown.
      leads.append(BirdsEyeLead(d, y, math.nan, radar=False))
  return leads


class _DefaultText:
  """gui_app fonts; screens with their own bitmap fonts pass a drawer instead."""
  def measure(self, text: str, size: float) -> tuple[float, float]:
    from openpilot.system.ui.lib.application import FontWeight, gui_app
    from openpilot.system.ui.lib.text_measure import measure_text_cached
    measured = measure_text_cached(gui_app.font(FontWeight.SEMI_BOLD), text, int(size))
    return measured.x, measured.y

  def draw(self, text: str, size: float, x: float, y: float, color: rl.Color) -> None:
    from openpilot.system.ui.lib.application import FontWeight, gui_app
    rl.draw_text_ex(gui_app.font(FontWeight.SEMI_BOLD), text, rl.Vector2(x, y), size, 0, color)


def _color(rgb, alpha) -> rl.Color:
  return rl.Color(int(rgb[0]), int(rgb[1]), int(rgb[2]), int(max(0, min(255, alpha))))


def _fan(points: np.ndarray, color: rl.Color) -> None:
  """Filled convex polygon; raylib wants counter-clockwise as seen on a y-down screen."""
  pts = np.asarray(points, dtype=np.float64)
  if pts.shape[0] < 3:
    return
  x, y = pts[:, 0], pts[:, 1]
  if np.dot(x, np.roll(y, -1)) - np.dot(y, np.roll(x, -1)) > 0:
    pts = pts[::-1]
  centre = pts.mean(axis=0)
  fan = [rl.Vector2(float(centre[0]), float(centre[1]))] + [rl.Vector2(float(px), float(py)) for px, py in pts]
  fan.append(fan[1])
  rl.draw_triangle_fan(fan, len(fan), color)


def _outline(points: np.ndarray, thickness: float, color: rl.Color) -> None:
  pts = np.asarray(points, dtype=np.float64)
  for i in range(len(pts)):
    a, b = pts[i], pts[(i + 1) % len(pts)]
    rl.draw_line_ex(rl.Vector2(float(a[0]), float(a[1])), rl.Vector2(float(b[0]), float(b[1])), thickness, color)


class BirdsEyeRenderer(Widget):
  """Top-down chase view. Feed it with poll(sm, started_frame) or ingest(...), then render(rect).

  panel=True draws a rounded translucent card (overlay); panel=False fills the rect edge to edge.
  """

  def __init__(self, config: bem.BirdsEyeConfig | None = None, text=None,
               source: Callable[[], tuple] | None = None):
    super().__init__()
    self.config = config or bem.BirdsEyeConfig()
    self.text = text or _DefaultText()
    # Returns (sm, started_frame); None means the caller ingests data itself.
    self.source = source
    self.panel = True
    self.opacity = 1.0
    self.road_style: dict = {}
    self.metric = True
    self.long_control = True
    self.parent_clip: rl.Rectangle | None = None
    self.clock = time.monotonic
    self.scene = BirdsEyeScene(grid=bem.sample_grid(self.config))
    self._throttle_blend = 1.0
    self._last_model_frame = -1
    self._rainbow = None
    self._lead_model = None

  # ---- data -------------------------------------------------------------------------------------

  def ingest(self, model=None, radar_state=None, car_state=None, *, engaged=None, experimental=None,
             allow_throttle=None) -> None:
    """Accept one observation. model=None keeps the last good road geometry."""
    scene = self.scene
    if model is not None:
      path, accel, lanes, probs, edges, stds = model_lines(model, scene.grid)
      scene.path = bem.smooth(scene.path, path, SMOOTHING) if scene.valid else path
      scene.path_accel = accel
      scene.lanes = [bem.smooth(old, new, SMOOTHING) for old, new in zip(scene.lanes, lanes, strict=False)] \
        if scene.valid and len(scene.lanes) == len(lanes) else lanes
      scene.edges = [bem.smooth(old, new, SMOOTHING) for old, new in zip(scene.edges, edges, strict=False)] \
        if scene.valid and len(scene.edges) == len(edges) else edges
      scene.lane_probs, scene.edge_stds = probs, stds
      scene.valid = bool(np.isfinite(path).any() or any(np.isfinite(lane).any() for lane in lanes))
    if model is not None:
      self._lead_model = model
    if radar_state is not None or model is not None:
      scene.leads = self._smooth_leads(scene.leads, scene_leads(radar_state, self._lead_model))
    if car_state is not None:
      scene.steering_deg = float(car_state.steeringAngleDeg)
      scene.left_blinker, scene.right_blinker = bool(car_state.leftBlinker), bool(car_state.rightBlinker)
    if engaged is not None:
      scene.engaged = bool(engaged)
    if experimental is not None:
      scene.experimental = bool(experimental)
    if allow_throttle is not None:
      scene.allow_throttle = bool(allow_throttle)

  @staticmethod
  def _smooth_leads(previous, current):
    out = []
    for lead in current:
      match = next((p for p in previous if p.radar == lead.radar and abs(p.d_rel - lead.d_rel) < 5 and abs(p.y - lead.y) < 1.5), None)
      if match is not None:
        lead = BirdsEyeLead(match.d_rel + (lead.d_rel - match.d_rel) * LEAD_SMOOTHING,
                            match.y + (lead.y - match.y) * LEAD_SMOOTHING, lead.v_rel, lead.radar)
      out.append(lead)
    return out

  def poll(self, sm, started_frame: int) -> None:
    """Read SubMaster services; road geometry changes only on a fresh modelV2."""
    def fresh(name):
      return bool(sm.valid.get(name, False) if hasattr(sm.valid, 'get') else sm.valid[name]) and sm.recv_frame[name] >= started_frame

    model = None
    if fresh('modelV2') and sm.updated['modelV2'] and sm.recv_frame['modelV2'] != self._last_model_frame:
      model = sm['modelV2']
      self._last_model_frame = sm.recv_frame['modelV2']
    radar = sm['radarState'] if fresh('radarState') else None
    car = sm['carState'] if fresh('carState') else None
    engaged = experimental = allow = None
    if fresh('selfdriveState'):
      engaged = sm['selfdriveState'].enabled
      experimental = sm['selfdriveState'].experimentalMode
    if fresh('longitudinalPlan'):
      allow = sm['longitudinalPlan'].allowThrottle or not self.long_control
    # Without a new modelV2 the road keeps its last good frame; leads, wheel and blinkers still follow.
    self.ingest(model, radar, car, engaged=engaged, experimental=experimental, allow_throttle=allow)

  # ---- drawing ----------------------------------------------------------------------------------

  def _render(self, rect: rl.Rectangle):
    if self.source is not None:
      sm, started_frame = self.source()
      self.poll(sm, started_frame)
    self.draw(rect)

  def draw(self, rect: rl.Rectangle) -> None:
    from openpilot.starpilot.ui import clip
    view = bem.view_params(rect, self.config)
    self._background(rect)
    clip.begin_scissor_mode(rect.x, rect.y, rect.width, rect.height)
    rl.rl_disable_backface_culling()
    try:
      scene = self.scene
      if scene.valid:
        self._road(scene, rect, view)
      self._ticks(rect, view)
      if scene.valid:
        self._lines(scene, rect, view)
        self._path(scene, rect, view)
      self._leads(scene, rect, view)
      self._ego(scene, rect, view)
      self._blinkers(scene, rect, view)
    finally:
      rl.rl_draw_render_batch_active()
      rl.rl_enable_backface_culling()
      if self.parent_clip is not None:
        clip.begin_scissor_mode(self.parent_clip.x, self.parent_clip.y, self.parent_clip.width, self.parent_clip.height)
      else:
        clip.end_scissor_mode()
    if self.panel:
      rl.draw_rectangle_rounded_lines_ex(rect, self._roundness(rect), 12, 2, rl.Color(255, 255, 255, int(40 * self.opacity)))

  def _roundness(self, rect) -> float:
    return min(1.0, 2 * 28 / max(1.0, min(rect.width, rect.height)))

  def _background(self, rect) -> None:
    if self.panel:
      rl.draw_rectangle_rounded(rect, self._roundness(rect), 12, rl.Color(10, 12, 18, int(235 * self.opacity)))
    else:
      rl.draw_rectangle_gradient_v(int(rect.x), int(rect.y), int(rect.width), int(rect.height),
                                   rl.Color(26, 31, 44, 255), rl.Color(8, 9, 13, 255))

  @staticmethod
  def _height_fraction(ys: np.ndarray, rect) -> np.ndarray:
    """0 at the bottom of the rect, 1 at the top: the axis every fade and gradient here runs along."""
    return np.clip((rect.y + rect.height - ys) / max(1.0, rect.height), 0.0, 1.0)

  @staticmethod
  def _fade(rgb, alpha, t: np.ndarray, hold: float = 0.6, end: float = 0.95) -> np.ndarray:
    """Per-row RGBA holding alpha near the car and fading out toward the far edge."""
    a = alpha * np.interp(t, [0.0, hold, end], [1.0, 0.8, 0.0])
    rows = np.empty((t.size, 4))
    rows[:, :3] = rgb
    rows[:, 3] = a
    return rows

  @staticmethod
  def _sample(gradient: Gradient, t: np.ndarray, alpha_scale: float = 1.0) -> np.ndarray:
    """CPU sample of a shader Gradient (colors[0] at the bottom) at height fractions t."""
    stops = np.asarray(gradient.stops, dtype=np.float64)
    cols = np.array([(c.r, c.g, c.b, c.a) for c in gradient.colors], dtype=np.float64)
    out = np.stack([np.interp(t, stops, cols[:, i]) for i in range(4)], axis=1)
    out[:, 3] *= alpha_scale
    return out

  @staticmethod
  def _strip(poly: np.ndarray, rows: np.ndarray) -> None:
    """Draw a ribbon [L0..Ln-1, Rn-1..R0] with one RGBA per row, as per-vertex colored quads.

    Shader gradients map through gl_FragCoord, whose origin differs between a window and a render
    target; vertex colors put each fade exactly where its row is on every target."""
    n = poly.shape[0] // 2
    if n < 2:
      return
    rgba = np.clip(rows, 0, 255).astype(np.int32).tolist()
    lp, rp = poly[:n].tolist(), poly[n:][::-1].tolist()
    color, vertex = rl.rl_color4ub, rl.rl_vertex2f
    rl.rl_begin(rl.RL_QUADS)
    for i in range(n - 1):
      c0, c1 = rgba[i], rgba[i + 1]
      if c0[3] == 0 and c1[3] == 0:
        continue
      color(*c0)
      vertex(*lp[i])
      vertex(*rp[i])
      color(*c1)
      vertex(*rp[i + 1])
      vertex(*lp[i + 1])
    rl.rl_end()

  def _project_line(self, ys, grid, view, rect):
    pts = np.stack((grid, np.nan_to_num(ys)), axis=1)
    return bem.car_to_widget(pts, rect, self.config, view)

  def _line_ribbon(self, ys, grid, view, rect, width_m, min_px):
    ok = np.isfinite(ys)
    pieces = []
    for start, end in bem.valid_runs(ok):
      if end - start < 2:
        continue
      centre = self._project_line(ys[start:end], grid[start:end], view, rect)
      half = np.maximum(width_m * bem.scale_at(grid[start:end], view), min_px) / 2
      pieces.append((bem.ribbon(centre, half), centre))
    return pieces

  def _road(self, scene, rect, view) -> None:
    grid = scene.grid
    pair = scene.edges if len(scene.edges) == 2 else []
    if not pair and len(scene.lanes) == 4:
      pair = [scene.lanes[0], scene.lanes[3]]
    if len(pair) != 2:
      return
    left, right = pair
    ok = np.isfinite(left) & np.isfinite(right)
    for start, end in bem.valid_runs(ok):
      if end - start < 2:
        continue
      lpts = self._project_line(left[start:end], grid[start:end], view, rect)
      rpts = self._project_line(right[start:end], grid[start:end], view, rect)
      t = self._height_fraction(lpts[:, 1], rect)
      self._strip(np.vstack((lpts, rpts[::-1])), self._fade((40, 46, 60), 215 * self.opacity, t, hold=0.55, end=0.97))

  def _font_size(self, rect) -> float:
    return max(15.0, min(26.0, rect.height * 0.026))

  def _label(self, text, size, x, y, color) -> None:
    self.text.draw(text, size, x + 1.5, y + 1.5, rl.Color(0, 0, 0, int(color.a * 0.7)))
    self.text.draw(text, size, x, y, color)

  def _ticks(self, rect, view) -> None:
    """Faint distance rungs every 10 m (50 ft), labeled at 20/40/60 m (100/200 ft)."""
    size = self._font_size(rect)
    step, labeled, unit, per_unit = (10.0, (20, 40, 60), 'm', 1.0) if self.metric else (50.0, (100, 200), 'ft', 0.3048)
    count = int(self.config.x_max / (step * per_unit))
    for index in range(1, count + 1):
      value = index * step
      distance = value * per_unit
      y = float(bem.car_to_widget(np.array([[distance, 0.0]]), rect, self.config, view)[0, 1])
      if y < rect.y + size * 1.5:
        break
      half = min(rect.width / 2 - 14, 11.0 * float(bem.scale_at(distance, view)))
      fade = float(np.interp(self._height_fraction(np.array([y]), rect)[0], [0.0, 0.6, 0.95], [1.0, 0.8, 0.0]))
      rl.draw_line_ex(rl.Vector2(view.cx - half, y), rl.Vector2(view.cx + half, y), 1.0,
                      rl.Color(255, 255, 255, int(24 * fade * self.opacity)))
      if int(value) in labeled:
        text = f"{int(value)} {unit}"
        _, h = self.text.measure(text, size)
        self._label(text, size, rect.x + 14, y - h / 2, rl.Color(170, 180, 198, int(160 * self.opacity)))

  def _lines(self, scene, rect, view) -> None:
    grid = scene.grid
    for i, edge in enumerate(scene.edges):
      std = scene.edge_stds[i] if i < len(scene.edge_stds) else 1.0
      alpha = float(np.clip(1.0 - std, 0.0, 1.0))
      if alpha < 0.05:
        continue
      for poly, centre in self._line_ribbon(edge, grid, view, rect, EDGE_WIDTH, 2.5):
        self._strip(poly, self._fade((150, 158, 176), 230 * alpha * self.opacity, self._height_fraction(centre[:, 1], rect)))
    from openpilot.starpilot.ui.road_colors import lane_color
    for i, lane in enumerate(scene.lanes):
      prob = float(scene.lane_probs[i]) if i < len(scene.lane_probs) else 0.0
      alpha = float(np.clip(prob, 0.0, 0.7)) / 0.7
      if alpha < 0.05:
        continue
      default = _color((236, 240, 248), 235 * alpha)
      color = lane_color(self.road_style, False, default, prob)
      a = color.a if color is default else min(255, color.a / 0.7)
      for poly, centre in self._line_ribbon(lane, grid, view, rect, LANE_WIDTH, 1.8):
        self._strip(poly, self._fade((color.r, color.g, color.b), a * self.opacity, self._height_fraction(centre[:, 1], rect)))

  def _path_end(self, scene) -> float:
    end = self.config.x_max
    for lead in scene.leads:
      if abs(lead.y - bem.lateral_at(scene.grid, scene.path, lead.d_rel + EGO_FRONT, lead.y)) < 1.6:
        end = min(end, lead.d_rel + EGO_FRONT - 1.0)
    return end

  def _path_rows(self, scene, t: np.ndarray, accel: np.ndarray) -> np.ndarray:
    """Per-row path color, chosen the way ModelRenderer picks the camera path's color."""
    from openpilot.starpilot.ui.road_colors import path_mode, solid_gradient
    a = self.opacity
    if not scene.engaged:
      return self._fade((205, 210, 222), 150 * a, t, hold=0.3, end=0.9)
    style = self.road_style
    mode = path_mode(style)
    if mode == 'rainbow':
      if self._rainbow is None:
        from openpilot.starpilot.ui.rainbow_path import RainbowPath
        self._rainbow = RainbowPath()
      self._rainbow.update(time.monotonic_ns(), None, source_alive=False)
      return self._sample(self._rainbow.gradient(), t, 1.8 * a)
    if mode == 'color':
      from openpilot.starpilot.ui.onroad_customization import ROAD_COLORS
      return self._sample(solid_gradient(style.get('path', ROAD_COLORS['path'])), t, a)
    if scene.experimental and accel.size == t.size and np.isfinite(accel).any():
      acc = np.nan_to_num(accel)
      hue = np.clip(60 + acc * 35, 0, 120) / 360
      sat = np.minimum(np.abs(acc * 1.5), 1)
      light = np.interp(sat, [0, 1], [0.95, 0.62])
      import colorsys
      rgb = np.array([colorsys.hls_to_rgb(h, l, s_) for h, l, s_ in zip(hue, light, sat, strict=True)]) * 255
      rows = self._fade((0, 0, 0), 215 * a, t, hold=0.35, end=0.9)
      rows[:, :3] = rgb
      return rows
    blend = self._throttle_blend
    rgb = np.array([n + (th - n) * blend for n, th in zip(NO_THROTTLE[0], THROTTLE[0], strict=True)])
    return self._fade(rgb, 210 * a, t, hold=0.35, end=0.9)

  def _path(self, scene, rect, view) -> None:
    if scene.path is None:
      return
    target = 1.0 if scene.allow_throttle else 0.0
    self._throttle_blend += (target - self._throttle_blend) * 0.2
    grid = scene.grid
    end = self._path_end(scene)
    ys = scene.path.copy()
    ys[(grid > end) | (grid < EGO_FRONT - 0.5)] = np.nan
    runs = bem.valid_runs(np.isfinite(ys))
    if not runs or runs[0][1] - runs[0][0] < 2:
      return
    lo, hi = runs[0]
    g, y = grid[lo:hi], ys[lo:hi]
    u = (g - g[0]) / max(1e-3, self.config.x_max - g[0])
    centre = self._project_line(y, g, view, rect)
    half = np.maximum(PATH_WIDTH * (1.0 - 0.35 * u) * bem.scale_at(g, view), 3.0) / 2
    accel = scene.path_accel[lo:hi] if scene.path_accel is not None else np.empty(0)
    t = self._height_fraction(centre[:, 1], rect)
    self._strip(bem.ribbon(centre, half), self._path_rows(scene, t, accel))

  def _car(self, view, rect, cx, cy, heading, length, width, body, glass, outline, wheel_angle=None) -> np.ndarray:
    hull = bem.car_outline(cx, cy, heading, length, width, chamfer=0.38 * width / 1.9)
    screen = bem.car_to_widget(hull, rect, self.config, view)
    if wheel_angle is not None:
      for fx, side in ((length / 2 - 0.9, -1), (length / 2 - 0.9, 1), (-length / 2 + 0.85, -1), (-length / 2 + 0.85, 1)):
        angle = wheel_angle if fx > 0 else 0.0
        local = bem.rotate_offsets(np.array([(0.36, -0.12), (-0.36, -0.12), (-0.36, 0.12), (0.36, 0.12)]), 0, 0, angle)
        wheel = bem.rotate_offsets(local + np.array([fx, side * (width / 2 - 0.02)]), cx, cy, heading)
        _fan(bem.car_to_widget(wheel, rect, self.config, view), rl.Color(16, 18, 24, body.a))
    _fan(screen, body)
    scale = np.array([length / 4.6, width / 1.9])
    roof = bem.rotate_offsets(np.array([(0.45, -0.64), (-1.05, -0.66), (-1.05, 0.66), (0.45, 0.64)]) * scale, cx, cy, heading)
    _fan(bem.car_to_widget(roof, rect, self.config, view), glass)
    shield = bem.rotate_offsets(np.array([(1.2, -0.62), (0.6, -0.7), (0.6, 0.7), (1.2, 0.62)]) * scale, cx, cy, heading)
    _fan(bem.car_to_widget(shield, rect, self.config, view), rl.Color(glass.r, glass.g, glass.b, int(glass.a * 0.7)))
    if outline is not None:
      _outline(screen, 1.5, outline)
    return screen

  def _leads(self, scene, rect, view) -> None:
    size = self._font_size(rect)
    for lead in sorted(scene.leads, key=lambda lead: -lead.d_rel):
      cx = lead.d_rel + EGO_FRONT + LEAD_LENGTH / 2
      if cx - LEAD_LENGTH / 2 > self.config.x_max:
        continue
      heading = bem.heading_at(scene.grid, scene.path, cx) if scene.path is not None else 0.0
      rgb = bem.lead_color(lead.d_rel, lead.v_rel if math.isfinite(lead.v_rel) else 0.0)
      a = (1.0 if lead.radar else 0.7) * self.opacity
      screen = self._car(view, rect, cx, lead.y, heading, LEAD_LENGTH, LEAD_WIDTH, _color(rgb, 245 * a),
                         rl.Color(64, 72, 88, int(170 * a)), _color((255, 255, 255), 60 * a))
      text = bem.distance_label(lead.d_rel, self.metric)
      w, h = self.text.measure(text, size)
      x = float(screen[:, 0].max()) + 10
      if x + w > rect.x + rect.width - 8:
        x = float(screen[:, 0].min()) - 10 - w
      self._label(text, size, x, float(screen[:, 1].mean()) - h / 2, _color(rgb, 255 * self.opacity))

  def _ego(self, scene, rect, view) -> None:
    a = self.opacity
    cx = EGO_FRONT - EGO_LENGTH / 2
    centre = bem.car_to_widget(np.array([[cx, 0.0]]), rect, self.config, view)[0]
    glow_r = EGO_WIDTH * float(bem.scale_at(cx, view)) * 1.3
    rl.draw_circle_gradient(rl.Vector2(float(centre[0]), float(centre[1])), glow_r,
                            rl.Color(120, 170, 255, int(55 * a)), rl.Color(120, 170, 255, 0))
    self._car(view, rect, cx, 0.0, 0.0, EGO_LENGTH, EGO_WIDTH, rl.Color(238, 241, 247, int(255 * a)),
              rl.Color(48, 58, 76, int(255 * a)), rl.Color(255, 255, 255, int(110 * a)),
              wheel_angle=bem.ego_wheel_angle(scene.steering_deg))

  def _blinkers(self, scene, rect, view) -> None:
    if not (scene.left_blinker or scene.right_blinker) or (self.clock() % 0.8) > 0.5:
      return
    cx = EGO_FRONT - EGO_LENGTH / 2 + 0.6
    for on, side in ((scene.left_blinker, -1), (scene.right_blinker, 1)):
      if on:
        base = side * (EGO_WIDTH / 2 + 0.7)
        arrow = np.array([(cx + 0.8, base), (cx, base + side * 1.1), (cx - 0.8, base)])
        _fan(bem.car_to_widget(arrow, rect, self.config, view), rl.Color(64, 220, 120, int(235 * self.opacity)))


class BitmapText:
  """Adapter for StarPilot BitmapFonts (headless-safe), used by the projection and the preview."""
  def __init__(self, fonts, role=None):
    from openpilot.starpilot.ui.presentation import FontRole
    self.fonts = fonts
    self.role = FontRole.SEMI_BOLD if role is None else role

  def measure(self, text: str, size: float) -> tuple[float, float]:
    measured = self.fonts.measure(text, self.role, size)
    return measured.width, measured.height

  def draw(self, text: str, size: float, x: float, y: float, color: rl.Color) -> None:
    self.fonts.draw(text, self.role, size, x, y, color)
