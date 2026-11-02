"""Tesla-style birds-eye (top-down chase view) of the model's road, plan and leads.

Pure 2D: everything is mapped through birds_eye_math.car_to_widget, no camera matrix.
Geometry is resampled onto one forward-distance grid so every line has the same sample
count, which keeps the ribbons simple and lets frames be blended without jitter.
"""
from collections.abc import Callable
from dataclasses import dataclass, field
from functools import lru_cache
from itertools import chain, repeat, starmap
import math
import operator
import time

import numpy as np
import pyray as rl
from raylib import rl as _raw  # the C functions pyray wraps; per-vertex calls skip its argument conversion

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



def _rgba(rgb, alpha) -> tuple[int, int, int, int]:
  return int(rgb[0]), int(rgb[1]), int(rgb[2]), int(max(0, min(255, alpha)))


def _color(rgb, alpha) -> rl.Color:
  return rl.Color(*_rgba(rgb, alpha))


# A layer is a ready-made list of raw raylib calls, (function, *args), replayed by _emit. Each quad is
# color0, vertex0, vertex1, color1, vertex2, vertex3, so a ribbon row fades along its length.
Layer = list
EMPTY: Layer = []
_call = getattr(operator, 'call', lambda function, *args: function(*args))


def _layer(cols, verts) -> Layer:
  """Quads from per-quad colors (Q, 8) [r0, g0, b0, a0, r1, g1, b1, a1] and vertices (Q, 8) [x0, y0 .. x3, y3].

  Built with C-level zips: no Python bytecode runs per vertex, here or in _emit."""
  cols = np.asarray(cols, dtype=np.int32).reshape(-1, 8)
  if not cols.shape[0]:
    return []
  c = cols.T.tolist()
  v = np.asarray(verts, dtype=np.float64).reshape(-1, 8).T.tolist()
  color, vertex = _raw.rlColor4ub, _raw.rlVertex2f

  def calls(function, *columns):  # (function, *args) per quad; repeat() is endless, so not strict
    return zip(repeat(function), *columns, strict=False)

  quads = zip(calls(color, *c[:4]), calls(vertex, *v[0:2]), calls(vertex, *v[2:4]),
              calls(color, *c[4:]), calls(vertex, *v[4:6]), calls(vertex, *v[6:8]), strict=True)
  return list(chain.from_iterable(quads))


def _emit(*layers: Layer) -> None:
  """All quads of the given layers in one rlBegin/rlEnd, through the raw C functions (no pyray wrapping).

  Vertex colors put each fade exactly where its row is on every target (shader gradients map through
  gl_FragCoord, whose origin differs between a window and a render target)."""
  if not any(layers):
    return
  # Bind the 1x1 white texture the way raylib's own shapes do: a quad batch opened right after text would
  # otherwise continue the font's draw call and sample its atlas.
  _raw.rlSetTexture(_raw.rlGetTextureIdDefault())
  _raw.rlBegin(rl.RL_QUADS)
  for layer in layers:
    any(starmap(_call, layer))  # every call returns None, so any() just drains the iterator in C
  _raw.rlEnd()
  _raw.rlSetTexture(0)


def _ribbon_quads(left: np.ndarray, right: np.ndarray, rows: np.ndarray, valid: np.ndarray, bottom: float) -> tuple[Layer, np.ndarray]:
  """Quads between consecutive valid rows of (R, N, 2) ribbon sides with one RGBA per row, and each ribbon's quad count.

  Rows fully transparent at both ends and quads wholly below the rect bottom are dropped."""
  rgba = np.clip(rows, 0, 255).astype(np.int32)
  verts = np.concatenate((left[..., :-1, :], right[..., :-1, :], right[..., 1:, :], left[..., 1:, :]), axis=-1)
  keep = valid[..., :-1] & valid[..., 1:] & ((rgba[..., :-1, 3] > 0) | (rgba[..., 1:, 3] > 0))
  keep &= verts[..., 1::2].min(axis=-1) <= bottom
  cols = np.concatenate((rgba[..., :-1, :], rgba[..., 1:, :]), axis=-1)
  return _layer(cols[keep], verts[keep]), keep.sum(axis=-1)


def _fan_indices(sizes) -> tuple[np.ndarray, np.ndarray]:
  """Quad vertex indices filling consecutive convex polygons as fans, and each quad's polygon number."""
  quads, owner, offset = [], [], 0
  for number, n in enumerate(sizes):
    for i in range(1, n - 1, 2):
      quads.append((offset, offset + i, offset + i + 1, offset + min(i + 2, n - 1)))
      owner.append(number)
    offset += n
  return np.array(quads), np.array(owner)


def _polygon_quads(screen: np.ndarray, layout, colors) -> Layer:
  """Fill polygons laid out by _fan_indices with one color each."""
  index, owner = layout
  rgba = np.asarray(colors, dtype=np.int32)[owner]
  return _layer(np.concatenate((rgba, rgba), axis=1), screen[index])


@lru_cache(maxsize=8)
def _car_body(length: float, width: float) -> np.ndarray:
  """Hull 8, roof 4 and windshield 4 points of a car glyph centered at the origin, car-frame meters."""
  scale = np.array([length / 4.6, width / 1.9])
  return np.concatenate((bem.car_outline(0.0, 0.0, 0.0, length, width, chamfer=0.38 * width / 1.9), ROOF * scale, SHIELD * scale))


WHEEL = np.array([(0.36, -0.12), (-0.36, -0.12), (-0.36, 0.12), (0.36, 0.12)])
ROOF = np.array([(0.45, -0.64), (-1.05, -0.66), (-1.05, 0.66), (0.45, 0.64)])
SHIELD = np.array([(1.2, -0.62), (0.6, -0.7), (0.6, 0.7), (1.2, 0.62)])
# Car glyph point order: [4 wheels x 4,] hull 8, roof 4, windshield 4.
CAR_LAYOUT = _fan_indices((8, 4, 4))
EGO_LAYOUT = _fan_indices((4, 4, 4, 4, 8, 4, 4))
TRIANGLE_LAYOUT = _fan_indices((3,))
TICK_FADE = ([0.0, 0.6, 0.95], [1.0, 0.8, 0.0])
WHEEL_STEP = math.radians(0.25)  # ego front-wheel angle resolution


class BirdsEyeRenderer(Widget):
  """Top-down chase view. Feed it with poll(sm, started_frame) or ingest(...), then render(rect).

  panel=True draws a rounded translucent card (overlay); panel=False fills the rect edge to edge.

  Geometry that only changes with the rect, the settings or a new model frame (road, lane lines, distance
  rungs, the ego glyph) is built once and replayed from cache. Scene arrays are treated as immutable:
  ingest() replaces them, and the cache keys on their identity.
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
    self._cache: dict[str, tuple] = {}


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

  # ---- caching ----------------------------------------------------------------------------------

  def _cached(self, name: str, key, build, keep=None):
    """build() once per key. keep holds the objects whose ids are in the key, so those ids stay unique."""
    hit = self._cache.get(name)
    if hit is not None and hit[0] == key:
      return hit[1]
    value = build()
    self._cache[name] = (key, value, keep)
    return value

  # ---- drawing ----------------------------------------------------------------------------------

  def _render(self, rect: rl.Rectangle):
    if self.source is not None:
      sm, started_frame = self.source()
      self.poll(sm, started_frame)
    self.draw(rect)

  def draw(self, rect: rl.Rectangle) -> None:
    from openpilot.starpilot.ui import clip
    rect_key = (float(rect.x), float(rect.y), float(rect.width), float(rect.height), self.config)
    view = self._cached('view', rect_key, lambda: bem.view_params(rect, self.config))
    self._background(rect)
    clip.begin_scissor_mode(rect.x, rect.y, rect.width, rect.height)
    rl.rl_disable_backface_culling()
    try:
      scene = self.scene
      road = lines = path = EMPTY
      if scene.valid:
        if scene.path is not None:
          target = 1.0 if scene.allow_throttle else 0.0
          self._throttle_blend += (target - self._throttle_blend) * 0.2
        road, lines, path = self._scene_layers(scene, rect, rect_key, view)
      ticks, labels = self._cached('ticks', (rect_key, self.opacity, self.metric), lambda: self._ticks(rect, view))
      _emit(road, ticks)
      for text, size, x, y, shadow, color in labels:
        self.text.draw(text, size, x + 1.5, y + 1.5, shadow)
        self.text.draw(text, size, x, y, color)
      _emit(lines, path)
      self._leads(scene, rect, view)
      self._ego(scene, rect, rect_key, view)
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
    """Per-row RGBA holding alpha near the car and fading out toward the far edge.

    rgb (3,) or (..., 3) and alpha scalar or (...,) broadcast against the rows t (..., N)."""
    t = np.asarray(t, dtype=np.float64)
    rows = np.empty((*t.shape, 4))
    rows[..., :3] = np.asarray(rgb, dtype=np.float64)[..., None, :] if np.ndim(rgb) > 1 else rgb
    rows[..., 3] = np.asarray(alpha, dtype=np.float64)[..., None] * np.interp(t, [0.0, hold, end], [1.0, 0.8, 0.0])
    return rows

  @staticmethod
  def _sample(gradient: Gradient, t: np.ndarray, alpha_scale: float = 1.0) -> np.ndarray:
    """CPU sample of a shader Gradient (colors[0] at the bottom) at height fractions t."""
    stops = np.asarray(gradient.stops, dtype=np.float64)
    cols = np.array([(c.r, c.g, c.b, c.a) for c in gradient.colors], dtype=np.float64)
    out = np.stack([np.interp(t, stops, cols[:, i]) for i in range(4)], axis=1)
    out[:, 3] *= alpha_scale
    return out

  def _project_lines(self, ys: np.ndarray, grid: np.ndarray, view, rect) -> np.ndarray:
    """(..., N) lateral offsets on the grid -> (..., N, 2) widget pixels (NaN offsets project as 0)."""
    ys = np.nan_to_num(ys)
    pts = np.stack((np.broadcast_to(grid, ys.shape), ys), axis=-1)
    return bem.car_to_widget(pts.reshape(-1, 2), rect, self.config, view).astype(np.float64).reshape(*ys.shape, 2)

  def _scene_layers(self, scene, rect, rect_key, view) -> tuple[Layer, Layer, Layer]:
    """(road, lines, path) layers; rebuilt only when the geometry, the path end or a setting changes."""
    from openpilot.starpilot.ui.road_colors import path_mode
    end = self._path_end(scene) if scene.path is not None else self.config.x_max
    style = self.road_style
    mode = path_mode(style)
    arrays = (scene.grid, *scene.lanes, *scene.edges, scene.lane_probs, scene.edge_stds, scene.path, scene.path_accel)
    build = lambda: self._build_scene(scene, rect, view, end)  # noqa: E731
    if scene.engaged and mode == 'rainbow' and scene.path is not None:  # animated: rebuilt every frame
      return build()
    key = (rect_key, self.opacity, style.get('laneLines'), tuple(map(id, arrays)), end, self._throttle_blend,
           scene.engaged, scene.experimental, mode, style.get('path'))
    return self._cached('scene', key, build, keep=arrays)

  def _road_pair(self, scene) -> list:
    if len(scene.edges) == 2:
      return scene.edges
    return [scene.lanes[0], scene.lanes[3]] if len(scene.lanes) == 4 else []

  def _line_styles(self, scene) -> list[tuple]:
    """(offsets, width m, min width px, rgb, alpha) of each road edge then lane line worth drawing."""
    from openpilot.starpilot.ui.road_colors import lane_color
    lines = []
    for i, edge in enumerate(scene.edges):
      std = scene.edge_stds[i] if i < len(scene.edge_stds) else 1.0
      alpha = float(np.clip(1.0 - std, 0.0, 1.0))
      if alpha >= 0.05:
        lines.append((edge, EDGE_WIDTH, 2.5, (150, 158, 176), 230 * alpha * self.opacity))
    for i, lane in enumerate(scene.lanes):
      prob = float(scene.lane_probs[i]) if i < len(scene.lane_probs) else 0.0
      alpha = float(np.clip(prob, 0.0, 0.7)) / 0.7
      if alpha < 0.05:
        continue
      default = _color((236, 240, 248), 235 * alpha)
      color = lane_color(self.road_style, False, default, prob)
      lines.append((lane, LANE_WIDTH, 1.8, (color.r, color.g, color.b),
                    (color.a if color is default else min(255, color.a / 0.7)) * self.opacity))
    return lines

  def _path_offsets(self, scene, end) -> tuple[np.ndarray, np.ndarray] | None:
    """Path offsets on the full grid, NaN outside the drawn run, and its half widths in pixels need u."""
    grid = scene.grid
    ys = scene.path.copy()
    ys[(grid > end) | (grid < EGO_FRONT - 0.5)] = np.nan
    runs = bem.valid_runs(np.isfinite(ys))
    if not runs or runs[0][1] - runs[0][0] < 2:
      return None
    lo, hi = runs[0]
    ys[:lo] = np.nan
    ys[hi:] = np.nan
    u = (grid - grid[lo]) / max(1e-3, self.config.x_max - grid[lo])
    return ys, u

  def _build_scene(self, scene, rect, view, end) -> tuple[Layer, Layer, Layer]:
    """Road, edges, lane lines and path as one batch: one projection, one ribbon pass, one quad build."""
    grid = scene.grid
    n = grid.size
    pair = self._road_pair(scene)
    lines = self._line_styles(scene)
    path = self._path_offsets(scene, end) if scene.path is not None else None
    offsets = [*pair, *(line[0] for line in lines)] + ([path[0]] if path is not None else [])
    if not offsets:
      return EMPTY, EMPTY, EMPTY
    stack = np.stack(offsets)
    centre = self._project_lines(stack, grid, view, rect)
    valid = np.isfinite(stack)
    t = self._height_fraction(centre[..., 1], rect)
    road = 1 if pair else 0
    lefts, rights, rows, oks = [], [], [], []
    if pair:
      lefts.append(centre[:1])
      rights.append(centre[1:2])
      rows.append(self._fade((40, 46, 60), 215 * self.opacity, t[0], hold=0.55, end=0.97)[None])
      oks.append((valid[0] & valid[1])[None])
    ribbon = centre[2 * road:]
    if ribbon.shape[0]:
      scale = bem.scale_at(grid, view)
      half = np.empty((ribbon.shape[0], n))
      if lines:
        width_m, min_px = np.array([(line[1], line[2]) for line in lines]).T
        half[:len(lines)] = np.maximum(width_m[:, None] * scale[None, :], min_px[:, None]) / 2
        rows.append(self._fade(np.array([line[3] for line in lines], dtype=np.float64), np.array([line[4] for line in lines]),
                               t[2 * road:2 * road + len(lines)]))
      if path is not None:
        half[-1] = np.maximum(PATH_WIDTH * (1.0 - 0.35 * path[1]) * scale, 3.0) / 2
        accel = scene.path_accel if scene.path_accel is not None else np.empty(0)
        rows.append(self._path_rows(scene, t[-1], accel)[None])
      left, right = bem.ribbon_sides(ribbon, half, valid[2 * road:])
      lefts.append(left)
      rights.append(right)
      oks.append(valid[2 * road:])
    layer, counts = _ribbon_quads(np.concatenate(lefts), np.concatenate(rights), np.concatenate(rows), np.concatenate(oks),
                                  rect.y + rect.height)
    # Split the one call list back into road | edges and lanes | path (6 calls per quad).
    cut_road = 6 * int(counts[:road].sum())
    cut_path = len(layer) - (6 * int(counts[-1]) if path is not None else 0)
    return layer[:cut_road], layer[cut_road:cut_path], layer[cut_path:]

  def _font_size(self, rect) -> float:
    return max(15.0, min(26.0, rect.height * 0.026))

  def _ticks(self, rect, view) -> tuple[Layer, list]:
    """Faint distance rungs every 10 m (50 ft), labeled at 20/40/60 m (100/200 ft)."""
    size = self._font_size(rect)
    step, labeled, unit, per_unit = (10.0, (20, 40, 60), 'm', 1.0) if self.metric else (50.0, (100, 200), 'ft', 0.3048)
    count = int(self.config.x_max / (step * per_unit))
    values = np.arange(1, count + 1) * step
    distances = values * per_unit
    ys = bem.car_to_widget(np.stack((distances, np.zeros_like(distances)), axis=1), rect, self.config, view)[:, 1].astype(np.float64)
    shown = int(np.argmax(ys < rect.y + size * 1.5)) if (ys < rect.y + size * 1.5).any() else len(ys)
    values, distances, ys = values[:shown], distances[:shown], ys[:shown]
    half = np.minimum(rect.width / 2 - 14, 11.0 * bem.scale_at(distances, view))
    fade = np.interp(self._height_fraction(ys, rect), *TICK_FADE)
    alpha = (24 * fade * self.opacity).astype(np.int32)
    verts = np.stack((view.cx - half, ys - 0.5, view.cx - half, ys + 0.5, view.cx + half, ys + 0.5, view.cx + half, ys - 0.5), axis=1)
    labels = []
    color = rl.Color(170, 180, 198, int(160 * self.opacity))
    shadow = rl.Color(0, 0, 0, int(color.a * 0.7))
    for value, y in zip(values.tolist(), ys.tolist(), strict=True):
      if int(value) in labeled:
        text = f"{int(value)} {unit}"
        _, h = self.text.measure(text, size)
        labels.append((text, size, rect.x + 14, y - h / 2, shadow, color))
    return _layer([[255, 255, 255, a] * 2 for a in alpha.tolist()], verts), labels

  def _path_profile(self, scene):
    """(distances, offsets, slopes) of the path's finite points, or None: what lateral_at/heading_at derive per call."""
    def build():
      ok = np.isfinite(scene.path)
      if np.count_nonzero(ok) < 2:
        return None
      gx, gy = scene.grid[ok], scene.path[ok]
      return gx, gy, np.gradient(gy, gx)
    return self._cached('path_profile', (id(scene.path), id(scene.grid)), build, keep=(scene.path, scene.grid))

  def _heading_at(self, scene, x: float) -> float:
    """bem.heading_at on the cached path profile."""
    profile = self._path_profile(scene) if scene.path is not None else None
    return 0.0 if profile is None else math.atan(float(np.interp(x, profile[0], profile[2])))

  def _path_end(self, scene) -> float:
    end = self.config.x_max
    profile = self._path_profile(scene)
    for lead in scene.leads:
      x = lead.d_rel + EGO_FRONT
      # bem.lateral_at on the cached profile
      lateral = lead.y if profile is None or x < profile[0][0] or x > profile[0][-1] else float(np.interp(x, profile[0], profile[1]))
      if abs(lead.y - lateral) < 1.6:
        end = min(end, x - 1.0)
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
      rows = self._fade((0, 0, 0), 215 * a, t, hold=0.35, end=0.9)
      rows[:, :3] = bem.hls_to_rgb(hue, light, sat) * 255
      return rows
    blend = self._throttle_blend
    rgb = np.array([n + (th - n) * blend for n, th in zip(NO_THROTTLE[0], THROTTLE[0], strict=True)])
    return self._fade(rgb, 210 * a, t, hold=0.35, end=0.9)

  def _cars(self, view, rect, poses, local, layout, hull_at: int, colors, outlines) -> tuple[list[Layer], np.ndarray]:
    """Car glyphs in one batch: poses (C, 3) of (x, y, heading), shared car-frame points local (P, 2) filled per
    layout with colors (C, polygons, 4), then outlines (C, 4). Returns one layer per car and the hulls (C, 8, 2)."""
    poses = np.asarray(poses, dtype=np.float64).reshape(-1, 3)
    cos_h, sin_h = np.cos(poses[:, 2]), np.sin(poses[:, 2])
    pts = np.empty((poses.shape[0], local.shape[0], 2))
    pts[..., 0] = poses[:, None, 0] + local[None, :, 0] * cos_h[:, None] - local[None, :, 1] * sin_h[:, None]
    pts[..., 1] = poses[:, None, 1] + local[None, :, 0] * sin_h[:, None] + local[None, :, 1] * cos_h[:, None]
    screen = bem.car_to_widget(pts.reshape(-1, 2), rect, self.config, view).astype(np.float64).reshape(pts.shape)
    hulls = screen[:, hull_at:hull_at + 8]
    index, owner = layout
    rgba = np.asarray(colors, dtype=np.int32)[:, owner]
    cols = np.concatenate((rgba, rgba), axis=-1)
    verts = screen[:, index].reshape(poses.shape[0], -1, 8)
    # Outline: one 1.5 px quad per hull edge, the shape draw_line_ex draws.
    b = np.roll(hulls, -1, axis=1)
    d = b - hulls
    length = np.hypot(d[..., 0], d[..., 1])
    length[length == 0] = 1.0
    nrm = d[..., ::-1] * (np.array([-1.0, 1.0]) * (0.75 / length)[..., None])
    line_cols = np.broadcast_to(np.tile(np.asarray(outlines, dtype=np.int32), 2)[:, None, :], (poses.shape[0], 8, 8))
    cols = np.concatenate((cols, line_cols), axis=1)
    verts = np.concatenate((verts, np.concatenate((hulls + nrm, hulls - nrm, b - nrm, b + nrm), axis=-1)), axis=1)
    calls = _layer(cols, verts)
    per_car = 6 * cols.shape[1]
    return [calls[i * per_car:(i + 1) * per_car] for i in range(poses.shape[0])], hulls

  def _leads(self, scene, rect, view) -> None:
    size = self._font_size(rect)
    leads = [lead for lead in sorted(scene.leads, key=lambda lead: -lead.d_rel)
             if lead.d_rel + EGO_FRONT <= self.config.x_max]
    if not leads:
      return
    poses, colors, outlines, rgbs = [], [], [], []
    for lead in leads:
      cx = lead.d_rel + EGO_FRONT + LEAD_LENGTH / 2
      poses.append((cx, lead.y, self._heading_at(scene, cx)))
      rgb = bem.lead_color(lead.d_rel, lead.v_rel if math.isfinite(lead.v_rel) else 0.0)
      a = (1.0 if lead.radar else 0.7) * self.opacity
      glass = (64, 72, 88, int(170 * a))
      colors.append((_rgba(rgb, 245 * a), glass, (*glass[:3], int(glass[3] * 0.7))))
      outlines.append(_rgba((255, 255, 255), 60 * a))
      rgbs.append(rgb)
    layers, hulls = self._cars(view, rect, poses, _car_body(LEAD_LENGTH, LEAD_WIDTH), CAR_LAYOUT, 0, colors, outlines)
    # Far to near, each car then its label, so a nearer car covers a farther label.
    for lead, rgb, layer, screen in zip(leads, rgbs, layers, hulls, strict=True):
      _emit(layer)
      text = bem.distance_label(lead.d_rel, self.metric)
      w, h = self.text.measure(text, size)
      x = float(screen[:, 0].max()) + 10
      if x + w > rect.x + rect.width - 8:
        x = float(screen[:, 0].min()) - 10 - w
      color = _color(rgb, 255 * self.opacity)
      y = float(screen[:, 1].mean()) - h / 2
      self.text.draw(text, size, x + 1.5, y + 1.5, rl.Color(0, 0, 0, int(color.a * 0.7)))
      self.text.draw(text, size, x, y, color)

  def _ego_glyph(self, rect, view, wheel_angle: float) -> tuple[Layer, tuple]:
    a = self.opacity
    cx = EGO_FRONT - EGO_LENGTH / 2
    centre = bem.car_to_widget(np.array([[cx, 0.0]]), rect, self.config, view)[0]
    glow = (rl.Vector2(float(centre[0]), float(centre[1])), EGO_WIDTH * float(bem.scale_at(cx, view)) * 1.3,
            rl.Color(120, 170, 255, int(55 * a)), rl.Color(120, 170, 255, 0))
    length, width = EGO_LENGTH, EGO_WIDTH
    front = bem.rotate_offsets(WHEEL, 0, 0, wheel_angle)
    wheels = [(front if fx > 0 else WHEEL) + np.array([fx, side * (width / 2 - 0.02)])
              for fx, side in ((length / 2 - 0.9, -1), (length / 2 - 0.9, 1), (-length / 2 + 0.85, -1), (-length / 2 + 0.85, 1))]
    body, glass = (238, 241, 247, int(255 * a)), (48, 58, 76, int(255 * a))
    colors = [[(16, 18, 24, body[3])] * 4 + [body, glass, (*glass[:3], int(glass[3] * 0.7))]]
    layers, _ = self._cars(view, rect, [(cx, 0.0, 0.0)], np.concatenate((*wheels, _car_body(length, width))), EGO_LAYOUT, 16,
                           colors, [(255, 255, 255, int(110 * a))])
    return layers[0], glow

  def _ego(self, scene, rect, rect_key, view) -> None:
    # Rounded to WHEEL_STEP so small steering jitter replays the cached glyph (0.25 deg is ~0.1 px on a wheel).
    wheel_angle = round(bem.ego_wheel_angle(scene.steering_deg) / WHEEL_STEP) * WHEEL_STEP
    layer, glow = self._cached('ego', (rect_key, self.opacity, wheel_angle), lambda: self._ego_glyph(rect, view, wheel_angle))
    rl.draw_circle_gradient(*glow)
    _emit(layer, self._blinkers(scene, rect, rect_key, view))

  def _blinkers(self, scene, rect, rect_key, view) -> Layer:
    if not (scene.left_blinker or scene.right_blinker) or (self.clock() % 0.8) > 0.5:
      return EMPTY
    return self._cached('blinkers', (rect_key, self.opacity, scene.left_blinker, scene.right_blinker),
                        lambda: self._blinker_arrows(scene, rect, view))

  def _blinker_arrows(self, scene, rect, view) -> Layer:
    cx = EGO_FRONT - EGO_LENGTH / 2 + 0.6
    arrows = []
    for on, side in ((scene.left_blinker, -1), (scene.right_blinker, 1)):
      if on:
        base = side * (EGO_WIDTH / 2 + 0.7)
        arrows.append((cx + 0.8, base, cx, base + side * 1.1, cx - 0.8, base))
    screen = bem.car_to_widget(np.array(arrows).reshape(-1, 2), rect, self.config, view).astype(np.float64).reshape(-1, 3, 2)
    color = _rgba((64, 220, 120), 235 * self.opacity)
    return [call for tri in screen for call in _polygon_quads(tri, TRIANGLE_LAYOUT, [color])]


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
