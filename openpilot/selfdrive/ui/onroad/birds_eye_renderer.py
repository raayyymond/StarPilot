"""Tesla-style birds-eye (top-down chase view) of the model's road, plan and leads.

Pure 2D: everything is mapped through birds_eye_math.car_to_widget, no camera matrix.
Geometry is resampled onto one forward-distance grid so every line has the same sample
count, which keeps the ribbons simple and lets frames be blended without jitter.

Drawing cost: a frame issues O(ribbons) C calls, not O(vertices). Each road edge, lane line and the road fill is
one single-coloured DrawTriangleStrip per valid run plus a few constant-alpha strips for its soft end (each line
ends at its own distance, bem.line_rows); the far-end fade is a few background-coloured gradient rectangles laid
over them (bem.overlay_bands). The path is drawn after those
overlays as a handful of constant-alpha strips that step through its own fade; only the per-row colour modes
(experimental accel tint, rainbow, colour gradient) still replay per-vertex quads, for the path alone. Car glyphs
are one DrawTriangleFan per polygon plus one outline strip.
"""
from collections.abc import Callable
from dataclasses import dataclass, field
from functools import lru_cache
from itertools import chain, repeat, starmap
import math
import operator
import time
from typing import NamedTuple

import numpy as np
import pyray as rl
from raylib import rl as _raw  # the C functions pyray wraps; per-vertex calls skip its argument conversion

from openpilot.selfdrive.ui.onroad import birds_eye_math as bem
from openpilot.system.ui.lib.shader_polygon import Gradient
from openpilot.system.ui.widgets import Widget

_ffi = rl.ffi

EGO_FRONT = 1.5      # meters from the camera to the ego front bumper
EGO_LENGTH = 4.6     # car outlines are true to scale: 4.6 x 1.9 m, the same px/m as the lane lines
EGO_WIDTH = 1.9
LEAD_LENGTH = 4.6
LEAD_WIDTH = 1.9
PATH_WIDTH = 1.7
LANE_WIDTH = 0.14
EDGE_WIDTH = 0.28
MODEL_LEAD_PROB = 0.5
SMOOTHING = 0.55     # blend toward each new model frame
LEAD_SMOOTHING = 0.5
# Lines narrower than this many pixels are widened to it: what the zoomed-out (highway) view shows. Zoomed in,
# the metre widths are wider anyway.
LANE_MIN_PX = 3.0
EGO_LANE_MIN_PX = 3.5
EDGE_MIN_PX = 3.5
# model_renderer.MIN/MAX_DRAW_DISTANCE (not imported: that module needs Params and the UI state at import).
MIN_DRAW_DISTANCE = 10.0
MAX_DRAW_DISTANCE = 100.0
LABEL_PAD = 6.0
LABEL_BG = (10, 12, 18)
LABEL_BG_ALPHA = 0.85
LABEL_SMOOTHING = 0.5   # per frame, toward the glyph-attached label position
LABEL_SNAP_PX = 80.0    # a jump larger than this (side flip, a new lead) is taken directly
SUBLABEL_SCALE = 0.62   # curve tag, relative to the distance label
SHOW_CURVE_MARK = False  # the curve-speed bar and tag on the path (scene.curve is still ingested)

LANE_HEADING_PROB = 0.3  # an ego lane line needs this probability to steer the lead glyph's heading
LAT_ACTIVE_LANE = (52, 216, 104)  # ego lane lines while lateral control is active
LANE_CENTERING_LANE = (0, 176, 220)  # = lane_feedback.BLUE, while the lane-centering correction applies

THROTTLE = ((13, 248, 122), (114, 255, 92))
NO_THROTTLE = ((242, 242, 242), (242, 242, 242))

BACKGROUND_TOP, BACKGROUND_BOTTOM = (26, 31, 44), (8, 9, 13)  # the full-frame gradient (birds_eye_hud copies it)
PANEL_BG = (10, 12, 18)
PANEL_ALPHA = 235
FADE_HOLD, FADE_END = 0.6, 0.95       # the lane lines' far fade (height fractions), now drawn as an overlay
LINE_ALPHA_STEP = 0.1                 # each line's soft end: strips within +/-5 % of a linear fade
PATH_ALPHA_STEP = 0.04                # the path's stepped fade: each strip within +/-2 % of the smooth alpha
LEAD_LATERAL_GATE = 1.6               # a lead within this of the path (m) ends it 1 m short of its rear bumper
LEAD_PATH_GAP = 1.0

GHOST_PRE = (190, 198, 212)           # lane-change ghost: preLaneChange
GHOST_GO = LAT_ACTIVE_LANE            # laneChangeStarting / laneChangeFinishing
GHOST_BLOCKED = (240, 84, 72)         # the blind spot on that side is set
CURVE_CANDIDATE = (138, 156, 184)     # curve tag: a candidate only
CURVE_CEILING = (222, 228, 240)       # a ceiling the planner is not using
CURVE_CONTROLLING = (246, 184, 64)    # = bem.CAUTION_LEAD: the curve ceiling is controlling
STOP_RED = (240, 64, 56)
HARD_BRAKE_HZ = 2.0


@dataclass
class BirdsEyeLead:
  d_rel: float
  y: float            # right-positive, meters
  v_rel: float
  radar: bool = True


@dataclass(frozen=True)
class CurveMark:
  """A fresh curve-speed observation worth showing: the speed (ceiling, else candidate) and where it binds."""
  speed_mps: float
  distance: float
  controlling: bool = False
  ceiling: bool = False


@dataclass
class BirdsEyeScene:
  grid: np.ndarray = field(default_factory=lambda: bem.sample_grid())
  path: np.ndarray | None = None          # lateral offset per grid point, NaN where absent
  path_accel: np.ndarray | None = None
  path_curvature: np.ndarray | None = None  # signed 1/m per grid point (right-positive), NaN where absent
  lanes: list[np.ndarray] = field(default_factory=list)
  lane_probs: np.ndarray = field(default_factory=lambda: np.zeros(4))
  lane_stds: np.ndarray = field(default_factory=lambda: np.zeros(4))
  edges: list[np.ndarray] = field(default_factory=list)
  edge_stds: np.ndarray = field(default_factory=lambda: np.ones(2))
  # Per-point lateral std (XYZTData.yStd) of each lane line / road edge on the grid, None where not published.
  lane_ystds: list = field(default_factory=list)
  edge_ystds: list = field(default_factory=list)
  leads: list[BirdsEyeLead] = field(default_factory=list)
  model_leads: list = field(default_factory=list)  # bem.model_lead_tracks: (x, y) at t = 0, 2 .. 10 s
  steering_deg: float = 0.0
  left_blinker: bool = False
  right_blinker: bool = False
  left_blindspot: bool = False
  right_blindspot: bool = False
  lane_change_state: int = 0      # bem.lane_change_code: 0 off, 1 pre, 2 starting, 3 finishing
  lane_change_direction: int = 0  # -1 left, +1 right
  hard_brake: bool = False        # modelV2.meta.hardBrakePredicted
  should_stop: bool = False       # modelV2.action.shouldStop
  curve: CurveMark | None = None
  speed_limit_mps: float | None = None
  engaged: bool = False
  experimental: bool = False
  allow_throttle: bool = True
  lat_active: bool = False        # carControl.latActive
  # lane_feedback.direction(): -1 the correction pulls toward the left line (laneLines[1]), +1 toward the right
  # line (laneLines[2]), 0 none. Only that line turns blue, as in the camera view.
  lane_centering_direction: int = 0
  v_ego: float | None = None      # carState.vEgo: drives the speed-based zoom
  draw_distance: float = MAX_DRAW_DISTANCE  # camera-view visibility: modelV2.position.x[-1] clipped
  lead_one_d: float | None = None  # radarState.leadOne.dRel while present: where the camera view cuts the path
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


def _line_ystd(line, grid) -> np.ndarray | None:
  """XYZTData.yStd resampled on the grid like the line itself; None when not published (this fork's modeld fills
  laneLines/roadEdges without stds: fill_model_msg.py fill_xyzt(..) for them passes no y_std)."""
  try:
    std = _floats(line.yStd)
    xs = _floats(line.x)
  except (AttributeError, TypeError, ValueError):
    return None
  n = min(std.size, xs.size)
  if n < 2 or not (np.nan_to_num(std[:n]) > 0).any():
    return None
  return bem.resample_line(xs[:n], std[:n], grid)


@dataclass(frozen=True)
class ModelExtras:
  lane_stds: np.ndarray
  lane_ystds: list
  edge_ystds: list
  curvature: np.ndarray | None
  tracks: list
  lane_change_state: int
  lane_change_direction: int
  hard_brake: bool
  should_stop: bool


def model_extras(model, grid) -> ModelExtras:
  """Everything else the birds-eye view shows from a modelV2: lane line stds, path curvature, model lead tracks,
  lane-change intent, hardBrakePredicted and action.shouldStop. Missing fields read as absent."""
  try:
    stds = _floats(model.laneLineStds)[:4]
  except (AttributeError, TypeError, ValueError):
    stds = np.zeros(0)
  stds = np.pad(stds, (0, 4 - stds.size))
  profile = bem.path_curvature(model)
  curvature = bem.resample_line(*profile, grid, extend_back=False) if profile is not None else None
  meta = getattr(model, 'meta', None)
  action = getattr(model, 'action', None)
  lane_ystds = [_line_ystd(line, grid) for line in list(model.laneLines)[:4]]
  edge_ystds = [_line_ystd(edge, grid) for edge in list(model.roadEdges)[:2]]
  return ModelExtras(stds, lane_ystds, edge_ystds, curvature, bem.model_lead_tracks(getattr(model, 'leadsV3', []), MODEL_LEAD_PROB),
                     bem.lane_change_code(getattr(meta, 'laneChangeState', 0)),
                     bem.lane_change_side(getattr(meta, 'laneChangeDirection', 0)),
                     bool(getattr(meta, 'hardBrakePredicted', False)), bool(getattr(action, 'shouldStop', False)))


def camera_draw_distance(model) -> float:
  """How far the camera view draws lane lines, edges and path: modelV2.position.x[-1], clipped (model_renderer)."""
  xs = list(model.position.x)
  last = float(xs[-1]) if xs else math.nan
  return float(np.clip(last, MIN_DRAW_DISTANCE, MAX_DRAW_DISTANCE)) if math.isfinite(last) else MAX_DRAW_DISTANCE


def camera_path_end(lead_d_rel: float | None, draw_distance: float) -> float:
  """Path end with a radar leadOne present, the camera view's rule (model_renderer._update_model)."""
  if lead_d_rel is None:
    return draw_distance
  lead_d = lead_d_rel * 2.0
  return float(np.clip(lead_d - min(lead_d * 0.35, 10.0), 0.0, draw_distance))


def scene_leads(radar_state, model, tracks=None) -> list[BirdsEyeLead]:
  """Radar leads first, then each confident model lead (leadsV3 prob >= MODEL_LEAD_PROB) where no drawn lead is
  within 6 m / 2 m of it (the three entries are the lead at three probability horizons, usually one car).

  A radar (radard-fused) lead's lateral position is -yRel and nothing else: it is drawn where radard puts it,
  unclamped, so a lead changing lanes moves across the lane lines."""
  if tracks is None:
    tracks = bem.model_lead_tracks(getattr(model, 'leadsV3', []), MODEL_LEAD_PROB) if model is not None else []
  leads: list[BirdsEyeLead] = []
  if radar_state is not None:
    for lead in (radar_state.leadOne, radar_state.leadTwo):
      if lead.present and math.isfinite(lead.dRel) and lead.dRel > 0:
        # radarState yRel is left-positive.
        leads.append(BirdsEyeLead(float(lead.dRel), -float(lead.yRel), float(lead.vRel), radar=True))
  for xs, ys in tracks:
    d, y = float(xs[0]) - EGO_FRONT, float(ys[0])
    if d > 0 and not any(abs(o.d_rel - d) < 6.0 and abs(o.y - y) < 2.0 for o in leads):
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


# ---- C-call lists -------------------------------------------------------------------------------
# An op list is a ready-made list of raw raylib calls, (function, *args), replayed by _run. Strip and fan ops hold
# their float32 points through ffi.from_buffer (which keeps the array alive) and a prebuilt Color struct, so a
# replay converts nothing.

_call = getattr(operator, 'call', lambda function, *args: function(*args))


def _ccolor(rgba) -> object:
  r, g, b, a = rgba
  return _ccolor_cached(int(r), int(g), int(b), int(max(0, min(255, round(a)))))


@lru_cache(maxsize=1024)
def _ccolor_cached(r: int, g: int, b: int, a: int) -> object:
  """A Color struct, shared: ops only read it."""
  return _ffi.new('Color *', (r, g, b, a))[0]


def _points(pts: np.ndarray):
  arr = np.ascontiguousarray(pts, dtype=np.float32)
  return _ffi.from_buffer('Vector2[]', arr), arr.shape[0]


def _strip(pts: np.ndarray, rgba) -> tuple:
  """One DrawTriangleStrip call: interleaved [L0, R0, L1, R1, ..] points in one colour."""
  ptr, n = _points(pts)
  return _raw.DrawTriangleStrip, ptr, n, _ccolor(rgba)


def _fan(pts: np.ndarray, rgba) -> tuple:
  """One DrawTriangleFan call: a convex polygon in one colour."""
  ptr, n = _points(pts)
  return _raw.DrawTriangleFan, ptr, n, _ccolor(rgba)


def _strips(points: np.ndarray, runs, colors) -> list:
  """DrawTriangleStrip ops over row runs [(lo, hi)] of one ribbon's interleaved points (bem.strip_points), each in
  its own colour. All share one buffer: each op's pointer is an offset into it, and the buffer itself rides along
  as a trailing no-op entry so the pointers stay valid as long as the op list lives."""
  if not runs:
    return []
  base = _ffi.from_buffer('Vector2[]', np.ascontiguousarray(points, dtype=np.float32))
  ops: list = [(_raw.DrawTriangleStrip, base + 2 * lo, 2 * (hi - lo), _ccolor(rgba)) for (lo, hi), rgba in zip(runs, colors, strict=True)]
  ops.append((_keep, base))
  return ops


def _keep(*_args) -> None:
  """The no-op an op list ends with to hold its shared point buffer."""


def _run(ops) -> None:
  if ops:
    any(starmap(_call, ops))  # every call returns None, so any() just drains the iterator in C


# Per-vertex quads, kept for the path's per-row colour modes only. Each quad is color0, vertex0, vertex1, color1,
# vertex2, vertex3, so a ribbon row fades along its length.
Layer = list
EMPTY: Layer = []


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
  """All quads of the given layers in one rlBegin/rlEnd, through the raw C functions (no pyray wrapping)."""
  if not any(layers):
    return
  # Bind the 1x1 white texture the way raylib's own shapes do: a quad batch opened right after text would
  # otherwise continue the font's draw call and sample its atlas.
  _raw.rlSetTexture(_raw.rlGetTextureIdDefault())
  _raw.rlBegin(rl.RL_QUADS)
  for layer in layers:
    any(starmap(_call, layer))
  _raw.rlEnd()
  _raw.rlSetTexture(0)


def _ribbon_quads(left: np.ndarray, right: np.ndarray, rows: np.ndarray, valid: np.ndarray, bottom: float) -> Layer:
  """Quads between consecutive valid rows of (N, 2) ribbon sides with one RGBA per row.

  Rows fully transparent at both ends and quads wholly below the rect bottom are dropped."""
  rgba = np.clip(rows, 0, 255).astype(np.int32)
  verts = np.concatenate((left[:-1], right[:-1], right[1:], left[1:]), axis=-1)
  keep = valid[:-1] & valid[1:] & ((rgba[:-1, 3] > 0) | (rgba[1:, 3] > 0))
  keep &= verts[:, 1::2].min(axis=-1) <= bottom
  cols = np.concatenate((rgba[:-1], rgba[1:]), axis=-1)
  return _layer(cols[keep], verts[keep])


@lru_cache(maxsize=8)
def _car_body(length: float, width: float) -> np.ndarray:
  """Hull 8, roof 4 and windshield 4 points of a car glyph centered at the origin, car-frame meters."""
  scale = np.array([length / 4.6, width / 1.9])
  return np.concatenate((bem.car_outline(0.0, 0.0, 0.0, length, width, chamfer=0.38 * width / 1.9), ROOF * scale, SHIELD * scale))


WHEEL = np.array([(0.36, -0.12), (-0.36, -0.12), (-0.36, 0.12), (0.36, 0.12)])
ROOF = np.array([(0.45, -0.64), (-1.05, -0.66), (-1.05, 0.66), (0.45, 0.64)])
SHIELD = np.array([(1.2, -0.62), (0.6, -0.7), (0.6, 0.7), (1.2, 0.62)])
# Car glyph polygon sizes, in point order: [4 wheels x 4,] hull 8, roof 4, windshield 4. One fan each.
CAR_LAYOUT = (8, 4, 4)
EGO_LAYOUT = (4, 4, 4, 4, 8, 4, 4)
WHEEL_STEP = math.radians(0.25)  # ego front-wheel angle resolution
OUTLINE_PX = 1.5


class RoadDraw(NamedTuple):
  """Cached ops for one road geometry: road fill, edges + lane lines, and the fade overlays drawn over them."""
  road: list
  lines: list
  overlay: list


class PathDraw(NamedTuple):
  """Cached path ops: constant-alpha strips (or per-vertex quads in a per-row colour mode), the curvature edge,
  and where the path ends (car frame x, y, heading) for the stop bar."""
  strips: list
  quads: Layer
  curve_edge: list
  end: tuple[float, float, float] | None


NO_ROAD = RoadDraw([], [], [])
NO_PATH = PathDraw([], EMPTY, [], None)
_UNSET = object()


class BirdsEyeRenderer(Widget):
  """Top-down chase view. Feed it with poll(sm, started_frame) or ingest(...), then render(rect).

  panel=True draws a rounded translucent card (overlay); panel=False fills the rect edge to edge.

  Geometry that only changes with the rect, the settings, the lateral state or a new model frame (road, lane
  lines, path, the ego glyph) is built once and replayed from cache. Scene arrays are treated as immutable:
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
    self.clock_ns = time.monotonic_ns  # the logMonoTime clock lane-centering freshness is judged against
    self.scene = BirdsEyeScene(grid=bem.sample_grid(bem.zoomed_config(self.config, bem.far_x_max(self.config))))
    self._throttle_blend = 1.0
    self._last_model_frame = -1
    self._rainbow = None
    self._lead_model = None
    self._cache: dict[str, tuple] = {}
    self._zoom = bem.ZoomFilter(self.config)
    self._zoom_clock: float | None = None
    self._labels: list[tuple[float, float]] = []  # last drawn label positions, for the jitter filter
    self._limit_frame: tuple[tuple, float | None] | None = None

  # ---- data -------------------------------------------------------------------------------------

  def ingest(self, model=None, radar_state=None, car_state=None, *, engaged=None, experimental=None,
             allow_throttle=None, lat_active=None, lane_centering_direction=None, curve=_UNSET, speed_limit=_UNSET) -> None:
    """Accept one observation. model=None keeps the last good road geometry.

    lane_centering_direction is lane_feedback.direction(): -1, 0 or +1 (the side whose line turns blue).
    curve (CurveMark or None) and speed_limit (m/s or None) replace the last value when given."""
    scene = self.scene
    tracks = None
    if model is not None:
      path, accel, lanes, probs, edges, stds = model_lines(model, scene.grid)
      scene.path = bem.smooth(scene.path, path, SMOOTHING) if scene.valid else path
      scene.path_accel = accel
      scene.lanes = [bem.smooth(old, new, SMOOTHING) for old, new in zip(scene.lanes, lanes, strict=False)] \
        if scene.valid and len(scene.lanes) == len(lanes) else lanes
      scene.edges = [bem.smooth(old, new, SMOOTHING) for old, new in zip(scene.edges, edges, strict=False)] \
        if scene.valid and len(scene.edges) == len(edges) else edges
      scene.lane_probs, scene.edge_stds = probs, stds
      scene.draw_distance = camera_draw_distance(model)
      scene.valid = bool(np.isfinite(path).any() or any(np.isfinite(lane).any() for lane in lanes))
      extras = model_extras(model, scene.grid)
      if not np.array_equal(extras.lane_stds, scene.lane_stds):
        scene.lane_stds = extras.lane_stds
      scene.path_curvature = extras.curvature
      scene.lane_ystds, scene.edge_ystds = extras.lane_ystds, extras.edge_ystds
      scene.model_leads = tracks = extras.tracks
      scene.lane_change_state, scene.lane_change_direction = extras.lane_change_state, extras.lane_change_direction
      scene.hard_brake, scene.should_stop = extras.hard_brake, extras.should_stop
      self._lead_model = model
    if radar_state is not None:
      one = radar_state.leadOne
      scene.lead_one_d = float(one.dRel) if one.present and math.isfinite(one.dRel) else None
    if radar_state is not None or model is not None:
      scene.leads = self._smooth_leads(scene.leads, scene_leads(radar_state, self._lead_model,
                                                                scene.model_leads if tracks is None else tracks))
    if car_state is not None:
      scene.steering_deg = float(car_state.steeringAngleDeg)
      scene.left_blinker, scene.right_blinker = bool(car_state.leftBlinker), bool(car_state.rightBlinker)
      scene.left_blindspot = bool(getattr(car_state, 'leftBlindspot', False))
      scene.right_blindspot = bool(getattr(car_state, 'rightBlindspot', False))
      v_ego = getattr(car_state, 'vEgo', None)
      scene.v_ego = float(v_ego) if v_ego is not None and math.isfinite(v_ego) else scene.v_ego
    if engaged is not None:
      scene.engaged = bool(engaged)
    if experimental is not None:
      scene.experimental = bool(experimental)
    if allow_throttle is not None:
      scene.allow_throttle = bool(allow_throttle)
    if lat_active is not None:
      scene.lat_active = bool(lat_active)
    if lane_centering_direction is not None:
      scene.lane_centering_direction = int(np.sign(lane_centering_direction))
    if curve is not _UNSET:
      scene.curve = curve
    if speed_limit is not _UNSET:
      scene.speed_limit_mps = float(speed_limit) if speed_limit is not None and math.isfinite(speed_limit) and speed_limit > 0 else None

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
    lat_active = bool(fresh('carControl') and sm['carControl'].latActive)
    # Blue only while the fork's own freshness and consistency rules say the correction applies, on its side.
    lane_dir = self._lane_centering_direction(sm, started_frame) if lat_active else 0
    now_ns = self.clock_ns()
    # Without a new modelV2 the road keeps its last good frame; leads, wheel and blinkers still follow.
    self.ingest(model, radar, car, engaged=engaged, experimental=experimental, allow_throttle=allow,
                lat_active=lat_active, lane_centering_direction=lane_dir,
                curve=read_curve(sm, now_ns, started_frame), speed_limit=self._read_speed_limit(sm, now_ns, started_frame))

  def _lane_centering_direction(self, sm, started_frame: int) -> int:
    try:
      from openpilot.starpilot.lateral.lane_feedback import direction
    except ImportError:
      return 0
    return direction(sm, self.clock_ns(), started_frame)

  def _read_speed_limit(self, sm, now_ns: int, started_frame: int) -> float | None:
    """The valid SLC speed limit (m/s) through the UI's own qualified readers; decoded once per slcState frame."""
    try:
      from openpilot.starpilot.ui.runtime_snapshot import display_message
      from openpilot.starpilot.ui.onroad_state import ObservationKind, speed_limit_from_message
    except ImportError:
      return None
    message = display_message(sm, 'slcState', now_ns, after_frame=started_frame)
    if message is None:
      return None
    frame = (int(sm.recv_frame['slcState']), id(message))
    if self._limit_frame is None or self._limit_frame[0] != frame:
      observation = speed_limit_from_message(message)
      value = observation.speed_limit_mps if observation.kind == ObservationKind.VALID else None
      self._limit_frame = (frame, float(value) if value is not None and math.isfinite(value) and value > 0 else None)
    return self._limit_frame[1]

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

  def view_config(self) -> bem.BirdsEyeConfig:
    """self.config zoomed for the current speed, widened to keep a radar leadOne on screen (filtered and quantized: see
    bem.ZoomFilter)."""
    if self._zoom.cfg is not self.config:
      self._zoom = bem.ZoomFilter(self.config)
    now = self.clock()
    dt = 0.0 if self._zoom_clock is None else now - self._zoom_clock
    self._zoom_clock = now
    lead = self.scene.lead_one_d
    lead_front = lead + EGO_FRONT + LEAD_LENGTH if lead is not None else None
    return bem.zoomed_config(self.config, self._zoom.update(self.scene.v_ego, dt, lead_front))

  def draw(self, rect: rl.Rectangle) -> None:
    from openpilot.starpilot.ui import clip
    cfg = self.view_config()
    rect_key = (float(rect.x), float(rect.y), float(rect.width), float(rect.height), cfg)
    view = self._cached('view', rect_key, lambda: bem.view_params(rect, cfg))
    self._background(rect)
    clip.begin_scissor_mode(rect.x, rect.y, rect.width, rect.height)
    rl.rl_disable_backface_culling()
    try:
      scene = self.scene
      if scene.valid:
        if scene.path is not None:
          target = 1.0 if scene.allow_throttle else 0.0
          self._throttle_blend += (target - self._throttle_blend) * 0.2
        road = self._road_draw(scene, rect, rect_key, view)
        _run(road.road)
        _run(road.lines)
        _run(road.overlay)
        path = self._path_draw(scene, rect, rect_key, view)
        _run(path.strips)
        _emit(path.quads)
        _run(path.curve_edge)
        self._marks(scene, rect, view, path)
      self._lane_change_ghost(scene, rect, rect_key, view)
      self._leads(scene, rect, view)
      self._ego(scene, rect, rect_key, view)
      self._speed_limit_sign(scene, rect)
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
      rl.draw_rectangle_rounded(rect, self._roundness(rect), 12, rl.Color(*PANEL_BG, int(PANEL_ALPHA * self.opacity)))
    else:
      rl.draw_rectangle_gradient_v(int(rect.x), int(rect.y), int(rect.width), int(rect.height),
                                   rl.Color(*BACKGROUND_TOP, 255), rl.Color(*BACKGROUND_BOTTOM, 255))

  def _background_at(self, y: float, rect, alpha: float) -> tuple[float, float, float, float]:
    """The background's colour at screen row y, with an overlay alpha 0..1."""
    if self.panel:
      return (*PANEL_BG, 255 * alpha * PANEL_ALPHA / 255 * self.opacity)
    t = (y - rect.y) / max(1.0, rect.height)
    return (*(a + (b - a) * t for a, b in zip(BACKGROUND_TOP, BACKGROUND_BOTTOM, strict=True)), 255 * alpha)

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

  # ---- road and lines ---------------------------------------------------------------------------

  def _road_draw(self, scene, rect, rect_key, view) -> RoadDraw:
    """Road, edges and lane lines; rebuilt only when the geometry, the draw distance or a setting changes."""
    reach = round(scene.draw_distance * 2) / 2  # 0.5 m steps steady the cache key
    style = self.road_style
    arrays = (scene.grid, *scene.lanes, *scene.edges, scene.lane_probs, scene.lane_stds, scene.edge_stds,
              *scene.lane_ystds, *scene.edge_ystds)
    key = (rect_key, self.opacity, self.panel, style.get('laneLines'), tuple(map(id, arrays)), reach,
           scene.lat_active, scene.lane_centering_direction)
    return self._cached('road', key, lambda: self._build_road(scene, rect, view, reach), keep=arrays)

  def _road_pair(self, scene) -> list:
    """(offsets, yStd) of the two lines bounding the road fill: the road edges, else the outer lane lines."""
    if len(scene.edges) == 2:
      return list(zip(scene.edges, self._ystds(scene.edge_ystds, 2), strict=True))
    lanes = self._ystds(scene.lane_ystds, 4)
    return [(scene.lanes[0], lanes[0]), (scene.lanes[3], lanes[3])] if len(scene.lanes) == 4 else []

  @staticmethod
  def _ystds(ystds, n) -> list:
    return [ystds[i] if i < len(ystds) else None for i in range(n)]

  @staticmethod
  def _lateral_rgb(scene, i: int):
    """Ego lane line i's color for the lateral state: blue on the side lane centering pulls toward (laneLines[1]
    for -1, laneLines[2] for +1, as in the camera view), green while lateral control is active, None (the normal
    lane color) otherwise or for the outer lines."""
    if not scene.lat_active or i not in (1, 2):
      return None
    direction = scene.lane_centering_direction
    if (i == 1 and direction < 0) or (i == 2 and direction > 0):
      return LANE_CENTERING_LANE
    return LAT_ACTIVE_LANE

  def _line_styles(self, scene) -> list[tuple]:
    """(offsets, width m, min width px, rgb, alpha, yStd) of each road edge then lane line worth drawing.

    A lane line's alpha follows its probability and, through bem.std_alpha, its laneLineStds entry."""
    from openpilot.starpilot.ui.road_colors import lane_color
    lines = []
    edge_ystds, lane_ystds = self._ystds(scene.edge_ystds, len(scene.edges)), self._ystds(scene.lane_ystds, len(scene.lanes))
    for i, edge in enumerate(scene.edges):
      std = scene.edge_stds[i] if i < len(scene.edge_stds) else 1.0
      alpha = float(np.clip(1.0 - std, 0.0, 1.0))
      if alpha >= 0.05:
        lines.append((edge, EDGE_WIDTH, EDGE_MIN_PX, (150, 158, 176), 230 * alpha * self.opacity, edge_ystds[i]))
    for i, lane in enumerate(scene.lanes):
      prob = float(scene.lane_probs[i]) if i < len(scene.lane_probs) else 0.0
      certainty = bem.std_alpha(float(scene.lane_stds[i])) if i < len(scene.lane_stds) else 1.0
      alpha = float(np.clip(prob, 0.0, 0.7)) / 0.7
      if alpha < 0.05:
        continue
      lateral = self._lateral_rgb(scene, i)
      if lateral is not None:  # the ego lane's own left and right lines
        lines.append((lane, LANE_WIDTH * 1.6, EGO_LANE_MIN_PX, lateral, 245 * alpha * certainty * self.opacity, lane_ystds[i]))
        continue
      default = _color((236, 240, 248), 235 * alpha)
      color = lane_color(self.road_style, False, default, prob)
      lines.append((lane, LANE_WIDTH, LANE_MIN_PX, (color.r, color.g, color.b),
                    (color.a if color is default else min(255, color.a / 0.7)) * certainty * self.opacity, lane_ystds[i]))
    return lines

  def _build_road(self, scene, rect, view, reach=MAX_DRAW_DISTANCE) -> RoadDraw:
    """The road fill, edges and lane lines, each ending at its own distance with a soft end.

    Each line stops at min(reach, where its own yStd passes bem.LINE_STD_LIMIT, its data end) and fades out over its
    last bem.end_fade_length metres as a few constant-alpha strips (LINE_ALPHA_STEP). The far end of the view fades
    through the overlay (the old per-row fade, one set of gradient rectangles)."""
    grid = scene.grid
    bottom = rect.y + rect.height
    xs_px = []
    road_ops: list = []
    pair = self._road_pair(scene)
    if pair:
      (a, a_std), (b, b_std) = pair
      end = min(reach, bem.std_end(grid, a_std), bem.std_end(grid, b_std))
      rows = bem.line_rows(grid, np.where(np.isfinite(b), a, np.nan), end, view.x_min, self._tail_step(view), view.x_max)
      if rows is not None:
        xs, ya, ok, end, fade = rows
        ok_b = np.isfinite(b)
        yb = np.interp(xs, grid[ok_b], b[ok_b]) if np.count_nonzero(ok_b) >= 2 else np.full(xs.shape, np.nan)
        left = bem.car_to_widget(np.stack((xs, ya), axis=1), rect, self.config, view).astype(np.float64)
        right = bem.car_to_widget(np.stack((xs, yb), axis=1), rect, self.config, view).astype(np.float64)
        road_ops = self._faded_strips(left, right, ok & np.isfinite(yb), bem.alpha_groups(bem.taper(xs, end, fade), LINE_ALPHA_STEP),
                                      (40, 46, 60), 215 * self.opacity, bottom)
        xs_px += [left[ok, 0], right[ok, 0]]
    # Lines with the same rows (the usual case: one model frame, one end) share one projection and ribbon pass.
    styles = self._line_styles(scene)
    step = self._tail_step(view)
    rows = [bem.line_rows(grid, line[0], min(reach, bem.std_end(grid, line[5])), view.x_min, step, view.x_max) for line in styles]
    batches: dict = {}
    for i, r in enumerate(rows):
      if r is not None:
        batches.setdefault((r[0].size, float(r[0][0]), r[3], r[4]), []).append(i)
    sides: dict = {}
    for members in batches.values():
      xs, _, _, end, fade = rows[members[0]]
      lateral = np.stack([rows[i][1] for i in members])
      ok = np.stack([rows[i][2] for i in members])
      pts = np.stack((np.broadcast_to(xs, lateral.shape), lateral), axis=-1).reshape(-1, 2)
      centre = bem.car_to_widget(pts, rect, self.config, view).astype(np.float64).reshape(*lateral.shape, 2)
      width_m, min_px = np.array([(styles[i][1], styles[i][2]) for i in members]).T
      half = np.maximum(width_m[:, None] * bem.scale_at(xs, view)[None, :], min_px[:, None]) / 2
      left, right = bem.ribbon_sides(centre, half, ok)
      groups = bem.alpha_groups(bem.taper(xs, end, fade), LINE_ALPHA_STEP)
      for k, i in enumerate(members):
        sides[i] = (left[k], right[k], ok[k], groups)
    line_ops: list = []
    for i, line in enumerate(styles):
      if i in sides:
        left, right, ok, groups = sides[i]
        line_ops += self._faded_strips(left, right, ok, groups, line[3], line[4], bottom)
        xs_px += [left[ok, 0], right[ok, 0]]
    xs_px = np.concatenate(xs_px) if xs_px else np.empty(0)
    overlay = self._overlay(rect, xs_px) if xs_px.size else []
    return RoadDraw(road_ops, line_ops, overlay)

  @staticmethod
  def _faded_strips(left, right, valid, groups, rgb, alpha, bottom) -> list:
    """Strip ops for one ribbon: rows in the constant-alpha groups of its soft end (bem.alpha_groups), each split
    at gaps; strips wholly below the rect bottom dropped."""
    low = np.minimum(left[:, 1], right[:, 1]) > bottom
    runs, colors = [], []
    for s, e, f in groups:
      if f * alpha < 1.0 or low[s:e].all():
        continue
      for lo, hi in ([(s, e)] if valid[s:e].all() else [(s + lo, s + hi) for lo, hi in bem.valid_runs(valid[s:e])]):
        if hi - lo >= 2:
          runs.append((lo, hi))
          colors.append((*rgb, f * alpha))
    return _strips(bem.strip_points(left, right), runs, colors)

  @staticmethod
  def _tail_step(view) -> float:
    """Row spacing (m) in a fading stretch: ~6 px on screen, so zoomed in a step of the fade stays a thin band."""
    return float(np.clip(6.0 / max(view.lateral_ppm, 1e-3), 0.25, 1.0))

  def _overlay(self, rect, xs: np.ndarray) -> list:
    """Gradient-rectangle ops that fade the road toward the background at the far end of the view (bem.overlay_bands),
    over the road's horizontal extent only."""
    x0 = int(max(rect.x, math.floor(float(xs.min())) - 2))
    x1 = int(min(rect.x + rect.width, math.ceil(float(xs.max())) + 2))
    ops = []
    if x1 <= x0:
      return ops
    for y0, y1, a0, a1 in bem.overlay_bands(rect.y, rect.height, None, None, FADE_HOLD, FADE_END):
      top, bot = int(round(y0)), int(round(y1))
      if bot > top:
        ops.append((_raw.DrawRectangleGradientV, x0, top, x1 - x0, bot - top,
                    _ccolor(self._background_at(top, rect, a0)), _ccolor(self._background_at(bot, rect, a1))))
    return ops

  # ---- path -------------------------------------------------------------------------------------

  def _path_end(self, scene, reach: float) -> float:
    """min(camera rule, reach, a lead's rear bumper - 1 m for each lead within LEAD_LATERAL_GATE of the path)."""
    end = min(camera_path_end(scene.lead_one_d, scene.draw_distance), reach)
    profile = self._path_profile(scene) if scene.leads else None
    for lead in scene.leads:
      x = lead.d_rel + EGO_FRONT
      lateral = lead.y if profile is None or x < profile[0][0] or x > profile[0][-1] else float(np.interp(x, profile[0], profile[1]))
      if abs(lead.y - lateral) < LEAD_LATERAL_GATE:
        end = min(end, x - LEAD_PATH_GAP)
    return end

  def _path_draw(self, scene, rect, rect_key, view) -> PathDraw:
    from openpilot.starpilot.ui.road_colors import path_mode
    if scene.path is None:
      return NO_PATH
    end = round(self._path_end(scene, scene.draw_distance) * 4) / 4  # 0.25 m steps: a lead's jitter replays the cache
    style = self.road_style
    mode = path_mode(style)
    build = lambda: self._build_path(scene, rect, view, end)  # noqa: E731
    if scene.engaged and mode == 'rainbow':  # animated: rebuilt every frame
      return build()
    arrays = (scene.grid, scene.path, scene.path_accel, scene.path_curvature)
    key = (rect_key, self.opacity, tuple(map(id, arrays)), end, self._throttle_blend, scene.engaged, scene.experimental,
           mode, style.get('path'))
    return self._cached('path', key, build, keep=arrays)

  def _per_vertex_path(self, scene) -> bool:
    """True in the path colour modes that need a colour per row: rainbow, colour gradient, experimental accel tint."""
    from openpilot.starpilot.ui.road_colors import path_mode
    if not scene.engaged:
      return False
    mode = path_mode(self.road_style)
    if mode in ('rainbow', 'color'):
      return True
    accel = scene.path_accel
    return bool(scene.experimental and accel is not None and np.isfinite(accel).any())

  def _path_rgb_alpha(self, scene) -> tuple[tuple, float, float, float]:
    """(rgb, alpha, hold, end) of a single-colour path: the old _path_rows fade parameters."""
    a = self.opacity
    if not scene.engaged:
      return (205, 210, 222), 150 * a, 0.3, 0.9
    blend = self._throttle_blend
    rgb = tuple(n + (th - n) * blend for n, th in zip(NO_THROTTLE[0], THROTTLE[0], strict=True))
    return rgb, 210 * a, 0.35, 0.9

  def _build_path(self, scene, rect, view, end: float) -> PathDraw:
    """The path from just ahead of the ego to `end`, ending softly over its last bem.end_fade_length metres.

    Single-colour modes step the fade through constant-alpha strips (resampled every ~6 px, so each strip is
    within PATH_ALPHA_STEP / 2 of the smooth per-row alpha); per-row colour modes keep per-vertex quads."""
    grid, ys_all = scene.grid, scene.path
    x0 = EGO_FRONT - 0.5
    runs = bem.valid_runs(np.isfinite(ys_all) & (grid >= x0) & (grid <= end))
    if not runs or runs[0][1] - runs[0][0] < 1:
      return NO_PATH
    lo, hi = runs[0]
    gx, gy = grid[lo:hi], ys_all[lo:hi]
    if hi < grid.size and math.isfinite(ys_all[hi]) and end > gx[-1] + 0.05:  # end between grid rows: cut exactly
      gx = np.append(gx, end)
      gy = np.append(gy, np.interp(end, grid[hi - 1:hi + 1], ys_all[hi - 1:hi + 1]))
    if gx.size < 2:
      return NO_PATH
    per_vertex = self._per_vertex_path(scene)
    # Single-colour: resampled every ~6 px (the centreline is piecewise linear between grid rows anyway).
    xs = gx if per_vertex else np.append(np.arange(gx[0], gx[-1] - 0.1, self._tail_step(view)), gx[-1])
    ys = np.interp(xs, gx, gy)
    centre = bem.car_to_widget(np.stack((xs, ys), axis=1), rect, self.config, view).astype(np.float64)
    u = (xs - xs[0]) / max(1e-3, view.x_max - xs[0])
    half = np.maximum(PATH_WIDTH * (1.0 - 0.35 * u) * bem.scale_at(xs, view), 3.0) / 2
    ok = np.ones(xs.size, dtype=bool)
    left, right = bem.ribbon_sides(centre, half, ok)
    t = self._height_fraction(centre[:, 1], rect)
    taper = bem.taper(xs, xs[-1], bem.end_fade_length(xs[0], xs[-1]))
    bottom = rect.y + rect.height
    strips: list = []
    quads: Layer = EMPTY
    if per_vertex:
      accel = scene.path_accel
      accel_rows = np.interp(xs, grid, np.nan_to_num(accel)) if accel is not None else np.empty(0)
      rows = self._path_rows(scene, t, accel_rows)
      rows[:, 3] *= taper
      quads = _ribbon_quads(left, right, rows, ok, bottom)
      factor = rows[:, 3] / 255
    else:
      rgb, alpha, hold, fade_end = self._path_rgb_alpha(scene)
      factor = np.interp(t, [0.0, hold, fade_end], [1.0, 0.8, 0.0]) * taper * alpha / 255
      low = np.minimum(left[:, 1], right[:, 1]) > bottom
      groups = [(s, e, a) for s, e, a in bem.alpha_groups(factor, PATH_ALPHA_STEP) if a * 255 >= 1.0 and not low[s:e].all()]
      strips = _strips(bem.strip_points(left, right), [(s, e) for s, e, _ in groups], [(*rgb, 255 * a) for _, _, a in groups])
    edge = self._curve_edge(scene, xs, left, right, factor)
    heading = math.atan2(ys[-1] - ys[-2], xs[-1] - xs[-2])
    return PathDraw(strips, quads, edge, (float(xs[-1]), float(ys[-1]), heading))

  @staticmethod
  def _curve_edge(scene, xs, left, right, factor) -> list:
    """A brighter band along the path's outer edge where |curvature| >= bem.CURVE_MARK (right turn: the left edge)."""
    curvature = scene.path_curvature
    if curvature is None or not (np.abs(np.nan_to_num(curvature)) >= bem.CURVE_MARK).any():
      return []
    ok = np.isfinite(curvature)
    kappa = np.interp(xs, scene.grid[ok], scene.path_curvature[ok], left=0.0, right=0.0)
    ops = []
    for sign in (1, -1):
      mark = sign * kappa >= bem.CURVE_MARK
      outer, other = (left, right) if sign > 0 else (right, left)
      inner = outer + (other - outer) * 0.22
      for lo, hi in bem.valid_runs(mark):
        lo, hi = max(0, lo - 1), min(xs.size, hi + 1)  # a single marked row still gets a segment
        a = float(np.mean(factor[lo:hi]))
        if hi - lo >= 2 and a * 255 >= 4:
          ops.append(_strip(bem.strip_points(outer[lo:hi], inner[lo:hi]), (255, 255, 255, 120 * a)))
    return ops

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

  def _lateral_at(self, scene, x: float) -> float:
    profile = self._path_profile(scene) if scene.path is not None else None
    if profile is None or x < profile[0][0] or x > profile[0][-1]:
      return 0.0
    return float(np.interp(x, profile[0], profile[1]))

  def _road_profiles(self, scene) -> list[list]:
    """Slope profiles of the ego lane's two lines (if confident), then of the road edges; cached per model frame."""
    def build():
      lanes = [scene.lanes[i] for i in (1, 2) if i < len(scene.lanes) and i < len(scene.lane_probs) and
               scene.lane_probs[i] >= LANE_HEADING_PROB]
      return [[p for p in (bem.slope_profile(scene.grid, ys) for ys in group) if p is not None] for group in (lanes, scene.edges)]
    arrays = (scene.grid, scene.lane_probs, *scene.lanes, *scene.edges)
    return self._cached('road_profiles', tuple(map(id, arrays)), build, keep=arrays)

  def _road_heading(self, scene, x: float) -> float:
    """Road direction at x: mean slope of the ego lane lines, else of the road edges, else the planned path.

    The path bends with steering intent; the lane lines are the road, so a lead glyph does not turn with the wheel."""
    for profiles in self._road_profiles(scene):
      heading = bem.mean_heading(profiles, x)
      if heading is not None:
        return heading
    return self._heading_at(scene, x)

  # ---- marks on the path ------------------------------------------------------------------------

  def _font_size(self, rect) -> float:
    """Label size: bem.label_font_size (the stream HUD's readout is a multiple of it)."""
    return bem.label_font_size(rect.height)

  def _bar(self, rect, view, x: float, y: float, heading: float, along: float, across: float, rgba) -> np.ndarray:
    """A bar across the path drawn as one fan; returns its screen corners."""
    pts = bem.car_to_widget(bem.segment_quad(x, y, heading, along, across), rect, self.config, view)
    _run([_fan(pts, rgba)])
    return pts

  def _marks(self, scene, rect, view, path: PathDraw) -> None:
    """action.shouldStop: a red bar at the path's end. Curve speed (only with SHOW_CURVE_MARK): a bar at the binding
    distance with its speed."""
    a = self.opacity
    if scene.should_stop and path.end is not None:
      x, y, heading = path.end
      self._bar(rect, view, x, y, heading, max(0.6, 4.0 / float(bem.depth_scale_at(x, view))), PATH_WIDTH * 1.5,
                _rgba(STOP_RED, 235 * a))
    curve = scene.curve
    if curve is None or not SHOW_CURVE_MARK:
      return
    x = float(np.clip(curve.distance, EGO_FRONT + 3.0, view.x_max - 3.0))
    y, heading = self._lateral_at(scene, x), self._heading_at(scene, x)
    rgb = CURVE_CONTROLLING if curve.controlling else CURVE_CEILING if curve.ceiling else CURVE_CANDIDATE
    pts = self._bar(rect, view, x, y, heading, max(0.45, 3.0 / float(bem.depth_scale_at(x, view))), PATH_WIDTH * 1.9,
                    _rgba(rgb, 225 * a))
    text = f"{bem.speed_text(curve.speed_mps, self.metric)} {'km/h' if self.metric else 'mph'}"
    size = self._font_size(rect) * SUBLABEL_SCALE
    w, h = self.text.measure(text, size)
    # Left of the bar: lead labels sit right of their cars, so the two rarely meet.
    tx = float(pts[:, 0].min()) - 8 - LABEL_PAD - w
    if tx - LABEL_PAD < rect.x + 4:
      tx = float(pts[:, 0].max()) + 8 + LABEL_PAD
    ty = float(pts[:, 1].mean()) - h / 2
    rl.draw_rectangle_rounded(rl.Rectangle(round(tx) - LABEL_PAD, round(ty) - LABEL_PAD / 2, w + 2 * LABEL_PAD, h + LABEL_PAD),
                              1.0, 8, _color(LABEL_BG, 255 * LABEL_BG_ALPHA * a))
    self.text.draw(text, size, round(tx), round(ty), _color(rgb, 255 * a))

  # ---- cars -------------------------------------------------------------------------------------

  def _cars(self, view, rect, poses, local, layout, hull_at: int, colors, outlines, outline_px: float = OUTLINE_PX
            ) -> tuple[list[list], np.ndarray]:
    """Car glyphs: poses (C, 3) of (x, y, heading), shared car-frame points local (P, 2) split into convex polygons
    of the sizes in layout, filled with colors (C, polygons, 4) as one fan each, then a hull outline strip in
    outlines (C, 4). Returns one op list per car and the hulls (C, 8, 2) in pixels; transparent parts are skipped.

    Drawn in car-frame meters through the view mapping, so with the default isotropic view they are true to scale."""
    poses = np.asarray(poses, dtype=np.float64).reshape(-1, 3)
    cos_h, sin_h = np.cos(poses[:, 2]), np.sin(poses[:, 2])
    pts = np.empty((poses.shape[0], local.shape[0], 2))
    pts[..., 0] = poses[:, None, 0] + local[None, :, 0] * cos_h[:, None] - local[None, :, 1] * sin_h[:, None]
    pts[..., 1] = poses[:, None, 1] + local[None, :, 0] * sin_h[:, None] + local[None, :, 1] * cos_h[:, None]
    screen = bem.car_to_widget(pts.reshape(-1, 2), rect, self.config, view).astype(np.float64).reshape(pts.shape)
    hulls = screen[:, hull_at:hull_at + 8]
    cars = []
    for c in range(poses.shape[0]):
      ops, offset = [], 0
      for k, n in enumerate(layout):
        if colors[c][k][3] > 0:
          ops.append(_fan(screen[c, offset:offset + n], colors[c][k]))
        offset += n
      if outlines[c][3] > 0:
        ops.append(_strip(bem.ring_strip(hulls[c], outline_px), outlines[c]))
      cars.append(ops)
    return cars, hulls

  def _lane_change_ghost(self, scene, rect, rect_key, view) -> None:
    """While the model's lane change is pending or under way, an outline of the ego car one lane over on that side:
    dim before the change starts, green once it does, red when that side's blind spot is set."""
    state, side = scene.lane_change_state, scene.lane_change_direction
    if state == 0 or side == 0:
      return
    width = round(bem.lane_width(scene.lanes, scene.grid, EGO_FRONT, side), 1)
    blocked = scene.left_blindspot if side < 0 else scene.right_blindspot
    rgb, alpha = (GHOST_BLOCKED, 0.6) if blocked else (GHOST_GO, 0.5) if state >= 2 else (GHOST_PRE, 0.35)

    def build():
      cx = EGO_FRONT - EGO_LENGTH / 2
      cars, _ = self._cars(view, rect, [(cx, side * width, 0.0)], _car_body(EGO_LENGTH, EGO_WIDTH), CAR_LAYOUT, 0,
                           [[(0, 0, 0, 0), (*rgb, int(40 * alpha * self.opacity)), (0, 0, 0, 0)]],
                           [_rgba(rgb, 255 * alpha * self.opacity)], outline_px=2.5)
      return cars[0]
    _run(self._cached('ghost', (rect_key, self.opacity, side, width, rgb, alpha), build))

  def _leads(self, scene, rect, view) -> None:
    size = self._font_size(rect)
    # A lead whose rear bumper is beyond the zoomed view is dropped, label and all.
    leads = [lead for lead in sorted(scene.leads, key=lambda lead: -lead.d_rel)
             if lead.d_rel + EGO_FRONT <= view.x_max]
    if not leads:
      self._labels = []
      return
    poses, colors, outlines, rgbs = [], [], [], []
    for lead in leads:
      # The outline's rear bumper is at dRel ahead of the ego front bumper: exactly the gap the label prints.
      cx = lead.d_rel + EGO_FRONT + LEAD_LENGTH / 2
      poses.append((cx, lead.y, self._road_heading(scene, cx)))
      rgb = bem.lead_color(lead.d_rel, lead.v_rel if math.isfinite(lead.v_rel) else 0.0)
      a = (1.0 if lead.radar else 0.7) * self.opacity
      glass = (64, 72, 88, int(170 * a))
      colors.append((_rgba(rgb, 245 * a), glass, (*glass[:3], int(glass[3] * 0.7))))
      outlines.append(_rgba((255, 255, 255), 60 * a))
      rgbs.append(rgb)
    cars, hulls = self._cars(view, rect, poses, _car_body(LEAD_LENGTH, LEAD_WIDTH), CAR_LAYOUT, 0, colors, outlines)
    nearest = leads[-1] if scene.hard_brake else None
    # Far to near, each car then its label, so a nearer car covers a farther label.
    previous, self._labels = self._labels, []
    for lead, pose, rgb, ops, screen in zip(leads, poses, rgbs, cars, hulls, strict=True):
      self._predictions(scene, lead, pose, rect, view)
      _run(ops)
      self._lead_label(lead, rgb, screen, rect, size, previous, lead is nearest)

  def _predictions(self, scene, lead, pose, rect, view) -> None:
    """Where the matching model lead (leadsV3) puts this lead at t = 2 s and 4 s: two small fading dots."""
    if not scene.model_leads:
      return
    moves = bem.lead_prediction(lead.d_rel, lead.y, scene.model_leads, EGO_FRONT)
    if moves is None:
      return
    pts = bem.car_to_widget(np.array([pose[0], pose[1]]) + moves, rect, self.config, view)
    radius = max(3.0, 0.4 * float(bem.scale_at(pose[0], view)))
    for (px, py), alpha in zip(pts, (120, 70), strict=True):
      rl.draw_circle_v(rl.Vector2(float(px), float(py)), radius, _color((214, 219, 228), alpha * self.opacity))

  def _lead_label(self, lead, rgb, screen, rect, size, previous, hard_brake: bool) -> None:
    text = bem.distance_label(lead.d_rel, self.metric)
    box_w, box_h = self.text.measure(text, size)
    pad = LABEL_PAD
    x = float(screen[:, 0].max()) + 10 + pad
    if x + box_w + pad > rect.x + rect.width - 8:
      x = float(screen[:, 0].min()) - 10 - pad - box_w
    y = min(max(float(screen[:, 1].mean()) - box_h / 2, rect.y + 4 + pad), rect.y + rect.height - box_h - 4 - pad)
    x, y = self._label_position(previous, x, y)
    # An opaque pill instead of a drop shadow: the stream encoder smears thin text over a changing background,
    # and the pill keeps the digits on one steady colour so a changed number leaves no residue around it.
    pill = rl.Rectangle(x - pad, y - pad, box_w + 2 * pad, box_h + 2 * pad)
    roundness = 1.0
    rl.draw_rectangle_rounded(pill, roundness, 8, _color(LABEL_BG, 255 * LABEL_BG_ALPHA * self.opacity))
    if hard_brake:  # modelV2.meta.hardBrakePredicted: the nearest lead's pill border pulses red
      pulse = 0.5 + 0.5 * math.sin(2 * math.pi * HARD_BRAKE_HZ * self.clock())
      rl.draw_rectangle_rounded_lines_ex(pill, roundness, 8, 3.0, _color(STOP_RED, (110 + 145 * pulse) * self.opacity))
    self.text.draw(text, size, x, y, _color(rgb, 255 * self.opacity))

  def _label_position(self, previous: list[tuple[float, float]], x: float, y: float) -> tuple[float, float]:
    """Follow the glyph with a light filter, then snap to whole pixels, so a label neither jitters nor slides
    by sub-pixel amounts (each would redraw every glyph edge on the stream)."""
    near = min(previous, key=lambda p: math.hypot(p[0] - x, p[1] - y), default=None)
    if near is not None and math.hypot(near[0] - x, near[1] - y) < LABEL_SNAP_PX:
      x, y = near[0] + (x - near[0]) * LABEL_SMOOTHING, near[1] + (y - near[1]) * LABEL_SMOOTHING
    self._labels.append((x, y))
    return float(round(x)), float(round(y))

  def _ego_glyph(self, rect, view, wheel_angle: float) -> tuple[list, tuple]:
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
    cars, _ = self._cars(view, rect, [(cx, 0.0, 0.0)], np.concatenate((*wheels, _car_body(length, width))), EGO_LAYOUT, 16,
                         colors, [(255, 255, 255, int(110 * a))])
    return cars[0], glow

  def _ego(self, scene, rect, rect_key, view) -> None:
    # Rounded to WHEEL_STEP so small steering jitter replays the cached glyph (0.25 deg is ~0.1 px on a wheel).
    wheel_angle = round(bem.ego_wheel_angle(scene.steering_deg) / WHEEL_STEP) * WHEEL_STEP
    ops, glow = self._cached('ego', (rect_key, self.opacity, wheel_angle), lambda: self._ego_glyph(rect, view, wheel_angle))
    rl.draw_circle_gradient(*glow)
    # No blinker arrows here: the HUD draws a large corner arrow (scene.left/right_blinker are kept for it).
    _run(ops)

  # ---- speed limit ------------------------------------------------------------------------------

  def _sign_top(self, rect) -> float:
    """Below the HUD's top-right turn-signal arrow when full-frame; a small margin in the overlay panel."""
    if self.panel:
      return rect.y + 12
    try:
      from openpilot.starpilot.system.starport import birds_eye_hud as hud
      reserve = hud.SIGNAL_MARGIN + hud.SIGNAL_HEIGHT
      frame_rows = 1080 - (hud.TORQUE_MAX + 2 * hud.TORQUE_MARGIN)  # the view rect stops above the torque dock
    except ImportError:
      reserve, frame_rows = 288, 940
    return rect.y + reserve * rect.height / frame_rows

  def _speed_limit_sign(self, scene, rect) -> None:
    """A compact speed-limit sign at the top right: a US white rectangle in mph, a red-ringed disc in km/h."""
    if scene.speed_limit_mps is None:
      return
    a = self.opacity
    value = bem.speed_text(scene.speed_limit_mps, self.metric)
    height = max(56.0, rect.height * 0.12)
    top = self._sign_top(rect)
    right = rect.x + rect.width - 16
    black = _color((16, 18, 24), 255 * a)
    if self.metric:
      r = height / 2
      centre = rl.Vector2(right - r, top + r)
      rl.draw_circle_v(centre, r, _color((255, 255, 255), 250 * a))
      rl.draw_ring(centre, r * 0.8, r, 0, 360, 36, _color((214, 40, 40), 255 * a))
      size = r * (0.95 if len(value) <= 2 else 0.75)
      w, h = self.text.measure(value, size)
      self.text.draw(value, size, round(centre.x - w / 2), round(centre.y - h / 2), black)
      return
    width = height * 0.8
    sign = rl.Rectangle(right - width, top, width, height)
    rl.draw_rectangle_rounded(sign, 0.18, 8, _color((255, 255, 255), 250 * a))
    rl.draw_rectangle_rounded_lines_ex(rl.Rectangle(sign.x + 3, sign.y + 3, width - 6, height - 6), 0.16, 8, 2.0, black)
    small, big = height * 0.17, height * 0.46
    y = top + height * 0.1
    for word in ('SPEED', 'LIMIT'):
      w, h = self.text.measure(word, small)
      self.text.draw(word, small, round(sign.x + (width - w) / 2), round(y), black)
      y += h * 0.95
    w, h = self.text.measure(value, big)
    self.text.draw(value, big, round(sign.x + (width - w) / 2), round(top + height * 0.93 - h), black)


def read_curve(sm, now_ns: int, started_frame: int) -> CurveMark | None:
  """A fresh slcState.curve with a candidate or a ceiling, through the UI's qualified reader (stale data rejected)."""
  try:
    from openpilot.starpilot.ui.runtime_snapshot import current_curve_message, curve_observation
  except ImportError:
    return None
  message = current_curve_message(sm, now_ns, after_frame=started_frame)
  observation = curve_observation(message, now_ns) if message is not None else None
  if observation is None:
    return None
  speed = observation.ceiling_mps if observation.ceiling_mps is not None else observation.candidate_mps
  if speed is None:
    return None
  return CurveMark(float(speed), float(observation.binding_distance), bool(observation.controlling),
                   observation.ceiling_mps is not None)


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
