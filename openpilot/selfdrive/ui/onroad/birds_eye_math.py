"""Pure geometry for the birds-eye (top-down chase view) renderer. No GL, no messaging.

Car frame follows modelV2 (device frame): x forward, y right, z down, meters. radarState
yRel is left-positive and must be negated before it is mapped here.

The view is a ground-plane homography: with u = (x - x_min) / (x_max - x_min) and
d = 1 + k*u, a point lands at

  px = cx + y * lateral_ppm / d
  py = bottom - (1 + k) * u / d * depth_px

k = 0 is a plain orthographic top-down view. With isotropic=True (the default) it is a true-to-scale map:
one pixels-per-meter forward and sideways at every distance, so lane widths, car outlines and gaps all
keep their real proportions and a 30 m gap is twice a 15 m one. k > 0 is a mild chase-cam: straight
lines stay straight, the ego region keeps its scale and distant geometry narrows by 1 / (1 + k).
"""
from dataclasses import dataclass, replace
import math

import numpy as np


@dataclass(frozen=True)
class BirdsEyeConfig:
  """View extent and scale.

  Defaults (chosen for the 1280x720 Android Auto stream, also fine for the overlay panel): a true-to-scale,
  linear top-down map. perspective 0 + isotropic: one pixels-per-meter in both directions, the forward fit
  (x_max - x_min over the rect height) sets it unless lateral_range would not fit the width.
  - x_min -5 / x_max 68: 73 m over ~677 px = ~9.3 px/m at 720 p. The ego outline (front bumper at 1.5 m,
    4.6 m long) fits above the bottom and a lead at 60 m (rear bumper 61.5 m, front 66.1 m) fits below the top.
  - Trade-off: true scale makes the road small. A 3.7 m lane is ~34 px and a 4.6 x 1.9 m car
    ~43 x 18 px at 720 p (half a lane wide);
    a shorter x_max would enlarge it but cut leads off before 60 m. A 1280 px wide frame shows ~69 m either side,
    so the neighbouring lanes always fit.
  - The old default (perspective 0.6) drew a 30 m radar gap as ~17.5 m of ego-car lengths at 1280x720, so the
    printed distance looked 50-100 % too large for the picture.
  """
  x_min: float = -5.0          # meters behind the camera shown below the ego origin
  x_max: float = 68.0          # meters ahead at the top edge
  lateral_range: float = 12.0  # meters either side that must fit the width (it lowers the scale if it does not)
  perspective: float = 0.0     # 0 = orthographic (linear distance), 1 = isotropic at the ego (strong chase cam)
  isotropic: bool = True       # with perspective 0: one scale forward and sideways (true to scale)
  max_ppm_fraction: float = 0.045  # non-isotropic only: lateral pixels-per-meter cap, as a fraction of the rect height
  top_pad: float = 0.03        # fractions of the rect height kept clear at the top and bottom
  bottom_pad: float = 0.03
  sample_step: float = 1.5     # meters between resampled line points near the car


DEFAULT_CONFIG = BirdsEyeConfig()

# Speed-based zoom. The view shows roughly the next 2.3-3 s of travel: 30 m ahead at city speed (<= 10 m/s,
# ~20 px/m at 720 p, a 3.7 m lane ~75 px) widening linearly to the default 68 m at highway speed (>= 30 m/s,
# ~9.3 px/m). 20 m/s (72 km/h) shows 49 m. With a radar leadOne the view tightens to the lead's front bumper plus
# ZOOM_LEAD_BUFFER (never below ZOOM_NEAR_X_MAX), so the gap fills the screen instead of a thin strip.
ZOOM_NEAR_X_MAX = 30.0
ZOOM_SLOW_SPEED = 10.0
ZOOM_FAST_SPEED = 30.0
ZOOM_TAU = 1.0    # s, first-order filter time constant
ZOOM_STEP = 0.5   # m, x_max quantum: the view (and every cached layer) only changes when the zoom moves a step
ZOOM_LEAD_BUFFER = 6.0  # m shown beyond a lead's front bumper


def zoom_x_max(v_ego: float, cfg: BirdsEyeConfig = DEFAULT_CONFIG) -> float:
  """Unfiltered top-edge distance for an ego speed: ZOOM_NEAR_X_MAX up to ZOOM_SLOW_SPEED, cfg.x_max from ZOOM_FAST_SPEED."""
  if not math.isfinite(v_ego):
    return cfg.x_max
  near = min(ZOOM_NEAR_X_MAX, cfg.x_max)
  t = (v_ego - ZOOM_SLOW_SPEED) / (ZOOM_FAST_SPEED - ZOOM_SLOW_SPEED)
  return near + (cfg.x_max - near) * min(1.0, max(0.0, t))


def lead_zoom_x_max(v_ego: float, lead_front: float | None, cfg: BirdsEyeConfig = DEFAULT_CONFIG) -> float:
  """zoom_x_max, tightened to lead_front + ZOOM_LEAD_BUFFER (never below ZOOM_NEAR_X_MAX) when a lead is given.

  lead_front: the lead's front bumper ahead of the camera (m), dRel + ego front + lead length; None without a lead."""
  preset = zoom_x_max(v_ego, cfg)
  if lead_front is None or not math.isfinite(lead_front):
    return preset
  return max(min(ZOOM_NEAR_X_MAX, cfg.x_max), min(preset, lead_front + ZOOM_LEAD_BUFFER))


def zoomed_config(cfg: BirdsEyeConfig, x_max: float) -> BirdsEyeConfig:
  """cfg showing x_max ahead: same x_min (the ego stays put), lateral_range scaled with the span (same aspect)."""
  if x_max == cfg.x_max:
    return cfg
  ratio = (x_max - cfg.x_min) / max(1e-3, cfg.x_max - cfg.x_min)
  return replace(cfg, x_max=x_max, lateral_range=cfg.lateral_range * ratio)


class ZoomFilter:
  """First-order filtered, quantized x_max for the speed-based (and lead-tightened) zoom.

  The quantized value only moves once the filter is more than 3/4 of a step away from it, so speed noise near a
  step boundary does not toggle the view (and rebuild the cached layers) back and forth."""
  def __init__(self, cfg: BirdsEyeConfig = DEFAULT_CONFIG, tau: float = ZOOM_TAU, step: float = ZOOM_STEP):
    self.cfg, self.tau, self.step = cfg, tau, step
    self.value: float | None = None
    self.x_max = cfg.x_max

  def update(self, v_ego: float | None, dt: float, lead_front: float | None = None) -> float:
    if v_ego is None or not math.isfinite(v_ego):
      return self.x_max
    target = lead_zoom_x_max(v_ego, lead_front, self.cfg)
    if self.value is None:
      self.value = target
    else:
      alpha = 1.0 - math.exp(-max(0.0, min(dt, 0.5)) / self.tau)
      self.value += (target - self.value) * alpha
    if abs(self.value - self.x_max) >= 0.75 * self.step:
      self.x_max = min(self.cfg.x_max, round(self.value / self.step) * self.step)
    return self.x_max


@dataclass(frozen=True)
class ViewParams:
  cx: float
  bottom: float
  depth_px: float
  lateral_ppm: float
  k: float
  x_min: float
  x_max: float


def view_params(rect, cfg: BirdsEyeConfig = DEFAULT_CONFIG) -> ViewParams:
  """Metres-per-pixel and perspective strength derived from the widget rect."""
  width, height = float(rect.width), float(rect.height)
  depth_px = max(1.0, height * (1.0 - cfg.top_pad - cfg.bottom_pad))
  span = max(1e-3, cfg.x_max - cfg.x_min)
  if cfg.isotropic and cfg.perspective <= 0:
    ppm = max(1e-3, min(depth_px / span, width / (2.0 * cfg.lateral_range)))
    return ViewParams(cx=float(rect.x) + width / 2.0, bottom=float(rect.y) + height * (1.0 - cfg.bottom_pad),
                      depth_px=ppm * span, lateral_ppm=ppm, k=0.0, x_min=cfg.x_min, x_max=cfg.x_max)
  lateral_ppm = max(1e-3, min(width / (2.0 * cfg.lateral_range), height * cfg.max_ppm_fraction))
  depth_ppm = depth_px / span
  # k at which the ego region is isotropic (equal lateral and forward pixels per meter).
  k_iso = min(max(lateral_ppm / depth_ppm - 1.0, 0.0), 4.0)
  k = max(0.0, cfg.perspective) * k_iso
  return ViewParams(cx=float(rect.x) + width / 2.0, bottom=float(rect.y) + height * (1.0 - cfg.bottom_pad),
                    depth_px=depth_px, lateral_ppm=lateral_ppm, k=k, x_min=cfg.x_min, x_max=cfg.x_max)


def _u_and_d(x, view: ViewParams):
  u = (np.asarray(x, dtype=np.float64) - view.x_min) / (view.x_max - view.x_min)
  if view.k > 0:
    u = np.maximum(u, -0.9 / view.k)  # keep the homography's denominator positive far behind the view
  return u, 1.0 + view.k * u


def car_to_widget(points, rect, cfg: BirdsEyeConfig = DEFAULT_CONFIG, view: ViewParams | None = None) -> np.ndarray:
  """Map (N, 2+) car-frame points (x fwd, y right, meters) to (N, 2) widget pixels."""
  view = view_params(rect, cfg) if view is None else view
  pts = np.asarray(points, dtype=np.float64).reshape(-1, np.shape(points)[-1] if np.ndim(points) > 1 else 2)
  u, d = _u_and_d(pts[:, 0], view)
  out = np.empty((pts.shape[0], 2), dtype=np.float32)
  out[:, 0] = view.cx + pts[:, 1] * view.lateral_ppm / d
  out[:, 1] = view.bottom - (1.0 + view.k) * u / d * view.depth_px
  return out


def scale_at(x, view: ViewParams) -> np.ndarray:
  """Lateral pixels per meter at forward distance x."""
  _, d = _u_and_d(x, view)
  return view.lateral_ppm / d


def depth_scale_at(x, view: ViewParams) -> np.ndarray:
  """Forward pixels per meter at forward distance x (derivative of py)."""
  _, d = _u_and_d(x, view)
  return (1.0 + view.k) * view.depth_px / (view.x_max - view.x_min) / (d * d)


def slope_profile(grid: np.ndarray, ys: np.ndarray) -> tuple[np.ndarray, np.ndarray] | None:
  """(distances, dy/dx) of a resampled line's finite points, or None with fewer than two."""
  ok = np.isfinite(ys)
  if np.count_nonzero(ok) < 2:
    return None
  gx, gy = grid[ok], ys[ok]
  return gx, np.gradient(gy, gx)


def mean_heading(profiles, x: float) -> float | None:
  """atan of the mean slope at x over the slope_profiles that cover x; None when none does."""
  slopes = [float(np.interp(x, gx, dy)) for gx, dy in profiles if gx[0] <= x <= gx[-1]]
  return math.atan(sum(slopes) / len(slopes)) if slopes else None


def widget_y_for_distance(x: float, rect, cfg: BirdsEyeConfig = DEFAULT_CONFIG) -> float:
  return float(car_to_widget(np.array([[x, 0.0]]), rect, cfg)[0, 1])


def sample_grid(cfg: BirdsEyeConfig = DEFAULT_CONFIG) -> np.ndarray:
  """Forward distances to sample: sample_step near the car, coarser far away where perspective shrinks it."""
  # Start a little behind the view so lanes run off the bottom edge instead of stopping short of it.
  near = np.arange(cfg.x_min - 4.0, min(15.0, cfg.x_max), cfg.sample_step)
  mid = np.arange(near[-1] + cfg.sample_step if near.size else cfg.x_min, min(40.0, cfg.x_max), cfg.sample_step * 2)
  start = mid[-1] + cfg.sample_step * 2 if mid.size else (near[-1] + cfg.sample_step if near.size else cfg.x_min)
  far = np.arange(start, cfg.x_max, cfg.sample_step * 3)
  return np.concatenate((near, mid, far, [cfg.x_max]))


def resample_line(xs, ys, grid: np.ndarray, *, extend_back: bool = True) -> np.ndarray:
  """Lateral offset of a model line at each grid distance; NaN beyond the line's data.

  Behind its first point the line keeps its first lateral offset (lanes continue under the car)."""
  xs = np.asarray(xs, dtype=np.float64)
  ys = np.asarray(ys, dtype=np.float64)
  n = min(xs.size, ys.size)
  out = np.full(grid.shape, np.nan)
  if n < 2:
    return out
  xs, ys = xs[:n], ys[:n]
  finite = np.isfinite(xs) & np.isfinite(ys)
  xs, ys = xs[finite], ys[finite]
  if xs.size < 2:
    return out
  xs = np.maximum.accumulate(xs)  # tolerate a non-monotonic tail (stopped car); interp needs increasing xp
  keep = np.concatenate(([True], np.diff(xs) > 1e-4))
  xs, ys = xs[keep], ys[keep]
  if xs.size < 2:
    return out
  inside = (grid >= xs[0]) & (grid <= xs[-1])
  out[inside] = np.interp(grid[inside], xs, ys)
  if extend_back:
    out[grid < xs[0]] = ys[0]
  return out


def ribbon(centre: np.ndarray, half_widths) -> np.ndarray:
  """Screen-space ribbon polygon [L0..Ln-1, Rn-1..R0] around an (N, 2) pixel centreline."""
  centre = np.asarray(centre, dtype=np.float64)
  if centre.shape[0] < 2:
    return np.empty((0, 2), dtype=np.float32)
  tangent = np.gradient(centre, axis=0)
  norm = np.linalg.norm(tangent, axis=1, keepdims=True)
  norm[norm < 1e-6] = 1.0
  tangent /= norm
  # For a centreline running up the screen this normal points screen-left, so the first chain is the
  # left one, which is the winding draw_polygon's triangle strip needs to survive back-face culling.
  normal = np.stack((tangent[:, 1], -tangent[:, 0]), axis=1)
  half = np.broadcast_to(np.asarray(half_widths, dtype=np.float64), (centre.shape[0],))[:, None]
  left, right = centre + normal * half, centre - normal * half
  return np.vstack((left, right[::-1])).astype(np.float32)


def valid_runs(mask: np.ndarray) -> list[tuple[int, int]]:
  """[start, end) index runs where mask is true."""
  edges = np.flatnonzero(np.diff(np.concatenate(([0], np.asarray(mask, dtype=np.int8).ravel(), [0]))))
  return list(zip(edges[::2].tolist(), edges[1::2].tolist(), strict=True))


def ribbon_sides(centre: np.ndarray, half_widths, valid: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
  """Left and right edges of (..., N, 2) pixel centrelines, one ribbon per leading index.

  Batched form of ribbon(): each row's tangent matches np.gradient over the valid run the row belongs
  to (central inside a run, one-sided at its ends), so lines split by gaps need no per-run loop."""
  c = np.asarray(centre, dtype=np.float64)
  ok = np.asarray(valid, dtype=bool)
  fwd = np.zeros_like(c)
  fwd[..., :-1, :] = c[..., 1:, :] - c[..., :-1, :]
  has_fwd = np.zeros_like(ok)
  has_fwd[..., :-1] = ok[..., 1:] & ok[..., :-1]
  bwd = np.zeros_like(c)
  bwd[..., 1:, :] = fwd[..., :-1, :]
  has_bwd = np.zeros_like(ok)
  has_bwd[..., 1:] = has_fwd[..., :-1]
  tangent = np.where((has_fwd & has_bwd)[..., None], (fwd + bwd) / 2, np.where(has_fwd[..., None], fwd, bwd))
  norm = np.sqrt(tangent[..., 0] ** 2 + tangent[..., 1] ** 2)
  norm[norm < 1e-6] = 1.0
  half = np.broadcast_to(np.asarray(half_widths, dtype=np.float64), ok.shape) / norm
  normal = np.stack((tangent[..., 1] * half, -tangent[..., 0] * half), axis=-1)
  return c + normal, c - normal


def hls_to_rgb(hue, light, sat) -> np.ndarray:
  """Vectorized colorsys.hls_to_rgb: (..., 3) RGB in 0..1."""
  hue, light, sat = np.broadcast_arrays(*(np.asarray(v, dtype=np.float64) for v in (hue, light, sat)))
  m2 = np.where(light <= 0.5, light * (1.0 + sat), light + sat - light * sat)
  m1 = 2.0 * light - m2

  def channel(h):
    h = np.mod(h, 1.0)
    rise = m1 + (m2 - m1) * h * 6.0
    fall = m1 + (m2 - m1) * (2.0 / 3.0 - h) * 6.0
    return np.select([h < 1 / 6, h < 0.5, h < 2 / 3], [rise, m2, fall], m1)

  rgb = np.stack((channel(hue + 1 / 3), channel(hue), channel(hue - 1 / 3)), axis=-1)
  return np.where((sat == 0.0)[..., None], light[..., None], rgb)


def heading_at(grid: np.ndarray, ys: np.ndarray, x: float) -> float:
  """Path heading (radians, right-positive) at forward distance x."""
  ok = np.isfinite(ys)
  if np.count_nonzero(ok) < 2:
    return 0.0
  gx, gy = grid[ok], ys[ok]
  dy = np.gradient(gy, gx)
  return float(math.atan(np.interp(x, gx, dy)))


def lateral_at(grid: np.ndarray, ys: np.ndarray, x: float, default: float = 0.0) -> float:
  ok = np.isfinite(ys)
  if np.count_nonzero(ok) < 2 or x < grid[ok][0] or x > grid[ok][-1]:
    return default
  return float(np.interp(x, grid[ok], ys[ok]))


def car_outline(x: float, y: float, heading: float = 0.0, length: float = 4.6, width: float = 1.9,
                chamfer: float = 0.45) -> np.ndarray:
  """Convex octagonal car footprint in car-frame meters, centered at (x, y), counter-clockwise on screen."""
  hl, hw, c = length / 2.0, width / 2.0, min(chamfer, width / 2.0 - 1e-3)
  # (forward, right) corners going clockwise on the ground when viewed from above = CCW on screen (y down).
  local = np.array([(hl, -hw + c), (hl - c, -hw), (-hl + c, -hw), (-hl, -hw + c),
                    (-hl, hw - c), (-hl + c, hw), (hl - c, hw), (hl, hw - c)], dtype=np.float64)
  cos_h, sin_h = math.cos(heading), math.sin(heading)
  rot = np.array([[cos_h, -sin_h], [sin_h, cos_h]])
  return local @ rot.T + np.array([x, y])


def rotate_offsets(local: np.ndarray, x: float, y: float, heading: float) -> np.ndarray:
  cos_h, sin_h = math.cos(heading), math.sin(heading)
  rot = np.array([[cos_h, -sin_h], [sin_h, cos_h]])
  return np.asarray(local, dtype=np.float64) @ rot.T + np.array([x, y])


def time_to_collision(d_rel: float, v_rel: float) -> float:
  """Seconds until contact at the current closing speed; inf when not closing."""
  if not math.isfinite(d_rel) or not math.isfinite(v_rel) or v_rel >= -0.1:
    return math.inf
  return max(0.0, d_rel) / -v_rel


NEUTRAL_LEAD = (214, 219, 228)
ALERT_LEAD = (240, 64, 56)
CAUTION_LEAD = (246, 184, 64)


def lead_color(d_rel: float, v_rel: float) -> tuple[int, int, int]:
  """Neutral grey above 6 s TTC, amber around 3 s, red at 1.5 s or less (or closer than 4 m)."""
  ttc = time_to_collision(d_rel, v_rel)
  if d_rel < 4.0:
    ttc = min(ttc, 1.5)
  if ttc >= 6.0:
    return NEUTRAL_LEAD
  if ttc >= 3.0:
    t = (6.0 - ttc) / 3.0
    a, b = NEUTRAL_LEAD, CAUTION_LEAD
  else:
    t = min(1.0, (3.0 - max(ttc, 1.5)) / 1.5)
    a, b = CAUTION_LEAD, ALERT_LEAD
  return tuple(int(round(p + (q - p) * t)) for p, q in zip(a, b, strict=True))  # type: ignore[return-value]


YARDS_PER_METER = 1.09361


def distance_label(d_rel: float, metric: bool = True) -> str:
  """The radar gap (dRel, ego front bumper to lead rear bumper) as printed, with no offset: metres or yards."""
  if metric:
    return f"{max(0.0, d_rel):.0f} m"
  return f"{max(0.0, d_rel) * YARDS_PER_METER:.0f} yd"


def ego_wheel_angle(steering_angle_deg: float, steer_ratio: float = 15.0, exaggerate: float = 2.0) -> float:
  """Front-wheel angle in radians for the ego glyph (right-positive, matching car-frame y)."""
  if not math.isfinite(steering_angle_deg):
    return 0.0
  # steeringAngleDeg is left-positive; the glyph's frame is right-positive.
  return -math.radians(max(-35.0, min(35.0, steering_angle_deg / steer_ratio * exaggerate)))


def smooth(previous: np.ndarray | None, current: np.ndarray, alpha: float) -> np.ndarray:
  """Exponential blend toward current; NaNs and shape changes take the new value directly."""
  if previous is None or previous.shape != current.shape:
    return current.copy()
  out = previous + (current - previous) * alpha
  fresh = ~np.isfinite(previous) | ~np.isfinite(current)
  out[fresh] = current[fresh]
  return out


# ---- strip drawing ------------------------------------------------------------------------------

def strip_points(left: np.ndarray, right: np.ndarray) -> np.ndarray:
  """(N, 2) left and right ribbon sides -> contiguous (2N, 2) float32 [L0, R0, L1, R1, ..]: a triangle strip."""
  left, right = np.asarray(left), np.asarray(right)
  out = np.empty((2 * left.shape[0], 2), dtype=np.float32)
  out[0::2] = left
  out[1::2] = right
  return out


def ring_strip(poly: np.ndarray, thickness: float) -> np.ndarray:
  """A closed outline of a convex (or simple) polygon as one triangle strip: (2(n+1), 2) float32.

  The stroke is centred on the edges and mitred at the corners, so every edge is `thickness` wide."""
  p = np.asarray(poly, dtype=np.float64)
  e_in = p - np.roll(p, 1, axis=0)
  e_out = np.roll(p, -1, axis=0) - p
  for e in (e_in, e_out):
    n = np.hypot(e[:, 0], e[:, 1])
    n[n < 1e-9] = 1.0
    e /= n[:, None]
  n_in = np.stack((e_in[:, 1], -e_in[:, 0]), axis=1)
  n_out = np.stack((e_out[:, 1], -e_out[:, 0]), axis=1)
  denom = np.maximum(1.0 + np.sum(n_in * n_out, axis=1), 0.25)  # caps the miter at sharp corners
  offset = (n_in + n_out) / denom[:, None] * (thickness / 2.0)
  out = strip_points(p + offset, p - offset)
  return np.concatenate((out, out[:2]))


def overlay_bands(top: float, height: float, cut_y: float | None = None, soft_y: float | None = None,
                  hold: float = 0.6, end: float = 0.95, hold_alpha: float = 0.8) -> list[tuple[float, float, float, float]]:
  """Background-coloured gradient bands (y_top, y_bottom, alpha_top, alpha_bottom) drawn over the road to fade it.

  The far fade is 1 - the old per-row fade profile (alpha 1 at the bottom, hold_alpha at `hold`, 0 at `end`, as
  height fractions). With cut_y, everything above cut_y is covered and the band from soft_y up to cut_y fades in,
  so lines end softly at the draw distance. Overlays of one colour compose multiplicatively, so the two stack."""
  bottom = top + height

  def y(t):
    return bottom - t * height
  bands = [(y(hold), bottom, 1.0 - hold_alpha, 0.0), (y(end), y(hold), 1.0, 1.0 - hold_alpha), (top, y(end), 1.0, 1.0)]
  if cut_y is not None and cut_y > top:
    cut_y = min(cut_y, bottom)
    soft_y = cut_y if soft_y is None else min(max(soft_y, cut_y), bottom)
    clipped = []
    for y0, y1, a0, a1 in bands:
      if y1 <= cut_y:
        continue
      if y0 < cut_y:
        a0 = a0 + (a1 - a0) * (cut_y - y0) / (y1 - y0)
        y0 = cut_y
      clipped.append((y0, y1, a0, a1))
    bands = clipped + [(cut_y, soft_y, 1.0, 0.0), (top, cut_y, 1.0, 1.0)]
  return [b for b in bands if b[1] - b[0] >= 0.5 and max(b[2], b[3]) > 0.0]


def end_fade_length(start: float, end: float, fraction: float = 0.25, low: float = 4.0, high: float = 12.0) -> float:
  """Length of the soft end of a line drawn from start to end: `fraction` of it, clipped to [low, high] metres and
  never longer than the line."""
  return max(0.0, min(max(fraction * (end - start), low), high, end - start))


LINE_STD_LIMIT = 1.0  # m: a lane line or road edge stops where its own per-point lateral std (yStd) passes this


def std_end(grid: np.ndarray, y_std: np.ndarray | None, limit: float = LINE_STD_LIMIT) -> float:
  """First grid distance where a line's resampled yStd exceeds limit; inf when it never does or is not published
  (missing, or all zero / non-finite, as when modeld leaves XYZTData.yStd empty)."""
  if y_std is None:
    return math.inf
  std = np.asarray(y_std, dtype=np.float64)
  ok = np.isfinite(std)
  if not (std[ok] > 0).any():
    return math.inf
  over = np.flatnonzero(ok & (std > limit))
  return float(grid[over[0]]) if over.size else math.inf


def line_rows(grid: np.ndarray, ys: np.ndarray, end: float, start: float, step: float = 0.75):
  """Rows to draw a resampled line up to `end` with a soft end: its grid rows before the fade, then every `step`
  metres over the fade (bem.end_fade_length from `start`, the view's bottom), the last row exactly at the end.

  Returns (xs, lateral, valid, end, fade length) or None with nothing to draw. The end is also cut at the line's
  own data (no fade when that is the grid's end); rows inside a gap of the line are invalid."""
  ok = np.isfinite(ys)
  if np.count_nonzero(ok) < 2:
    return None
  gx = grid[ok]
  end = min(end, float(gx[-1]))
  first = float(gx[0])
  if end - first < 1e-3:
    return None
  # A line running to the grid's end runs off the widest view: no soft end there.
  fade = 0.0 if end >= grid[-1] - 1e-6 else end_fade_length(max(start, first), end)
  body = grid[(grid >= first) & (grid < end - fade)]
  xs = np.concatenate((body, np.arange(end - fade, end, step), [end]))
  lateral = np.interp(xs, gx, ys[ok])
  valid = np.interp(xs, grid, ok.astype(np.float64)) > 0.999
  return xs, lateral, valid, end, fade


def taper(xs, end: float, length: float) -> np.ndarray:
  """1 up to end - length, falling linearly to 0 at end."""
  xs = np.asarray(xs, dtype=np.float64)
  if length <= 1e-6:
    return (xs <= end).astype(np.float64)
  return np.clip((end - xs) / length, 0.0, 1.0)


def alpha_groups(alpha: np.ndarray, step: float = 0.05) -> list[tuple[int, int, float]]:
  """Split rows with per-row alpha into strips of one alpha: (first row, last row + 1, alpha).

  Each segment between consecutive rows takes the mean of its two rows; consecutive segments share a strip while
  they fall in the same `step`-wide alpha bin, and the strip's alpha is the middle of its segments' range, so it
  is within step / 2 of every segment it covers."""
  a = np.asarray(alpha, dtype=np.float64)
  if a.size < 2:
    return []
  seg = (a[:-1] + a[1:]) / 2
  starts = np.concatenate(([0], np.flatnonzero(np.diff(np.floor(seg / step))) + 1))
  values = (np.minimum.reduceat(seg, starts) + np.maximum.reduceat(seg, starts)) / 2
  ends = np.append(starts[1:], seg.size) + 1
  return list(zip(starts.tolist(), ends.tolist(), values.tolist(), strict=True))


# ---- extra model data -----------------------------------------------------------------------------

LANE_CHANGE_STATES = {'off': 0, 'preLaneChange': 1, 'laneChangeStarting': 2, 'laneChangeFinishing': 3}
LANE_CHANGE_SIDES = {'none': 0, 'left': -1, 'right': 1}
DEFAULT_LANE_WIDTH = 3.7


def lane_change_code(state) -> int:
  """modelV2.meta.laneChangeState (enum, its name or raw int) -> 0 off, 1 pre, 2 starting, 3 finishing."""
  if isinstance(state, (int, np.integer)) and not isinstance(state, bool):
    return int(state) if 0 <= state <= 3 else 0
  return LANE_CHANGE_STATES.get(str(state), 0)


def lane_change_side(direction) -> int:
  """modelV2.meta.laneChangeDirection -> -1 left, +1 right (car-frame y sign), 0 none."""
  if isinstance(direction, (int, np.integer)) and not isinstance(direction, bool):
    return {1: -1, 2: 1}.get(int(direction), 0)
  return LANE_CHANGE_SIDES.get(str(direction), 0)


def lane_width(lanes, grid: np.ndarray, x: float, side: int, default: float = DEFAULT_LANE_WIDTH,
               low: float = 2.5, high: float = 5.0) -> float:
  """Width of the lane on `side` (-1 left, +1 right) at x from laneLines [0]/[1] or [2]/[3]; default if implausible."""
  if side == 0 or len(lanes) < 4:
    return default
  a, b = (lanes[0], lanes[1]) if side < 0 else (lanes[2], lanes[3])
  width = lateral_at(grid, b, x, math.nan) - lateral_at(grid, a, x, math.nan)
  return width if math.isfinite(width) and low <= width <= high else default


def std_alpha(std: float) -> float:
  """Lane line alpha factor from its laneLineStds entry: clip(1 - std, 0.35, 1); 1 when unknown."""
  return float(np.clip(1.0 - std, 0.35, 1.0)) if math.isfinite(std) else 1.0


def model_lead_tracks(leads, min_prob: float) -> list[tuple[np.ndarray, np.ndarray]]:
  """(x, y) trajectories (6 points at t = 0, 2 .. 10 s, camera frame, y right-positive) of leadsV3 entries
  with prob >= min_prob and a finite first point."""
  tracks = []
  for lead in list(leads)[:3]:
    try:
      prob = float(lead.prob)
      xs = np.asarray(list(lead.x), dtype=np.float64)
      ys = np.asarray(list(lead.y), dtype=np.float64)
    except (AttributeError, TypeError, ValueError):
      continue
    n = min(xs.size, ys.size)
    if prob >= min_prob and n and math.isfinite(xs[0]) and math.isfinite(ys[0]):
      tracks.append((xs[:n], ys[:n]))
  return tracks


def lead_prediction(d_rel: float, y: float, tracks, front: float, gap: float = 6.0, lateral: float = 2.0) -> np.ndarray | None:
  """(2, 2) displacement (forward, right) of a lead at t = 2 s and 4 s from the nearest matching model track
  (first point within `gap` m of the gap d_rel and `lateral` m of y); None without a finite match."""
  best, best_d = None, gap
  for xs, ys in tracks:
    d = abs(xs[0] - front - d_rel)
    if d < best_d and abs(ys[0] - y) < lateral and xs.size >= 3 and ys.size >= 3:
      best, best_d = (xs, ys), d
  if best is None:
    return None
  xs, ys = best
  out = np.stack((xs[1:3] - xs[0], ys[1:3] - ys[0]), axis=1)
  return out if np.isfinite(out).all() else None


MPH_PER_MPS = 2.2369362921
KPH_PER_MPS = 3.6


def label_font_size(view_height: float) -> float:
  """The birds-eye lead label's text size for a view rect this tall: ~2x the first version (unreadable on a car
  screen), ~37 px at 720 p. The stream HUD's speed readout is a multiple of it."""
  return max(30.0, min(54.0, view_height * 0.052))


def speed_text(mps: float, metric: bool) -> str:
  return f"{round(mps * (KPH_PER_MPS if metric else MPH_PER_MPS)):d}"


def closing_speed_label(v_rel: float, metric: bool = True, minimum: float = 0.5) -> str | None:
  """'-5 mph' / '+8 km/h' from radar vRel (negative = closing); None below `minimum` m/s or when unknown."""
  if not math.isfinite(v_rel) or abs(v_rel) < minimum:
    return None
  value = round(v_rel * (KPH_PER_MPS if metric else MPH_PER_MPS))
  return f"{value:+d} {'km/h' if metric else 'mph'}"


CURVE_MARK = 0.01  # 1/m: path points tighter than this get a brighter outer edge


def path_curvature(model) -> tuple[np.ndarray, np.ndarray] | None:
  """(position.x, signed curvature 1/m) per plan point: orientationRate.z / velocity.x, right-positive.

  The curve_speed.target formula (zero below 3 m/s, magnitude capped at 0.1); None when the fields are missing."""
  try:
    rates = np.asarray(list(model.orientationRate.z), dtype=np.float64)
    speeds = np.asarray(list(model.velocity.x), dtype=np.float64)
    xs = np.asarray(list(model.position.x), dtype=np.float64)
  except (AttributeError, TypeError, ValueError):
    return None
  n = min(rates.size, speeds.size, xs.size)
  if n < 2:
    return None
  rates, speeds = rates[:n], speeds[:n]
  with np.errstate(divide='ignore', invalid='ignore'):
    kappa = np.where(speeds >= 3.0, np.clip(rates / speeds, -0.1, 0.1), 0.0)
  return xs[:n], np.nan_to_num(kappa)


def segment_quad(x: float, y: float, heading: float, along: float, across: float) -> np.ndarray:
  """Car-frame corners (4, 2) of a bar centred at (x, y): `along` metres in the heading, `across` metres across it."""
  half = np.array([(along / 2, -across / 2), (along / 2, across / 2), (-along / 2, across / 2), (-along / 2, -across / 2)])
  return rotate_offsets(half, x, y, heading)
