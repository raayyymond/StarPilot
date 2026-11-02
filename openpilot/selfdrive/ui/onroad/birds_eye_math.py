"""Pure geometry for the birds-eye (top-down chase view) renderer. No GL, no messaging.

Car frame follows modelV2 (device frame): x forward, y right, z down, meters. radarState
yRel is left-positive and must be negated before it is mapped here.

The view is a ground-plane homography: with u = (x - x_min) / (x_max - x_min) and
d = 1 + k*u, a point lands at

  px = cx + y * lateral_ppm / d
  py = bottom - (1 + k) * u / d * depth_px

k = 0 is a plain orthographic top-down view. k > 0 is a mild chase-cam: straight lines stay
straight, the ego region keeps its scale and distant geometry narrows by 1 / (1 + k).
"""
from dataclasses import dataclass
import math

import numpy as np


@dataclass(frozen=True)
class BirdsEyeConfig:
  x_min: float = -6.0          # meters behind the camera shown below the ego origin
  x_max: float = 80.0          # meters ahead at the top edge
  lateral_range: float = 10.0  # meters either side visible at the ego, before the scale cap
  perspective: float = 0.6     # 0 = orthographic, 1 = isotropic at the ego (strong chase cam)
  max_ppm_fraction: float = 0.045  # lateral pixels-per-meter cap, as a fraction of the rect height
  top_pad: float = 0.03        # fractions of the rect height kept clear at the top and bottom
  bottom_pad: float = 0.03
  sample_step: float = 1.5     # meters between resampled line points near the car


DEFAULT_CONFIG = BirdsEyeConfig()


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


def distance_label(d_rel: float, metric: bool = True) -> str:
  if metric:
    return f"{max(0.0, d_rel):.0f} m"
  return f"{max(0.0, d_rel) * 3.28084:.0f} ft"


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
