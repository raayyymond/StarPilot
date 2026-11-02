"""Deterministic kinematic road, ego and lead model for the hardware-free UI loop.

Everything here is pure numpy. A Simulation is integrated once up front at DT and
then sampled by time, so any thread may ask for the state at any instant.

Frames: the road/world frame is x forward, y left, z up. ScenarioFrame converts
geometry into the frame modelV2 actually uses (openpilot's calibrated device frame:
x forward, y right, z down, origin at the road camera), so lane lines sit at
z = +camera height and the planned path at z = 0 (the UI adds the height).
"""
from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
import math

import numpy as np

from openpilot.selfdrive.modeld.constants import ModelConstants

DT = 0.01
LANE_WIDTH = 3.7
EDGE_MARGIN = 0.6
LANE_COUNT = 3  # lane centers at -W, 0, +W around the centerline
CAMERA_HEIGHT = 1.22  # calibrationd.HEIGHT_INIT
RADAR_TO_CAMERA = 1.52  # radard.RADAR_TO_CAMERA
WHEELBASE = 2.7
STEER_RATIO = 15.0
LOOKAHEAD = 260.0
LEAD_IN_LANE = 2.0  # m lateral gap for a lead to count as in the ego lane
T_IDXS = np.array(ModelConstants.T_IDXS)
X_IDXS = np.array(ModelConstants.X_IDXS)
LEAD_T_IDXS = np.array(ModelConstants.LEAD_T_IDXS)
SCENARIOS = ("straight", "s_curve", "lane_change", "lead_stop", "merge")

# Absolute painted-line offsets from the road centerline (left positive), with dashed flags.
PAINTED_LINES = ((1.5 * LANE_WIDTH, False), (0.5 * LANE_WIDTH, True), (-0.5 * LANE_WIDTH, True), (-1.5 * LANE_WIDTH, False))
ROAD_EDGE = 1.5 * LANE_WIDTH + EDGE_MARGIN


def smoothstep(x):
  x = np.clip(x, 0.0, 1.0)
  return x * x * (3.0 - 2.0 * x)


def _cumtrapz(y, dx):
  out = np.zeros_like(y)
  out[1:] = np.cumsum((y[1:] + y[:-1]) * 0.5 * dx)
  return out


class Road:
  """Planar road centerline integrated from a curvature-vs-distance profile."""

  def __init__(self, curvature: Callable[[np.ndarray], np.ndarray], length: float, ds: float = 0.5):
    self.ds = ds
    self.s = np.arange(0.0, length + ds, ds)
    self.k = np.asarray(curvature(self.s), dtype=np.float64)
    self.psi = _cumtrapz(self.k, ds)
    self.x = _cumtrapz(np.cos(self.psi), ds)
    self.y = _cumtrapz(np.sin(self.psi), ds)

  def curvature(self, s):
    return np.interp(s, self.s, self.k)

  def heading(self, s):
    return np.interp(s, self.s, self.psi)

  def point(self, s, offset):
    """World point at arc length s and lateral offset (left positive)."""
    psi = self.heading(s)
    return (np.interp(s, self.s, self.x) - offset * np.sin(psi),
            np.interp(s, self.s, self.y) + offset * np.cos(psi))


@dataclass(frozen=True)
class Lead:
  x: float  # camera-frame forward distance, m
  y: float  # lateral, left positive, m
  v: float  # absolute speed, m/s
  a: float
  v_rel: float
  in_lane: bool

  @property
  def d_rel(self) -> float:
    return self.x - RADAR_TO_CAMERA


@dataclass(frozen=True)
class ScenarioFrame:
  """One sample of the scene, ready for message construction."""
  t: float
  v_ego: float
  a_ego: float
  curvature: float  # left positive, 1/m
  steering_angle_deg: float
  yaw_rate: float  # left positive, rad/s
  lane_lines: np.ndarray  # (4, 3, 33) model frame
  lane_line_probs: np.ndarray  # (4,)
  road_edges: np.ndarray  # (2, 3, 33)
  road_edge_stds: np.ndarray  # (2,)
  position: np.ndarray  # (3, 33)
  velocity: np.ndarray
  acceleration: np.ndarray
  orientation: np.ndarray
  orientation_rate: np.ndarray
  leads: tuple[Lead, ...]
  lane_change_state: str
  lane_change_direction: str
  left_blinker: bool
  right_blinker: bool
  # dense camera-frame centerline for image rendering (x forward, y left)
  centerline_u: np.ndarray
  centerline_x: np.ndarray
  centerline_y: np.ndarray
  centerline_heading: np.ndarray
  s_ego: float
  rpy_calib: tuple[float, float, float] = (0.0, 0.0, 0.0)
  height: float = CAMERA_HEIGHT

  @property
  def lead_one(self) -> Lead | None:
    in_lane = [lead for lead in self.leads if lead.in_lane and lead.x > 0]
    return min(in_lane, key=lambda lead: lead.x) if in_lane else None

  @property
  def lead_two(self) -> Lead | None:
    one = self.lead_one
    others = [lead for lead in self.leads if lead is not one and lead.x > 0]
    return min(others, key=lambda lead: lead.x) if others else None


def _curvature_profile(name: str):
  if name == "s_curve":
    amplitude, period, start = 1.0 / 300.0, 700.0, 60.0
    return lambda s: np.where(s < start, 0.0, amplitude * np.sin(2 * np.pi * (s - start) / period))
  if name == "lane_change":
    return lambda s: np.where(s < 100.0, 0.0, 1.0 / 1500.0) * np.ones_like(s)
  if name == "merge":
    return lambda s: (1.0 / 900.0) * np.sin(2 * np.pi * s / 1500.0)
  return lambda s: np.zeros_like(s)


class Simulation:
  def __init__(self, name: str = "s_curve", *, seed: int = 0, speed: float = 25.0, duration: float = 60.0,
               lateral_offset: float = 0.0, noise: float = 0.08):
    if name not in SCENARIOS:
      raise ValueError(f"unknown scenario {name!r}; choose from {', '.join(SCENARIOS)}")
    if speed <= 0 or duration <= 0:
      raise ValueError("speed and duration must be positive")
    self.name, self.seed, self.speed, self.duration = name, seed, float(speed), float(duration)
    rng = np.random.default_rng(seed)
    n = int(round(duration / DT)) + 1
    self.t = np.arange(n) * DT

    # Ego lateral offset from the road centerline: scenario base + constant + smooth noise.
    freqs = rng.uniform(0.05, 0.3, size=4)
    phases = rng.uniform(0, 2 * np.pi, size=4)
    wobble = noise * np.sum(np.sin(2 * np.pi * freqs[:, None] * self.t[None, :] + phases[:, None]), axis=0) / 2.0
    base = np.zeros(n)
    self.lane_change = np.zeros(n)  # 0 off, 1 pre, 2 starting, 3 finishing
    if name == "lane_change":
      go_left = smoothstep((self.t - 6.0) / 5.0)
      go_back = smoothstep((self.t - 26.0) / 5.0)
      base = LANE_WIDTH * (go_left - go_back)
      for start in (6.0, 26.0):
        self.lane_change[(self.t >= start - 1.5) & (self.t < start)] = 1
        self.lane_change[(self.t >= start) & (self.t < start + 3.5)] = 2
        self.lane_change[(self.t >= start + 3.5) & (self.t < start + 5.0)] = 3
    self.lane_change_left = np.ones(n, dtype=bool)
    if name == "lane_change":
      self.lane_change_left = self.t < 16.0
    self.d = base + lateral_offset + wobble
    self.d_dot = np.gradient(self.d, DT)
    self.d_ddot = np.gradient(self.d_dot, DT)

    # Lead definition (absolute arc length, speed, offset) and ego longitudinal integration.
    self.has_lead = name in ("lead_stop", "merge", "lane_change")
    s = np.zeros(n)
    v = np.zeros(n)
    a = np.zeros(n)
    ls = np.zeros(n)
    lv = np.zeros(n)
    la = np.zeros(n)
    ld = np.zeros(n)
    v[0] = self.speed
    if name == "lead_stop":
      ls[0], lv[0] = 45.0, self.speed
    elif name == "merge":
      ls[0], lv[0] = 28.0, max(self.speed - 4.0, 2.0)
    elif name == "lane_change":
      ls[0], lv[0] = 55.0, max(self.speed - 6.0, 2.0)
    for i in range(n):
      ti = self.t[i]
      if name == "lead_stop":
        la[i] = -2.5 if ti >= 6.0 and lv[i] > 0 else 0.0
      if name == "merge":
        ld[i] = -LANE_WIDTH * (1.0 - smoothstep((ti - 4.0) / 3.0))
      a_cmd = float(np.clip(0.6 * (self.speed - v[i]), -1.0, 1.0))
      if self.has_lead and abs(ld[i] - self.d[i]) < LEAD_IN_LANE:
        gap = ls[i] - s[i]
        a_follow = 0.25 * (gap - (6.0 + 1.6 * v[i])) + 0.8 * (lv[i] - v[i])
        a_cmd = min(a_cmd, a_follow)
      a[i] = float(np.clip(a_cmd, -4.0, 1.5))
      if i + 1 < n:
        v[i + 1] = max(0.0, v[i] + a[i] * DT)
        s[i + 1] = s[i] + 0.5 * (v[i] + v[i + 1]) * DT
        lv[i + 1] = max(0.0, lv[i] + la[i] * DT)
        ls[i + 1] = ls[i] + 0.5 * (lv[i] + lv[i + 1]) * DT
    self.s, self.v, self.a = s, v, a
    self.lead_s, self.lead_v, self.lead_a, self.lead_d = ls, lv, la, ld
    self.road = Road(_curvature_profile(name), max(s[-1], ls[-1]) + LOOKAHEAD + self.speed * T_IDXS[-1] + 50.0)

  def _sample(self, values: np.ndarray, t):
    return np.interp(t, self.t, values)

  def _future_s(self, t: float, horizon: np.ndarray) -> np.ndarray:
    times = t + horizon
    s = self._sample(self.s, times)
    beyond = times > self.t[-1]
    if np.any(beyond):
      s[beyond] = self.s[-1] + self.v[-1] * (times[beyond] - self.t[-1])
    return s

  def frame(self, t: float) -> ScenarioFrame:
    """State at time t; t wraps around the scenario duration so live runs can loop."""
    t = float(t) % self.duration
    s_e = float(self._sample(self.s, t))
    v_e = float(self._sample(self.v, t))
    a_e = float(self._sample(self.a, t))
    d_e = float(self._sample(self.d, t))
    d_dot = float(self._sample(self.d_dot, t))
    d_ddot = float(self._sample(self.d_ddot, t))
    v_safe = max(v_e, 1.0)
    heading_error = math.atan(d_dot / v_safe) if v_e > 0.1 else 0.0
    psi_e = float(self.road.heading(s_e)) + heading_error
    ex, ey = self.road.point(s_e, d_e)
    cos_p, sin_p = math.cos(psi_e), math.sin(psi_e)

    def to_car(wx, wy):
      dx, dy = np.asarray(wx) - ex, np.asarray(wy) - ey
      return dx * cos_p + dy * sin_p, -dx * sin_p + dy * cos_p

    curvature = float(self.road.curvature(s_e)) + (d_ddot / (v_safe * v_safe) if v_e > 0.1 else 0.0)
    steering = math.degrees(math.atan(curvature * WHEELBASE)) * STEER_RATIO

    # Dense centerline ahead of the camera.
    u = np.arange(-10.0, LOOKAHEAD + 1.0, 1.0)
    cx, cy = to_car(*self.road.point(s_e + u, 0.0))
    c_heading = self.road.heading(s_e + u) - psi_e

    def line_at(offset: float) -> np.ndarray:
      lx, ly = to_car(*self.road.point(s_e + u, offset))
      lx = np.maximum.accumulate(lx)
      return np.interp(X_IDXS, lx, ly)

    lane_center = LANE_WIDTH * round(d_e / LANE_WIDTH)
    lane_offsets = [lane_center + 1.5 * LANE_WIDTH, lane_center + 0.5 * LANE_WIDTH,
                    lane_center - 0.5 * LANE_WIDTH, lane_center - 1.5 * LANE_WIDTH]
    lane_lines = np.zeros((4, 3, X_IDXS.size))
    probs = np.zeros(4)
    for i, offset in enumerate(lane_offsets):
      lane_lines[i, 0] = X_IDXS
      lane_lines[i, 1] = -line_at(offset)
      lane_lines[i, 2] = CAMERA_HEIGHT
      exists = abs(offset) <= 1.5 * LANE_WIDTH + 1e-6
      probs[i] = (0.95 if i in (1, 2) else 0.75) if exists else 0.02
    road_edges = np.zeros((2, 3, X_IDXS.size))
    for i, offset in enumerate((ROAD_EDGE, -ROAD_EDGE)):
      road_edges[i, 0] = X_IDXS
      road_edges[i, 1] = -line_at(offset)
      road_edges[i, 2] = CAMERA_HEIGHT

    # Planned path: the ego's own future trajectory.
    fs = self._future_s(t, T_IDXS)
    fd = self._sample(self.d, np.minimum(t + T_IDXS, self.t[-1]))
    px, py = to_car(*self.road.point(fs, fd))
    fv = np.interp(np.minimum(t + T_IDXS, self.t[-1]), self.t, self.v)
    fa = np.interp(np.minimum(t + T_IDXS, self.t[-1]), self.t, self.a)
    yaw = self.road.heading(fs) - psi_e
    yaw_rate = v_e * curvature
    position = np.stack([px, -py, np.zeros_like(px)])
    velocity = np.stack([fv * np.cos(yaw), -fv * np.sin(yaw), np.zeros_like(px)])
    acceleration = np.stack([fa, np.zeros_like(px), np.zeros_like(px)])
    orientation = np.stack([np.zeros_like(px), np.zeros_like(px), -yaw])
    orientation_rate = np.stack([np.zeros_like(px), np.zeros_like(px), -np.gradient(yaw, T_IDXS)])

    leads: list[Lead] = []
    if self.has_lead:
      ls = float(self._sample(self.lead_s, t))
      ld = float(self._sample(self.lead_d, t))
      lx, ly = to_car(*self.road.point(ls, ld))
      lv = float(self._sample(self.lead_v, t))
      la = float(self._sample(self.lead_a, t))
      if 0.0 < lx < 200.0:
        lane_gap = abs(ld - d_e)
        leads.append(Lead(float(lx), float(ly), lv, la, lv - v_e, lane_gap < LEAD_IN_LANE))

    stage = int(self._sample(self.lane_change, t) + 0.5)
    states = ("off", "preLaneChange", "laneChangeStarting", "laneChangeFinishing")
    left = bool(np.interp(t, self.t, self.lane_change_left.astype(float)) > 0.5)
    direction = "none" if stage == 0 else ("left" if left else "right")
    return ScenarioFrame(
      t=t, v_ego=v_e, a_ego=a_e, curvature=curvature, steering_angle_deg=steering, yaw_rate=yaw_rate,
      lane_lines=lane_lines, lane_line_probs=probs, road_edges=road_edges, road_edge_stds=np.array([0.3, 0.3]),
      position=position, velocity=velocity, acceleration=acceleration, orientation=orientation,
      orientation_rate=orientation_rate, leads=tuple(leads), lane_change_state=states[stage],
      lane_change_direction=direction, left_blinker=stage in (1, 2) and left, right_blinker=stage in (1, 2) and not left,
      centerline_u=u, centerline_x=cx, centerline_y=cy, centerline_heading=c_heading, s_ego=s_e,
    )
