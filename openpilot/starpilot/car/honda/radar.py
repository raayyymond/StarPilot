from __future__ import annotations

import json
import math
import time
from collections import deque
from pathlib import Path
from types import SimpleNamespace
from typing import TYPE_CHECKING, Any

import capnp

from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.realtime import DT_MDL
from openpilot.common.simple_kalman import KF1D
from openpilot.starpilot.car.honda import radar_point_status
from openpilot.starpilot.car.honda.radar_camera_slow_bound import CameraSlowBound
from openpilot.starpilot.car.honda.radar_camera_witness import (CAMERA_PROB_MIN, RADAR_TO_CAMERA, TREND_SAMPLES,
                                                               CameraWitness, camera_contradiction, camera_passed_ratio,
                                                               closing_ttc, range_slope, urgent)

if TYPE_CHECKING:
  from openpilot.selfdrive.controls.radard import KalmanParams

_LEAD_ACCEL_TAU = 0.6
# Bosch-A lead-accel extrapolation bounds (2026-10-22). This Track is only built by BoschALeadPolicy, so both are
# Bosch-A only; the generic radard Track is untouched. The MPC extrapolates a radar lead as
# aLeadK * exp(-aLeadTau * t^2 / 2). A hard-braking lead decays aLeadTau toward 0 (update(0.0) below) while the KF
# accel can run past -3 m/s^2 on a rail/rejoin vLead step, so the MPC held a large phantom decel over the whole
# horizon. Evidence (replay only, no road validation): window (a) 206-208.2 s of the investigated route, MPC obstacle
# distance replays with each bound applied in isolation (patch_tau, patch_akc). Neither changes dRel/vRel/vLead, the
# KF state or which track is selected (D-048 arbitration reads dRel/vRel, not aLeadK).
#   BOSCH_A_LEAD_ACCEL_TAU_MIN: the published aLeadTau never decays below this (s^-2 in the MPC exponent).
#   BOSCH_A_LEAD_ACCEL_MIN: the published aLeadK is clamped below at this; the KF itself is not. Cost: a lead that
#     really brakes harder than -3 m/s^2 is extrapolated at -3; its real slowdown still arrives through vLead/dRel.
BOSCH_A_LEAD_ACCEL_TAU_MIN = 0.3
BOSCH_A_LEAD_ACCEL_MIN = -3.0
V_EGO_STATIONARY = 4.0
HONDA_BOSCH_A_LOW_SPEED_MIN_COUNT = 3
HONDA_BOSCH_A_CHALLENGER_STALE_CYCLES = 2
HONDA_BOSCH_A_GROSS_DISTANCE_STALE_CYCLES = 3
HONDA_BOSCH_A_GROSS_DISTANCE_M = 25.0
# In-path radar keep (route 11c8fa231c0499ed/0000009e--9704dd1da2 seg 24, replay of radard at fd3b0ec99d by Job,
# 2026-11-06; level: replay / limited road evidence). Track 39 (dRel 28.1, yRel -0.86, vRel -7.9, cnt 342) was a real
# in-path closing lead; the model lead's x jumped 33 -> 50.6 m and the relaxed match in match_vision_to_track failed
# on the gap alone, so the preferred track was dropped and aLeadK flipped -3.0 -> +0.33. Deleting a real radar lead
# is worse than publishing a degraded one (repo rule), so when the relaxed vision match fails, a preferred track
# that is still in lane, still closer than the vision lead, and still closing is kept rather than dropped. Release
# is normal: when the track leaves lane, stops closing, or disappears, the old path runs.
#   KEEP_Y_MAX: in-lane bound in the radar frame (ego-centred). First set to 1.5; Job's camera stills of the census
#   events at |yRel| 1.44-1.5 (2026-11-06) showed two next-lane cars kept (7b i5143 box truck right lane, 71 i16009
#   left lane) and three real in-lane leads, so |yRel| cannot separate them near 1.5. 1.2 drops all five back to the
#   old behavior and still holds the route-9e track for ~1 s (its yRel reached -1.06 at -5.36 s, -1.45 at -5.0 s).
#   KEEP_VREL_MAX: closing bound; 0.0 m/s means the track must be closing, not holding or separating.
#   KEEP_VLEAD_MIN: the kept track must not itself be moving toward us. Job's census of all 46 keep events over
#   pool14 + r99 + r9b (2026-11-06, replay + camera stills): the only two with vEgo+vRel < -1 were cross traffic seen
#   while stopped at an intersection (route 72 i2493 at -6.6 m/s, route 7e i5814 at -9.0 m/s); every other event was
#   >= -0.24. n=46, so this separates the sample, it is not proof it generalizes.
KEEP_Y_MAX = 1.2
KEEP_VREL_MAX = 0.0
KEEP_VLEAD_MIN = -1.0
# Bosch-A urgency on a point with no direct speed reading (b1 seg14 47.7 s, 11c8fa231c0499ed/000000b1--c417f37f1c;
# parser replay on logged CAN, and logged radarState/carControl). Track 4 at 69-59 m was measured closing at 11.7 m/s,
# then for 0.75 s the parser had no direct reading and published its floor estimate, vRel -31 -> -8.4. On that floor
# the reversal/weak-lead bounds took it as urgent (TTC 2.2 s) and dropped their weight at once; leadOne read vRel -21,
# vLead 0, and the plan ramped to -3.5 for about 1 s until Peter pressed the gas. Its own last reading and its range
# slope (-9 to -15) both put TTC above 4 s. So while the parser marks a point unmeasured, the urgency TTC uses the
# last direct reading if it is no older than URGENT_LAST_DIRECT_S, else the published vRel as before. The range slope
# is not used: on b1 its 82 -> 69 m reflector hop read -17 to -22 and kept the lead urgent 47.80-48.45 s (probe), while
# the last reading alone keeps TTC 5.1-5.9 s. Cost (Peter chose it, 2026-11-08): a lead that starts closing faster while
# unmeasured is judged on a reading up to 1 s old. Only the urgency test moves: the published point, its vRel and dRel
# are untouched, and dRel < 30 m stays urgent. One event, one road (limited road evidence).
URGENT_LAST_DIRECT_S = 1.0
# Held-vRel range-slope bound (scratch candidate, 2026-11-11; level: replay only, not road tested). Scorecard episode 26,
# route 00000071--37216af48f seg 9 (t0_ns 763779770390, labeled PHANTOM by the owner): track 58 at 81.6 -> 84.7 m. The
# parser marked the point NOT measured (track.direct False) for ~1.8 s and held vRel at exactly -9.51 while the track's own
# range slope was +1.6..+2.5 m/s (the gap opening); the planner braked to -1.11. Counter-example that must not change:
# REAL episode 292 (000000a4--34c412c0ef seg 16): direct True throughout, range slope -1.5..-5.8 agreeing with vRel.
# So while the parser is holding (not measuring) a far point's vRel and the point's own range is flat or opening while the
# held vRel says it closes fast, the published vRel/vLead/vLeadK are raised to the range slope (capped at 0) less a margin,
# never lowered; dRel, aLeadK and the point are untouched. The radar's own measurement wins the cycle `direct` returns.
# The existing urgency test (urgency_v_rel + witness urgent(): range < 30 m or TTC < 4 s) vetoes it.
#   HELD_VREL_MIN_D: far leads only (m); ep26 was 81-85 m.
#   HELD_VREL_CLOSING: the published vRel must be at or below this (m/s); ep26 -9.51.
#   HELD_VREL_SLOPE_MIN: range slope at or above this = flat or opening (m/s); ep26 +1.6..+2.5, ep292 <= -1.5.
#   HELD_VREL_DISAGREE: slope - vRel must be at least this (m/s); ep26 11-12.
#   HELD_VREL_MIN_S: the condition must hold continuously this long before the bound acts (s).
#   HELD_VREL_MARGIN: the bound is min(slope, 0) - margin (m/s); ep26 gives about -1.0.
# One event, one road; the numbers are shaped on ep26 and ep292 only.
HELD_VREL_SLOPE_BOUND_ENABLED = True
HELD_VREL_MIN_D = 50.0
HELD_VREL_CLOSING = -4.0
HELD_VREL_SLOPE_MIN = -1.0
HELD_VREL_DISAGREE = 8.0
HELD_VREL_MIN_S = 0.3
HELD_VREL_MARGIN = 1.0
POST_STANDSTILL_RADAR_LEAD_PERSISTENCE_FRAMES = 3
POST_STANDSTILL_RADAR_LEAD_URGENT_TTC = 1.5
POST_STANDSTILL_RADAR_LEAD_URGENT_DISTANCE = 1.5
SPEED, ACCEL = (0, 1)
XCP_SHM_PATH = Path("/dev/shm/xcp_radar_assist.json")
XCP_ASSIST_MAX_SNAPSHOT_AGE_NS = 200_000_000
XCP_ASSIST_MAX_FUTURE_SKEW_NS = 60_000_000
XCP_ASSIST_MAX_KIN_AGE_MS = 200.0
XCP_ASSIST_MIN_AGE = 5
XCP_ASSIST_HOLD_FRAMES = 0
XCP_ASSIST_MAX_DY_M = 1.2
XCP_ASSIST_SCALE_ALIAS_MIN_D_M = 28.0
XCP_ASSIST_SCALE_ALIAS_MAX_DY_M = 0.75
XCP_ASSIST_MAX_DV_MPS = 3.0
XCP_ASSIST_LOCKED_CLOSING_MAX_DV_MPS = 8.0
XCP_ASSIST_LOCKED_CLOSING_MAX_DY_M = 1.0
XCP_ASSIST_LOCKED_CLOSING_MAX_ABS_Y_M = 1.5
XCP_ASSIST_LOCKED_CLOSING_MIN_DD_M = 3.0
XCP_ASSIST_LOCKED_CLOSING_DD_RATIO = 0.12
XCP_ASSIST_LOCKED_CLOSING_MIN_VREL_MPS = -1.0
XCP_ASSIST_LOCKED_MIN_VLEAD_MPS = -0.5
XCP_ASSIST_KF_JERK_Q_OVER_R = 30.0
XCP_ASSIST_KF_MIN_DT_S = 0.02
XCP_ASSIST_KF_MAX_DT_S = 0.20
XCP_ASSIST_MAX_STEP_DV_MPS = 0.15
XCP_ASSIST_MAX_STEP_DECEL_MPS2 = 7.0
XCP_ASSIST_MAX_STEP_DV_CAP_MPS = 0.85
XCP_ASSIST_LATCH_GUARD_ACCEL = -1.95
XCP_TRAILER_OFFSET_M = 434.0 / 128.0


def _xcp_target_qualifies(
  t: dict[str, Any],
  point,
  v_ego: float,
  *,
  locked_slot_key: tuple[Any, Any] | None = None,
) -> tuple[bool, float]:
  """Check XCP Array B assist gates, lateral validity, track ID, and range/velocity consistency."""
  if t.get("stage") != "measured":
    return False, math.inf
  if "lat_valid" in t and not bool(t.get("lat_valid")):
    return False, math.inf
  if "lat_updated_ns" in t and int(t.get("lat_updated_ns") or 0) <= 0:
    return False, math.inf
  if "yRel_m" not in t or t.get("yRel_m") is None:
    return False, math.inf
  try:
    kin_age_ms = float(t.get("kin_age_ms", 9999.0))
    flags = int(t.get("flags", 0))
    age = int(t.get("age", 0))
    xd = float(t["dRel_m"])
    xy = float(t["yRel_m"])
    xv = float(t["vRel_mps"])
  except (KeyError, TypeError, ValueError):
    return False, math.inf
  if not (math.isfinite(kin_age_ms) and math.isfinite(xd) and math.isfinite(xy) and math.isfinite(xv)):
    return False, math.inf
  if "lat_valid" not in t and "lat_updated_ns" not in t and xy == 0.0:
    return False, math.inf
  if kin_age_ms > XCP_ASSIST_MAX_KIN_AGE_MS:
    return False, math.inf
  if bool(t.get("ghost", False)) or (flags & 0x1020) != 0:
    return False, math.inf
  if age < XCP_ASSIST_MIN_AGE:
    return False, math.inf
  slot_key = (t.get("slot"), t.get("rtid", t.get("radar_track_id")))
  is_locked = locked_slot_key is not None and slot_key == locked_slot_key
  pd = float(point.dRel)
  py = float(point.yRel)
  pv = float(point.vRel)
  dy = abs(xy - py)
  if dy > XCP_ASSIST_MAX_DY_M:
    return False, math.inf
  op_tid = t.get("op_track_id")
  if op_tid is not None:
    try:
      if int(op_tid) != int(point.trackId):
        return False, math.inf
      op_confirmed = True
    except (TypeError, ValueError):
      op_confirmed = False
  else:
    op_confirmed = False
  dd_raw = abs(xd - pd)
  allow_scale_alias = xd >= pd and (
    op_confirmed or (pd >= XCP_ASSIST_SCALE_ALIAS_MIN_D_M and dy <= XCP_ASSIST_SCALE_ALIAS_MAX_DY_M and abs(py) <= 1.0)
  )
  non_stationary_closing = v_ego < 3.0 or (v_ego + xv) >= XCP_ASSIST_LOCKED_MIN_VLEAD_MPS
  locked_lat_valid = (
    xy != 0.0
    and dy <= XCP_ASSIST_LOCKED_CLOSING_MAX_DY_M
    and abs(py) <= XCP_ASSIST_LOCKED_CLOSING_MAX_ABS_Y_M
  )
  allow_closing_dd = (
    is_locked
    and xd < pd
    and xv < XCP_ASSIST_LOCKED_CLOSING_MIN_VREL_MPS
    and locked_lat_valid
    and non_stationary_closing
  )
  if allow_scale_alias:
    dd = min(dd_raw, abs((xd - XCP_TRAILER_OFFSET_M) - pd))
    max_dd = max(2.5, 0.12 * pd)
  elif allow_closing_dd:
    dd = dd_raw
    max_dd = max(XCP_ASSIST_LOCKED_CLOSING_MIN_DD_M, XCP_ASSIST_LOCKED_CLOSING_DD_RATIO * pd)
  else:
    dd = dd_raw
    max_dd = max(2.5, 0.08 * pd)
  if dd > max_dd:
    return False, math.inf
  dv = abs(xv - pv)
  if pv <= -11.5 and xv <= pv + 1.0:
    if dv > 25.0:
      return False, math.inf
  elif is_locked and locked_lat_valid and (pv - xv) > 0.0 and non_stationary_closing:
    if (pv - xv) > XCP_ASSIST_LOCKED_CLOSING_MAX_DV_MPS:
      return False, math.inf
  elif dv > XCP_ASSIST_MAX_DV_MPS:
    return False, math.inf
  return True, dd * dd + (2.0 * dy) ** 2 + (0.5 * dv) ** 2


class _XcpDopplerKF:
  """Variable-dt 2D constant-acceleration Kalman filter tuned for clean Bosch-A XCP Doppler velocity."""

  def __init__(self, v0: float, a0: float = 0.0, q_over_r: float = XCP_ASSIST_KF_JERK_Q_OVER_R):
    self.q = float(q_over_r)
    self.v = float(v0)
    self.a = float(a0)
    self.p00 = 0.3
    self.p01 = 0.5
    self.p11 = 4.0

  def reset(self, v0: float, a0: float = 0.0) -> None:
    self.v = float(v0)
    self.a = float(a0)
    self.p00 = 0.3
    self.p01 = 0.5
    self.p11 = 4.0

  def update(self, v_meas: float, dt: float, *, max_neg_step: float | None = None) -> tuple[float, float]:
    dt_clamped = max(XCP_ASSIST_KF_MIN_DT_S, min(XCP_ASSIST_KF_MAX_DT_S, float(dt)))
    v_pred = self.v + self.a * dt_clamped
    a_pred = self.a
    q = self.q
    p00 = self.p00 + 2.0 * dt_clamped * self.p01 + dt_clamped * dt_clamped * self.p11 + q * (dt_clamped ** 3) / 3.0
    p01 = self.p01 + dt_clamped * self.p11 + q * (dt_clamped ** 2) / 2.0
    p11 = self.p11 + q * dt_clamped
    s = p00 + 1.0
    k0 = p00 / s
    k1 = p01 / s
    meas = float(v_meas)
    if max_neg_step is not None:
      meas = max(meas, self.v - max_neg_step)
    innov = meas - v_pred
    self.v = v_pred + k0 * innov
    self.a = a_pred + k1 * innov
    self.p00 = (1.0 - k0) * p00
    self.p01 = (1.0 - k0) * p01
    self.p11 = max(1e-6, p11 - k1 * p01)
    return self.v, self.a


class Track:
  def __init__(self, identifier: int, v_lead: float, kalman_params: KalmanParams):
    self.identifier = identifier
    self.cnt = 0
    self.objectStatus = 0  # Bosch-A F0 STATUS of the latest point, from card (radar_point_status); 0 = not known
    self.range_history: deque[float] = deque(maxlen=TREND_SAMPLES)  # dRel per radard cycle, for the CameraWitness weak tell
    self.direct = True                 # the parser's measured flag on the latest point (radarTracks, not this cycle's freshness)
    self.last_direct_vrel = math.nan   # vRel of the latest point the parser marked measured
    self.direct_age = 0                # radard cycles since then
    self.held_vrel_cycles = 0          # radard cycles the held-vRel range-slope condition has held continuously
    self.aLeadTau = FirstOrderFilter(_LEAD_ACCEL_TAU, 0.45, DT_MDL)
    self.K_A = kalman_params.A
    self.K_C = kalman_params.C
    self.K_K = kalman_params.K
    self.kf = KF1D([[v_lead], [0.0]], self.K_A, self.K_C, self.K_K)
    self.base_kf = KF1D([[v_lead], [0.0]], self.K_A, self.K_C, self.K_K)
    self.base_cnt = 0
    self.xcp_var_kf = _XcpDopplerKF(v_lead, 0.0)
    self.xcp_last_kin_ts_ns: int = 0
    self.xcp_active = False
    self.xcp_hold_frames = 0
    self.last_xcp_match: dict[str, Any] | None = None
    self.xcp_slot_key: tuple[Any, Any] | None = None
    self.xcp_kin_sig: Any = None
    self.xcp_v_lead: float = float(v_lead)

  def update(
    self,
    d_rel: float,
    y_rel: float,
    v_rel: float,
    v_lead: float,
    measured: bool,
    measurement_update: bool | None = None,
    *,
    kf_v_lead: float | None = None,
    xcp_kf_state: tuple[float, float] | None = None,
  ):
    self.dRel = d_rel
    self.range_history.append(float(d_rel))
    self.yRel = y_rel
    self.vRel = v_rel
    self.vLead = v_lead
    self.measured = measured
    if measurement_update is None:
      measurement_update = True
    if xcp_kf_state is not None:
      self.kf.set_x([[float(xcp_kf_state[0])], [float(xcp_kf_state[1])]])
    elif measurement_update and self.cnt > 0:
      self.kf.update(self.vLead if kf_v_lead is None else float(kf_v_lead))
    self.vLeadK = float(self.kf.x[SPEED][0])
    self.aLeadK = float(self.kf.x[ACCEL][0])
    if measurement_update:
      if abs(self.aLeadK) < 0.5:
        self.aLeadTau.x = min(max(self.aLeadTau.x, 0.01) * 1.1, _LEAD_ACCEL_TAU)
      else:
        self.aLeadTau.update(0.0)
      self.aLeadTau.x = max(self.aLeadTau.x, BOSCH_A_LEAD_ACCEL_TAU_MIN)
      self.cnt += 1
    # Published value only; the KF accel state (and the tau decision above) keep the unclamped estimate.
    self.aLeadK = max(self.aLeadK, BOSCH_A_LEAD_ACCEL_MIN)

  def get_RadarState(self, model_prob: float = 0.0):
    return {
      'dRel': float(self.dRel),
      'yRel': float(self.yRel),
      'vRel': float(self.vRel),
      'vLead': float(self.vLead),
      'vLeadK': float(self.vLeadK),
      'aLeadK': float(self.aLeadK),
      'aLeadTau': float(self.aLeadTau.x),
      'present': True,
      'deprecated': {'fcw': self.is_potential_fcw(model_prob)},
      'modelProb': model_prob,
      'radar': True,
      'radarTrackId': self.identifier,
    }

  def potential_low_speed_lead(self, v_ego: float):
    return abs(self.yRel) < 1.0 and v_ego < V_EGO_STATIONARY and (0.75 < self.dRel < 25)

  def is_potential_fcw(self, model_prob: float):
    return model_prob > 0.9

  def __str__(self):
    ret = f'x: {self.dRel:4.1f}  y: {self.yRel:4.1f}  v: {self.vRel:4.1f}  a: {self.aLeadK:4.1f}'
    return ret


def laplacian_pdf(x: float, mu: float, b: float):
  b = max(b, 0.0001)
  return math.exp(-abs(x - mu) / b)


def vision_track_probability(track: Track, lead: capnp._DynamicStructReader, v_ego: float) -> float:
  offset_vision_dist = lead.x[0] - RADAR_TO_CAMERA
  prob_d = laplacian_pdf(track.dRel, offset_vision_dist, lead.xStd[0])
  prob_y = laplacian_pdf(track.yRel, -lead.y[0], lead.yStd[0])
  prob_v = laplacian_pdf(track.vRel + v_ego, lead.v[0], lead.vStd[0])
  return prob_d * prob_y * prob_v


def honda_bosch_a_low_speed_radar_lead_sane(track: Track, v_ego: float) -> bool:
  return track.cnt >= HONDA_BOSCH_A_LOW_SPEED_MIN_COUNT and track.potential_low_speed_lead(v_ego)


def post_standstill_radar_lead_is_urgent(lead: dict[str, Any]) -> bool:
  d_rel = float(lead.get('dRel', math.inf))
  v_rel = float(lead.get('vRel', 0.0))
  closing_speed = max(-v_rel, 0.0)
  ttc = closing_ttc(d_rel, closing_speed, 0.1)
  return d_rel <= POST_STANDSTILL_RADAR_LEAD_URGENT_DISTANCE or ttc <= POST_STANDSTILL_RADAR_LEAD_URGENT_TTC


def track_matches_vision(
  track: Track, lead: capnp._DynamicStructReader, v_ego: float, *, dist_scale: float, dist_floor: float, vel_limit: float, y_std_scale: float, y_floor: float
) -> bool:
  offset_vision_dist = lead.x[0] - RADAR_TO_CAMERA
  dist_sane = abs(track.dRel - offset_vision_dist) < max(abs(offset_vision_dist) * dist_scale, dist_floor)
  vel_sane = abs(track.vRel + v_ego - lead.v[0]) < vel_limit or v_ego + track.vRel > 3
  lat_sane = abs(track.yRel + lead.y[0]) < max(y_floor, y_std_scale * max(float(lead.yStd[0]), 0.2))
  return dist_sane and vel_sane and lat_sane


def urgency_v_rel(track: Track | None) -> float | None:
  """The vRel the bounds' urgency test uses for this track; None keeps the published vRel (see URGENT_LAST_DIRECT_S)."""
  if track is None or track.direct:
    return None
  if track.direct_age * DT_MDL > URGENT_LAST_DIRECT_S or math.isnan(track.last_direct_vrel):
    return None
  return track.last_direct_vrel


def held_vrel_condition(track: Track) -> bool:
  """The held-vRel range-slope condition on one track this cycle (see HELD_VREL_*). Reads the raw track, not a lead."""
  if track.direct or track.xcp_active or track.dRel < HELD_VREL_MIN_D or track.vRel > HELD_VREL_CLOSING:
    return False
  slope = range_slope(track.range_history)
  if math.isnan(slope) or slope < HELD_VREL_SLOPE_MIN or slope - track.vRel < HELD_VREL_DISAGREE:
    return False
  return not urgent({'dRel': track.dRel, 'vRel': track.vRel}, urgency_v_rel(track))


def held_vrel_active(track: Track | None) -> bool:
  return track is not None and track.held_vrel_cycles * DT_MDL >= HELD_VREL_MIN_S - 1e-9


def held_vrel_bound(lead: dict[str, Any], track: Track) -> dict[str, Any]:
  """Raise the published vRel/vLead/vLeadK to min(range slope, 0) - HELD_VREL_MARGIN; never lowers, nothing else moves."""
  v_bound = min(range_slope(track.range_history), 0.0) - HELD_VREL_MARGIN
  delta = v_bound - float(lead['vRel'])
  if not delta > 0.0:
    return lead
  out = dict(lead)
  for key in ('vRel', 'vLead', 'vLeadK'):
    out[key] = float(lead[key]) + delta
  return out


def weak_lead_distance_ratio(track: Track, lead: capnp._DynamicStructReader, model_v_ego: float) -> float:
  # the CameraWitness weak-tell distance tell: how far the model lead is past this track, while it calls for no braking
  return camera_passed_ratio(track.dRel, lead.x[0] - RADAR_TO_CAMERA, model_v_ego - lead.v[0], lead.a[0])


def weak_lead_contradiction(track: Track, camera_ranges, lead: capnp._DynamicStructReader, model_v_ego: float) -> float:
  # the CameraWitness weak-tell contradiction tell: how much faster this track closes than the model lead's range does
  return camera_contradiction(track.vRel, range_slope(track.range_history), range_slope(camera_ranges),
                              model_v_ego - lead.v[0], lead.a[0])


def in_path_radar_keep(preferred_track: Track | None, lead: capnp._DynamicStructReader) -> bool:
  # See the KEEP_Y_MAX/KEEP_VREL_MAX evidence block above. Only reached once the relaxed vision match has already
  # failed; keeps a preferred track that is still in lane, still closer than the vision lead, and still closing.
  if preferred_track is None or preferred_track.cnt < 3:
    return False
  if abs(preferred_track.yRel) >= KEEP_Y_MAX:
    return False
  if preferred_track.dRel >= lead.x[0] - RADAR_TO_CAMERA:
    return False
  if preferred_track.vLead <= KEEP_VLEAD_MIN:
    return False
  return preferred_track.vRel < KEEP_VREL_MAX


def match_vision_to_track(
  v_ego: float,
  lead: capnp._DynamicStructReader,
  tracks: dict[int, Track],
  preferred_track_id: int = -1,
):
  if not tracks:
    return None
  track = max(tracks.values(), key=lambda candidate: vision_track_probability(candidate, lead, v_ego))
  if track_matches_vision(track, lead, v_ego, dist_scale=0.25, dist_floor=5.0, vel_limit=10.0, y_std_scale=1.0, y_floor=1.0):
    return track
  preferred_track = tracks.get(preferred_track_id)
  if preferred_track is not None and preferred_track.cnt >= 3:
    if track_matches_vision(preferred_track, lead, v_ego, dist_scale=0.4, dist_floor=8.0, vel_limit=13.0, y_std_scale=2.0, y_floor=1.5):
      return preferred_track
  if in_path_radar_keep(preferred_track, lead):
    return preferred_track
  return None


def get_RadarState_from_vision(lead_msg: capnp._DynamicStructReader, v_ego: float, model_v_ego: float, model_prob: float, vision_state):
  prev_aLeadK = getattr(vision_state, 'prev_aLeadK', 0.0)
  blended_aLeadK = 0.8 * float(lead_msg.a[0]) + 0.2 * prev_aLeadK
  vision_state.prev_aLeadK = blended_aLeadK
  return {
    'dRel': float(lead_msg.x[0] - RADAR_TO_CAMERA),
    'yRel': float(-lead_msg.y[0]),
    'vRel': float(lead_msg.v[0] - model_v_ego),
    'vLead': float(v_ego + (lead_msg.v[0] - model_v_ego)),
    'vLeadK': float(v_ego + (lead_msg.v[0] - model_v_ego)),
    'aLeadK': blended_aLeadK,
    'aLeadTau': 0.3,
    'deprecated': {'fcw': False},
    'modelProb': float(model_prob),
    'present': True,
    'radar': False,
    'radarTrackId': -1,
  }


def get_lead(
  v_ego: float,
  ready: bool,
  tracks: dict[int, Track],
  lead_msg: capnp._DynamicStructReader,
  model_v_ego: float,
  starpilot_toggles: SimpleNamespace,
  low_speed_override: bool = True,
  lead_prob: float | None = None,
  preferred_track_id: int = -1,
  *,
  vision_state,
) -> dict[str, Any]:
  lead_detection_probability = float(getattr(starpilot_toggles, 'lead_detection_probability', 0.35))
  filtered_lead_prob = float(lead_msg.prob if lead_prob is None else lead_prob)
  if len(tracks) > 0 and ready and (filtered_lead_prob > lead_detection_probability):
    track = match_vision_to_track(v_ego, lead_msg, tracks, preferred_track_id=preferred_track_id)
  else:
    track = None
  lead_dict = {'present': False}
  if track is not None:
    lead_dict = track.get_RadarState(filtered_lead_prob)
  elif track is None and ready and (filtered_lead_prob > lead_detection_probability):
    lead_dict = get_RadarState_from_vision(lead_msg, v_ego, model_v_ego, filtered_lead_prob, vision_state)
  if low_speed_override:
    low_speed_tracks = [c for c in tracks.values() if honda_bosch_a_low_speed_radar_lead_sane(c, v_ego)]
    model_lead_available = ready and filtered_lead_prob > lead_detection_probability
    preferred_track = tracks.get(preferred_track_id)
    if preferred_track is not None and honda_bosch_a_low_speed_radar_lead_sane(preferred_track, v_ego):
      preferred_matches_model = not model_lead_available or track_matches_vision(
        preferred_track, lead_msg, v_ego, dist_scale=0.25, dist_floor=5.0, vel_limit=10.0, y_std_scale=1.0, y_floor=1.0
      )
      preferred_is_current = (
        not lead_dict.get('present', False)
        or lead_dict.get('radarTrackId', -1) == preferred_track_id
        or (lead_dict.get('present', False) and (not lead_dict.get('radar', False)))
      )
      if preferred_is_current and preferred_matches_model:
        lead_dict = preferred_track.get_RadarState(filtered_lead_prob)

    def candidate_is_established(candidate: Track) -> bool:
      if candidate.cnt < HONDA_BOSCH_A_LOW_SPEED_MIN_COUNT:
        return False
      if not lead_dict.get('present', False):
        return True
      if lead_dict.get('radarTrackId', -1) == candidate.identifier:
        return True
      return model_lead_available and track_matches_vision(
        candidate, lead_msg, v_ego, dist_scale=0.25, dist_floor=5.0, vel_limit=10.0, y_std_scale=1.0, y_floor=1.0
      )

    low_speed_tracks = [c for c in low_speed_tracks if candidate_is_established(c)]
    if len(low_speed_tracks) > 0:
      closest_track = min(low_speed_tracks, key=lambda c: c.dRel)
      if not lead_dict['present'] or closest_track.dRel < lead_dict['dRel']:
        lead_dict = closest_track.get_RadarState()
  return lead_dict


class BoschALeadPolicy:
  def _reset_preferred_stale_evidence(self, lead_index: int, track_id: int = -1) -> None:
    self.preferred_stale_track_ids[lead_index] = track_id
    self.preferred_challenger_stale_counts[lead_index] = 0
    self.preferred_gross_distance_stale_counts[lead_index] = 0

  def _update_honda_bosch_a_preferred_staleness(self, lead_index: int, lead: capnp._DynamicStructReader, lead_prob: float) -> None:
    preferred_id = self.prev_lead_track_ids[lead_index]
    if self.preferred_stale_track_ids[lead_index] != preferred_id:
      self._reset_preferred_stale_evidence(lead_index, preferred_id)
    lead_detection_probability = float(getattr(self.starpilot_toggles, 'lead_detection_probability', 0.35))
    preferred_track = self.tracks.get(preferred_id)
    if preferred_id < 0 or preferred_track is None or (not self.ready) or (lead_prob <= lead_detection_probability):
      self._reset_preferred_stale_evidence(lead_index, preferred_id)
      return
    strict_match = track_matches_vision(preferred_track, lead, self.v_ego, dist_scale=0.25, dist_floor=5.0, vel_limit=10.0, y_std_scale=1.0, y_floor=1.0)
    relaxed_match = track_matches_vision(preferred_track, lead, self.v_ego, dist_scale=0.4, dist_floor=8.0, vel_limit=13.0, y_std_scale=2.0, y_floor=1.5)
    if relaxed_match:
      self.preferred_challenger_stale_counts[lead_index] = 0
    else:
      best_track = max(self.tracks.values(), key=lambda candidate: vision_track_probability(candidate, lead, self.v_ego))
      preferred_score = vision_track_probability(preferred_track, lead, self.v_ego)
      best_score = vision_track_probability(best_track, lead, self.v_ego)
      # A challenger that only scores better (no strict match) must not retire a track the in-path keep holds; a
      # challenger that matches strictly already wins in match_vision_to_track.
      if best_track.identifier != preferred_id and best_score > preferred_score and not in_path_radar_keep(preferred_track, lead):
        self.preferred_challenger_stale_counts[lead_index] += 1
      else:
        self.preferred_challenger_stale_counts[lead_index] = 0
    distance_mismatch = abs(preferred_track.dRel - (lead.x[0] - RADAR_TO_CAMERA))
    if strict_match:
      self.preferred_gross_distance_stale_counts[lead_index] = 0
    elif distance_mismatch > HONDA_BOSCH_A_GROSS_DISTANCE_M and not in_path_radar_keep(preferred_track, lead):
      # Gross-distance staleness is exactly "vision disagrees with the track by a lot"; that is the case the
      # in-path keep exists for, so it must not count against the track while the keep conditions hold.
      self.preferred_gross_distance_stale_counts[lead_index] += 1
    else:
      self.preferred_gross_distance_stale_counts[lead_index] = 0
    challenger_stale = self.preferred_challenger_stale_counts[lead_index] >= HONDA_BOSCH_A_CHALLENGER_STALE_CYCLES
    distance_stale = self.preferred_gross_distance_stale_counts[lead_index] >= HONDA_BOSCH_A_GROSS_DISTANCE_STALE_CYCLES
    if challenger_stale or distance_stale:
      self.prev_lead_track_ids[lead_index] = -1
      self._reset_preferred_stale_evidence(lead_index)

  def _prepare_post_standstill_gate(self, standstill: bool) -> None:
    if standstill:
      self._post_standstill_gate_active = False
      self._post_standstill_candidate_id = -1
      self._post_standstill_candidate_frames = 0
    elif self._was_standstill and (not self._standstill_had_lead):
      self._post_standstill_gate_active = True
      self._post_standstill_candidate_id = -1
      self._post_standstill_candidate_frames = 0

  def _filter_post_standstill_lead(self, lead: dict[str, Any]) -> dict[str, Any]:
    if not self._post_standstill_gate_active:
      return lead
    model_lead = float(lead.get('modelProb', 0.0)) > float(getattr(self.starpilot_toggles, 'lead_detection_probability', 0.35))
    radar_only = bool(lead.get('present', False) and lead.get('radar', False) and (not model_lead))
    if not radar_only:
      if lead.get('present', False):
        self._post_standstill_gate_active = False
      self._post_standstill_candidate_id = -1
      self._post_standstill_candidate_frames = 0
      return lead
    track_id = int(lead.get('radarTrackId', -1))
    if track_id == self._post_standstill_candidate_id:
      self._post_standstill_candidate_frames += 1
    else:
      self._post_standstill_candidate_id = track_id
      self._post_standstill_candidate_frames = 1
    persistent = self._post_standstill_candidate_frames >= POST_STANDSTILL_RADAR_LEAD_PERSISTENCE_FRAMES
    if persistent or post_standstill_radar_lead_is_urgent(lead):
      self._post_standstill_gate_active = False
      return lead
    return {'present': False}

  def _remember_post_standstill_state(self, standstill: bool, lead_status: bool) -> None:
    if standstill:
      self._was_standstill = True
      self._standstill_had_lead |= lead_status
    else:
      self._was_standstill = False
      self._standstill_had_lead = False

  def __init__(self, *, xcp_assist: bool = False, xcp_path: Path | None = XCP_SHM_PATH):
    self.starpilot_toggles = SimpleNamespace(lead_detection_probability=0.35)
    self.vision_state = SimpleNamespace(prev_aLeadK=0.0)
    self.prev_lead_track_ids = [-1, -1]
    self.preferred_stale_track_ids = [-1, -1]
    self.preferred_challenger_stale_counts = [0, 0]
    self.preferred_gross_distance_stale_counts = [0, 0]
    self._was_standstill = False
    self._standstill_had_lead = False
    self._post_standstill_gate_active = False
    self._post_standstill_candidate_id = -1
    self._post_standstill_candidate_frames = 0
    self._last_tracks_frame = -1
    self.camera_witness = CameraWitness()  # C-012: the 1 s tapered cap hold is always on
    self.camera_slow_bound = CameraSlowBound()
    self.camera_ranges: list[deque[float]] = [deque(maxlen=TREND_SAMPLES) for _ in range(2)]  # model lead x per cycle
    self.xcp_assist = bool(xcp_assist)
    self._xcp_path = Path(xcp_path) if xcp_path is not None else None
    self._xcp_file_sig: tuple[int, int] | None = None
    self._xcp_snapshot: dict[str, Any] | None = None
    self._replay_xcp_snapshot: dict[str, Any] | None = None

  def set_xcp_snapshot(self, snapshot: dict[str, Any] | None, *, mono_ns: int | None = None) -> None:
    """Inject an XCP snapshot (from rlog xcp_array_b_20hz or /dev/shm JSON) for offline replay/tests."""
    if snapshot is None:
      self._replay_xcp_snapshot = None
      return
    snap_mono = snapshot.get("mono_ns", 0) if mono_ns is None else mono_ns
    targets = list(snapshot.get("targets") or ())
    by_id = dict(snapshot.get("tracks_by_op_id") or {})
    if not by_id and targets:
      for t in targets:
        op_tid = t.get("op_track_id")
        if op_tid is not None and t.get("stage") == "measured" and not t.get("ghost", False):
          by_id[str(op_tid)] = t
    self._replay_xcp_snapshot = {"mono_ns": int(snap_mono), "tracks_by_op_id": by_id, "targets": targets}

  def _read_xcp_snapshot(self, now_ns: int, *, used_boot_clock: bool = False) -> dict[str, Any] | None:
    if not self.xcp_assist:
      return None
    from_disk = self._replay_xcp_snapshot is None
    snap = self._replay_xcp_snapshot
    if snap is None:
      if self._xcp_path is None:
        return None
      try:
        st = self._xcp_path.stat()
      except OSError:
        self._xcp_file_sig = None
        self._xcp_snapshot = None
        return None
      sig = (int(getattr(st, "st_ino", 0)), int(st.st_mtime_ns))
      if sig != self._xcp_file_sig:
        try:
          data = json.loads(self._xcp_path.read_bytes())
          self._xcp_file_sig = sig
          self._xcp_snapshot = data if isinstance(data, dict) else None
        except Exception:
          self._xcp_file_sig = None
          self._xcp_snapshot = None
          return None
      snap = self._xcp_snapshot
    if snap is None:
      return None
    try:
      snap_mono = int(snap.get("mono_ns", 0))
      if from_disk and used_boot_clock:
        snap_mono += int(snap.get("boot_offset_ns", 0) or 0)
    except (TypeError, ValueError):
      return None
    if not (-XCP_ASSIST_MAX_FUTURE_SKEW_NS <= (now_ns - snap_mono) <= XCP_ASSIST_MAX_SNAPSHOT_AGE_NS):
      return None
    return snap

  @staticmethod
  def _match_xcp_targets_one_to_one(
    points: dict[int, Any],
    xcp_snapshot: dict[str, Any] | None,
    v_ego: float,
    *,
    locked_slot_keys: dict[int, tuple[Any, Any]] | None = None,
  ) -> dict[int, dict[str, Any]]:
    if xcp_snapshot is None or not points:
      return {}
    targets_raw = [t for t in (xcp_snapshot.get("targets") or ()) if isinstance(t, dict)]
    candidates_list: list[dict[str, Any]] = list(targets_raw)
    seen_keys: set[tuple[Any, Any]] = {
      (t.get("slot"), t.get("rtid", t.get("radar_track_id")))
      for t in candidates_list
      if t.get("slot") is not None
    }
    by_id = xcp_snapshot.get("tracks_by_op_id")
    if isinstance(by_id, dict):
      for tid_str, dt in by_id.items():
        if not isinstance(dt, dict):
          continue
        skey = (dt.get("slot"), dt.get("rtid", dt.get("radar_track_id")))
        if dt.get("slot") is not None and skey in seen_keys:
          continue
        dt_copy = dict(dt)
        if dt_copy.get("op_track_id") is None:
          try:
            dt_copy["op_track_id"] = int(tid_str)
          except (TypeError, ValueError):
            pass
        candidates_list.append(dt_copy)
        if dt_copy.get("slot") is not None:
          seen_keys.add(skey)

    pairs: list[tuple[int, float, int, int]] = []
    for identifier, point in points.items():
      locked_sk = locked_slot_keys.get(identifier) if locked_slot_keys else None
      for ti, t in enumerate(candidates_list):
        ok, cost = _xcp_target_qualifies(t, point, v_ego, locked_slot_key=locked_sk)
        if not ok:
          continue
        op_tid = t.get("op_track_id")
        try:
          op_confirmed = op_tid is not None and int(op_tid) == int(identifier)
        except (TypeError, ValueError):
          op_confirmed = False
        pairs.append((0 if op_confirmed else 1, cost, int(identifier), ti))
    pairs.sort(key=lambda item: item)

    used_points: set[int] = set()
    used_targets: set[int] = set()
    used_slots: set[Any] = set()
    matched: dict[int, dict[str, Any]] = {}
    for _, _, identifier, ti in pairs:
      if identifier in used_points or ti in used_targets:
        continue
      t = candidates_list[ti]
      slot = t.get("slot")
      if slot is not None and slot in used_slots:
        continue
      used_points.add(identifier)
      used_targets.add(ti)
      if slot is not None:
        used_slots.add(slot)
      matched[identifier] = t
    return matched

  @classmethod
  def _match_xcp_for_point(cls, point, xcp_snapshot: dict[str, Any] | None, v_ego: float) -> dict[str, Any] | None:
    if point is None or xcp_snapshot is None:
      return None
    return cls._match_xcp_targets_one_to_one({int(point.trackId): point}, xcp_snapshot, v_ego).get(int(point.trackId))

  def update_tracks(self, sm, rr, radar):
    self.ready = radar.ready
    self.v_ego = radar.v_ego
    self.tracks = radar.tracks
    self._prepare_post_standstill_gate(bool(sm['carState'].standstill))
    fresh = sm.recv_frame['radarTracks'] != self._last_tracks_frame
    self._last_tracks_frame = sm.recv_frame['radarTracks']
    points = {point.trackId: point for point in rr.points}
    statuses = radar_point_status.read(sm)
    for identifier in list(self.tracks):
      if identifier not in points:
        del self.tracks[identifier]
    if len(sm['modelV2'].leadsV3) < 2:
      self._remember_post_standstill_state(bool(sm['carState'].standstill), False)
    log_times = getattr(sm, 'logMonoTime', None)
    now_ns = max(
      int(log_times.get('modelV2', 0) or 0),
      int(log_times.get('radarTracks', 0) or 0),
    ) if isinstance(log_times, dict) else 0
    used_boot_clock = now_ns > 0
    if now_ns <= 0:
      now_ns = time.monotonic_ns()
    xcp_snapshot = self._read_xcp_snapshot(now_ns, used_boot_clock=used_boot_clock) if self.xcp_assist else None
    locked_slot_keys = (
      {
        ident: tr.xcp_slot_key
        for ident, tr in self.tracks.items()
        if tr.xcp_active and tr.xcp_slot_key is not None
      }
      if xcp_snapshot is not None
      else None
    )
    matches = (
      self._match_xcp_targets_one_to_one(points, xcp_snapshot, self.v_ego, locked_slot_keys=locked_slot_keys)
      if xcp_snapshot is not None
      else {}
    )
    for identifier, point in points.items():
      v_lead_base = float(point.vRel + radar.v_ego_hist[0])
      measured_base = bool(point.deprecated.measured and fresh)
      existing_track = self.tracks.get(identifier)
      if not self.xcp_assist:
        if existing_track is None:
          existing_track = Track(identifier, v_lead_base, radar.kalman_params)
          self.tracks[identifier] = existing_track
        track = existing_track
        track.xcp_active = False
        track.update(point.dRel, point.yRel, point.vRel, v_lead_base, measured_base, measured_base)
        if not point.deprecated.measured:
          # Fork seam, the one line not from bob-unify-car: upstream's parser merge (5e44b4bd34) has it and pins it in
          # test_bosch_a_integration. A range-based coast can close faster than its frozen velocity filter.
          track.vLeadK = min(track.vLeadK, track.vLead)
      else:
        if existing_track is None:
          existing_track = Track(identifier, v_lead_base, radar.kalman_params)
          self.tracks[identifier] = existing_track
        track = existing_track
        if measured_base:
          if track.base_cnt > 0:
            track.base_kf.update(v_lead_base)
          track.base_cnt += 1
        xcp_match = matches.get(identifier)
        if xcp_match is not None:
          slot_key = (xcp_match.get("slot"), xcp_match.get("rtid", xcp_match.get("radar_track_id")))
          kin_ts = xcp_match.get("kin_updated_ns")
          if kin_ts is not None:
            try:
              kin_ts_int = int(kin_ts)
            except (TypeError, ValueError):
              kin_ts_int = 0
          else:
            kin_ts_int = 0
          kin_sig = (slot_key, kin_ts_int) if kin_ts_int > 0 else (
            slot_key, xcp_match.get("dRel_m"), xcp_match.get("vRel_mps"), xcp_match.get("ax_mps2")
          )
          v_rel_raw = float(xcp_match["vRel_mps"])
          v_lead_raw = v_rel_raw + float(radar.v_ego_hist[0])
          handoff = (not track.xcp_active) or (track.xcp_slot_key != slot_key)
          if handoff:
            init_a = float(track.base_kf.x[ACCEL][0]) if track.cnt > 0 else 0.0
            track.xcp_var_kf.reset(v_lead_raw, init_a)
            track.kf.set_x([[float(v_lead_raw)], [init_a]])
            track.xcp_slot_key = slot_key
            track.xcp_kin_sig = kin_sig
            track.xcp_last_kin_ts_ns = kin_ts_int
            track.xcp_v_lead = float(v_lead_raw)
            track.xcp_active = True
            track.last_xcp_match = xcp_match
            track.update(point.dRel, point.yRel, v_rel_raw, v_lead_raw, measured_base, False)
            if track.cnt == 0 and measured_base:
              track.cnt = 1
          elif kin_sig != track.xcp_kin_sig:
            if kin_ts_int > 0 and track.xcp_last_kin_ts_ns > 0 and kin_ts_int > track.xcp_last_kin_ts_ns:
              dt_real = max(
                XCP_ASSIST_KF_MIN_DT_S,
                min(XCP_ASSIST_KF_MAX_DT_S, (kin_ts_int - track.xcp_last_kin_ts_ns) * 1e-9),
              )
            else:
              dt_real = DT_MDL
            max_neg_step = max(
              XCP_ASSIST_MAX_STEP_DV_MPS,
              min(XCP_ASSIST_MAX_STEP_DV_CAP_MPS, XCP_ASSIST_MAX_STEP_DECEL_MPS2 * dt_real),
            )
            vk, ak = track.xcp_var_kf.update(v_lead_raw, dt_real, max_neg_step=max_neg_step)
            track.xcp_kin_sig = kin_sig
            track.xcp_last_kin_ts_ns = kin_ts_int
            track.xcp_v_lead = float(v_lead_raw)
            track.xcp_active = True
            track.last_xcp_match = xcp_match
            track.update(point.dRel, point.yRel, v_rel_raw, v_lead_raw, measured_base, True, xcp_kf_state=(vk, ak))
          else:
            v_lead_hold = float(track.xcp_v_lead)
            v_rel_hold = v_lead_hold - float(radar.v_ego_hist[0])
            track.xcp_active = True
            track.last_xcp_match = xcp_match
            track.update(point.dRel, point.yRel, v_rel_hold, v_lead_hold, measured_base, False)
          a_base = float(track.base_kf.x[ACCEL][0]) if track.base_cnt > 0 else 0.0
          if a_base > -2.0 and track.aLeadK <= -2.0:
            track.aLeadK = XCP_ASSIST_LATCH_GUARD_ACCEL
        else:
          if track.xcp_active:
            track.kf.set_x([[float(track.base_kf.x[SPEED][0])], [float(track.base_kf.x[ACCEL][0])]])
            track.xcp_active = False
            track.last_xcp_match = None
            track.xcp_slot_key = None
            track.xcp_kin_sig = None
            track.xcp_last_kin_ts_ns = 0
            track.update(point.dRel, point.yRel, point.vRel, v_lead_base, measured_base, False)
            if track.cnt == 0 and measured_base:
              track.cnt = 1
          else:
            track.xcp_active = False
            track.update(point.dRel, point.yRel, point.vRel, v_lead_base, measured_base, measured_base)
      track.direct = bool(point.deprecated.measured)
      if track.direct:
        track.last_direct_vrel = float(point.vRel)
        track.direct_age = 0
      else:
        track.direct_age += 1
      track.held_vrel_cycles = track.held_vrel_cycles + 1 if held_vrel_condition(track) else 0
      self.tracks[identifier].objectStatus = statuses.get(identifier, 0)

  def leads(self, sm, radar, model_v_ego):
    leads = sm['modelV2'].leadsV3
    for index in range(2):
      self._update_honda_bosch_a_preferred_staleness(index, leads[index], radar.lead_prob_filters[index].x)
      self.camera_ranges[index].append(float(leads[index].x[0]) if leads[index].prob >= CAMERA_PROB_MIN else math.nan)
    output = [
      get_lead(
        self.v_ego,
        self.ready,
        self.tracks,
        leads[index],
        model_v_ego,
        self.starpilot_toggles,
        low_speed_override=index == 0,
        lead_prob=radar.lead_prob_filters[index].x,
        preferred_track_id=self.prev_lead_track_ids[index],
        vision_state=self.vision_state,
      )
      for index in range(2)
    ]
    output[0] = self._filter_post_standstill_lead(output[0])
    lead_probs = [radar.lead_prob_filters[index].x for index in range(2)]
    lead_tracks = [self.tracks.get(int(lead.get('radarTrackId', -1))) if lead.get('present', False) and lead.get('radar', False)
                   else None for lead in output]
    if HELD_VREL_SLOPE_BOUND_ENABLED:
      # Before the witness stages see the lead; track.vRel stays raw (the witness's camera_contradiction reads it).
      output = [held_vrel_bound(lead, track) if held_vrel_active(track) else lead for lead, track in zip(output, lead_tracks, strict=False)]  # ty: ignore[invalid-argument-type]
    urgent_v_rels = [urgency_v_rel(track) for track in lead_tracks]
    output = self.camera_witness.apply(
      output,
      leads,
      self.v_ego,
      model_v_ego,
      lead_probs,
      self.tracks,
      [track.objectStatus if track is not None else 0 for track in lead_tracks],
      [weak_lead_distance_ratio(track, leads[index], model_v_ego) if track is not None else 0.0
       for index, track in enumerate(lead_tracks)],
      [weak_lead_contradiction(track, self.camera_ranges[index], leads[index], model_v_ego) if track is not None else 0.0
       for index, track in enumerate(lead_tracks)],
      self.camera_ranges,
      urgent_v_rels,
    )
    output = self.camera_slow_bound.apply(output, leads, self.v_ego, model_v_ego, lead_probs)
    for index, lead in enumerate(output):
      if lead.get('present', False) and lead.get('radar', False):
        identifier = int(lead.get('radarTrackId', -1))
        if identifier != self.prev_lead_track_ids[index]:
          self._reset_preferred_stale_evidence(index, identifier)
        self.prev_lead_track_ids[index] = identifier
      elif not lead.get('present', False) or self.prev_lead_track_ids[index] not in self.tracks:
        self.prev_lead_track_ids[index] = -1
        self._reset_preferred_stale_evidence(index)
    self._remember_post_standstill_state(bool(sm['carState'].standstill), bool(output[0].get('present', False)))
    return output
