"""Display the lane contribution that survives the host curvature limiter, or the side Blendful steers toward."""

import math
from pathlib import Path

from openpilot.cereal import messaging
from openpilot.starpilot.lateral.lane_centering import MAX_CORRECTION

SERVICE = 'starpilotLateralState'
MAX_AGE_NS = 250_000_000
MAX_PAIR_SKEW_NS = 100_000_000
DIRECTION_EPSILON = 1e-6
BLUE = (0, 176, 220)
BLENDFUL_DIRECTION_KEY = 'LaneCenteringBlendfulDirection'  # written by modeld, blendful_runtime.BlendfulHost
BLENDFUL_POLL_NS = 100_000_000
_blendful = {'params': None, 'read_ns': None, 'direction': 0}


def feedback_message(result, applied, lateral_active, *, model_mono_time, car_control_mono_time, valid, inactive_reason='inactive'):
  event = messaging.new_message(SERVICE, valid=valid)
  state = event.starpilotLateralState.laneCentering
  state.version = 1
  state.modelMonoTime = model_mono_time
  state.carControlMonoTime = car_control_mono_time
  state.lateralActive = lateral_active
  state.requestedCorrection = result.correction if result is not None else 0.0
  state.appliedCorrection = applied if lateral_active else 0.0
  state.reason = result.reason if result is not None else inactive_reason
  return event


def _saved_blendful_direction() -> int:
  if _blendful['params'] is None:
    from openpilot.common.params import Params
    _blendful['params'] = Params()
  return {b'-1': -1, b'1': 1}.get(Path(_blendful['params'].get_param_path(BLENDFUL_DIRECTION_KEY)).read_bytes(), 0)


def blendful_direction(sm, now_ns: int, after_frame: int, *, read=_saved_blendful_direction) -> int:
  """The side modeld's Blendful correction steers toward, while this drive's model and lateral control are live."""
  try:
    for service in ('carControl', 'modelV2'):
      stamp = int(sm.logMonoTime[service])
      if (not sm.alive[service] or not sm.valid[service] or sm.recv_frame[service] <= after_frame or
          not 0 < stamp <= now_ns or now_ns - stamp > MAX_AGE_NS):
        return 0
    if not sm['carControl'].latActive:
      return 0
    last = _blendful['read_ns']
    if last is None or not 0 <= now_ns - last < BLENDFUL_POLL_NS:
      _blendful['read_ns'] = now_ns
      try:
        _blendful['direction'] = read()
      except Exception:
        _blendful['direction'] = 0  # no saved side, an unregistered key or no params on this host
    return _blendful['direction']
  except (AttributeError, KeyError, TypeError, ValueError, OverflowError):
    return 0


def direction(sm, now_ns: int, after_frame: int) -> int:
  """-1 left line blue, +1 right: the classic correction's applied side, else Blendful's (the two never run together)."""
  return _classic_direction(sm, now_ns, after_frame) or blendful_direction(sm, now_ns, after_frame)


def _classic_direction(sm, now_ns: int, after_frame: int) -> int:
  try:
    clocks = {}
    for service in (SERVICE, 'carControl', 'modelV2'):
      stamp = int(sm.logMonoTime[service])
      if (not sm.alive[service] or not sm.valid[service] or sm.recv_frame[service] <= after_frame or
          not 0 < stamp <= now_ns or now_ns - stamp > MAX_AGE_NS):
        return 0
      clocks[service] = stamp
    state = sm[SERVICE].laneCentering
    requested, applied = float(state.requestedCorrection), float(state.appliedCorrection)
    if (state.version != 1 or not state.lateralActive or not sm['carControl'].latActive or
        not 0 < state.modelMonoTime <= now_ns or now_ns - state.modelMonoTime > MAX_AGE_NS or
        not 0 < state.carControlMonoTime <= clocks[SERVICE] or
        abs(state.carControlMonoTime - clocks['carControl']) > MAX_PAIR_SKEW_NS or
        abs(state.modelMonoTime - clocks['modelV2']) > MAX_PAIR_SKEW_NS or
        not math.isfinite(requested) or not math.isfinite(applied) or
        abs(requested) > MAX_CORRECTION + 1e-8 or abs(applied) > abs(requested) + 1e-8 or
        requested * applied < 0):
      return 0
    return (applied > DIRECTION_EPSILON) - (applied < -DIRECTION_EPSILON)
  except (AttributeError, KeyError, TypeError, ValueError, OverflowError):
    return 0
