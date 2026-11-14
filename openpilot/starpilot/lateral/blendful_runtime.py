"""Saved Blendful preference and the one modeld join that applies its curvature correction."""

from pathlib import Path

from openpilot.starpilot.lateral.blendful import Blendful

KEY = 'LaneCenteringBlendful'
CLASSIC_KEY = 'LaneCentering'
REFRESH_FRAMES = 20  # 1 s of model frames


def enabled(params) -> bool:
  """Blendful replaces the classic correction, never stacks on it: both saved on leaves Blendful off."""
  try:
    def raw(key):
      try:
        return Path(params.get_param_path(key)).read_bytes()
      except FileNotFoundError:
        return None
    return raw(KEY) == b'1' and raw(CLASSIC_KEY) != b'1'
  except Exception:
    # An unregistered key (a stale params build) or unreadable storage must not enable a correction.
    return False


class BlendfulHost:
  """Keeps the correction out of the model's own smoothing state and adds it to the published action."""

  def __init__(self, params):
    self.params = params
    self.blendful = Blendful()
    self.enabled = False
    self.frame = 0
    self.applied = 0.0

  def strip(self, prev_action, lat_active: bool) -> None:
    """Before the model's action call: the previous action it smooths from is its own, without the correction."""
    if lat_active and self.applied:
      prev_action.desiredCurvature = float(prev_action.desiredCurvature) - self.applied
    self.applied = 0.0

  def apply(self, action, model_output, v_ego: float, active: bool) -> float:
    """After the model's action is final: add the correction and remember exactly what was added."""
    if self.frame % REFRESH_FRAMES == 0:
      self.enabled = enabled(self.params)
    self.frame += 1
    base = float(action.desiredCurvature)
    correction = self.blendful.update(model_output, v_ego, active, base, self.enabled)
    if correction:
      action.desiredCurvature = base + correction
    self.applied = float(action.desiredCurvature) - base
    return self.applied
