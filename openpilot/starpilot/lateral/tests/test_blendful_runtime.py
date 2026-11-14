"""The modeld join for Blendful: the saved choice, and a correction that never enters the model's own smoothing state."""
from types import SimpleNamespace

import numpy as np
import pytest

from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.starpilot.lateral.blendful_runtime import BlendfulHost, CLASSIC_KEY, DIRECTION_KEY, KEY, REFRESH_FRAMES, enabled

X = np.array(ModelConstants.X_IDXS)


class SavedParams:
  def __init__(self, root, registered=(KEY, CLASSIC_KEY, DIRECTION_KEY)):
    self.root, self.registered, self.writes = root, registered, []

  def put(self, key, value):
    self.get_param_path(key)
    self.writes.append((key, value))

  def get_param_path(self, key):
    if key not in self.registered:
      raise KeyError(key)  # stands in for UnknownKeyName on a params build that predates the key
    return str(self.root / key)

  def save(self, **values):
    for key, value in values.items():
      (self.root / key).write_bytes(value)


def frame(center):
  lines = np.zeros((1, 4, 33, 2))
  stds = np.ones((1, 4, 33, 2))
  for i, offset in enumerate((-5.25, -1.75, 1.75, 5.25)):
    lines[0, i, :, 0] = center + offset
  stds[0, 1:3, :, 0] = 0.1
  probs = np.zeros((1, 8))
  probs[0, 3] = probs[0, 5] = 0.9
  plan = np.zeros((1, 33, 3))
  plan[0, :, 0] = X
  return {'lane_lines': lines, 'lane_lines_stds': stds, 'lane_lines_prob': probs, 'plan': plan}


@pytest.mark.parametrize('saved, expected', [
  ({}, False),
  ({KEY: b'0'}, False),
  ({KEY: b'1'}, True),
  ({KEY: b'1', CLASSIC_KEY: b'0'}, True),
  ({KEY: b'1', CLASSIC_KEY: b'1'}, False),  # the classic correction keeps the wheel; the two never stack
  ({KEY: b'true'}, False),
])
def test_saved_choice(tmp_path, saved, expected):
  params = SavedParams(tmp_path)
  params.save(**saved)
  assert enabled(params) is expected


def test_unregistered_key_is_off(tmp_path):
  params = SavedParams(tmp_path, registered=(CLASSIC_KEY,))
  (tmp_path / KEY).write_bytes(b'1')
  assert enabled(params) is False


def run(host, seconds, *, center=0.5, v=20.0, active=True, model=0.0, tau=0.0):
  """modeld's join: the model smooths from its own previous action, then the correction is added."""
  prev = SimpleNamespace(desiredCurvature=0.0)
  published, model_only = [], []
  alpha = 1 - np.exp(-DT_MDL / tau) if tau > 0 else 1
  for _ in range(int(seconds / DT_MDL)):
    host.strip(prev, True)
    action = SimpleNamespace(desiredCurvature=alpha * model + (1 - alpha) * prev.desiredCurvature)
    model_only.append(action.desiredCurvature)
    prev = action
    host.apply(action, frame(center), v, active)
    published.append(action.desiredCurvature)
  return np.array(published), np.array(model_only)


def test_off_changes_nothing(tmp_path):
  host = BlendfulHost(SavedParams(tmp_path))
  published, model_only = run(host, 5.0, model=0.001)
  assert np.array_equal(published, model_only)
  assert host.applied == 0.0


def test_on_pulls_toward_the_lane_center_and_stays_out_of_the_models_smoothing(tmp_path):
  params = SavedParams(tmp_path)
  params.save(**{KEY: b'1'})
  host = BlendfulHost(params)
  published, model_only = run(host, 5.0, model=0.001, tau=0.5)
  correction = published - model_only
  assert correction[-1] * 20.0 ** 2 > 0.02  # m/s^2 toward a lane center 0.5 m to the right
  assert np.all(np.diff(correction) * 20.0 ** 2 <= 0.5 * DT_MDL + 1e-9)  # builds at Blendful's own jerk limit
  # the model's smoothed action converges on the model's demand alone; the correction does not compound through it
  assert model_only[-1] == pytest.approx(0.001, rel=1e-3)


def test_inactive_releases_and_live_toggle_off_drops_the_correction(tmp_path):
  params = SavedParams(tmp_path)
  params.save(**{KEY: b'1'})
  host = BlendfulHost(params)
  run(host, 5.0)
  assert host.applied > 0
  published, model_only = run(host, 3.0, active=False)  # lane change, navigation desire or lateral off
  assert abs(published[-1] - model_only[-1]) * 20.0 ** 2 < 0.005  # m/s^2: released, the tail is Blendful's own low-pass
  run(host, 5.0)
  params.save(**{KEY: b'0'})
  published, model_only = run(host, (REFRESH_FRAMES + 1) * DT_MDL)
  assert published[-1] == model_only[-1] and host.applied == 0.0


def test_publishes_the_side_it_steers_toward_only_when_it_changes(tmp_path):
  params = SavedParams(tmp_path)
  params.save(**{KEY: b'1'})
  host = BlendfulHost(params)
  run(host, 5.0, center=0.5)  # lane center to the right (model y is positive right)
  run(host, 8.0, center=-0.5)
  run(host, 3.0, active=False)
  assert params.writes == [(DIRECTION_KEY, 0), (DIRECTION_KEY, 1), (DIRECTION_KEY, -1), (DIRECTION_KEY, 0)]


def test_a_params_build_without_the_side_key_still_corrects(tmp_path):
  params = SavedParams(tmp_path, registered=(KEY, CLASSIC_KEY))
  params.save(**{KEY: b'1'})
  host = BlendfulHost(params)
  run(host, 5.0)
  assert host.applied > 0 and host.direction == 1 and params.writes == []
