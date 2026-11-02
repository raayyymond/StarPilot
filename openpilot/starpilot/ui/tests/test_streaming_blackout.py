"""While Android Auto streams the car view, the comma's own onroad screen is black with alerts only."""
from types import SimpleNamespace as NS
from unittest.mock import Mock, patch

import pytest

from openpilot.starpilot.ui import onroad
from openpilot.starpilot.ui.onroad_state import AlertSize, OnroadAlert, OnroadState, SpeedLimitObservation
from openpilot.starpilot.ui.presentation import Profile


def state():
  return OnroadState(True, True, 20, 80, SpeedLimitObservation(), alert=OnroadAlert('Take Control', 'x', AlertSize.FULL))


def view(profile, active):
  view = object.__new__(onroad.OnroadView)
  view.fonts = NS(profile=profile)
  view.projection_viewport = None
  view.camera_layer = Mock()
  view.alert = Mock()
  view.streaming_blackout = Mock(return_value=active)
  view._large, view._compact = Mock(), Mock()
  return view


@pytest.mark.parametrize('profile', [Profile.COMPACT, Profile.LARGE])
def test_active_lease_draws_black_and_only_the_alert(profile):
  v = view(profile, True)
  with patch.object(onroad, 'rl') as rl, patch.object(onroad, 'clip'):
    v.render(state())
  v.camera_layer.assert_not_called()  # no VisionIPC receive and no texture upload either: both live in this callback
  v._large.assert_not_called()
  v._compact.assert_not_called()
  v.alert.render.assert_called_once()
  assert v.alert.render.call_args.args[1] == state().alert
  assert (rl.draw_rectangle.call_args or rl.draw_rectangle_rec.call_args).args[-1] is rl.BLACK


@pytest.mark.parametrize('profile', [Profile.COMPACT, Profile.LARGE])
def test_inactive_lease_keeps_the_normal_view(profile):
  v = view(profile, False)
  v.render(state())
  (v._large if profile == Profile.LARGE else v._compact).assert_called_once()
  v.alert.render.assert_not_called()


def test_native_session_wires_the_cached_lease_reader():
  from openpilot.starpilot.system.starport.projection_stream_lease import StreamLeaseReader
  import ast
  from pathlib import Path
  source = (Path(onroad.__file__).with_name('runtime_app.py')).read_text()
  assert 'self.view.onroad.streaming_blackout = StreamLeaseReader()' in source
  ast.parse(source)
  assert callable(StreamLeaseReader('/nonexistent'))
