"""Birds-eye stream HUD: acceleration bar colours and geometry, torque arc wiring, and per-frame cost with fakes."""
from types import SimpleNamespace
import time

import pytest

from openpilot.starpilot.system.starport import birds_eye_hud as hud_module
from openpilot.starpilot.system.starport.birds_eye_hud import (AccelReading, BirdsEyeHud, RED, accel_geometry, braking,
                                                               dock_height, fill_color, read_accel, torque_rect,
                                                               view_rect)
from openpilot.starpilot.ui.torque_feedback import TorqueFeedback

LOGICAL_720P = (1860, 1080)  # projection_geometry's logical canvas for a 1280x720 head unit


class FakeRl:
  def __init__(self):
    self.calls = []

  @staticmethod
  def Color(*values):
    return values

  @staticmethod
  def Rectangle(*values):
    return values

  def __getattr__(self, name):
    if name.startswith('draw_'):
      return lambda *args: self.calls.append((name, *args))
    raise AttributeError(name)


class FakeText:
  def __init__(self):
    self.drawn = []

  def measure(self, text, size):
    return len(text) * size * 0.5, size

  def draw(self, text, size, x, y, color):
    self.drawn.append((text, color))


class FakePainter:
  def __init__(self):
    self.rendered = []
    self.closed = False

  def render(self, rect, torque, alpha, screen_width, cap_radius, offset=(0, 0)):
    self.rendered.append((rect, torque, alpha, screen_width, cap_radius, offset))

  def close(self):
    self.closed = True


class FakeSM:
  def __init__(self, **messages):
    self.messages = messages
    self.seen = {name: name in messages for name in ('carState', 'carControl', 'longitudinalPlan', 'controlsState',
                                                      'carOutput', 'vehicleParameters')}

  def __getitem__(self, name):
    return self.messages[name]


def sm(*, desired=0.0, actual=0.0, long_active=True, brake=False, plan=0.5):
  return FakeSM(carState=SimpleNamespace(aEgo=actual, brakePressed=brake),
                carControl=SimpleNamespace(longActive=long_active, actuators=SimpleNamespace(accel=desired)),
                longitudinalPlan=SimpleNamespace(accels=[plan], aTarget=plan))


def hud(source, feedback=lambda sm, cp: TorqueFeedback(0.4, 'angle_error', 'observed')):
  rl, text, painter = FakeRl(), FakeText(), FakePainter()
  return BirdsEyeHud(rl, text, source, painter=painter, feedback=feedback), rl, text, painter


def fills(rl):
  return [call for call in rl.calls if call[0] == 'draw_rectangle_rec' and call[2][-1] in (235, 240, 120)]


def test_reading_prefers_actuators_while_long_active_and_the_plan_otherwise():
  assert read_accel(sm(desired=-1.0)).desired == -1.0
  reading = read_accel(sm(desired=-1.0, long_active=False, plan=0.7))
  assert reading.desired == pytest.approx(0.7) and not reading.long_active
  assert read_accel(FakeSM()) is None


@pytest.mark.parametrize('reading, expected', [
  (AccelReading(-0.5, 0.0, True, False), True),
  (AccelReading(-0.1, 0.0, True, False), False),
  (AccelReading(-2.0, 0.0, False, False), False),  # not commanding: the plan's decel is not braking
  (AccelReading(1.0, 0.0, False, True), True),     # the pedal always is
  (None, False),
])
def test_braking(reading, expected):
  assert braking(reading) is expected


def test_braking_hysteresis():
  assert braking(AccelReading(-0.18, 0, True, False), was_braking=True)
  assert not braking(AccelReading(-0.18, 0, True, False), was_braking=False)


def test_fill_turns_red_on_braking_and_green_when_accelerating():
  assert fill_color(-1.0, True, True)[:3] == RED
  assert fill_color(0.8, False, True)[:3] == hud_module.GREEN
  assert fill_color(-0.1, False, True)[:3] == hud_module.NEUTRAL


def test_bar_is_red_while_braking_on_screen():
  view, rl, text, _ = hud(lambda: (sm(desired=-1.5, actual=-1.2), None))
  view.render(*LOGICAL_720P)
  (fill,) = fills(rl)
  assert fill[2][:3] == RED
  assert text.drawn[0][0] == '-1.5' and text.drawn[0][1][:3] == RED

  view, rl, _, _ = hud(lambda: (sm(desired=1.0, brake=True), None))
  view.render(*LOGICAL_720P)
  assert fills(rl)[0][2][:3] == RED

  view, rl, _, _ = hud(lambda: (sm(desired=1.0), None))
  view.render(*LOGICAL_720P)
  assert fills(rl)[0][2][:3] == hud_module.GREEN


def test_geometry_at_1280x720():
  g = accel_geometry(LOGICAL_720P[1])
  assert g.strip == 84 and g.x + g.width < g.strip
  assert g.y(0) == pytest.approx((g.top + g.bottom) / 2)
  assert g.y(3.0) == pytest.approx(g.top) and g.y(-3.0) == pytest.approx(g.bottom)
  assert g.y(-9.0) == pytest.approx(g.bottom), 'clipped to the track'
  x, y, w, h = torque_rect(*LOGICAL_720P, g.strip)
  assert x + w / 2 == pytest.approx(g.strip + (LOGICAL_720P[0] - g.strip) / 2), 'centred on the birds-eye view'
  assert y + h == LOGICAL_720P[1] and w < LOGICAL_720P[0] / 3
  # The view sits right of the strip and above the dock; the resting arc (no torque) fits inside the dock.
  from openpilot.starpilot.ui.onroad_torque_geometry import MIN_HEIGHT, MIN_OFFSET, arc_scale
  vx, vy, vw, vh = view_rect(*LOGICAL_720P)
  assert (vx, vy, vx + vw) == (g.strip, 0, LOGICAL_720P[0]) and vh == LOGICAL_720P[1] - dock_height(LOGICAL_720P[1]) == 1016
  scale = arc_scale(w, h, w)
  assert scale == pytest.approx(1.0)
  assert LOGICAL_720P[1] - (MIN_OFFSET + MIN_HEIGHT) * scale > vh
  # Physical 720 rows: the strip is 56 px, the bar about 9 px wide.
  physical = accel_geometry(720)
  assert physical.strip == 56 and physical.width == 9


def test_filter_smooths_steps():
  view, rl, _, _ = hud(lambda: (sm(desired=2.0, actual=0.0), None))
  view._desired.x = 0.0
  view.render(*LOGICAL_720P)
  assert 0.0 < view._desired.x < 1.0


def test_torque_arc_draws_bottom_centre_and_hides_when_lateral_inactive():
  view, _, _, painter = hud(lambda: (sm(), 'CP'))
  view.render(*LOGICAL_720P)
  view.render(*LOGICAL_720P)
  (rect, torque, alpha, screen_width, cap, offset), second = painter.rendered
  assert rect == torque_rect(*LOGICAL_720P, 84) and screen_width == rect[2] and offset == (-8, 0)
  assert torque == pytest.approx(0.4) and alpha == 0.0 and second[2] > 0.0, 'fades in like the full composition'

  view, _, _, painter = hud(lambda: (sm(), 'CP'), feedback=lambda sm, cp: TorqueFeedback(0.0, 'lateral_acceleration_estimate',
                                                                                          'lateral_inactive'))
  view.render(*LOGICAL_720P)
  assert painter.rendered == []
  view.close()
  assert painter.closed


def test_torque_feedback_is_the_accord_angle_error():
  """EVIDENCE path: the HUD feeds observe_torque_feedback, whose Accord branch is accord_angle_error_fraction."""
  from opendbc.car.honda.values import HondaFlags
  cp = SimpleNamespace(brand='honda', flags=int(HondaFlags.EPS_ANGLE_LOOP_FW), maxLateralAccel=3.0)
  lateral = SimpleNamespace(which=lambda: 'angleState')
  messages = FakeSM(carState=SimpleNamespace(canValid=True, canTimeout=False, steeringAngleDeg=1.0, vEgoRaw=20.0),
                    carControl=SimpleNamespace(latActive=True), controlsState=SimpleNamespace(lateralControlState=lateral),
                    carOutput=SimpleNamespace(actuatorsOutput=SimpleNamespace(steeringAngleDeg=2.0)))
  feedback = hud_module.torque_feedback(messages, cp)
  from openpilot.starpilot.ui.torque_feedback import accord_angle_error_fraction
  assert feedback.source == 'angle_error'
  assert feedback.utilization == pytest.approx(accord_angle_error_fraction(2.0, 1.0, 20.0))


def test_missing_shader_drops_only_the_arc():
  class Broken(FakePainter):
    def render(self, *args):
      raise RuntimeError('Torque bar shader unavailable')
  view = BirdsEyeHud(FakeRl(), FakeText(), lambda: (sm(), None), painter=Broken(),
                     feedback=lambda sm, cp: TorqueFeedback(0.4, 'angle_error', 'observed'))
  view.render(*LOGICAL_720P)
  view.render(*LOGICAL_720P)
  assert view._painter_failed


def test_cpu_cost_per_frame_is_small():
  view, _, _, _ = hud(lambda: (sm(desired=-0.6, actual=-0.4), None))
  view.render(*LOGICAL_720P)
  start = time.perf_counter()
  for _ in range(300):
    view.render(*LOGICAL_720P)
  assert (time.perf_counter() - start) / 300 < 0.001
