"""Birds-eye stream HUD: acceleration bar colours, limits and geometry, flat torque bar, turn-signal arrows, and per-frame
cost with fakes."""
from types import SimpleNamespace
import time

import pytest

from openpilot.starpilot.system.starport import birds_eye_hud as hud_module
from openpilot.starpilot.system.starport.birds_eye_hud import (DEFAULT_SCALE, LIMIT_FRACTION, AccelReading, AccelScale,
                                                               BirdsEyeHud, RED, accel_geometry, braking, dock_height,
                                                               fill_color, read_accel, read_limits, read_speeds,
                                                               readout_size, signal_alpha, torque_color, torque_geometry,
                                                               torque_strength, view_rect)
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

  @staticmethod
  def Vector2(*values):
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


SERVICES = ('carState', 'carControl', 'longitudinalPlan', 'starpilotLongitudinalPlan', 'controlsState', 'carOutput',
            'vehicleParameters')


class FakeSM:
  def __init__(self, **messages):
    self.messages = messages
    self.seen = {name: name in messages for name in SERVICES}

  def __getitem__(self, name):
    return self.messages[name]


def sm(*, desired=0.0, actual=0.0, long_active=True, brake=False, plan=None, limits=None, left=False, right=False,
       v_ego=None, v_cruise=255.0):
  """plan=None: no longitudinalPlan, so the bar falls back to the actuators' accel (`desired`)."""
  car_state = SimpleNamespace(aEgo=actual, brakePressed=brake, leftBlinker=left, rightBlinker=right,
                              vCruise=v_cruise, vCruiseCluster=v_cruise)
  if v_ego is not None:
    car_state.vEgo = v_ego
  messages = {'carState': car_state,
              'carControl': SimpleNamespace(longActive=long_active, actuators=SimpleNamespace(accel=desired))}
  if plan is not None:
    messages['longitudinalPlan'] = SimpleNamespace(accels=[plan], aTarget=plan)
  if limits is not None:
    messages['starpilotLongitudinalPlan'] = SimpleNamespace(accelLimitMin=limits[0], accelLimitMax=limits[1])
  return FakeSM(**messages)


def hud(source, feedback=lambda sm, cp: TorqueFeedback(0.4, 'angle_error', 'observed'), clock=lambda: 0.0):
  rl, text = FakeRl(), FakeText()
  return BirdsEyeHud(rl, text, source, feedback=feedback, clock=clock), rl, text


def accel_fills(rl, g):
  """Rounded rectangles centred in the acceleration strip, other than the full-height track."""
  return [call for call in rl.calls if call[0] == 'draw_rectangle_rounded' and call[1][0] + call[1][2] <= g.strip
          and call[1][3] < g.bottom - g.top - 1]


def torque_rects(rl):
  g = accel_geometry(LOGICAL_720P[1])
  return [call for call in rl.calls if call[0] == 'draw_rectangle_rounded' and call[1][0] > g.strip]


def test_reading_shows_the_plan_target_only_while_long_active():
  assert read_accel(sm(desired=-1.0, plan=-0.6)).desired == -0.6, 'longitudinalPlan.aTarget, not the PID output'
  assert read_accel(sm(desired=-1.0)).desired == -1.0, 'no plan yet: the actuators fall back'
  reading = read_accel(sm(desired=-1.0, long_active=False, plan=0.7))
  assert reading.desired == 0.0 and not reading.long_active, 'the plan is not shown while disengaged'
  assert read_accel(FakeSM()) is None


def test_reading_falls_back_to_the_actuators_on_a_stale_plan():
  messages = sm(desired=-1.0, plan=-0.6)
  messages.recv_time = {'longitudinalPlan': 10.0}
  assert read_accel(messages, now_ns=int(10.5e9)).desired == -0.6
  assert read_accel(messages, now_ns=int(11.2e9)).desired == -1.0, 'older than 1 s'
  messages.alive = {'longitudinalPlan': False}
  assert read_accel(messages, now_ns=int(10.5e9)).desired == -1.0, 'not alive'


def test_measured_accel_tick_is_lightly_filtered():
  view, _, _ = hud(lambda: (sm(actual=0.0), None))
  assert view._actual.alpha == pytest.approx((1 / 30) / (hud_module.AEGO_RC + 1 / 30))
  assert hud_module.AEGO_RC == pytest.approx(0.15)


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


def test_fill_is_coloured_only_while_engaged():
  assert fill_color(-1.0, True, True)[:3] == RED
  assert fill_color(0.8, False, True)[:3] == hud_module.GREEN
  assert fill_color(-0.1, False, True)[:3] == hud_module.NEUTRAL
  for accel, is_braking in ((1.0, False), (-1.0, True)):
    color = fill_color(accel, is_braking, False)
    assert color[:3] == hud_module.IDLE and color[3] < 128


def test_bar_is_red_while_braking_on_screen():
  g = accel_geometry(LOGICAL_720P[1])
  view, rl, text = hud(lambda: (sm(desired=-1.5, actual=-1.2), None))
  view.render(*LOGICAL_720P)
  (fill,) = accel_fills(rl, g)
  assert fill[-1][:3] == RED
  assert text.drawn[0][0] == '-1.5' and text.drawn[0][1][:3] == RED

  view, rl, _ = hud(lambda: (sm(desired=1.0, brake=True), None))
  view.render(*LOGICAL_720P)
  assert accel_fills(rl, g)[0][-1][:3] == RED

  view, rl, _ = hud(lambda: (sm(desired=1.0), None))
  view.render(*LOGICAL_720P)
  assert accel_fills(rl, g)[0][-1][:3] == hud_module.GREEN


def test_disengaged_bar_is_grey_with_the_setpoint_at_zero():
  g = accel_geometry(LOGICAL_720P[1])
  view, rl, text = hud(lambda: (sm(desired=1.5, actual=0.8, long_active=False, plan=1.2, limits=(-1.2, 1.6)), None))
  view.render(*LOGICAL_720P)
  assert accel_fills(rl, g) == [], 'no fill away from zero'
  assert text.drawn == []
  track = next(call for call in rl.calls if call[0] == 'draw_rectangle_rounded' and call[1][1] == g.top)
  assert track[-1][:3] == hud_module.IDLE
  limit_ticks = [call for call in rl.calls if call[0] == 'draw_rectangle_rec' and call[-1][:3] == hud_module.LIMIT]
  assert limit_ticks == [], 'limit ticks only while engaged'
  actual = [call for call in rl.calls if call[0] == 'draw_rectangle_rec' and call[-1][:3] == hud_module.ACTUAL]
  assert len(actual) == 1 and actual[0][1][1] < g.zero, 'the measured accel tick still shows'
  # A residual setpoint decays back to zero, drawn grey.
  view._desired.x = 1.0
  rl.calls.clear()
  view.render(*LOGICAL_720P)
  (fill,) = accel_fills(rl, g)
  assert fill[-1][:3] == hud_module.IDLE and view._desired.x < 1.0


def test_limits_sit_at_the_fixed_fraction_with_ticks():
  g = accel_geometry(LOGICAL_720P[1])
  scale = AccelScale(-1.2, 1.6)
  assert scale.y(g, 1.6) == pytest.approx(g.zero - LIMIT_FRACTION * g.half)
  assert scale.y(g, -1.2) == pytest.approx(g.zero + LIMIT_FRACTION * g.half)
  assert scale.y(g, 9.0) == pytest.approx(g.top) and scale.y(g, -9.0) == pytest.approx(g.bottom), 'past the tick to the end'
  assert DEFAULT_SCALE.y(g, 3.0) == pytest.approx(g.top) and DEFAULT_SCALE.y(g, -3.0) == pytest.approx(g.bottom)

  view, rl, _ = hud(lambda: (sm(desired=0.4, limits=(-1.2, 1.6)), None))
  view.render(*LOGICAL_720P)
  ticks = sorted(call[1][1] + call[1][3] / 2 for call in rl.calls
                 if call[0] == 'draw_rectangle_rec' and call[-1][:3] == hud_module.LIMIT)
  assert ticks == pytest.approx([scale.y(g, 1.6), scale.y(g, -1.2)])


def test_bar_grows_at_the_limit():
  g = accel_geometry(LOGICAL_720P[1])
  widths = {}
  for desired in (0.5, 1.6, 2.5):
    view, rl, _ = hud(lambda desired=desired: (sm(desired=desired, limits=(-1.2, 1.6)), None))
    view.render(*LOGICAL_720P)
    (fill,) = accel_fills(rl, g)
    widths[desired] = fill[1][2]
    if desired == 2.5:
      assert fill[1][1] == pytest.approx(g.top), 'passes the limit tick up to the end'
  assert widths[0.5] == pytest.approx(g.width) and widths[1.6] == pytest.approx(g.grown) == widths[2.5]
  assert g.grown == 2 * g.width


def test_limits_fall_back_when_absent_stale_or_implausible():
  assert read_limits(sm()) is None
  assert read_limits(sm(limits=(-1.0, 1.5))) == AccelScale(-1.0, 1.5)
  assert read_limits(sm(limits=(0.0, 1.5))) is None
  assert read_limits(sm(limits=(-1.0, float('nan')))) is None
  stale = sm(limits=(-1.0, 1.5))
  stale.alive = {'starpilotLongitudinalPlan': False}
  assert read_limits(stale) is None
  old = sm(limits=(-1.0, 1.5))
  old.recv_time = {'starpilotLongitudinalPlan': 1.0}
  assert read_limits(old, now_ns=3_000_000_000) is None and read_limits(old, now_ns=1_500_000_000) is not None
  view, _, _ = hud(lambda: (sm(desired=0.4), None))
  view.render(*LOGICAL_720P)
  assert (view._lower.x, view._upper.x) == (DEFAULT_SCALE.lower, DEFAULT_SCALE.upper)


def test_geometry_at_1280x720():
  g = accel_geometry(LOGICAL_720P[1])
  assert g.width == 28 and g.strip == 112 and g.x + g.width < g.strip
  assert g.center - g.grown / 2 > 0 and g.center + g.grown / 2 < g.strip
  assert g.bottom - g.top == pytest.approx(0.75 * LOGICAL_720P[1])
  t = torque_geometry(*LOGICAL_720P)
  vx, vy, vw, vh = view_rect(*LOGICAL_720P)
  assert (vx, vy, vx + vw) == (g.strip, 0, LOGICAL_720P[0])
  assert t.cx == pytest.approx(vx + vw / 2), 'centred under the view, on the ego car'
  assert t.right - t.left == pytest.approx(0.75 * LOGICAL_720P[0]) and t.left > g.strip
  assert (t.thin, t.thick) == (28, 112), '2x the arc thickness'
  assert vh == LOGICAL_720P[1] - dock_height(LOGICAL_720P[1])
  assert t.bottom - t.thick > vh, 'the grown bar never reaches into the view'
  physical = accel_geometry(720)
  assert physical.width == 19 and physical.strip == 74


def test_filter_smooths_steps():
  view, _, _ = hud(lambda: (sm(desired=2.0, actual=0.0), None))
  view._desired.x = 0.0
  view.render(*LOGICAL_720P)
  assert 0.0 < view._desired.x < 1.0


def test_torque_strength_and_colour_follow_the_arc():
  assert torque_strength(0.4) == 0 and torque_strength(0.75) == pytest.approx(0.5) and torque_strength(-1.0) == 1
  assert torque_color(0.5, 1.0)[:3] == (255, 255, 255)
  warm = torque_color(1.0, 1.0)
  assert warm[0] == 255 and warm[2] == 0 and warm[1] < 255


def test_torque_bar_is_flat_and_stays_visible_when_inactive():
  t = torque_geometry(*LOGICAL_720P)
  view, rl, _ = hud(lambda: (sm(), 'CP'))
  view.render(*LOGICAL_720P)
  rl.calls.clear()
  view.render(*LOGICAL_720P)
  track, indicator, notch = torque_rects(rl)
  assert track[1] == pytest.approx((t.left, t.bottom - t.thin, t.right - t.left, t.thin))
  assert indicator[1][0] == pytest.approx(t.cx) and indicator[1][2] == pytest.approx(0.4 * (t.right - t.left) / 2)
  assert notch[1][0] < t.cx < notch[1][0] + notch[1][2]

  def inactive(sm, cp):
    return TorqueFeedback(0.0, 'lateral_acceleration_estimate', 'lateral_inactive')
  for feedback in (inactive, lambda sm, cp: TorqueFeedback()):
    view, rl, _ = hud(lambda: (sm(), 'CP'), feedback=feedback)
    view.render(*LOGICAL_720P)
    track, notch = torque_rects(rl)
    assert track[1][2] == pytest.approx(t.right - t.left) and track[-1][:3] == hud_module.IDLE


def test_torque_bar_grows_and_returns_to_centre():
  t = torque_geometry(*LOGICAL_720P)
  state = {'feedback': TorqueFeedback(-1.0, 'angle_error', 'observed')}
  view, rl, _ = hud(lambda: (sm(), 'CP'), feedback=lambda sm, cp: state['feedback'])
  for _ in range(60):
    view.render(*LOGICAL_720P)
  rl.calls.clear()
  view.render(*LOGICAL_720P)
  track, indicator = torque_rects(rl)
  assert track[1][3] == pytest.approx(t.thick, rel=0.02)
  assert indicator[1][0] + indicator[1][2] == pytest.approx(t.cx) and indicator[1][0] < t.left + 5, 'reaches left'
  state['feedback'] = TorqueFeedback(0.0, 'lateral_acceleration_estimate', 'lateral_inactive')
  for _ in range(60):
    view.render(*LOGICAL_720P)
  assert abs(view._torque.x) < 0.01 and view._torque_alpha.x < 0.01


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


def test_signal_pulse_matches_the_comma_period():
  period = hud_module.TURN_SIGNAL_BLINK_PERIOD
  assert period == pytest.approx(0.75)
  assert signal_alpha(0.0) == 1.0 and signal_alpha(period * 0.95) < 0.45
  assert signal_alpha(period) == 1.0, 'restarts each period'


def test_turn_signal_arrows_in_the_corners_with_the_fallback_polygon():
  now = {'t': 0.0}
  view, rl, _ = hud(lambda: (sm(left=True), None), clock=lambda: now['t'])
  view.render(*LOGICAL_720P)
  assert view._icon_failed, 'the fake rl has no image loader'
  triangles = [call for call in rl.calls if call[0] == 'draw_triangle']
  assert len(triangles) == 1
  xs = [point[0] for point in triangles[0][1:4]]
  ys = [point[1] for point in triangles[0][1:4]]
  g = accel_geometry(LOGICAL_720P[1])
  assert g.strip < min(xs) and max(xs) < LOGICAL_720P[0] / 4, 'left edge, right of the accel strip'
  assert max(ys) - min(ys) == pytest.approx(0.6 * hud_module.SIGNAL_HEIGHT)
  assert (max(ys) + min(ys)) / 2 == pytest.approx(LOGICAL_720P[1] / 2), 'centred vertically on the frame'

  (ax, ay), (bx, by), (cx, cy) = triangles[0][1:4]
  assert (bx - ax) * (cy - ay) - (by - ay) * (cx - ax) < 0, 'counter-clockwise for raylib'
  bright = triangles[0][-1][3]
  now['t'] = 0.7
  rl.calls.clear()
  view.render(*LOGICAL_720P)
  assert next(call for call in rl.calls if call[0] == 'draw_triangle')[-1][3] < bright, 'pulses'

  view, rl, _ = hud(lambda: (sm(right=True), None))
  view.render(*LOGICAL_720P)
  (triangle,) = [call for call in rl.calls if call[0] == 'draw_triangle']
  assert min(point[0] for point in triangle[1:4]) > LOGICAL_720P[0] * 3 / 4, 'right edge'
  ys = [point[1] for point in triangle[1:4]]
  assert (max(ys) + min(ys)) / 2 == pytest.approx(LOGICAL_720P[1] / 2)
  (ax, ay), (bx, by), (cx, cy) = triangle[1:4]
  assert (bx - ax) * (cy - ay) - (by - ay) * (cx - ax) < 0

  view, rl, _ = hud(lambda: (sm(), None))
  view.render(*LOGICAL_720P)
  assert not [call for call in rl.calls if call[0] in ('draw_triangle', 'draw_texture_pro')]


def test_turn_signal_texture_is_loaded_once_flipped_for_the_right_and_unloaded():
  class TextureRl(FakeRl):
    def __init__(self):
      super().__init__()
      self.ffi = SimpleNamespace(NULL=None)
      self.TextureFilter = SimpleNamespace(TEXTURE_FILTER_BILINEAR=1)
      self.loaded, self.unloaded = 0, []

    def load_image(self, path):
      assert path.endswith('icons_mici/onroad/turn_signal_left.png')
      return SimpleNamespace(data=b'x', width=208, height=192)

    def image_resize(self, image, w, h):
      image.width, image.height = w, h

    def load_texture_from_image(self, image):
      self.loaded += 1
      return SimpleNamespace(id=7, width=image.width, height=image.height)

    def unload_image(self, image):
      pass

    def set_texture_filter(self, texture, mode):
      pass

    def unload_texture(self, texture):
      self.unloaded.append(texture.id)

  rl = TextureRl()
  view = BirdsEyeHud(rl, FakeText(), lambda: (sm(left=True, right=True), None),
                     feedback=lambda sm, cp: TorqueFeedback(), clock=lambda: 0.0)
  view.render(*LOGICAL_720P)
  view.render(*LOGICAL_720P)
  assert rl.loaded == 1
  draws = [call for call in rl.calls if call[0] == 'draw_texture_pro']
  left, right = draws[:2]
  assert left[2][2] > 0 and right[2][2] < 0, 'right side flipped horizontally'
  assert left[3][3] == pytest.approx(hud_module.SIGNAL_HEIGHT) and left[3][0] > accel_geometry(LOGICAL_720P[1]).strip
  assert right[3][0] + right[3][2] < LOGICAL_720P[0]
  for quad in (left[3], right[3]):
    assert quad[1] + quad[3] / 2 == pytest.approx(LOGICAL_720P[1] / 2), 'centred vertically on the frame'
  view.close()
  assert rl.unloaded == [7]


class SizedText(FakeText):
  def draw(self, text, size, x, y, color):
    self.drawn.append((text, size, x, y))


def test_speed_readout_shows_speed_and_set_speed_at_twice_the_lead_label():
  from openpilot.selfdrive.ui.onroad.birds_eye_math import label_font_size
  assert read_speeds(sm(v_ego=26.8224, v_cruise=112.654), metric=False) == ('60', '70')
  assert read_speeds(sm(v_ego=26.8224, v_cruise=112.654), metric=True) == ('97', '113')
  assert read_speeds(sm(v_ego=10.0, v_cruise=255.0), metric=False) == ('22', '--'), 'cruise not set'
  assert read_speeds(sm(v_ego=10.0, v_cruise=0.0), metric=False)[1] == '--'
  assert read_speeds(sm(), metric=False) is None, 'no vEgo'
  for width, height in (LOGICAL_720P, (1280, 720), (3840, 2160)):
    lead_label = label_font_size(view_rect(width, height)[3])
    assert readout_size(width, height) == pytest.approx(2 * lead_label)

  text = SizedText()
  view = BirdsEyeHud(FakeRl(), text, lambda: (sm(v_ego=26.8224, v_cruise=112.654, long_active=False), None),
                     feedback=lambda sm, cp: TorqueFeedback(), clock=lambda: 0.0)
  view.render(*LOGICAL_720P)
  (speed, speed_size, sx, sy), (mph, cap_size, _, cy), (cruise, cruise_size, cx, _), (unit, _, _, _) = text.drawn
  assert (speed, mph, cruise, unit) == ('60', 'mph', '70', 'SET')
  assert speed_size == cruise_size == pytest.approx(2 * label_font_size(view_rect(*LOGICAL_720P)[3]))
  assert cap_size < speed_size / 2
  g = accel_geometry(LOGICAL_720P[1])
  assert g.strip < sx < cx and sy < 0.1 * LOGICAL_720P[1] and cy > sy, 'top left of the view, set speed to the right'
  view.metric = True
  text.drawn.clear()
  view.render(*LOGICAL_720P)
  assert [entry[0] for entry in text.drawn] == ['97', 'km/h', '113', 'SET']


def test_cpu_cost_per_frame_is_small():
  view, _, _ = hud(lambda: (sm(desired=-0.6, actual=-0.4, limits=(-1.2, 1.6), left=True, v_ego=20.0, v_cruise=100.0), None))
  view.render(*LOGICAL_720P)
  start = time.perf_counter()
  for _ in range(300):
    view.render(*LOGICAL_720P)
  assert (time.perf_counter() - start) / 300 < 0.001
