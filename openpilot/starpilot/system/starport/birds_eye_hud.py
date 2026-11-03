"""HUD over the birds-eye-only car screen: a longitudinal acceleration bar on the left edge and the torque bar.

Acceleration bar: a slim vertical track hugging the left edge, zero at its middle, accelerating up, braking down.
The filled bar is the commanded accel (carControl.actuators.accel while longActive, else the plan's first accel,
drawn dimmer); a white tick is the measured carState.aEgo. The fill turns red while braking is applied:
commanded below BRAKE_ACCEL while longActive, or the driver's brake pedal.

Torque bar: the same arc and the same caller-qualified utilization as the full composition
(observe_torque_feedback -> TorqueBarPainter); on the angle-loop Accord that utilization is the angle error
(setpoint - wheel) over the carcontroller's error clip, see starpilot/ui/torque_feedback.py.

Per frame this is a handful of rectangles, one cached-text draw and one shader quad; nothing allocates a texture.
"""
from dataclasses import dataclass
import math

ACCEL_RANGE = 3.0          # m/s^2 at either end of the bar
BRAKE_ACCEL = -0.2         # commanded accel below this while longActive counts as braking
BRAKE_RELEASE = -0.15      # hysteresis: braking ends above this
FPS = 30.0                 # the stream's frame rate; filters are tuned for it
ACCEL_RC = 0.12            # seconds; enough to calm 30 fps jitter without visible lag
TORQUE_RC = 0.1            # the full composition's torque filter time constant

BACKGROUND_TOP, BACKGROUND_BOTTOM = (26, 31, 44), (8, 9, 13)   # birds_eye_renderer's full-frame gradient
TRACK = (255, 255, 255, 26)
TICK = (170, 180, 198)                                       # birds_eye_renderer's rung-label grey
GREEN = (64, 220, 120)                                       # birds_eye_renderer's accent green
NEUTRAL = (170, 180, 198)
RED = (255, 72, 72)
ACTUAL = (245, 247, 250)
SHADOW = (0, 0, 0)


def _finite(value, default=None):
  try:
    value = float(value)
  except (TypeError, ValueError):
    return default
  return value if math.isfinite(value) else default


@dataclass(frozen=True)
class AccelReading:
  desired: float | None
  actual: float | None
  long_active: bool
  brake_pressed: bool


@dataclass(frozen=True)
class AccelGeometry:
  strip: float     # reserved width at the left edge
  x: float         # bar left edge
  width: float     # bar width
  top: float
  bottom: float
  zero: float      # y of 0 m/s^2
  per_unit: float  # px per m/s^2

  def y(self, accel: float) -> float:
    return self.zero - max(-ACCEL_RANGE, min(ACCEL_RANGE, accel)) * self.per_unit


def accel_geometry(height: float) -> AccelGeometry:
  """Sized from the frame height so 1080 logical rows (a 1280x720 head unit) keep the designed proportions."""
  k = height / 1080
  strip = round(84 * k)
  width = max(4.0, round(14 * k))
  top, bottom = height * 0.16, height * 0.80
  zero = (top + bottom) / 2
  return AccelGeometry(strip, strip / 2 - width / 2, width, top, bottom, zero, (bottom - top) / 2 / ACCEL_RANGE)


def dock_height(height: float) -> float:
  """Bottom band reserved for the torque arc, so the arc never lands on the ego car the view anchors near its bottom."""
  return round(64 * height / 1080)


def view_rect(width: float, height: float) -> tuple[float, float, float, float]:
  """Where the birds-eye view draws: right of the acceleration strip and above the torque dock."""
  strip = accel_geometry(height).strip
  return strip, 0, width - strip, height - dock_height(height)


def torque_rect(width: float, height: float, strip: float) -> tuple[float, float, float, float]:
  """Bottom-centre box for TorqueBarPainter (arc_scale = box height / 240 when screen_width is the box width)."""
  scale = height / 1080
  box_w = 2 * (24 + 139 * scale)
  box_h = 240 * scale
  cx = strip + (width - strip) / 2
  return cx - box_w / 2, height - box_h, box_w, box_h


def read_accel(sm) -> AccelReading | None:
  """The commanded and measured accel from the SubMaster; None before carState has been seen."""
  seen = sm.seen
  if not seen['carState']:
    return None
  car_state = sm['carState']
  long_active, desired = False, None
  if seen['carControl']:
    control = sm['carControl']
    long_active = bool(control.longActive)
    if long_active:
      desired = _finite(control.actuators.accel)
  if desired is None and seen['longitudinalPlan']:
    plan = sm['longitudinalPlan']
    accels = plan.accels
    desired = _finite(accels[0]) if len(accels) else _finite(plan.aTarget)
  return AccelReading(desired, _finite(car_state.aEgo), long_active, bool(car_state.brakePressed))


def braking(reading: AccelReading | None, was_braking: bool = False) -> bool:
  if reading is None:
    return False
  if reading.brake_pressed:
    return True
  if not reading.long_active or reading.desired is None:
    return False
  return reading.desired < (BRAKE_RELEASE if was_braking else BRAKE_ACCEL)


def fill_color(accel: float, is_braking: bool, long_active: bool) -> tuple[int, int, int, int]:
  alpha = 235 if long_active else 120
  if is_braking:
    return (*RED, 240)
  return (*(GREEN if accel >= 0 else NEUTRAL), alpha)


class _Filter:
  def __init__(self, rc):
    self.alpha = (1 / FPS) / (rc + 1 / FPS)
    self.x = None

  def update(self, value):
    self.x = value if self.x is None else self.x + self.alpha * (value - self.x)
    return self.x


def torque_feedback(sm, car_params):
  """observe_torque_feedback over the SubMaster, as the full composition qualifies it."""
  from openpilot.starpilot.ui.torque_feedback import observe_torque_feedback
  seen = sm.seen
  def message(name):
    return sm[name] if seen[name] else None
  return observe_torque_feedback(message('carState'), message('carControl'), message('controlsState'),
                                 message('carOutput'), message('vehicleParameters'), car_params)


class BirdsEyeHud:
  def __init__(self, rl, text, source, *, painter=None, feedback=torque_feedback):
    """source() -> (SubMaster, CarParams or None). painter: TorqueBarPainter-like, built lazily when omitted."""
    self.rl, self.text, self.source = rl, text, source
    self._painter = painter
    self._painter_failed = False
    self._feedback = feedback
    self._desired = _Filter(ACCEL_RC)
    self._actual = _Filter(ACCEL_RC)
    self._torque = _Filter(TORQUE_RC)
    self._torque_alpha = _Filter(TORQUE_RC)
    self._braking = False
    self._label_key = None
    self._label = None
    self._geometry_key = None
    self._geometry = None

  @staticmethod
  def view_rect(width: float, height: float) -> tuple[float, float, float, float]:
    return view_rect(width, height)

  def geometry(self, height: float) -> AccelGeometry:
    if self._geometry_key != height:
      self._geometry, self._geometry_key = accel_geometry(height), height
    return self._geometry

  def render(self, width: float, height: float) -> None:
    sm, car_params = self.source()
    geometry = self.geometry(height)
    self._draw_dock(width, height, geometry.strip)
    self._draw_accel(geometry, read_accel(sm), height)
    self._draw_torque(sm, car_params, width, height, geometry.strip)

  def _draw_dock(self, width, height, strip) -> None:
    """The torque dock continues the view's own background; the view's last rows fade into it."""
    rl = self.rl
    dock = dock_height(height)
    fade = round(24 * height / 1080)
    top = int(height - dock)
    x, w = int(strip), int(width - strip)
    rl.draw_rectangle_gradient_v(x, top - fade, w, fade, rl.Color(*BACKGROUND_BOTTOM, 0), rl.Color(*BACKGROUND_BOTTOM, 255))
    rl.draw_rectangle(x, top, w, int(height) - top, rl.Color(*BACKGROUND_BOTTOM, 255))

  # --- acceleration bar ---
  def _draw_accel(self, g: AccelGeometry, reading: AccelReading | None, height: float) -> None:
    rl = self.rl
    strip = int(g.strip)
    rl.draw_rectangle_gradient_v(0, 0, strip, int(height), rl.Color(*BACKGROUND_TOP, 255), rl.Color(*BACKGROUND_BOTTOM, 255))
    rl.draw_rectangle(strip - 1, 0, 1, int(height), rl.Color(255, 255, 255, 14))
    rl.draw_rectangle_rounded(rl.Rectangle(g.x, g.top, g.width, g.bottom - g.top), 1.0, 6, rl.Color(*TRACK))
    k = height / 1080
    tick = 6 * k
    for unit in range(-int(ACCEL_RANGE), int(ACCEL_RANGE) + 1):
      y = g.y(unit)
      if unit == 0:
        rl.draw_rectangle_rec(rl.Rectangle(g.x - 2 * tick, y - k, g.width + 4 * tick, 2 * k), rl.Color(*TICK, 150))
      else:
        rl.draw_rectangle_rec(rl.Rectangle(g.x - tick - 2 * k, y - k / 2, tick, max(1.0, k)), rl.Color(*TICK, 90))
    if reading is None:
      self._desired.x = self._actual.x = None
      self._braking = False
      return
    self._braking = braking(reading, self._braking)
    if reading.desired is not None:
      desired = self._desired.update(reading.desired)
      color = rl.Color(*fill_color(desired, self._braking, reading.long_active))
      y = g.y(desired)
      top, bottom = min(y, g.zero), max(y, g.zero)
      if bottom - top >= 0.5:
        rl.draw_rectangle_rec(rl.Rectangle(g.x, top, g.width, bottom - top), color)
      self._draw_label(desired, color, g, k)
    if reading.actual is not None:
      y = g.y(self._actual.update(reading.actual))
      w, h = g.width + 16 * k, max(3.0, 4 * k)
      x = g.x + g.width / 2 - w / 2
      rl.draw_rectangle_rec(rl.Rectangle(x - k, y - h / 2 - k, w + 2 * k, h + 2 * k), rl.Color(*SHADOW, 170))
      rl.draw_rectangle_rec(rl.Rectangle(x, y - h / 2, w, h), rl.Color(*ACTUAL, 245))

  def _draw_label(self, accel: float, color, g: AccelGeometry, k: float) -> None:
    if self.text is None:
      return
    value = round(accel, 1)
    size = 28 * k
    key = (value, size)
    if key != self._label_key:
      text = f"{value:+.1f}" if value != 0 else "0.0"
      w, h = self.text.measure(text, size)
      self._label, self._label_key = (text, w, h), key
    text, w, _ = self._label
    self.text.draw(text, size, g.strip / 2 - w / 2, g.bottom + 16 * k, color)

  # --- torque bar ---
  def _draw_torque(self, sm, car_params, width, height, strip) -> None:
    if self._painter_failed:
      return
    try:
      feedback = self._feedback(sm, car_params)
    except (AttributeError, KeyError, TypeError, ValueError):
      feedback = None
    utilization = getattr(feedback, 'utilization', None)
    active = utilization is not None and getattr(feedback, 'reason', '') != 'lateral_inactive'
    if not active:
      # The full composition hides the arc and restarts its fade whenever lateral is not active.
      self._torque.x = self._torque_alpha.x = None
      return
    torque = self._torque.update(max(-1.0, min(1.0, utilization)))
    alpha = self._torque_alpha.update(1.0) if self._torque_alpha.x is not None else self._torque_alpha.update(0.0)
    x, y, w, h = torque_rect(width, height, strip)
    try:
      if self._painter is None:
        from openpilot.starpilot.ui.onroad_torque_draw import TorqueBarPainter
        self._painter = TorqueBarPainter()
      from openpilot.starpilot.ui.onroad_torque_geometry import CAP_RADIUS
      # offset -8 undoes the camera-feed nudge the painter applies; screen_width = box width -> scale = h / 240.
      self._painter.render(self.rl.Rectangle(x, y, w, h), torque, alpha, w, CAP_RADIUS, (-8, 0))
    except RuntimeError:
      self._painter_failed = True  # no shader on this GL context: keep the stream, drop the arc

  def close(self) -> None:
    if self._painter is not None:
      self._painter.close()
      self._painter = None
