"""HUD over the birds-eye-only car screen: an acceleration bar on the left edge, a flat torque bar at the bottom and
large blinking turn-signal arrows in the top corners.

Acceleration bar: a vertical track hugging the left edge, zero at its middle, accelerating up, braking down. Only while
carControl.longActive is it coloured: the fill is the commanded accel (carControl.actuators.accel), green accelerating,
neutral below zero, red while braking (commanded below BRAKE_ACCEL, or the brake pedal). The scale then puts the
planner's published cruise limits (starpilotLongitudinalPlan.accelLimitMin/Max, the floor and ceiling the StarPilot
profile toggles and overrides leave in force) at LIMIT_FRACTION of each half, with a tick at each; as the command
reaches a limit the bar grows thicker and may pass the tick up to the bar's end. Disengaged, the track is neutral
grey, the setpoint stays at zero (the plan's accel is not shown) and the scale is +/-ACCEL_RANGE. A white tick is
always the measured carState.aEgo.

Torque bar: a flat, rounded bar centred at the bottom, LENGTH_FRACTION of the frame width, always visible. Its
indicator runs from the centre toward the sign of the same caller-qualified utilization as the full composition's arc
(observe_torque_feedback; on the angle-loop Accord the angle error over the carcontroller's error clip, see
starpilot/ui/torque_feedback.py). Thickness and colour follow the arc: 2x its base thickness, growing to 2x its
maximum as |utilization| passes 0.5, warming toward orange past 0.75. Grey with the indicator at centre while lateral
is not active.

Turn signals: carState.leftBlinker/rightBlinker pulse the comma's turn_signal_left icon, scaled up, in the top-left /
top-right corner with the comma screen's period; a filled arrow when the texture cannot be loaded.

Per frame this is a handful of rectangles, one cached-text draw and at most one texture quad; nothing allocates.
"""
from dataclasses import dataclass
import math
from pathlib import Path
import time

ACCEL_RANGE = 3.0          # m/s^2 at either end of the bar when no planner limits are known
LIMIT_FRACTION = 0.8       # the planner's limits sit at this fraction of each half of the bar
BRAKE_ACCEL = -0.2         # commanded accel below this while longActive counts as braking
BRAKE_RELEASE = -0.15      # hysteresis: braking ends above this
FPS = 30.0                 # the stream's frame rate; filters are tuned for it
ACCEL_RC = 0.12            # seconds; enough to calm 30 fps jitter without visible lag
TORQUE_RC = 0.1            # the full composition's torque filter time constant
LIMITS_MAX_AGE_NS = 1_000_000_000  # starpilotLongitudinalPlan older than this falls back to +/-ACCEL_RANGE

ACCEL_WIDTH, ACCEL_GROWN = 28, 56              # logical px at 1080 rows: 2x the former bar, doubling at the limit
TORQUE_MIN, TORQUE_MAX = 2 * 14, 2 * 56        # 2x the arc's MIN_HEIGHT / MAX_HEIGHT (onroad_torque_geometry)
TORQUE_MARGIN = 14                             # gap under the torque bar and above its grown top
LENGTH_FRACTION = 0.75                         # torque bar length over the frame width
SIGNAL_HEIGHT = 260                            # turn-signal arrow height at 1080 rows (the icon has glow padding)
SIGNAL_MARGIN = 28
TURN_SIGNAL_BLINK_PERIOD = 1 / (80 / 60)       # the comma screen's period (mici alert_renderer)
TURN_SIGNAL_RC = 0.3                           # its pulse filter: jumps bright each period, then decays to 20 %
TURN_SIGNAL_ICON = Path(__file__).parents[3] / 'selfdrive/assets/icons_mici/onroad/turn_signal_left.png'

BACKGROUND_TOP, BACKGROUND_BOTTOM = (26, 31, 44), (8, 9, 13)   # birds_eye_renderer's full-frame gradient
TRACK = (255, 255, 255, 26)
IDLE = (220, 224, 232)                                       # disengaged fill / track
TICK = (170, 180, 198)                                       # birds_eye_renderer's rung-label grey
LIMIT = (235, 238, 245)
GREEN = (64, 220, 120)                                       # birds_eye_renderer's accent green
SIGNAL_GREEN = (40, 230, 110)                                # the comma icon's green
NEUTRAL = (170, 180, 198)
RED = (255, 72, 72)
ACTUAL = (245, 247, 250)
SHADOW = (0, 0, 0)
DOT = (182, 182, 182)                                        # the arc's centre dot


def _finite(value, default=None):
  try:
    value = float(value)
  except (TypeError, ValueError):
    return default
  return value if math.isfinite(value) else default


def _clamp(value, low=0.0, high=1.0):
  return max(low, min(high, value))


@dataclass(frozen=True)
class AccelReading:
  desired: float | None
  actual: float | None
  long_active: bool
  brake_pressed: bool


@dataclass(frozen=True)
class AccelGeometry:
  strip: float     # reserved width at the left edge
  x: float         # bar left edge at the base width
  width: float     # base bar width
  grown: float     # bar width at a limit
  top: float
  bottom: float
  zero: float      # y of 0 m/s^2

  @property
  def center(self) -> float:
    return self.x + self.width / 2

  @property
  def half(self) -> float:
    return (self.bottom - self.top) / 2


@dataclass(frozen=True)
class AccelScale:
  """Bar scale: `upper` m/s^2 lands at LIMIT_FRACTION of the upper half, `lower` (negative) of the lower half."""
  lower: float
  upper: float

  def y(self, g: AccelGeometry, accel: float) -> float:
    limit = self.upper if accel >= 0 else -self.lower
    fraction = _clamp(abs(accel) / limit * LIMIT_FRACTION) if limit > 0 else 0.0
    return g.zero - math.copysign(fraction * g.half, accel)

  def utilization(self, accel: float) -> float:
    """Fraction of the limit on the accel's side; 1.0 at the limit."""
    limit = self.upper if accel >= 0 else -self.lower
    return abs(accel) / limit if limit > 0 else 0.0


DEFAULT_SCALE = AccelScale(-ACCEL_RANGE * LIMIT_FRACTION, ACCEL_RANGE * LIMIT_FRACTION)  # +/-ACCEL_RANGE at the ends


def accel_geometry(height: float) -> AccelGeometry:
  """Sized from the frame height so 1080 logical rows (a 1280x720 head unit) keep the designed proportions."""
  k = height / 1080
  width = max(6.0, round(ACCEL_WIDTH * k))
  grown = max(width, round(ACCEL_GROWN * k))
  strip = round(grown + 56 * k)
  top, bottom = height * 0.10, height * 0.85   # 75 % of the height, like the torque bar's share of the width
  return AccelGeometry(strip, strip / 2 - width / 2, width, grown, top, bottom, (top + bottom) / 2)


def dock_height(height: float) -> float:
  """Bottom band reserved for the torque bar at its grown thickness, so the view never draws under it."""
  return round((TORQUE_MAX + 2 * TORQUE_MARGIN) * height / 1080)


def view_rect(width: float, height: float) -> tuple[float, float, float, float]:
  """Where the birds-eye view draws: right of the acceleration strip and above the torque dock."""
  strip = accel_geometry(height).strip
  return strip, 0, width - strip, height - dock_height(height)


@dataclass(frozen=True)
class TorqueGeometry:
  cx: float        # centre (zero) x
  left: float
  right: float
  bottom: float    # the bar grows upward from here
  thin: float      # thickness at |utilization| <= 0.5
  thick: float     # thickness at |utilization| >= 1


def torque_geometry(width: float, height: float) -> TorqueGeometry:
  """Flat bar LENGTH_FRACTION of the frame width, centred under the birds-eye view (on the ego car)."""
  k = height / 1080
  strip = accel_geometry(height).strip
  cx = strip + (width - strip) / 2
  half = min(LENGTH_FRACTION * width / 2, width - cx - TORQUE_MARGIN * k, cx - strip - TORQUE_MARGIN * k)
  return TorqueGeometry(cx, cx - half, cx + half, height - TORQUE_MARGIN * k, TORQUE_MIN * k, TORQUE_MAX * k)


def torque_strength(utilization: float) -> float:
  """The arc's growth: 0 up to |utilization| 0.5, 1 at full."""
  return _clamp((abs(utilization) - 0.5) * 2)


def torque_color(utilization: float, alpha: float) -> tuple[int, int, int, int]:
  """The arc's indicator colour mid-reach: white, warming to orange past 0.75."""
  warmth = _clamp((abs(utilization) - 0.75) * 4)
  green = 1 - warmth + warmth * (115 / 255) + warmth * (85 / 255) * 0.5
  return 255, round(255 * green), round(255 * (1 - warmth)), round(255 * 0.9 * alpha)


def read_accel(sm) -> AccelReading | None:
  """The commanded and measured accel from the SubMaster; None before carState has been seen.

  The setpoint is carControl.actuators.accel while longActive and 0.0 otherwise: the plan's accel is not shown
  while openpilot is not commanding.
  """
  seen = sm.seen
  if not seen['carState']:
    return None
  car_state = sm['carState']
  long_active, desired = False, 0.0
  if seen['carControl']:
    control = sm['carControl']
    long_active = bool(control.longActive)
    if long_active:
      desired = _finite(control.actuators.accel)
  return AccelReading(desired, _finite(car_state.aEgo), long_active, bool(car_state.brakePressed))


def read_limits(sm, now_ns: int | None = None) -> AccelScale | None:
  """The planner's cruise floor and ceiling from starpilotLongitudinalPlan; None when absent, stale or implausible."""
  name = 'starpilotLongitudinalPlan'
  try:
    if not sm.seen[name]:
      return None
    alive = getattr(sm, 'alive', None)
    if isinstance(alive, dict) and alive.get(name) is False:
      return None
    received = getattr(sm, 'recv_time', None)
    if isinstance(received, dict) and now_ns is not None and received.get(name):
      if now_ns - int(received[name] * 1e9) > LIMITS_MAX_AGE_NS:
        return None
    plan = sm[name]
    lower, upper = _finite(plan.accelLimitMin), _finite(plan.accelLimitMax)
  except (AttributeError, KeyError, TypeError, ValueError):
    return None
  if lower is None or upper is None or not lower < 0 < upper:
    return None
  return AccelScale(lower, upper)


def read_blinkers(sm) -> tuple[bool, bool]:
  try:
    if not sm.seen['carState']:
      return False, False
    car_state = sm['carState']
    return bool(getattr(car_state, 'leftBlinker', False)), bool(getattr(car_state, 'rightBlinker', False))
  except (AttributeError, KeyError, TypeError):
    return False, False


def braking(reading: AccelReading | None, was_braking: bool = False) -> bool:
  if reading is None:
    return False
  if reading.brake_pressed:
    return True
  if not reading.long_active or reading.desired is None:
    return False
  return reading.desired < (BRAKE_RELEASE if was_braking else BRAKE_ACCEL)


def fill_color(accel: float, is_braking: bool, long_active: bool) -> tuple[int, int, int, int]:
  """Coloured only while longActive; neutral grey/white otherwise."""
  if not long_active:
    return (*IDLE, 110)
  if is_braking:
    return (*RED, 240)
  return (*(GREEN if accel >= 0 else NEUTRAL), 235)


def signal_alpha(elapsed: float) -> float:
  """The comma's turn-signal pulse (0..1): bright at each period start, decaying toward 20 %."""
  alpha = (1 / FPS) / (TURN_SIGNAL_RC + 1 / FPS)
  frames = (elapsed % TURN_SIGNAL_BLINK_PERIOD) * FPS
  return min(1.0, 0.2 + (2 - 0.2) * (1 - alpha) ** frames)


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
  def __init__(self, rl, text, source, *, feedback=torque_feedback, clock=time.monotonic, icon_path=TURN_SIGNAL_ICON):
    """source() -> (SubMaster, CarParams or None). clock() -> seconds, for the turn-signal pulse."""
    self.rl, self.text, self.source = rl, text, source
    self._feedback = feedback
    self._clock = clock
    self._icon_path = icon_path
    self._desired = _Filter(ACCEL_RC)
    self._actual = _Filter(ACCEL_RC)
    self._grow = _Filter(ACCEL_RC)
    self._lower = _Filter(ACCEL_RC)
    self._upper = _Filter(ACCEL_RC)
    self._torque = _Filter(TORQUE_RC)
    self._torque_alpha = _Filter(TORQUE_RC)
    self._braking = False
    self._label_key = None
    self._label = None
    self._geometry_key = None
    self._geometry = None
    self._signal_since = None
    self._icon = None
    self._icon_failed = False

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
    self._draw_accel(geometry, read_accel(sm), read_limits(sm, time.monotonic_ns()), height)
    self._draw_torque(sm, car_params, width, height)
    self._draw_signals(sm, width, height, geometry.strip)

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
  def _scale(self, limits: AccelScale | None, long_active: bool) -> AccelScale:
    target = limits if long_active and limits is not None else DEFAULT_SCALE
    return AccelScale(self._lower.update(target.lower), self._upper.update(target.upper))

  def _draw_accel(self, g: AccelGeometry, reading: AccelReading | None, limits: AccelScale | None, height: float) -> None:
    rl = self.rl
    strip = int(g.strip)
    k = height / 1080
    rl.draw_rectangle_gradient_v(0, 0, strip, int(height), rl.Color(*BACKGROUND_TOP, 255), rl.Color(*BACKGROUND_BOTTOM, 255))
    rl.draw_rectangle(strip - 1, 0, 1, int(height), rl.Color(255, 255, 255, 14))
    long_active = reading is not None and reading.long_active
    scale = self._scale(limits, long_active)
    track = TRACK if long_active else (*IDLE, 46)
    rl.draw_rectangle_rounded(rl.Rectangle(g.x, g.top, g.width, g.bottom - g.top), 1.0, 8, rl.Color(*track))
    tick = 8 * k
    for unit in range(math.ceil(scale.lower / LIMIT_FRACTION), math.floor(scale.upper / LIMIT_FRACTION) + 1):
      y = scale.y(g, unit)
      if unit == 0:
        rl.draw_rectangle_rec(rl.Rectangle(g.x - 2 * tick, y - k, g.width + 4 * tick, 2 * k), rl.Color(*TICK, 150))
      else:
        rl.draw_rectangle_rec(rl.Rectangle(g.x - tick - 3 * k, y - k / 2, tick, max(1.0, k)), rl.Color(*TICK, 90))
    if long_active and limits is not None:
      # The planner's floor and ceiling: short ticks across the bar at LIMIT_FRACTION of each half.
      for accel in (scale.lower, scale.upper):
        y = scale.y(g, accel)
        rl.draw_rectangle_rec(rl.Rectangle(g.x - 2 * tick, y - 1.5 * k, g.width + 4 * tick, 3 * k), rl.Color(*LIMIT, 220))
    if reading is None:
      self._desired.x = self._actual.x = self._grow.x = None
      self._braking = False
      return
    self._braking = braking(reading, self._braking)
    if reading.desired is not None:
      desired = self._desired.update(reading.desired)
      color = rl.Color(*fill_color(desired, self._braking, long_active))
      # Like the torque bar's strength: thicker as the command reaches the limit (full at it).
      grow = self._grow.update(_clamp((scale.utilization(desired) - 0.8) * 5) if long_active else 0.0)
      width = g.width + (g.grown - g.width) * grow
      y = scale.y(g, desired)
      top, bottom = min(y, g.zero), max(y, g.zero)
      if bottom - top >= 0.5:
        rl.draw_rectangle_rounded(rl.Rectangle(g.center - width / 2, top, width, bottom - top), 1.0, 8, color)
      if long_active:
        self._draw_label(desired, color, g, k)
    if reading.actual is not None:
      y = scale.y(g, self._actual.update(reading.actual))
      w, h = g.grown + 12 * k, max(3.0, 5 * k)
      x = g.center - w / 2
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
  def _draw_torque(self, sm, car_params, width, height) -> None:
    try:
      feedback = self._feedback(sm, car_params)
    except (AttributeError, KeyError, TypeError, ValueError):
      feedback = None
    utilization = _finite(getattr(feedback, 'utilization', None))
    active = utilization is not None and getattr(feedback, 'reason', '') != 'lateral_inactive'
    # Inactive: the indicator settles back to the centre and fades; the track stays.
    torque = self._torque.update(_clamp(utilization, -1.0, 1.0) if active else 0.0)
    alpha = self._torque_alpha.update(1.0 if active else 0.0) if self._torque_alpha.x is not None else self._torque_alpha.update(0.0)
    g = torque_geometry(width, height)
    strength = torque_strength(torque) * alpha
    thickness = g.thin + (g.thick - g.thin) * strength
    top = g.bottom - thickness
    rl = self.rl
    track_alpha = round(255 * (0.25 + 0.25 * strength)) if active else 60
    track = (255, 255, 255, track_alpha) if active else (*IDLE, track_alpha)
    rl.draw_rectangle_rounded(rl.Rectangle(g.left, top, g.right - g.left, thickness), 1.0, 12, rl.Color(*track))
    reach = torque * (g.right - g.left) / 2
    if alpha > 0.01 and abs(reach) >= 1.0:
      x = min(g.cx, g.cx + reach)
      rl.draw_rectangle_rounded(rl.Rectangle(x, top, abs(reach), thickness), 1.0, 12, rl.Color(*torque_color(torque, alpha)))
    if abs(torque) < 0.5 or not active:
      # The arc's centre dot, as a short notch across the bar.
      k = height / 1080
      rl.draw_rectangle_rounded(rl.Rectangle(g.cx - 3 * k, top - 4 * k, 6 * k, thickness + 8 * k), 1.0, 6,
                                rl.Color(*DOT, 230))

  # --- turn signals ---
  def _load_icon(self, height):
    """The comma's turn-signal icon as a texture, shrunk once on the CPU; None (and never retried) on failure."""
    if self._icon is not None or self._icon_failed:
      return self._icon
    rl = self.rl
    try:
      image = rl.load_image(str(self._icon_path))
      try:
        if image.data == rl.ffi.NULL or image.width <= 0 or image.height <= 0:
          raise ValueError('turn signal icon unreadable')
        if image.height > height:
          rl.image_resize(image, round(image.width * height / image.height), round(height))
        texture = rl.load_texture_from_image(image)
      finally:
        if image.data != rl.ffi.NULL:
          rl.unload_image(image)
      if texture.id == 0:
        raise ValueError('turn signal texture not created')
      rl.set_texture_filter(texture, rl.TextureFilter.TEXTURE_FILTER_BILINEAR)
      self._icon = texture
    except Exception:
      self._icon_failed = True
    return self._icon

  def _draw_signals(self, sm, width, height, strip) -> None:
    left, right = read_blinkers(sm)
    if not (left or right):
      self._signal_since = None
      return
    now = self._clock()
    if self._signal_since is None or now < self._signal_since:
      self._signal_since = now
    alpha = signal_alpha(now - self._signal_since)
    k = height / 1080
    h = SIGNAL_HEIGHT * k
    icon = self._load_icon(h)
    w = h * (icon.width / icon.height if icon is not None else 208 / 192)
    margin = SIGNAL_MARGIN * k
    for on, x, flip in ((left, strip + margin, False), (right, width - margin - w, True)):
      if on:
        self._draw_arrow(icon, x, margin, w, h, flip, alpha)

  def _draw_arrow(self, icon, x, y, w, h, flip, alpha) -> None:
    rl = self.rl
    if icon is not None:
      source = rl.Rectangle(0, 0, -icon.width if flip else icon.width, icon.height)
      rl.draw_texture_pro(icon, source, rl.Rectangle(x, y, w, h), rl.Vector2(0, 0), 0.0,
                          rl.Color(255, 255, 255, round(255 * alpha)))
      return
    # Fallback: a filled arrow in the icon's footprint (head 55 % of the width, shaft 40 % of the height).
    color = rl.Color(*SIGNAL_GREEN, round(255 * alpha))
    pad = 0.12 * w
    head = 0.45 * (w - 2 * pad)
    shaft_h = 0.36 * h
    cy = y + h / 2
    if flip:
      tip, base = x + w - pad, x + w - pad - head
      rl.draw_rectangle_rec(rl.Rectangle(x + pad, cy - shaft_h / 2, base - x - pad, shaft_h), color)
    else:
      tip, base = x + pad, x + pad + head
      rl.draw_rectangle_rec(rl.Rectangle(base, cy - shaft_h / 2, x + w - pad - base, shaft_h), color)
    points = [(tip, cy), (base, y + 0.8 * h), (base, y + 0.2 * h)]
    (ax, ay), (bx, by), (cx, cy2) = points
    if (bx - ax) * (cy2 - ay) - (by - ay) * (cx - ax) > 0:
      points[1], points[2] = points[2], points[1]  # raylib wants counter-clockwise on screen
    rl.draw_triangle(*(rl.Vector2(px, py) for px, py in points), color)

  def close(self) -> None:
    if self._icon is not None:
      self.rl.unload_texture(self._icon)
      self._icon = None
