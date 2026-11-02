#!/usr/bin/env python3
"""Live desktop preview of the Android Auto projection stream (./aapreview).

Runs the real car-view renderer (``current_car_ui``, headless EGL) exactly the
way starportd does: a ``FrameConsumer`` publishes the negotiated geometry and
demand, ``ViewSource`` spawns the renderer, and each frame it publishes is shown
in a window scaled to fit. The title shows WxH@fps and the measured rates.

  ./aapreview                            1280x720 @ 30, NV12 like the comma
  ./aapreview --res 1080p --fps 30       any head-unit size (480p, 720p, 1080p)
  ./aapreview --width 1280 --height 720 --margins 0x240   a wide screen's safe area
  ./aapreview --encode                   libx264 round trip: what the head unit decodes
  ./aapreview --encode --mp4 out.mp4     also record the encoded stream
  ./aapreview --headless --frames 30 --out /tmp/aa   PNGs, no window
  ./aapreview --fake-head-unit           full TLS session/negotiation over localhost
  ./aapreview --stream-content full      the camera composition instead of the birds-eye-only view

Data: the renderer reads whatever cereal services are published in this
process's OPENPILOT_PREFIX. ``./dev`` gives every host command one shared
prefix, so with nothing publishing the car view shows its standby screen. To
watch a replay or the synthetic publisher, use their private prefix:

  ./onroad --replay-only --prefix replay-aa --demo     (terminal 1)
  ./aapreview --prefix replay-aa                       (terminal 2)

(``--prefix`` sets OPENPILOT_PREFIX for the renderer; ``--params-root`` sets
PARAMS_ROOT the same way when the publisher uses private Params.)

Window keys: K forces a keyframe (a new focus epoch); S saves a PNG; Esc quits.
Mouse clicks are sent to the renderer as Android Auto touches.
"""

from __future__ import annotations

import argparse
import os
import shutil
import sys
import tempfile
import time
from collections import deque
from fractions import Fraction
from pathlib import Path

RESOLUTIONS = {"480p": (800, 480), "720p": (1280, 720), "1080p": (1920, 1080)}
STARTUP_TIMEOUT = 90.0  # s; a cold renderer under llvmpipe compiles every shader and font


def parse_args(argv=None) -> argparse.Namespace:
  parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  parser.add_argument("--res", choices=sorted(RESOLUTIONS), help="head-unit resolution (overrides --width/--height)")
  parser.add_argument("--width", type=int, default=1280)
  parser.add_argument("--height", type=int, default=720)
  parser.add_argument("--fps", type=int, default=30, help="requested frame rate (the comma's hardware encoder runs 30)")
  parser.add_argument("--margins", default="0x0", help="head-unit margins WxH, e.g. 0x240 for a 1280x480 wide screen")
  parser.add_argument("--pixel-format", choices=("nv12", "rgba"), default=None,
                      help="frame format requested from the renderer (default: nv12, or rgba with --encode like the software encoder)")
  parser.add_argument("--sync-readback", action="store_true", help="disable asynchronous GPU readback")
  parser.add_argument("--encode", action="store_true", help="encode with the software H264Encoder and show the decoded stream")
  parser.add_argument("--bitrate", type=int, default=6000, help="kbps for --encode (starport default 6000)")
  parser.add_argument("--mp4", type=Path, help="write the encoded stream (implies --encode)")
  parser.add_argument("--headless", action="store_true", help="no window: write PNGs to --out")
  parser.add_argument("--frames", type=int, default=0, help="stop after this many frames (headless default 30)")
  parser.add_argument("--out", type=Path, help="directory for PNG frames")
  parser.add_argument("--every", type=int, default=1, help="headless: save every Nth frame")
  parser.add_argument("--prefix", help="OPENPILOT_PREFIX to read cereal data from (e.g. replay-NAME)")
  parser.add_argument("--params-root", help="PARAMS_ROOT for the renderer")
  parser.add_argument("--stream-content", choices=("birds_eye", "full"),
                      help="what the car view draws while driving (default: starport config stream_content, birds_eye)")
  parser.add_argument("--fake-head-unit", action="store_true",
                      help="negotiate and stream through a real ProjectionSession to a localhost FakeHeadUnit")
  parser.add_argument("--resolution", choices=("auto", "480p", "720p", "1080p"), default="auto",
                      help="--fake-head-unit: starport 'resolution' preference applied to the head unit's offer")
  parser.add_argument("--hw-max-height", type=int, choices=(720, 1080),
                      help="--fake-head-unit: negotiate as if the comma hardware encoder's limit were this (config hw_encoder_max_height)")
  parser.add_argument("--offer", default="480p,720p,1080p", help="--fake-head-unit: resolutions the fake head unit offers")
  parser.add_argument("--scale", type=float, default=0.0, help="initial window scale (default: fit 1280 wide)")
  parser.add_argument("--renderer-log", type=Path, help="renderer output (default: a temporary file, printed on failure)")
  args = parser.parse_args(argv)
  if args.res:
    args.width, args.height = RESOLUTIONS[args.res]
  try:
    margin_w, margin_h = (int(value) for value in args.margins.lower().split("x"))
  except ValueError:
    parser.error("--margins must be WxH")
  args.margin_w, args.margin_h = margin_w, margin_h
  if args.mp4 or args.fake_head_unit:
    args.encode = True
  if args.headless:
    args.frames = args.frames or 30
    if args.out is None:
      parser.error("--headless needs --out DIR")
  if args.pixel_format is None:
    args.pixel_format = "rgba" if args.encode else "nv12"
  if not 1 <= args.fps <= 60:
    parser.error("--fps must be 1-60")
  return args


def nv12_to_rgba(data: bytes, width: int, height: int):
  """BT.601 limited range, the inverse of gpu_nv12's conversion (for display only)."""
  import numpy as np
  try:
    import av  # swscale: ~3 ms at 1080p, against ~60 ms for numpy
    frame = av.VideoFrame(width, height, "nv12")
    if frame.planes[0].line_size == width and frame.planes[1].line_size == width:
      frame.planes[0].update(memoryview(data)[:width * height])
      frame.planes[1].update(memoryview(data)[width * height:width * height * 3 // 2])
      return frame.to_ndarray(format="rgba")
  except ImportError:
    pass
  frame = np.frombuffer(data, np.uint8, width * height * 3 // 2)
  y = frame[:width * height].reshape(height, width).astype(np.int32) - 16
  uv = frame[width * height:].reshape(height // 2, width // 2, 2).astype(np.int32) - 128
  u = uv[..., 0].repeat(2, axis=0).repeat(2, axis=1)
  v = uv[..., 1].repeat(2, axis=0).repeat(2, axis=1)
  c = 298 * y
  rgba = np.empty((height, width, 4), np.uint8)
  rgba[..., 0] = np.clip((c + 409 * v + 128) >> 8, 0, 255)
  rgba[..., 1] = np.clip((c - 100 * u - 208 * v + 128) >> 8, 0, 255)
  rgba[..., 2] = np.clip((c + 516 * u + 128) >> 8, 0, 255)
  rgba[..., 3] = 255
  return rgba


def frame_rgba(frame):
  import numpy as np
  from openpilot.starpilot.system.starport.frame_source import FORMAT_NV12
  if frame.pixel_format == FORMAT_NV12:
    return nv12_to_rgba(frame.data, frame.width, frame.height)
  return np.frombuffer(frame.data, np.uint8, frame.width * frame.height * 4).reshape(frame.height, frame.width, 4)


class EncodedStream:
  """The daemon's software encoder, plus a decoder and an optional MP4 of exactly what was sent."""

  def __init__(self, width: int, height: int, fps: int, bitrate_kbps: int, mp4: Path | None):
    from openpilot.starpilot.system.starport.encoder import H264Encoder
    self.encoder = H264Encoder(width, height, fps=fps, bitrate_kbps=bitrate_kbps)
    av = self.encoder.av
    self.av = av
    self.decoder = av.CodecContext.create("h264", "r")
    self.fps = fps
    self.packets = 0
    self.bytes = 0
    self.keyframes = 0
    self.needs_keyframe = True  # every media epoch starts with a keyframe
    self.mp4 = self.stream = None
    self._first_ns: int | None = None
    self._last_pts = -1
    if mp4 is not None:
      mp4.parent.mkdir(parents=True, exist_ok=True)
      self.mp4 = av.open(str(mp4), "w", format="mp4")
      self.stream = self.mp4.add_stream("h264", rate=fps)
      self.stream.width, self.stream.height = width, height

  def encode(self, rgba, captured_ns: int) -> tuple[bytes, bool]:
    data, keyframe = self.encoder.encode_rgba(memoryview(rgba).cast("B"), keyframe=self.needs_keyframe)
    self.needs_keyframe = False
    self.packets += 1
    self.bytes += len(data)
    self.keyframes += keyframe
    if self.mp4 is not None:
      packet = self.av.Packet(data)
      packet.stream = self.stream
      # Real capture times: onroad the renderer follows the 20 Hz camera, not the requested rate.
      if self._first_ns is None:
        self._first_ns = captured_ns
      pts = max(self._last_pts + 1, (captured_ns - self._first_ns) // 1_000_000)
      self._last_pts = pts
      packet.pts = packet.dts = pts
      packet.time_base = Fraction(1, 1000)
      packet.is_keyframe = keyframe
      self.mp4.mux(packet)
    return data, keyframe

  def decode(self, data: bytes):
    frames = self.decoder.decode(self.av.Packet(data))
    return frames[-1].to_ndarray(format="rgba") if frames else None

  def close(self) -> None:
    self.encoder.close()
    if self.mp4 is not None:
      self.mp4.close()


def save_png(rgba, path: Path) -> None:
  from PIL import Image
  path.parent.mkdir(parents=True, exist_ok=True)
  Image.fromarray(rgba[..., :3]).save(path)


class Window:
  def __init__(self, width: int, height: int, scale: float):
    import pyray as rl
    self.rl = rl
    scale = scale or min(1.0, 1280 / width)
    rl.set_config_flags(rl.ConfigFlags.FLAG_WINDOW_RESIZABLE | rl.ConfigFlags.FLAG_VSYNC_HINT)
    rl.set_trace_log_level(rl.TraceLogLevel.LOG_WARNING)
    rl.init_window(round(width * scale), round(height * scale), "Android Auto preview")
    rl.set_exit_key(rl.KeyboardKey.KEY_ESCAPE)
    image = rl.gen_image_color(width, height, rl.BLACK)
    rl.image_format(image, rl.PixelFormat.PIXELFORMAT_UNCOMPRESSED_R8G8B8A8)
    self.texture = rl.load_texture_from_image(image)
    rl.unload_image(image)
    rl.set_texture_filter(self.texture, rl.TextureFilter.TEXTURE_FILTER_BILINEAR)
    self.width, self.height = width, height
    self.destination = rl.Rectangle(0, 0, width, height)

  def should_close(self) -> bool:
    return self.rl.window_should_close()

  def show(self, rgba) -> None:
    rl = self.rl
    if rgba is not None:
      import numpy as np
      rgba = np.ascontiguousarray(rgba)
      rl.update_texture(self.texture, rl.ffi.cast("void *", rl.ffi.from_buffer(rgba)))
    screen_w, screen_h = rl.get_screen_width(), rl.get_screen_height()
    scale = min(screen_w / self.width, screen_h / self.height)
    w, h = self.width * scale, self.height * scale
    self.destination = rl.Rectangle((screen_w - w) / 2, (screen_h - h) / 2, w, h)
    rl.begin_drawing()
    rl.clear_background(rl.Color(24, 24, 24, 255))
    rl.draw_texture_pro(self.texture, rl.Rectangle(0, 0, self.width, self.height), self.destination,
                        rl.Vector2(0, 0), 0.0, rl.WHITE)
    rl.end_drawing()

  def frame_point(self) -> tuple[float, float] | None:
    """Mouse position in encoded-frame pixels, or None outside the picture."""
    rl, d = self.rl, self.destination
    mouse = rl.get_mouse_position()
    x, y = (mouse.x - d.x) / d.width * self.width, (mouse.y - d.y) / d.height * self.height
    return (x, y) if 0 <= x < self.width and 0 <= y < self.height else None

  def touches(self, request) -> list:
    """Clicks as Android Auto touches, normalized within the visible (margin-free) area like the session's TouchMapper."""
    from openpilot.starpilot.system.starport.touch import TouchEvent
    rl = self.rl
    kinds = []
    if rl.is_mouse_button_pressed(rl.MouseButton.MOUSE_BUTTON_LEFT):
      kinds.append("down")
    elif rl.is_mouse_button_down(rl.MouseButton.MOUSE_BUTTON_LEFT) and (delta := rl.get_mouse_delta()) and (delta.x or delta.y):
      kinds.append("move")
    if rl.is_mouse_button_released(rl.MouseButton.MOUSE_BUTTON_LEFT):
      kinds.append("up")
    point = self.frame_point()
    if not kinds:
      return []
    if point is None:
      return [TouchEvent("cancel", 0, 0)]
    visible_w, visible_h = request.width - request.margin_w, request.height - request.margin_h
    x = min(max((point[0] - request.left) / visible_w, 0.0), 1.0)
    y = min(max((point[1] - request.top) / visible_h, 0.0), 1.0)
    return [TouchEvent(kind, x, y) for kind in kinds]

  def key(self, key) -> bool:
    return self.rl.is_key_pressed(key)

  def title(self, text: str) -> None:
    self.rl.set_window_title(text)

  def close(self) -> None:
    self.rl.unload_texture(self.texture)
    self.rl.close_window()


class Rate:
  def __init__(self, window: float = 2.0):
    self.window = window
    self.times: deque[float] = deque()

  def tick(self, now: float) -> None:
    self.times.append(now)
    while self.times and now - self.times[0] > self.window:
      self.times.popleft()

  def value(self, now: float) -> float:
    while self.times and now - self.times[0] > self.window:
      self.times.popleft()
    return len(self.times) / self.window


def configure_environment(args) -> None:
  """Before anything imports messaging or Params: the renderer inherits this environment."""
  if args.prefix:
    ipc = Path("/tmp" if sys.platform == "darwin" else "/dev/shm") / f"msgq_{args.prefix}"
    if not ipc.is_dir():
      raise SystemExit(f"aapreview: nothing is publishing in prefix {args.prefix} ({ipc} does not exist); " +
                       f"start ./simui --prefix {args.prefix} or ./onroad --prefix {args.prefix} ... first")
    os.environ["OPENPILOT_PREFIX"] = args.prefix
  if args.params_root:
    os.environ["PARAMS_ROOT"] = args.params_root
  # The renderer runs 'onroad' layout code; never let it think it owns the comma's screen.
  os.environ.setdefault("STARPILOT_PROJECTION_READ_ONLY", "1")


def renderer_command(frames: str, touch: str, control: str, stream_content: str | None = None) -> list[str]:
  return [sys.executable, "-m", "openpilot.starpilot.system.starport.current_car_ui",
          "--frames", frames, "--touch", touch, "--control", control,
          *(["--stream-content", stream_content] if stream_content else [])]


def print_tail(path: Path, lines: int = 40) -> None:
  try:
    text = path.read_text(errors="replace").splitlines()[-lines:]
  except OSError:
    return
  if text:
    print(f"--- renderer output ({path}) ---", file=sys.stderr)
    print("\n".join(text), file=sys.stderr)


def frame_request(args, fps: int):
  from openpilot.starpilot.system.starport.frame_source import FLAG_ASYNC_READBACK, FLAG_NV12, FrameRequest
  flags = (FLAG_NV12 if args.pixel_format == "nv12" else 0) | (0 if args.sync_readback else FLAG_ASYNC_READBACK)
  return FrameRequest(args.width, args.height, args.margin_w, args.margin_h, int(1e6 / fps), flags)


def run(args) -> int:
  configure_environment(args)
  from openpilot.starpilot.system.starport.view import ViewSource

  fps = args.fps
  head_unit = None
  if args.fake_head_unit:
    head_unit = FakeLink(args)
    mode = head_unit.mode
    args.width, args.height, args.margin_w, args.margin_h = mode.width, mode.height, mode.margin_width, mode.margin_height
    fps = min(fps, mode.fps)
    args.encode = True
    print(f"fake head unit negotiated {mode.as_dict()}", flush=True)
  request = frame_request(args, fps)
  # Paths of our own, so a preview never collides with a starportd running on this machine.
  scratch = Path(tempfile.mkdtemp(prefix="aapreview-"))
  frames_path = f"/dev/shm/starpilot_aapreview_{os.getpid()}"
  touch_path, control_path = str(scratch / "touch.sock"), str(scratch / "control.sock")
  log_path = args.renderer_log or scratch / "car_ui.log"

  def log(event, **fields):
    print(f"[{event}] " + " ".join(f"{key}={value}" for key, value in fields.items()), flush=True)

  stream = EncodedStream(args.width, args.height, fps, args.bitrate, args.mp4) if args.encode else None
  window = None if args.headless else Window(args.width, args.height, args.scale)
  source = ViewSource("car", request, log, car_path=frames_path, touch_path=touch_path, control_path=control_path,
                      renderer_command=renderer_command(frames_path, touch_path, control_path, args.stream_content), renderer_log=log_path)
  pipeline = f"{args.pixel_format}{' -> libx264' if stream else ''}"
  print(f"preview {args.width}x{args.height}@{fps} margins {args.margin_w}x{args.margin_h} {pipeline}; " +
        f"content {args.stream_content or 'from config'}; " +
        f"prefix {os.environ.get('OPENPILOT_PREFIX', '(none)')}; renderer log {log_path}", flush=True)
  received = shown = 0
  render_rate, encode_rate = Rate(), Rate()
  started = last_title = time.monotonic()
  result = 0
  try:
    while True:
      now = time.monotonic()
      if window is not None and window.should_close():
        break
      source.demand(1.0)
      frame = source.latest()
      if frame is None and source.waiting_for_first_frame and now - started > STARTUP_TIMEOUT:
        raise RuntimeError(f"renderer produced no frame in {STARTUP_TIMEOUT:.0f} s")
      if source.fallback_reason:
        raise RuntimeError(f"renderer failed: {source.fallback_reason}")
      source.check(now)
      if window is not None:
        if window.key(window.rl.KeyboardKey.KEY_K) and stream is not None:
          stream.needs_keyframe = True
        if source.touch is not None and (touches := window.touches(request)):
          source.send_touches(touches)
      image = None
      if frame is not None:
        received += 1
        render_rate.tick(now)
        rgba = frame_rgba(frame)
        if stream is not None:
          data, keyframe = stream.encode(rgba, frame.captured_ns)
          encode_rate.tick(time.monotonic())
          if head_unit is not None:
            data = head_unit.send(data, frame.captured_ns, keyframe)
          image = stream.decode(data) if data is not None else None
        else:
          image = rgba
        if image is not None:
          shown += 1
          if args.out is not None and (shown - 1) % max(1, args.every) == 0:
            save_png(image, args.out / f"frame_{shown:05d}.png")
      if window is not None:
        if image is not None and window.key(window.rl.KeyboardKey.KEY_S):
          out = args.out or Path.cwd()
          save_png(image, out / f"aapreview_{int(time.monotonic() * 1000)}.png")
        window.show(image)
      elif frame is None:
        time.sleep(0.002)
      if args.frames and shown >= args.frames:
        break
      if now - last_title >= 0.5:
        last_title = now
        title = f"Android Auto {args.width}x{args.height}@{fps} - render {render_rate.value(now):.1f} fps"
        if stream is not None:
          kbps = stream.bytes * 8 / 1000 / max(1e-3, now - started)
          title += f", encode {encode_rate.value(now):.1f} fps {stream.encoder.last_encode_ms:.1f} ms {kbps:.0f} kbps"
        if head_unit is not None:
          title += f", head unit acked {head_unit.session.acked}"
        if window is not None:
          window.title(title)
        elif received and int(now - started) % 5 == 0:
          print(title, flush=True)
  except KeyboardInterrupt:
    pass
  except RuntimeError as error:
    print(f"aapreview: {error}", file=sys.stderr)
    result = 1
  finally:
    source.close()
    if window is not None:
      window.close()
    if stream is not None:
      stream.close()
    if head_unit is not None:
      head_unit.close()
    if result:
      print_tail(Path(log_path))
    shutil.rmtree(scratch, ignore_errors=True)
  elapsed = time.monotonic() - started
  print(f"{received} frames received, {shown} shown in {elapsed:.1f} s" +
        (f"; {stream.packets} encoded ({stream.keyframes} keyframes, {stream.bytes} bytes)" if stream else "") +
        (f"; PNGs in {args.out}" if args.out else "") + (f"; MP4 {args.mp4}" if args.mp4 else ""), flush=True)
  return result


class FakeLink:
  """A real ProjectionSession (TLS, discovery, video negotiation, focus, ACK window) to the test FakeHeadUnit.

  Negotiation applies the starport ``resolution`` preference to what the fake head
  unit offers, as ``Supervisor._project`` does. The head unit decodes nothing; the
  preview decodes the access units it accepted.
  """

  def __init__(self, args):
    import socket
    from openpilot.starpilot.system.starport.session import ProjectionSession
    from openpilot.starpilot.system.starport.tests import fake_head_unit as fake
    from openpilot.starpilot.system.starport.wire import field
    names = {"480p": 1, "720p": 2, "1080p": 3}
    offered = [names[name.strip()] for name in args.offer.split(",") if name.strip()]
    scripted = fake.discovery_response

    from openpilot.starpilot.system.starport.session import choose_video_mode
    from openpilot.starpilot.system.starport.wire import parse_fields
    configs = [fake.video_config(r, args.margin_w, args.margin_h) for r in offered]
    # The scripted head unit always accepts configuration index 1. Put the mode the phone will choose there
    # (behind an H.265 offer the phone skips), so negotiation runs unmodified.
    max_height = None if args.hw_max_height is None else args.hw_max_height
    chosen = choose_video_mode([{"id": 3, "video_configs": [parse_fields(c) for c in configs]}], args.resolution, max_height)
    winner = configs.pop(chosen.config_index)
    configs = [field(1, 2) + field(10, 7), winner, *configs]

    def discovery_response(video_channel: int = 3, input_channel: int = 1, **_):
      # The scripted head unit offers 480p and 720p; offer the requested list with the preview's margins instead.
      av = field(1, 3) + b"".join(field(4, config) for config in configs)
      video = field(1, video_channel) + field(3, av)
      audio = field(1, 4) + field(3, field(1, 1))
      touch = field(1, input_channel) + field(4, field(1, 84) + field(1, 4) + field(2, field(1, 1280) + field(2, 720)))
      return field(1, video) + field(1, audio) + field(1, touch) + field(2, "Honda") + field(3, "Preview")

    self.directory = Path(tempfile.mkdtemp(prefix="aapreview-hu-"))
    identity = fake.make_identity(self.directory)
    fake.discovery_response = discovery_response
    try:
      self.hu = fake.FakeHeadUnit(identity)
      sock = socket.create_connection(("127.0.0.1", self.hu.port), timeout=5)
      self.session = ProjectionSession(sock, str(identity["phone_cert"]), str(identity["phone_key"]),
                                       lambda event, **f: None, None)
      self.session.resolution = args.resolution
      self.session.max_video_height = max_height
      self.session.authenticate()
      self.mode = self.session.start("StarPilot", "comma.ai")
    finally:
      fake.discovery_response = scripted
    deadline = time.monotonic() + 5
    while not self.session.focused and time.monotonic() < deadline:
      self.session.pump(0.05)
    if not self.session.focused:
      raise RuntimeError("fake head unit did not grant video focus")

  def send(self, data: bytes, captured_ns: int, keyframe: bool) -> bytes | None:
    session = self.session
    deadline = time.monotonic() + 1.0
    while not session.can_send() and time.monotonic() < deadline:
      session.pump(0.01)
    session.pump(0.0)
    if not session.can_send():
      return None
    if session.needs_keyframe and not keyframe:
      return None  # the caller forces a keyframe at each new epoch
    session.send_frame(data, captured_ns // 1000, keyframe=keyframe)
    return self.hu.frames[-1][1] if self._wait_received() else data

  def _wait_received(self) -> bool:
    count = self.session.frames_sent
    deadline = time.monotonic() + 0.5
    while len(self.hu.frames) < count and time.monotonic() < deadline:
      time.sleep(0.001)
    return len(self.hu.frames) >= count

  def close(self) -> None:
    try:
      self.session.shutdown()
    except Exception:
      pass
    self.session.peer.close()
    self.hu.close()
    shutil.rmtree(self.directory, ignore_errors=True)


def main(argv=None) -> int:
  return run(parse_args(argv))


if __name__ == "__main__":
  raise SystemExit(main())
