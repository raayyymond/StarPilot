"""Development preview of the birds-eye renderer on synthetic data (curve + leads).

  python -m openpilot.starpilot.ui.preview_birds_eye --output /tmp/birds_eye.png [--show]

Writes <output> (1860x1080 frame, panel over a stand-in camera) and <stem>_full.png (full mode).
"""
import argparse
import math
from pathlib import Path
from types import SimpleNamespace

import numpy as np
from openpilot.starpilot.ui.raylib import rl

from openpilot.selfdrive.ui.onroad.birds_eye_renderer import BirdsEyeRenderer, BitmapText
from openpilot.starpilot.ui.presentation import BitmapFonts, Profile, default_font_directory

WIDTH, HEIGHT = 1860, 1080
PANEL = (1310, 290, 520, 600)
X_IDXS = 10.0 * (np.arange(33) / 32) ** 2 * 19.2  # ~0..192 m, the model's quadratic spacing


def _xyz(xs, ys):
  return SimpleNamespace(x=list(xs), y=list(ys), z=[0.0] * len(xs))


def synthetic_messages(radius=-220.0, steering_deg=8.0, lead_d=26.0, lead_vrel=-3.5, blinker=False):
  """A left curve (negative radius bends left; y is right-positive) with a closing lead and a far lead."""
  curve = X_IDXS ** 2 / (2 * radius)
  lanes = [_xyz(X_IDXS, curve + off) for off in (-5.4, -1.8, 1.8, 5.4)]
  edges = [_xyz(X_IDXS, curve + off) for off in (-7.2, 7.2)]
  t = np.arange(33) / 32 * 10
  path_x = np.minimum(t * 22.0, 150.0)
  model = SimpleNamespace(
    position=_xyz(path_x, path_x ** 2 / (2 * radius) + 0.15),
    acceleration=SimpleNamespace(x=list(np.linspace(-0.6, 0.4, 33))),
    laneLines=lanes, laneLineProbs=[0.55, 0.92, 0.94, 0.4],
    roadEdges=edges, roadEdgeStds=[0.25, 0.35],
    leadsV3=[SimpleNamespace(prob=0.9, x=[lead_d + 1.5], y=[lead_d ** 2 / (2 * radius)], v=[20.0])],
  )
  lead = lambda d, y, v: SimpleNamespace(present=True, dRel=d, yRel=y, vRel=v)  # noqa: E731
  radar = SimpleNamespace(leadOne=lead(lead_d, -(lead_d ** 2 / (2 * radius)), lead_vrel),
                          leadTwo=lead(58.0, -(58.0 ** 2 / (2 * radius) + 3.6), 1.0))
  car = SimpleNamespace(steeringAngleDeg=steering_deg, leftBlinker=blinker, rightBlinker=False)
  return model, radar, car


def _fake_camera():
  rl.draw_rectangle_gradient_v(30, 30, WIDTH - 60, HEIGHT // 2, rl.Color(70, 92, 120, 255), rl.Color(120, 130, 135, 255))
  rl.draw_rectangle_gradient_v(30, HEIGHT // 2, WIDTH - 60, HEIGHT // 2 - 30, rl.Color(60, 62, 66, 255), rl.Color(36, 37, 40, 255))
  rl.draw_triangle(rl.Vector2(930, 520), rl.Vector2(300, 1050), rl.Vector2(1560, 1050), rl.Color(50, 52, 56, 255))


def _export(target, path: Path) -> None:
  image = rl.load_image_from_texture(target.texture)
  try:
    rl.image_flip_vertical(image)
    if not rl.export_image(image, str(path)):
      raise RuntimeError(f"Unable to export {path}")
  finally:
    rl.unload_image(image)


def main() -> None:
  parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  parser.add_argument("--output", type=Path, default=Path("birds_eye.png"))
  parser.add_argument("--show", action="store_true", help="keep a window open, animating the scene (space toggles panel/full)")
  parser.add_argument("--blinker", action="store_true")
  parser.add_argument("--disengaged", action="store_true")
  parser.add_argument("--experimental", action="store_true")
  args = parser.parse_args()
  if not args.show:
    rl.set_config_flags(rl.ConfigFlags.FLAG_WINDOW_HIDDEN)
  rl.init_window(WIDTH // 2 if args.show else 64, HEIGHT // 2 if args.show else 64, "Birds-eye preview")
  target = rl.load_render_texture(WIDTH, HEIGHT)
  fonts = BitmapFonts(Profile.LARGE, default_font_directory())
  try:
    renderer = BirdsEyeRenderer(text=BitmapText(fonts))
    renderer.clock = lambda: 0.0  # blinkers in their visible phase for stills
    model, radar, car = synthetic_messages(blinker=args.blinker)
    for _ in range(3):  # several frames so the smoothing settles
      renderer.ingest(model, radar, car, engaged=not args.disengaged, experimental=args.experimental, allow_throttle=True)

    def frame(full: bool) -> None:
      rl.begin_texture_mode(target)
      try:
        rl.clear_background(rl.BLACK)
        if full:
          renderer.panel = False
          renderer.render(rl.Rectangle(30, 30, WIDTH - 60, HEIGHT - 60))
        else:
          _fake_camera()
          renderer.panel = True
          renderer.render(rl.Rectangle(*PANEL))
      finally:
        rl.end_texture_mode()

    for _ in range(4):
      frame(False)
    _export(target, args.output)
    frame(True)
    _export(target, args.output.with_name(args.output.stem + "_full" + args.output.suffix))
    print(f"wrote {args.output} and {args.output.stem}_full{args.output.suffix}")

    if args.show:
      renderer.clock = rl.get_time
      full = False
      while not rl.window_should_close():
        t = rl.get_time()
        model, radar, car = synthetic_messages(radius=-220 + 120 * math.sin(t * 0.3), steering_deg=12 * math.sin(t * 0.3),
                                               lead_d=26 + 8 * math.sin(t * 0.7), blinker=int(t / 4) % 2 == 1)
        renderer.ingest(model, radar, car, engaged=not args.disengaged, experimental=args.experimental, allow_throttle=True)
        if rl.is_key_pressed(rl.KeyboardKey.KEY_SPACE):
          full = not full
        frame(full)
        rl.begin_drawing()
        rl.clear_background(rl.BLACK)
        rl.draw_texture_pro(target.texture, rl.Rectangle(0, 0, WIDTH, -HEIGHT), rl.Rectangle(0, 0, WIDTH / 2, HEIGHT / 2),
                            rl.Vector2(0, 0), 0, rl.WHITE)
        rl.end_drawing()
  finally:
    fonts.close()
    rl.unload_render_texture(target)
    rl.close_window()


if __name__ == "__main__":
  main()
