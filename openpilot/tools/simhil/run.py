"""Drive the real compact/large StarPilot UI from a synthetic scenario, without hardware.

Run through the host runtime: ./simui --c4 --scenario s_curve
(or ./dev python -m openpilot.tools.simhil.run ...). The session gets a private
replay-sim... messaging prefix and disposable Params like ./onroad. No replay display
clock is created, so the UI runs in its live mode on the host monotonic clock.
"""
from __future__ import annotations

import argparse
import os
from pathlib import Path
import secrets
import signal
import subprocess
import sys
import tempfile
import time

from openpilot.tools.simhil.scenario import SCENARIOS

HEADLESS_SECONDS = 15.0
WARMUP_FRACTION = 0.4


def parse_args(argv: list[str]) -> argparse.Namespace:
  parser = argparse.ArgumentParser(prog="./simui", description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  ui = parser.add_mutually_exclusive_group()
  ui.add_argument("--c4", dest="ui", action="store_const", const="c4", help="compact comma 4 UI (default)")
  ui.add_argument("--c3", dest="ui", action="store_const", const="c3", help="large comma 3/3X UI")
  parser.add_argument("--scenario", choices=SCENARIOS, default="s_curve")
  parser.add_argument("--seed", type=int, default=0)
  parser.add_argument("--speed", type=float, default=25.0, help="cruise speed, m/s")
  parser.add_argument("--duration", type=float, default=60.0, help="scenario length in seconds (live runs loop it)")
  parser.add_argument("--headless", action="store_true", help="hidden window; record and dump PNG frames, then exit")
  parser.add_argument("--frames", type=int, default=5, help="PNG frames to write in --headless mode")
  parser.add_argument("--seconds", type=float, default=None, help=f"wall seconds to run (headless default {HEADLESS_SECONDS:g})")
  parser.add_argument("--out", type=Path, default=None, help="output directory for --headless (default: new temp dir)")
  parser.add_argument("--record", type=Path, default=None, help="also record the UI to this .mp4")
  parser.add_argument("--gpu", action="store_true", help="WSL: render with the host GPU (GALLIUM_DRIVER=d3d12)")
  parser.add_argument("--prefix", default=None, help="private session name replay-NAME (default: random)")
  parser.set_defaults(ui="c4")
  args = parser.parse_args(argv)
  if args.frames < 1:
    parser.error("--frames must be positive")
  if args.headless and args.seconds is None:
    args.seconds = HEADLESS_SECONDS
  return args


def seed_params(params_root: str) -> None:
  from openpilot.common.params import Params
  from openpilot.starpilot.schema_cache import put_cache
  from openpilot.tools.replay.onroad_config import seed_preview
  from openpilot.tools.simhil.publisher import car_params
  params = Params(params_root)
  seed_preview(None, params)  # terms/training acceptance only
  for key, value in (("OpenpilotEnabledToggle", True), ("IsMetric", False), ("ExperimentalMode", False)):
    params.put(key, value, block=True)
  put_cache(params, "CarParamsPersistent", car_params(), block=True)


def _ffmpeg(tool: str = "ffmpeg") -> str:
  local = Path(sys.executable).with_name(tool)  # the venv's minimal ffmpeg build has no select/fps filters
  return str(local) if local.is_file() else tool


def extract_frames(video: Path, out_dir: Path, count: int) -> list[Path]:
  probe = subprocess.run([_ffmpeg("ffprobe"), "-v", "error", "-select_streams", "v:0", "-show_entries", "format=duration",
                          "-of", "csv=p=0", str(video)], check=True, capture_output=True, text=True)
  duration = float(probe.stdout.strip() or 0.0)
  if duration <= 0.0:
    raise RuntimeError(f"no frames were recorded in {video}")
  first, last = duration * WARMUP_FRACTION, max(duration - 0.2, 0.0)
  frames = []
  for index in range(count):
    stamp = first + (last - first) * index / max(count - 1, 1)
    path = out_dir / f"frame_{index:03d}.png"
    subprocess.run([_ffmpeg(), "-v", "error", "-y", "-ss", f"{stamp:.3f}", "-i", str(video), "-frames:v", "1", str(path)], check=True)
    frames.append(path)
  return frames


def _stop_ui(child: subprocess.Popen) -> None:
  # SIGINT is the UI's clean exit (gui_app.close flushes a RECORD mp4); escalate if needed.
  for send, wait in ((lambda: child.send_signal(signal.SIGINT), 20.0), (child.terminate, 3.0), (child.kill, 1.0)):
    if child.poll() is not None:
      return
    send()
    try:
      child.wait(timeout=wait)
    except subprocess.TimeoutExpired:
      pass


def supervise(publisher_cmd: list[str], ui_cmd: list[str], env: dict[str, str], seconds: float | None) -> int:
  from openpilot.tools.replay.onroad import _exit_status, _stop
  stop_signal: int | None = None

  def interrupted(signum, _frame):
    nonlocal stop_signal
    stop_signal = signum
  handlers = {sig: signal.signal(sig, interrupted) for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP)}
  publisher = subprocess.Popen(publisher_cmd, env=env)
  ui = None
  try:
    time.sleep(0.5)
    if publisher.poll() is not None:
      return _exit_status(publisher.returncode)
    ui = subprocess.Popen(ui_cmd, env=env, start_new_session=True)  # our SIGINT handling owns its shutdown
    deadline = None if seconds is None else time.monotonic() + seconds
    while stop_signal is None and (deadline is None or time.monotonic() < deadline):
      for child in (publisher, ui):
        if child.poll() is not None:
          print(f"simhil: {'publisher' if child is publisher else 'UI'} exited with {child.returncode}", file=sys.stderr, flush=True)
          return _exit_status(child.returncode)
      time.sleep(0.1)
    return 128 + stop_signal if stop_signal is not None else 0
  finally:
    if ui is not None:
      _stop_ui(ui)
    _stop([publisher])
    for sig, handler in handlers.items():
      signal.signal(sig, handler)


def run(args: argparse.Namespace) -> int:
  from openpilot.starpilot.ui.host_launch import launch_environment
  from openpilot.starpilot.ui.presentation import Profile
  from openpilot.tools.replay.onroad import PREFIX_RE, _ipc_root, owned_ipc_namespace
  source = dict(os.environ)
  if source.get("SP_HOST_RUNTIME") != "1":
    raise RuntimeError("Start through ./simui (or ./dev python -m openpilot.tools.simhil.run) in the isolated host runtime")
  raw_host_params = source.get("SP_HOST_PARAMS_ROOT", "")
  if not raw_host_params or not Path(raw_host_params).is_absolute():
    raise RuntimeError("simhil requires the host runtime's absolute SP_HOST_PARAMS_ROOT")
  host_params = Path(raw_host_params).resolve()
  prefix = args.prefix or "replay-sim" + secrets.token_hex(6)
  if PREFIX_RE.fullmatch(prefix) is None:
    raise ValueError("--prefix must be replay-NAME with letters, digits, _ or -")
  if _ipc_root(prefix).exists():
    raise RuntimeError(f"Prefix already has an IPC namespace: {prefix}")
  profile = Profile.LARGE if args.ui == "c3" else Profile.COMPACT
  launch_environment(profile, {**source, "OPENPILOT_PREFIX": prefix, "PARAMS_ROOT": str(host_params)})  # font contract first

  out_dir = None
  if args.headless:
    out_dir = (args.out or Path(tempfile.mkdtemp(prefix="simhil-"))).resolve()
    out_dir.mkdir(parents=True, exist_ok=True)
  record = args.record.resolve() if args.record else (out_dir / "sim.mp4" if out_dir else None)

  host_params.parent.mkdir(parents=True, exist_ok=True)
  with tempfile.TemporaryDirectory(prefix="simhil-params-", dir=host_params.parent) as params_root:
    env = {**source, "OPENPILOT_PREFIX": prefix, "PARAMS_ROOT": params_root,
           "COMMA_CACHE": str(host_params.parent / "download-cache")}
    for key in ("SP_REPLAY_CLOCK_PATH", "SP_REPLAY_DEVICE_TYPE", "SP_ONROAD_VISUAL_PREVIEW"):
      env.pop(key, None)  # live UI clock; HARDWARE stays "pc", deviceState carries the device type
    env["PATH"] = str(Path(sys.executable).parent) + os.pathsep + env.get("PATH", "")  # venv ffmpeg for RECORD
    if args.gpu:
      env["GALLIUM_DRIVER"] = "d3d12"
    if record is not None:
      env.update(RECORD="1", RECORD_OUTPUT=str(record))
    if args.headless:
      env.update(HIDDEN_WINDOW="1", SCALE="1")
    seed_params(params_root)
    device_type = "tizi" if args.ui == "c3" else "mici"
    publisher_cmd = [sys.executable, "-m", "openpilot.tools.simhil.publisher", "--scenario", args.scenario,
                     "--seed", str(args.seed), "--speed", str(args.speed), "--duration", str(args.duration),
                     "--device-type", device_type]
    ui_cmd = [sys.executable, "-m", "openpilot.starpilot.ui.host_launch", "large" if args.ui == "c3" else "compact"]
    mode = ("; headless" if args.headless else "") + ("; GPU d3d12" if args.gpu else "")
    print(f"simhil prefix {prefix}; UI {args.ui}; scenario {args.scenario}{mode}", flush=True)
    with owned_ipc_namespace(prefix):
      status = supervise(publisher_cmd, ui_cmd, env, args.seconds)
  if args.headless and status == 0:
    assert out_dir is not None and record is not None
    frames = extract_frames(record, out_dir, args.frames)
    print(f"simhil: wrote {len(frames)} PNG frames and {record.name} to {out_dir}", flush=True)
  elif record is not None and record.exists():
    print(f"simhil: recording {record}", flush=True)
  return status


def main(argv: list[str] | None = None) -> int:
  args = parse_args(list(sys.argv[1:] if argv is None else argv))
  try:
    return run(args)
  except (OSError, RuntimeError, ValueError, subprocess.CalledProcessError) as error:
    print(f"simui: {error}", file=sys.stderr)
    return 2


if __name__ == "__main__":
  raise SystemExit(main())
