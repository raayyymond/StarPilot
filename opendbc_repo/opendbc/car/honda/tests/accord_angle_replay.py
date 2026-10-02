"""Replay harness for the Accord EPS angle-loop tests (test_honda_accord_angle_loop.py).

The fixture holds two windows of a RECORDED torque-mode drive, route 75604b0a432fdc89_00000071--a7b8ba5d9d (EPS
39990-TVA,A160 = the V294 torque map, always-on lateral, openpilot longitudinal):
  * segment 0, frames 3450-3949: an engage at 0-0.9 m/s, presses, torque to 0.6;
  * segment 16, frames 1700-1999: a disengage at 10-11 m/s with torque on the wheel.
Each window carries the carControl and carState fields the Honda CarController reads, and the 0xE4 STEERING_CONTROL
frames that Dom 20d24ab79's CarController emitted for exactly those inputs (this harness run on a git archive of
20d24ab79).  This module imports nothing the angle-loop change added, so it runs unchanged on 20d24ab79.
"""
import json
from pathlib import Path
from types import SimpleNamespace

from opendbc.car import structs

DATA = Path(__file__).resolve().parent / "data" / "accord_torque_mode_replay_r71b.json"

LongControlState = structs.CarControl.Actuators.LongControlState
VisualAlert = structs.CarControl.HUDControl.VisualAlert


class FakeParams:
  def get_float(self, key, block=False, return_default=False, default=0.0):
    return default

  def put_float(self, key, value):
    pass

  def get_bool(self, key, block=False, default=False):
    return default


def load_windows() -> list[dict]:
  return json.loads(DATA.read_text(encoding="utf-8"))["windows"]


def build_cc(cols: dict, i: int):
  CC = structs.CarControl.new_message()
  CC.enabled = bool(cols["cc.enabled"][i])
  CC.latActive = bool(cols["cc.latActive"][i])
  CC.longActive = bool(cols["cc.longActive"][i])
  CC.actuators.torque = float(cols["cc.torque"][i])
  CC.actuators.accel = float(cols["cc.accel"][i])
  CC.actuators.longControlState = getattr(LongControlState, cols["cc.longControlState"][i])
  CC.cruiseControl.cancel = bool(cols["cc.cancel"][i])
  CC.cruiseControl.resume = bool(cols["cc.resume"][i])
  CC.hudControl.setSpeed = float(cols["cc.setSpeed"][i])
  CC.hudControl.speedVisible = bool(cols["cc.speedVisible"][i])
  CC.hudControl.lanesVisible = bool(cols["cc.lanesVisible"][i])
  CC.hudControl.leadVisible = bool(cols["cc.leadVisible"][i])
  CC.hudControl.leadDistanceBars = int(cols["cc.leadDistanceBars"][i])
  CC.hudControl.visualAlert = getattr(VisualAlert, cols["cc.visualAlert"][i])
  CC.orientationNED = [0.0, float(cols["cc.pitch"][i]), 0.0]
  return CC


def build_cs(cols: dict, i: int):
  out = SimpleNamespace(
    vEgo=float(cols["cs.vEgo"][i]), vEgoRaw=float(cols["cs.vEgoRaw"][i]), aEgo=float(cols["cs.aEgo"][i]),
    steeringAngleDeg=float(cols["cs.steeringAngleDeg"][i]), steeringRateDeg=float(cols["cs.steeringRateDeg"][i]),
    steeringTorque=float(cols["cs.steeringTorque"][i]), steeringPressed=bool(cols["cs.steeringPressed"][i]),
    gasPressed=bool(cols["cs.gasPressed"][i]), brakePressed=bool(cols["cs.brakePressed"][i]),
    standstill=bool(cols["cs.standstill"][i]),
    cruiseState=SimpleNamespace(available=bool(cols["cs.cruiseAvailable"][i]), enabled=False, standstill=False),
  )
  return SimpleNamespace(out=out, v_cruise_factor=1.0, is_metric=False, acc_hud=False, lkas_hud=False)


def replay_steering_frames(controller, window: dict, toggles) -> list[tuple[int, str, int]]:
  """Run every recorded frame of one window through controller.update; return its 0xE4 frames as (addr, hex, bus)."""
  cols = window["cols"]
  frames = []
  for i in range(window["n"]):
    _, can_sends = controller.update(build_cc(cols, i).as_reader(), build_cs(cols, i), 0, toggles)
    frames.extend((addr, bytes(dat).hex(), bus) for addr, dat, bus in can_sends if addr == 0xE4)
  return frames
