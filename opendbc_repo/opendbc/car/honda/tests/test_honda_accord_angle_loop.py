"""Accord EPS angle-loop interface (V298 firmware, F181 39990-TVA,A16A): the car-side invariants.

I1 torque path byte-identical to Dom 20d24ab79 · I2 the 0xE4 field by state · I3 the angle limits · I4 the override ·
I5 no integrator · I7 the mode-3 steer fault · I8 the panda does not block the angle frames.  (I5's LatControlAngle half
and I6 live in selfdrive/controls/tests/test_latcontrol.py, next to the other Accord tests.)
"""
from collections import defaultdict
import hashlib
import inspect
import math
import re
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from opendbc.can import CANPacker, CANParser
from opendbc.car import Bus, gen_empty_fingerprint, structs
from opendbc.car.honda import carcontroller as honda_carcontroller
from opendbc.car.honda import interface as honda_interface
from opendbc.car.honda.carcontroller import CarController
from opendbc.car.honda.carstate import CarState
from opendbc.car.honda.hondacan import CanBus, create_steering_control
from opendbc.car.honda.interface import CarInterface
from opendbc.car.honda.tests.accord_angle_replay import FakeParams, load_windows, replay_steering_frames
from opendbc.car.honda.values import CAR, DBC, HONDA_ACCORD_EPS_ANGLE_LOOP_FW, CarControllerParams, HondaFlags
from opendbc.car.lateral import get_max_angle_delta_vm, get_max_angle_vm, apply_steer_angle_limits_vm
from opendbc.car.structs import CarParams
from opendbc.safety import ALTERNATIVE_EXPERIENCE

SteerControlType = CarParams.SteerControlType
P = CarControllerParams
FW_ANGLE = HONDA_ACCORD_EPS_ANGLE_LOOP_FW + b"\x00\x00"
FW_TORQUE = b"39990-TVA,A160\x00\x00"  # the torque-map image flying today (V294/V295)
HONDA_H = Path(__file__).resolve().parents[3] / "safety" / "modes" / "honda.h"


def toggles():
  return SimpleNamespace(always_on_lateral_lkas=False, force_torque_controller=False, nnff=False, nnff_lite=False)


def accord_cp(monkeypatch, fw, switch, aol=True, car_fw=None):
  class SwitchParams(FakeParams):
    def get_bool(self, key, block=False, default=False):
      return switch if key == "AccordEpsAngleLoop" else default

  monkeypatch.setattr(honda_interface, "Params", lambda: SwitchParams())
  if car_fw is None:
    car_fw = [CarParams.CarFw(ecu=CarParams.Ecu.eps, fwVersion=fw, address=0x18DA30F1, subAddress=0),
              CarParams.CarFw(ecu=CarParams.Ecu.eps, fwVersion=b"\x11L-130520-03461268             ", address=0x18DA30F1)]
  CP = CarInterface.get_params(CAR.HONDA_ACCORD, gen_empty_fingerprint(), car_fw, True, False, False, toggles())
  if aol:
    CP.alternativeExperience |= ALTERNATIVE_EXPERIENCE.ALWAYS_ON_LATERAL
  return CP


def controller(monkeypatch, CP):
  monkeypatch.setattr(honda_carcontroller, "Params", lambda: FakeParams())
  return CarController(DBC[CP.carFingerprint], CP)


def angle_controller(monkeypatch, aol=True):
  CP = accord_cp(monkeypatch, FW_ANGLE, True, aol=aol)
  assert CP.steerControlType == SteerControlType.angle
  return controller(monkeypatch, CP)


def step(ctrl, *, lat_active, desired=0.0, measured=0.0, rate=0.0, v=20.0, driver_torque=0.0, enabled=False, main_on=True):
  """One control frame.  Returns (apply_angle, decoded 0xE4 fields, raw frame bytes)."""
  CC = structs.CarControl.new_message()
  CC.enabled = enabled
  CC.latActive = lat_active
  CC.actuators.steeringAngleDeg = desired
  CC.hudControl.visualAlert = structs.CarControl.HUDControl.VisualAlert.none
  out = SimpleNamespace(vEgo=v, vEgoRaw=v, aEgo=0.0, steeringAngleDeg=measured, steeringRateDeg=rate, steeringTorque=driver_torque,
                        steeringPressed=abs(driver_torque) > 1200, gasPressed=False, brakePressed=False, standstill=v < 0.1,
                        cruiseState=SimpleNamespace(available=main_on, enabled=False, standstill=False))
  CS = SimpleNamespace(out=out, v_cruise_factor=1.0, is_metric=False, acc_hud=False, lkas_hud=False)
  new_actuators, can_sends = ctrl.update(CC.as_reader(), CS, 0, toggles())
  frames = [(a, d, b) for a, d, b in can_sends if a == 0xE4]
  assert len(frames) == 1
  # carOutput carries the sent setpoint (float32 there; the controller keeps the exact value)
  assert new_actuators.steeringAngleDeg == pytest.approx(ctrl.apply_angle_last, rel=1e-6, abs=1e-6)
  return ctrl.apply_angle_last, decode_e4(frames[0][1]), frames[0][1]


def decode_e4(dat):
  dat = bytes(dat)
  return SimpleNamespace(torque=int.from_bytes(dat[0:2], "big", signed=True), request=dat[2] >> 7, arm=(dat[2] >> 2) & 0x3,
                         byte2_rest=dat[2] & 0x73, byte3=dat[3])


def error_max(v):
  return float(np.interp(v, P.ANGLE_ERROR_MAX_BP, P.ANGLE_ERROR_MAX_V))


# ---------------------------------------------------------------------------------------------------- the interlock

class TestAccordAngleInterlock:
  def test_angle_mode_needs_the_angle_firmware_and_the_switch(self, monkeypatch):
    CP = accord_cp(monkeypatch, FW_ANGLE, True)
    assert CP.steerControlType == SteerControlType.angle
    assert CP.flags & HondaFlags.EPS_ANGLE_LOOP_FW and CP.flags & HondaFlags.EPS_MODIFIED
    assert not CP.flags & HondaFlags.EPS_ANGLE_LOOP_FW_MISSING
    assert CP.steerActuatorDelay == pytest.approx(0.15)
    assert CP.lateralTuning.which() == "pid"  # unused by LatControlAngle; keeps the torque conversion off

    for fw, switch, flagged in ((FW_ANGLE, False, True), (FW_TORQUE, True, False), (b"39990-TVA-A160\x00\x00", True, False),
                                (b"39990-TVA,A16B\x00\x00", True, False)):
      CP = accord_cp(monkeypatch, fw, switch)
      assert CP.steerControlType == SteerControlType.torque, (fw, switch)
      assert bool(CP.flags & HondaFlags.EPS_ANGLE_LOOP_FW) == flagged, fw
      # the switch on without the angle firmware is flagged for carstate's permanent fault
      assert bool(CP.flags & HondaFlags.EPS_ANGLE_LOOP_FW_MISSING) == (switch and not flagged), (fw, switch)
      assert CP.steerActuatorDelay == pytest.approx(0.1)

  @pytest.mark.parametrize("car_fw", [
    [],  # no EPS entry at all (the fw query timed out, or the EPS did not answer)
    [CarParams.CarFw(ecu=CarParams.Ecu.fwdCamera, fwVersion=FW_ANGLE, address=0x18DAB5F1)],  # the string on a non-EPS ECU
    [CarParams.CarFw(ecu=CarParams.Ecu.eps, fwVersion=FW_TORQUE, address=0x18DA30F1)],
    [CarParams.CarFw(ecu=CarParams.Ecu.eps, fwVersion=b"39990-TVA-A160\x00\x00", address=0x18DA30F1)],
    [CarParams.CarFw(ecu=CarParams.Ecu.eps, fwVersion=b"39990-TVA,A16AX", address=0x18DA30F1)],
  ])
  def test_switch_on_without_the_angle_firmware_is_flagged(self, monkeypatch, car_fw):
    CP = accord_cp(monkeypatch, None, True, car_fw=car_fw)
    assert CP.steerControlType == SteerControlType.torque
    assert CP.flags & HondaFlags.EPS_ANGLE_LOOP_FW_MISSING and not CP.flags & HondaFlags.EPS_ANGLE_LOOP_FW
    # switch off: nothing flagged, the torque path as before
    CP = accord_cp(monkeypatch, None, False, car_fw=car_fw)
    assert not CP.flags & (HondaFlags.EPS_ANGLE_LOOP_FW_MISSING | HondaFlags.EPS_ANGLE_LOOP_FW)

  def test_force_torque_controller_cannot_convert_the_angle_mode(self, monkeypatch):
    monkeypatch.setattr(honda_interface, "Params", lambda: SimpleNamespace(get_bool=lambda key, default=False, **kw: key == "AccordEpsAngleLoop" or default))
    car_fw = [CarParams.CarFw(ecu=CarParams.Ecu.eps, fwVersion=FW_ANGLE, address=0x18DA30F1, subAddress=0)]
    forced = SimpleNamespace(always_on_lateral_lkas=False, force_torque_controller=True, nnff=True, nnff_lite=False)
    CP = CarInterface.get_params(CAR.HONDA_ACCORD, gen_empty_fingerprint(), car_fw, True, False, False, forced)
    assert CP.steerControlType == SteerControlType.angle and CP.lateralTuning.which() == "pid"

  def test_unknown_switch_key_is_off(self, monkeypatch):
    # an on-device params library built before this key existed raises UnknownKeyName: the switch reads off
    def get_bool(key, default=False, **kw):
      if key == "AccordEpsAngleLoop":
        raise honda_interface.UnknownKeyName(key)
      return default
    monkeypatch.setattr(honda_interface, "Params", lambda: SimpleNamespace(get_bool=get_bool))
    car_fw = [CarParams.CarFw(ecu=CarParams.Ecu.eps, fwVersion=FW_ANGLE, address=0x18DA30F1, subAddress=0)]
    CP = CarInterface.get_params(CAR.HONDA_ACCORD, gen_empty_fingerprint(), car_fw, True, False, False, toggles())
    assert CP.steerControlType == SteerControlType.torque and CP.flags & HondaFlags.EPS_ANGLE_LOOP_FW


# ------------------------------------------------------------------------------- I1: the torque path is untouched

class TestAccordTorquePathUnchanged:
  @pytest.mark.parametrize("fw, switch", [(FW_TORQUE, False), (FW_TORQUE, True), (FW_ANGLE, False)])
  def test_replay_is_byte_identical_to_20d24ab79(self, monkeypatch, fw, switch):
    windows = load_windows()
    assert [(w["start_frame"], w["n"]) for w in windows] == [(3450, 500), (1700, 300)]
    for w in windows:
      golden = w["golden_e4"]
      assert golden["generated_at"] == "20d24ab79" and len(golden["hex"]) == w["n"]
      assert hashlib.sha256("".join(golden["hex"]).encode()).hexdigest() == golden["sha256"]

      CP = accord_cp(monkeypatch, fw, switch)
      assert CP.steerControlType == SteerControlType.torque
      frames = replay_steering_frames(controller(monkeypatch, CP), w, toggles())
      assert [h for _, h, _ in frames] == golden["hex"], w["source"]
      assert sorted({b for _, _, b in frames}) == golden["bus"]
      # the torque frame never carries the angle arm
      assert all((int(h[4:6], 16) >> 2) & 0x3 == 0 for h in golden["hex"])
    # the recordings exercise the torque path: an engage, a disengage with torque on the wheel, the slew limit
    edges = [[a != b for a, b in zip(w["cols"]["cc.latActive"], w["cols"]["cc.latActive"][1:], strict=False)].index(True) for w in windows]
    assert windows[0]["cols"]["cc.latActive"][edges[0] + 1] == 1 and windows[1]["cols"]["cc.latActive"][edges[1] + 1] == 0
    assert len({h[:4] for h in windows[0]["golden_e4"]["hex"]}) > 100

  def test_create_steering_control_default_is_the_torque_frame(self):
    packer = CANPacker(DBC[CAR.HONDA_ACCORD][Bus.pt])
    CAN = SimpleNamespace(lkas=1)
    assert create_steering_control(packer, CAN, 1234, False, False)[1][0:2] == b"\x00\x00"  # torque zeroed with request off
    assert decode_e4(create_steering_control(packer, CAN, -321, True, False)[1]).arm == 0


# --------------------------------------------------------------------------------------- I2: the field, by state

class TestAccordAngleField:
  @pytest.mark.parametrize("deg, raw, b01", [(12.3, -123, "ff85"), (-45.0, 450, "01c2"), (0.0, 0, "0000"), (400.0, -4000, "f060")])
  def test_spec_vectors_pack_and_decode(self, monkeypatch, deg, raw, b01):
    ctrl = angle_controller(monkeypatch)
    # hold the setpoint inside every limit: measured = desired, low speed, settled limiter
    ctrl.apply_angle_last = deg
    apply_angle, f, dat = step(ctrl, lat_active=True, desired=deg, measured=deg, v=2.0)
    assert apply_angle == pytest.approx(deg)
    assert f.torque == raw and dat[0:2].hex() == b01 and f.request == 1 and f.arm == 2
    assert f.byte2_rest == 0 and f.byte3 == 0

  def test_frames_decode_with_a_valid_checksum_and_counter(self, monkeypatch):
    ctrl = angle_controller(monkeypatch)
    CP_bus = CanBus(ctrl.CP).lkas
    parser = CANParser(DBC[CAR.HONDA_ACCORD][Bus.pt], [("STEERING_CONTROL", 100)], CP_bus)
    states = [dict(lat_active=True, desired=3.0, measured=2.5), dict(lat_active=False, measured=-17.3),
              dict(lat_active=False, measured=-17.3, main_on=False), dict(lat_active=True, desired=-1.0, measured=-0.8)]
    for i, kw in enumerate(states * 3):
      _, f, dat = step(ctrl, **kw)
      updated = parser.update([(int(1e9) + i * 10_000_000, [(0xE4, dat, CP_bus)])])
      assert 0xE4 in updated, f"frame {i} rejected (checksum or counter)"
      vl = parser.vl["STEERING_CONTROL"]
      assert vl["STEER_TORQUE"] == f.torque and vl["STEER_TORQUE_REQUEST"] == f.request
      assert f.arm == 2

  def test_state_table(self, monkeypatch):
    # active: the limited setpoint, request 1
    ctrl = angle_controller(monkeypatch)
    ctrl.apply_angle_last = 4.9
    apply_angle, f, _ = step(ctrl, lat_active=True, desired=5.0, measured=4.0)
    assert f.request == 1 and f.arm == 2 and f.torque == math.floor(-10 * apply_angle + 0.5)
    # inactive, allowed through always-on lateral (main on + the AOL flag, CC.enabled false): the 0x14A raw field
    for field in range(-4000, 4001, 7):
      measured = field * -0.1  # carstate: STEER_ANGLE = raw x -0.1
      apply_angle, f, _ = step(ctrl, lat_active=False, measured=measured, main_on=True)
      assert (f.torque, f.request, f.arm) == (field, 0, 2), field
      assert apply_angle == pytest.approx(measured)
    # inactive, allowed through engagement (no AOL flag)
    ctrl_no_aol = angle_controller(monkeypatch, aol=False)
    _, f, _ = step(ctrl_no_aol, lat_active=False, measured=-12.3, enabled=True, main_on=True)
    assert (f.torque, f.request, f.arm) == (123, 0, 2)
    # not allowed: AOL flag but main off / no AOL flag and not engaged -> 0 (the panda blocks anything else)
    _, f, _ = step(ctrl, lat_active=False, measured=-12.3, main_on=False)
    assert (f.torque, f.request, f.arm) == (0, 0, 2)
    _, f, _ = step(ctrl_no_aol, lat_active=False, measured=-12.3, enabled=False, main_on=True)
    assert (f.torque, f.request, f.arm) == (0, 0, 2)
    # engaged but ACC main off: the panda drops controls_allowed with main off, so this too is not allowed
    for c in (ctrl, ctrl_no_aol):
      _, f, dat = step(c, lat_active=False, measured=-12.3, enabled=True, main_on=False)
      assert dat[0:3].hex() == "000008" and (f.torque, f.request, f.arm) == (0, 0, 2)

  def test_first_active_frame_starts_from_the_wheel(self, monkeypatch):
    # a controller whose very first frame is active must not step the setpoint by the rate limit toward 0 deg
    for v, measured in ((20.0, 30.0), (5.0, -60.0), (30.0, 3.0)):
      ctrl = angle_controller(monkeypatch)
      assert ctrl.apply_angle_last is None
      apply_angle, f, _ = step(ctrl, lat_active=True, desired=measured, measured=measured, v=v)
      assert apply_angle == pytest.approx(measured) and f.torque == math.floor(-10 * measured + 0.5)
      # and from there the limiter runs from its own last output
      max_delta = min(get_max_angle_delta_vm(v, ctrl.VM, P), P.ANGLE_LIMITS.MAX_ANGLE_RATE)
      apply_angle, _, _ = step(ctrl, lat_active=True, desired=measured + 50.0, measured=measured, v=v)
      assert apply_angle == pytest.approx(measured + min(max_delta, error_max(v)))

  @pytest.mark.parametrize("deg", [5000.0, -5000.0, 1e9])
  def test_no_wrap(self, monkeypatch, deg):
    ctrl = angle_controller(monkeypatch)
    ctrl.apply_angle_last = math.copysign(400.0, deg)
    _, f, _ = step(ctrl, lat_active=True, desired=deg, measured=math.copysign(399.0, deg), v=1.0)
    assert f.torque == -math.copysign(P.ANGLE_RAW_MAX, deg)
    _, f, _ = step(ctrl, lat_active=False, measured=deg, v=1.0)
    assert abs(f.torque) == P.ANGLE_RAW_MAX and f.torque == -math.copysign(P.ANGLE_RAW_MAX, deg)

  def test_actuator_output_is_the_sent_angle(self, monkeypatch):
    ctrl = angle_controller(monkeypatch)
    for v in (5.0, 10.0):
      ctrl = angle_controller(monkeypatch)
      apply_angle, f, _ = step(ctrl, lat_active=True, desired=30.0, measured=0.0, v=v)
      # first frame from 0: one rate step, 1.2 deg at 5 m/s, the 3.589 m/s^3 jerk limit (1.02 deg) at 10 m/s
      assert apply_angle == pytest.approx(min(get_max_angle_delta_vm(v, ctrl.VM, P), P.ANGLE_LIMITS.MAX_ANGLE_RATE))
      assert f.torque == math.floor(-10 * apply_angle + 0.5)


# ----------------------------------------------------------------------------------------------- I3: the limits

class TestAccordAngleLimits:
  @pytest.mark.parametrize("v", [1.0, 3.1, 5.0, 8.0, 10.0, 11.75, 15.0, 17.5, 20.0, 26.9, 35.0])
  def test_limits_hold_on_an_adversarial_sequence(self, monkeypatch, v):
    ctrl = angle_controller(monkeypatch)
    rng = np.random.default_rng(int(v * 100))
    max_delta = min(get_max_angle_delta_vm(max(v, 1.0), ctrl.VM, P), P.ANGLE_LIMITS.MAX_ANGLE_RATE)
    max_angle = min(get_max_angle_vm(max(v, 1.0), ctrl.VM, P), P.ANGLE_LIMITS.STEER_ANGLE_MAX)
    measured, last = 0.0, 0.0
    for i in range(1500):
      # carControl carries the desired angle as float32
      desired = float(np.float32(rng.choice([rng.uniform(-600, 600), measured + rng.uniform(-60, 60), last])))
      if i % 300 < 150:  # a lagging plant half the time, a held wheel the other half
        measured += 0.2 * (last - measured)
      pre = apply_steer_angle_limits_vm(desired, last, v, measured, True, P, ctrl.VM)
      apply_angle, f, _ = step(ctrl, lat_active=True, desired=desired, measured=measured, v=v)
      # the error clip, the absolute clip and the field
      assert abs(apply_angle - measured) <= error_max(v) + 1e-9
      assert abs(apply_angle) <= P.ANGLE_LIMITS.STEER_ANGLE_MAX
      assert f.torque == max(-P.ANGLE_RAW_MAX, min(P.ANGLE_RAW_MAX, math.floor(-10 * apply_angle + 0.5)))
      # the rate and lateral-accel limits; a frame that moved further did so only by the error clip, toward the wheel
      assert abs(pre - last) <= max_delta + 1e-9 and abs(pre) <= max_angle + 1e-9
      if abs(apply_angle - pre) > 1e-9:
        assert abs(apply_angle - measured) == pytest.approx(error_max(v)) and abs(apply_angle - measured) < abs(pre - measured)
      last = apply_angle

  def test_error_clip_table_is_the_v298_sizing(self):
    assert P.ANGLE_ERROR_MAX_BP == [3.1, 8.0, 10.0, 11.75, 17.5, 26.9]
    stiffness = [51.5, 64.1, 33.2, 24.5, 46.7, 95.7]  # V298 DC stiffness, lane counts per degree
    caps = [s * d for s, d in zip(stiffness, P.ANGLE_ERROR_MAX_V, strict=True)]
    assert max(caps) < 0.45 * 2461  # never more than 45 % of the rail from the error clip alone
    assert P.ANGLE_LIMITS.STEER_ANGLE_MAX == 400 and P.ANGLE_RAW_MAX == 4000 and P.ANGLE_LIMITS.MAX_ANGLE_RATE == pytest.approx(1.2)
    assert P.ANGLE_LIMITS.MAX_LATERAL_ACCEL == pytest.approx(3.5886) and P.ANGLE_LIMITS.MAX_LATERAL_JERK == pytest.approx(3.5886)

  def test_held_wheel_pins_the_setpoint_at_the_clip(self, monkeypatch):
    for v in (5.0, 10.0, 20.0, 30.0):
      ctrl = angle_controller(monkeypatch)
      for _ in range(400):
        apply_angle, _, _ = step(ctrl, lat_active=True, desired=90.0, measured=0.0, v=v)
      assert apply_angle == pytest.approx(min(error_max(v), get_max_angle_vm(v, ctrl.VM, P)))


# --------------------------------------------------------------------------------------------- I4: the override

class TestAccordAngleOverride:
  def test_hysteresis_600_on_500_off(self, monkeypatch):
    ctrl = angle_controller(monkeypatch)
    pressed = []
    for torque in (0, 550, 599, 601, 550, 501, 499, 550, -601, -501, -499):
      step(ctrl, lat_active=True, desired=0.0, measured=0.0, driver_torque=torque)
      pressed.append(ctrl.angle_override)
    assert pressed == [False, False, False, True, True, True, False, False, True, True, False]

  def test_setpoint_follows_the_hand_then_slews_back(self, monkeypatch):
    v = 12.0
    ctrl = angle_controller(monkeypatch)
    for _ in range(100):
      step(ctrl, lat_active=True, desired=5.0, measured=4.0, v=v)
    # the driver turns the wheel away against the model's 5 deg
    measured = 4.0
    for _ in range(80):
      measured -= 0.6  # 60 deg/s
      apply_angle, f, _ = step(ctrl, lat_active=True, desired=5.0, measured=measured, rate=-60.0, v=v, driver_torque=900)
      assert apply_angle == pytest.approx(measured - 60.0 * P.ANGLE_OVERRIDE_LEAD_S)
      assert f.request == 1 and f.arm == 2
    # release: the limiter restarts from the measured angle (the lead is dropped) and the setpoint returns to the
    # model's angle under the normal rate limit (the wheel follows the setpoint here, a first-order plant)
    max_delta = min(get_max_angle_delta_vm(v, ctrl.VM, P), P.ANGLE_LIMITS.MAX_ANGLE_RATE)
    last = measured
    for _ in range(200):
      measured += 0.3 * (last - measured)
      apply_angle, _, _ = step(ctrl, lat_active=True, desired=5.0, measured=measured, v=v, driver_torque=100)
      assert 0.0 <= apply_angle - last <= max_delta + 1e-9
      last = apply_angle
    assert apply_angle == pytest.approx(5.0) and not ctrl.angle_override

  def test_release_restarts_the_limiter_from_the_wheel(self, monkeypatch):
    v = 12.0
    for desired, rate in ((-20.0, 60.0), (20.0, 60.0), (20.0, -60.0)):
      ctrl = angle_controller(monkeypatch)
      max_delta = min(get_max_angle_delta_vm(v, ctrl.VM, P), P.ANGLE_LIMITS.MAX_ANGLE_RATE)
      step(ctrl, lat_active=True, desired=0.0, measured=0.0, v=v)
      apply_angle, _, _ = step(ctrl, lat_active=True, desired=desired, measured=0.0, rate=rate, v=v, driver_torque=900)
      assert ctrl.angle_override and apply_angle == pytest.approx(rate * P.ANGLE_OVERRIDE_LEAD_S)  # the O1 lead
      apply_angle, _, _ = step(ctrl, lat_active=True, desired=desired, measured=0.0, rate=rate, v=v, driver_torque=0)
      assert not ctrl.angle_override
      # one rate step from the wheel toward the model's angle, not from the led setpoint
      assert apply_angle == pytest.approx(math.copysign(max_delta, desired))

  def test_override_lead_is_clipped_to_the_error_max(self, monkeypatch):
    ctrl = angle_controller(monkeypatch)
    apply_angle, _, _ = step(ctrl, lat_active=True, desired=0.0, measured=10.0, rate=900.0, v=20.0, driver_torque=2000)
    assert apply_angle == pytest.approx(10.0 + error_max(20.0))


# ------------------------------------------------------------------------------------------ I5: no integrator

class TestAccordAngleNoIntegrator:
  def test_constant_error_gives_a_constant_setpoint(self, monkeypatch):
    for gap in (2.0, 40.0):  # inside and beyond the error clip
      ctrl = angle_controller(monkeypatch)
      out = [step(ctrl, lat_active=True, desired=10.0 + gap, measured=10.0, v=20.0)[0] for _ in range(600)]
      settled = out[200:]
      assert max(settled) - min(settled) == 0.0
      assert settled[0] == pytest.approx(10.0 + min(gap, error_max(20.0)))

  def test_no_state_accumulates_in_the_angle_path(self):
    source = inspect.getsource(CarController._update_angle) + inspect.getsource(CarController._angle_hold_allowed)
    assert not re.search(r"self\.\w+\s*[+\-]=", source)
    assert "integr" not in source.lower()
    assigned = set(re.findall(r"self\.(\w+)\s*=", source))
    assert assigned == {"angle_override", "apply_angle_last"}  # a flag and the limiter's last output, nothing else


# ----------------------------------------------------------------------------------- I7: the mode-3 steer fault

class FakeCAN:
  def __init__(self, sensors):
    self.vl = defaultdict(lambda: defaultdict(float))
    self.vl_all = defaultdict(lambda: defaultdict(list))
    self.vl["STEERING_SENSORS"].update({f"STEER_SENSOR_STATUS_{i}": float(b) for i, b in zip((1, 2, 3), sensors, strict=True)})
    self.vl["SCM_FEEDBACK"]["MAIN_ON"] = 1.0
    self.vl["CAR_SPEED"]["IMPERIAL_UNIT"] = 1.0


def carstate_faults(CP, sensors):
  CS = CarState(CP, SimpleNamespace(flags=0))
  ret, _ = CS.update({Bus.pt: FakeCAN(sensors), Bus.cam: FakeCAN(sensors)}, toggles())
  return ret.steerFaultTemporary, ret.steerFaultPermanent


class TestAccordAngleSteerFault:
  @pytest.mark.parametrize("sensors", [(1, 1, 1), (0, 1, 1), (1, 0, 1), (1, 1, 0), (0, 0, 0)])
  def test_sensor_status_fault_in_angle_mode_only(self, monkeypatch, sensors):
    healthy = sensors == (1, 1, 1)
    assert carstate_faults(accord_cp(monkeypatch, FW_ANGLE, True), sensors) == (not healthy, False)
    # the switch on with the torque image: permanent fault (the switch and the image disagree), whatever the sensors
    assert carstate_faults(accord_cp(monkeypatch, FW_TORQUE, True), sensors) == (False, True)
    assert carstate_faults(accord_cp(monkeypatch, FW_TORQUE, False), sensors) == (False, False)

  def test_angle_firmware_without_the_angle_interface_is_a_permanent_fault(self, monkeypatch):
    assert carstate_faults(accord_cp(monkeypatch, FW_ANGLE, False), (1, 1, 1)) == (False, True)

  def test_switch_on_with_no_eps_in_carfw_is_a_permanent_fault(self, monkeypatch):
    # the switch can never silently leave lateral in torque mode on an EPS whose firmware was not read
    CP = accord_cp(monkeypatch, None, True, car_fw=[])
    assert CP.steerControlType == SteerControlType.torque
    assert carstate_faults(CP, (1, 1, 1)) == (False, True)
    assert carstate_faults(accord_cp(monkeypatch, None, False, car_fw=[]), (1, 1, 1)) == (False, False)


# ------------------------------------------------------------------------------- I8: what the panda checks on 0xE4

class TestAccordAnglePanda:
  def test_honda_safety_checks_only_bytes_0_1_and_only_when_not_allowed(self):
    src = HONDA_H.read_text(encoding="utf-8")
    hook = src[src.index("static bool honda_tx_hook("):]
    hook = hook[:hook.index("\n}\n")]
    block = hook[hook.index("// STEER: safety check"):hook.index("// Bosch supplemental control check")]
    # the 0xE4 check reads data[0] and data[1] only, and only while neither controls nor always-on lateral is allowed
    assert re.findall(r"data\[(\d+)\]", block) == ["0", "1"]
    assert "if (!(aol_allowed || controls_allowed))" in block
    # no other 0xE4 / STEERING_CONTROL rule anywhere in the TX hook (no byte-2, request, rate or magnitude check)
    assert hook.count("0xE4U") == 1
    # the Bosch TX allow-list carries 0xE4 as a 5-byte frame on the LKAS bus (bus 0 stock long, bus 1 openpilot long)
    assert "{0xE4, 0, 5, .check_relay = true}" in src and "{0xE4, 1, 5, .check_relay = true}" in src
