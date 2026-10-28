"""Accord on the angle-loop EPS firmware (39990-TVA,A16<letter>): detection, the 0xE4 frame, the setpoint limits."""
import math
from types import SimpleNamespace

import numpy as np
import pytest

from opendbc import get_generated_dbcs
from opendbc.can import CANPacker
from opendbc.car import Bus, gen_empty_fingerprint
from opendbc.car.honda.carcontroller import CarController
from opendbc.car.honda.carstate import CarState
from opendbc.car.honda.hondacan import CanBus, create_steering_control
from opendbc.car.honda.interface import CarInterface
from opendbc.car.honda.values import CAR, DBC, CarControllerParams, HondaFlags, is_accord_eps_angle_loop_fw, \
                                     accord_angle_steer_ratio, ACCORD_ANGLE_SR_BP, ACCORD_ANGLE_SR_V
from opendbc.car.interfaces import get_torque_params
from opendbc.car.lateral import MAX_LATERAL_ACCEL, apply_steer_angle_limits_vm
from opendbc.car.toyota.interface import CarInterface as ToyotaInterface
from opendbc.car.toyota.values import CAR as TOYOTA
from opendbc.car.structs import CarParams

P = CarControllerParams
STEERING_CONTROL = 0xE4


def _eps_fw(fw: bytes):
  return [CarParams.CarFw(ecu=CarParams.Ecu.eps, fwVersion=fw, address=0x18DA30F1, subAddress=0)]


def accord_cp(eps_fw: bytes | None):
  car_fw = [] if eps_fw is None else _eps_fw(eps_fw)
  return CarInterface.get_params(CAR.HONDA_ACCORD, gen_empty_fingerprint(), car_fw, False, False, False)


@pytest.mark.parametrize("fw, expected", [
  (b"39990-TVA,A16A\x00\x00", True),   # V298, NUL-padded as the car reports it
  (b"39990-TVA,A16B\x00\x00", True),   # V299
  (b"39990-TVA,A16C", True),           # the release
  (b"39990-TVA,A160\x00\x00", False),  # torque-map images
  (b"39990-TVA-A160\x00\x00", False),  # stock
  (b"39990-TVA,A16", False),
  (b"39990-TVA,A16AB", False),
  (b"39990-TVA,A16a", False),
  (b"39990-TVB,A16A", False),
])
def test_fw_detection(fw, expected):
  assert is_accord_eps_angle_loop_fw(fw) is expected


@pytest.mark.parametrize("fw", [b"39990-TVA,A16A\x00\x00", b"39990-TVA,A16B\x00\x00", b"39990-TVA,A16C\x00\x00"])
def test_angle_fw_selects_angle_control(fw):
  CP = accord_cp(fw)
  assert CP.steerControlType == CarParams.SteerControlType.angle
  assert CP.flags & HondaFlags.EPS_ANGLE_LOOP_FW
  assert CP.steerActuatorDelay == pytest.approx(P.ANGLE_STEER_ACTUATOR_DELAY)
  # the angle fw carries a ',' but is not a torque-modified EPS: it must not hit the MODIFIED_EPS_FW dashcam fallback
  assert not CP.dashcamOnly
  assert not CP.flags & HondaFlags.EPS_MODIFIED
  assert not CP.flags & HondaFlags.EPS_FW_UNREAD


def test_angle_fw_on_another_honda_is_not_angle_control():
  # the family is Accord-only; on any other platform the ',' read is the upstream modified-EPS path
  CP = CarInterface.get_params(CAR.HONDA_CIVIC_BOSCH, gen_empty_fingerprint(), _eps_fw(b"39990-TVA,A16C\x00\x00"),
                               False, False, False)
  assert CP.steerControlType == CarParams.SteerControlType.torque
  assert not CP.flags & HondaFlags.EPS_ANGLE_LOOP_FW


def test_mixed_eps_read_is_not_angle_control():
  car_fw = _eps_fw(b"39990-TVA,A16C\x00\x00") + _eps_fw(b"39990-TVA-A160\x00\x00")
  CP = CarInterface.get_params(CAR.HONDA_ACCORD, gen_empty_fingerprint(), car_fw, False, False, False)
  assert CP.steerControlType == CarParams.SteerControlType.torque
  assert not CP.flags & HondaFlags.EPS_ANGLE_LOOP_FW
  assert CP.dashcamOnly  # upstream: a ',' EPS outside MODIFIED_EPS_FW


def test_angle_control_needs_no_params():
  """A fresh install selects angle control from the EPS fw alone and keeps the pid tuning block untouched.

  Old tree (83fa2e68c8) also pinned this against stale torque-era toggles (ForceTorqueController, NNFF, NNFF Lite)
  passed into get_params; this tree's get_params takes no toggles and has no ForceTorqueController conversion at all,
  so only the fw-alone half of that test still has an API to exercise."""
  CP = CarInterface.get_params(CAR.HONDA_ACCORD, gen_empty_fingerprint(), _eps_fw(b"39990-TVA,A16C"), False, False, False)
  assert CP.steerControlType == CarParams.SteerControlType.angle
  assert CP.lateralTuning.which() == "pid"
  assert CarController({Bus.pt: DBC[CAR.HONDA_ACCORD][Bus.pt]}, CP).angle_mode


@pytest.mark.parametrize("fw", [None, b"39990-TVA-A160\x00\x00", b"39990-TVA,A160\x00\x00"])
def test_other_fw_keeps_torque_control(fw):
  CP = accord_cp(fw)
  assert CP.steerControlType == CarParams.SteerControlType.torque
  assert not CP.flags & HondaFlags.EPS_ANGLE_LOOP_FW
  assert not CarController({Bus.pt: DBC[CAR.HONDA_ACCORD][Bus.pt]}, CP).angle_mode


def test_max_lateral_accel_is_the_angle_limit_only_on_the_angle_fw():
  # the onroad torque bar divides by CP.maxLateralAccel; the torque fit's 0.32 m/s^2 pegged it on every turn
  torque_fit = get_torque_params()[CAR.HONDA_ACCORD]['MAX_LAT_ACCEL_MEASURED']
  assert torque_fit == pytest.approx(0.3247, abs=1e-4)
  assert accord_cp(b"39990-TVA,A16C").maxLateralAccel == pytest.approx(MAX_LATERAL_ACCEL)
  assert MAX_LATERAL_ACCEL == pytest.approx(3.589, abs=1e-3)
  for fw in (None, b"39990-TVA-A160\x00\x00", b"39990-TVA,A160\x00\x00"):
    assert accord_cp(fw).maxLateralAccel == pytest.approx(torque_fit)
  angle_fw = _eps_fw(b"39990-TVA,A16C")
  for car in (CAR.HONDA_CIVIC_BOSCH, CAR.HONDA_CRV_5G):
    CP = CarInterface.get_params(car, gen_empty_fingerprint(), angle_fw, False, False, False)
    assert CP.maxLateralAccel == pytest.approx(get_torque_params()[car]['MAX_LAT_ACCEL_MEASURED'])
  CP = ToyotaInterface.get_params(TOYOTA.TOYOTA_RAV4_TSS2, gen_empty_fingerprint(), [], False, False, False)
  assert CP.maxLateralAccel == pytest.approx(get_torque_params()[TOYOTA.TOYOTA_RAV4_TSS2]['MAX_LAT_ACCEL_MEASURED'])


def _steer_faults(CP):
  # an unfed parser set: only the fault logic, not the bus contents, is under test
  CS = CarState(CP)
  ret = CS.update(CS.get_can_parsers(CP))
  return ret.steerFaultPermanent


@pytest.mark.parametrize("fw, angle, unread", [
  (b"39990-TVA,A16C\x00\x00", True, False),   # angle EPS read -> angle mode
  (b"39990-TVA-A160\x00\x00", False, False),  # stock EPS read -> torque mode, no fault
  (b"39990-TVA,A160\x00\x00", False, False),  # torque-map EPS read -> torque mode, no fault
  (None, False, True),                         # no EPS fw at all -> permanent fault
])
def test_eps_fw_unread_faults(fw, angle, unread):
  CP = accord_cp(fw)
  assert (CP.steerControlType == CarParams.SteerControlType.angle) == angle
  assert bool(CP.flags & HondaFlags.EPS_FW_UNREAD) == unread
  if not angle:
    # same unfed bus for every case, so the fault is decided by the flag alone (the read cases are the control)
    assert _steer_faults(CP) == unread


def test_docs_params_do_not_fault():
  CP = CarInterface.get_params(CAR.HONDA_ACCORD, gen_empty_fingerprint(), [], False, False, True)
  assert not CP.flags & HondaFlags.EPS_FW_UNREAD


def _upstream_dbc(tmp_path):
  # the Accord DBC without STEER_CONTROL_ARM, i.e. as it was before the signal was added.  The *_generated DBCs are
  # built in memory in this tree (opendbc.get_generated_dbcs), so the stripped copy is written out from that content.
  name = DBC[CAR.HONDA_ACCORD][Bus.pt]
  content = get_generated_dbcs()[name]
  assert "STEER_CONTROL_ARM" in content
  lines = [ln for ln in content.splitlines(keepends=True) if "STEER_CONTROL_ARM" not in ln]
  path = tmp_path / f"{name}.dbc"
  path.write_text("".join(lines), encoding="utf-8")
  return str(path)


def test_torque_frame_unchanged_by_the_dbc_signal(tmp_path):
  CP = accord_cp(None)
  new = CANPacker(DBC[CAR.HONDA_ACCORD][Bus.pt])
  old = CANPacker(_upstream_dbc(tmp_path))
  can = CanBus(CP)
  for torque in range(-4096, 4097, 37):
    for active in (False, True):
      for tja in (False, True):
        assert create_steering_control(new, can, torque, active, tja) == create_steering_control(old, can, torque, active, tja)


@pytest.mark.parametrize("active", [False, True])
def test_angle_frame_carries_the_arm_and_a_valid_checksum(active):
  packer = CANPacker(DBC[CAR.HONDA_ACCORD][Bus.pt])
  can = CanBus(accord_cp(b"39990-TVA,A16C"))
  for raw in (-4000, -1234, -1, 0, 1, 987, 4000):
    addr, dat, _ = create_steering_control(packer, can, raw, active, False, angle_arm=P.ANGLE_ARM)
    assert addr == STEERING_CONTROL
    assert (dat[2] >> 2) & 0x3 == P.ANGLE_ARM          # byte 2 bits 3:2
    assert int.from_bytes(dat[0:2], "big", signed=True) == raw  # setpoint sent with the request off too
    assert bool(dat[2] & 0x80) == active               # STEER_TORQUE_REQUEST
    sig = packer.dbc.addr_to_msg[addr].sigs["CHECKSUM"]
    zeroed = bytearray(dat)
    zeroed[4] &= 0xF0
    assert sig.calc_checksum(addr, sig, zeroed) == dat[4] & 0xF  # CHECKSUM 35|4 = byte 4 low nibble


def _controller(aol=False):
  CP = accord_cp(b"39990-TVA,A16C")
  if aol:
    CP.alternativeExperience |= 32  # always-on lateral: the hold must NOT depend on it (the 7.0 panda predicate is a session grant)
  return CarController({Bus.pt: DBC[CAR.HONDA_ACCORD][Bus.pt]}, CP)


def _cc_cs(desired, measured, v, lat_active, enabled=True, main_on=True):
  CC = SimpleNamespace(actuators=SimpleNamespace(steeringAngleDeg=desired), latActive=lat_active, enabled=enabled)
  CS = SimpleNamespace(out=SimpleNamespace(steeringAngleDeg=measured, vEgoRaw=v, cruiseState=SimpleNamespace(available=main_on)))
  return CC, CS


def test_rate_limit_then_error_clip():
  cc = _controller()
  assert cc.angle_mode
  v, measured = 20.0, 0.0
  err = float(np.interp(v, P.ANGLE_ERROR_MAX_BP, P.ANGLE_ERROR_MAX_V))
  state = measured
  for _ in range(200):
    CC, CS = _cc_cs(90.0, measured, v, True)
    state = apply_steer_angle_limits_vm(90.0, state, v, measured, True, cc.params, cc.VM)
    apply_angle, raw = cc._update_angle(CC, CS)
    assert cc.apply_angle_last == state  # the limiter keeps its own output
    assert apply_angle == pytest.approx(float(np.clip(state, measured - err, measured + err)))  # the clip bounds the send
    assert raw == math.floor(-10.0 * apply_angle + 0.5)
  assert apply_angle == pytest.approx(measured + err)  # the wheel never moved: the clip holds the send at +err
  assert state > measured + err                       # while the limiter state ran on


def test_wheel_motion_does_not_feed_the_limiter_state():
  # same command, different wheel paths: while lateral is active the limiter state must not depend on the wheel
  rng = np.random.default_rng(0)
  a, b = _controller(), _controller()
  a.apply_angle_last = b.apply_angle_last = 0.0
  for i in range(300):
    desired = 60.0 * math.sin(i / 20.0)
    a._update_angle(*_cc_cs(desired, 0.0, 12.0, True))
    b._update_angle(*_cc_cs(desired, float(rng.normal(0.0, 15.0)), 12.0, True))
    assert a.apply_angle_last == b.apply_angle_last


def test_wheel_moving_under_a_held_setpoint_does_not_move_the_limiter_state():
  cc = _controller()
  cc.apply_angle_last = 20.0
  for i in range(100):
    wheel = 20.0 + 15.0 * math.sin(i / 5.0)  # the wheel moves (clip binds at the peaks); the command holds at 20 deg
    sent, _ = cc._update_angle(*_cc_cs(20.0, wheel, 20.0, True))
    assert cc.apply_angle_last == 20.0
    err = float(np.interp(20.0, P.ANGLE_ERROR_MAX_BP, P.ANGLE_ERROR_MAX_V))
    assert sent == pytest.approx(float(np.clip(20.0, wheel - err, wheel + err)))


def test_each_step_is_bounded_by_the_rate_limit():
  # MAX_ANGLE_RATE = 2.5 deg/frame (250 deg/s): no frame may step further than that
  cc = _controller()
  prev = 0.0
  for _ in range(5):
    a, _ = cc._update_angle(*_cc_cs(90.0, 0.0, 3.0, True))
    assert abs(a - prev) <= P.ANGLE_LIMITS.MAX_ANGLE_RATE + 1e-9
    prev = a


def test_first_frame_seeds_from_the_wheel():
  cc = _controller()
  a, _ = cc._update_angle(*_cc_cs(30.0, 25.0, 10.0, True))
  assert abs(a - 25.0) <= P.ANGLE_LIMITS.MAX_ANGLE_RATE + 1e-9


@pytest.mark.parametrize("enabled, main_on, aol, holds", [
  (True, True, False, True),
  (False, True, True, False),   # always-on lateral with ACC main on: the panda's grant is not visible here, so no hold
  (True, True, True, True),
  (False, True, False, False),
  (True, False, False, False),  # the panda drops controls_allowed with main off
  (False, False, True, False),
])
def test_idle_hold(enabled, main_on, aol, holds):
  cc = _controller(aol)
  measured = -12.34
  apply_angle, raw = cc._update_angle(*_cc_cs(50.0, measured, 15.0, False, enabled, main_on))
  assert apply_angle == measured  # lateral off: the limiter follows the wheel
  assert raw == (math.floor(-10.0 * measured + 0.5) if holds else 0)


def test_raw_is_clipped():
  cc = _controller()
  cc.apply_angle_last = 399.0
  _, raw = cc._update_angle(*_cc_cs(400.0, 399.0, 1.0, True))
  assert raw == -P.ANGLE_RAW_MAX


def test_steer_ratio_map_level():
  assert accord_angle_steer_ratio(0.0) == pytest.approx(16.84)
  for deg in (0.0, 45.0, 120.0, -200.0, 500.0):
    expected = float(np.interp(abs(deg), ACCORD_ANGLE_SR_BP, ACCORD_ANGLE_SR_V)) * 16.84 / 16.88
    assert accord_angle_steer_ratio(deg) == pytest.approx(expected)


def test_error_clip_is_the_p_room_under_the_firmware_sum_clamp():
  # Operator rule (2026-10-21): the clip is the angle error where the V300 firmware's P term alone equals
  # Kmax - Imax (sum clamp 15360 after the 254/256 taper, so 15481 before it; I cap 4096 / 6144 / 8192 by
  # speed).  Integer mirror of the image: E = 16 * 10 * deg, E' = (E * G(v)) >> 8, P = (E' * 112) >> 8.
  g_v = {3.1: 1178, 8.0: 1465, 10.0: 760, 11.75: 560, 17.5: 1068, 26.9: 2188}
  for v, clip in zip(P.ANGLE_ERROR_MAX_BP, P.ANGLE_ERROR_MAX_V, strict=True):
    imax = 4096 if v <= 6.0 else (6144 if v <= 12.5 else 8192)
    p_of = lambda deg, v=v: (((16 * round(10 * deg) * g_v[v]) >> 8) * 112) >> 8  # noqa: E731
    assert p_of(clip) >= 15481 - imax > p_of(clip - 0.1)


def test_angle_frame_is_sent_by_the_controller():
  # the full update(): angle mode packs the setpoint with the arm; the projection observer is never installed on angle
  CP = accord_cp(b"39990-TVA,A16C")
  ci = CarInterface(CP)
  from opendbc.car import structs
  CC = structs.CarControl(enabled=True, latActive=True)
  CC.actuators.steeringAngleDeg = 5.0
  ci.update([])
  out, sends = ci.apply(CC.as_reader(), 1_000_000_000)
  steer = [s for s in sends if s[0] == STEERING_CONTROL]
  assert len(steer) == 1
  dat = steer[0][1]
  assert (dat[2] >> 2) & 0x3 == P.ANGLE_ARM
  assert int.from_bytes(dat[0:2], "big", signed=True) == out.torqueOutputCan
  assert out.steeringAngleDeg == pytest.approx(-out.torqueOutputCan / 10.0, abs=0.05)
