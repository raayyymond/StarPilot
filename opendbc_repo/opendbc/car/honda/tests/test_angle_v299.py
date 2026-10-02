"""Accord angle interface, V299 (fork side): no fork override, the A16 firmware family, the EPS torque bar from 0x1AB,
the AccordAngle* params, the angle status word.

The route-79 fixture (data/accord_angle_r79_v299.npz) is RECORDED: route 75604b0a432fdc89_00000079--a1f5d2a272, V298
(A16A) on Dom 2712e1336.  Built by the kit's analysis-2020accord/studies/angle_loop/v299_design/fork/
build_v299_fork_fixture.py.  It holds
  lim_*   the limiter inputs and the 0xE4 field the car sent (carOutput.actuatorsOutput.torqueOutputCan) on every
          latActive frame plus the frame before each latActive run, with the card-loop pairing (carControl jc-1,
          carState i-1); steeringAngleDeg / rate / hand torque as the parser's integers, so each float is the one the
          controller saw;
  e1ab_*  segment 0's bus-1 0x1AB frames and the bus-1 CAN batch times, with the route's two 0x1AB gaps > 100 ms;
  bar_*   31,987 engaged hands-off frames: the 0x1AB frame there and the sign of the angle-mode bar the car drew.
"""
import inspect
import math
from collections import defaultdict
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from opendbc.car import Bus, gen_empty_fingerprint, structs
from opendbc.car.honda import carcontroller as honda_carcontroller
from opendbc.car.honda import interface as honda_interface
from opendbc.car.honda.carcontroller import CarController, read_accord_angle_param
from opendbc.car.honda.carstate import CarState, accord_eps_torque_s10
from opendbc.car.honda.hondacan import CanBus, honda_checksum
from opendbc.car.honda.interface import CarInterface
from opendbc.car.honda.tests.accord_angle_replay import FakeParams, load_windows, replay_steering_frames
from opendbc.car.honda.values import CAR, DBC, CarControllerParams, HondaFlags, is_accord_eps_angle_loop_fw
from opendbc.car.lateral import apply_steer_angle_limits_vm, get_max_angle_delta_vm
from opendbc.car.structs import CarParams
from opendbc.safety import ALTERNATIVE_EXPERIENCE
from openpilot.common.params import UnknownKeyName

SteerControlType = CarParams.SteerControlType
P = CarControllerParams
FIXTURE = Path(__file__).resolve().parent / "data" / "accord_angle_r79_v299.npz"
FW_A16A = b"39990-TVA,A16A\x00\x00"  # what route 79's EPS reported (NUL-padded to 16 bytes)
FW_A16B = b"39990-TVA,A16B\x00\x00"
FW_A160 = b"39990-TVA,A160\x00\x00"  # V294/V295, the torque map
ARTEFACT_FRAMES = [24872, 30533, 40759]  # route-79 card-loop pairing artefacts (test_the_fixture_is_route_79_as_flown)


def toggles():
  return SimpleNamespace(always_on_lateral_lkas=False, force_torque_controller=False, nnff=False, nnff_lite=False)


class ValueParams(FakeParams):
  def __init__(self, values=None, raise_unknown=()):
    self.values, self.raise_unknown = values or {}, set(raise_unknown)

  def get_float(self, key, block=False, return_default=False, default=0.0):
    if key in self.raise_unknown:
      raise UnknownKeyName(key)
    return self.values.get(key, default)

  def get_bool(self, key, block=False, default=False):
    if key in self.raise_unknown:
      raise UnknownKeyName(key)
    return self.values.get(key, default)


def make_cp(mp, fw, switch, aol=True):
  mp.setattr(honda_interface, "Params", lambda: ValueParams({"AccordEpsAngleLoop": switch}))
  car_fw = [CarParams.CarFw(ecu=CarParams.Ecu.eps, fwVersion=fw, address=0x18DA30F1, subAddress=0)]
  CP = CarInterface.get_params(CAR.HONDA_ACCORD, gen_empty_fingerprint(), car_fw, True, False, False, toggles())
  if aol:
    CP.alternativeExperience |= ALTERNATIVE_EXPERIENCE.ALWAYS_ON_LATERAL
  return CP


def make_controller(mp, CP, values=None, raise_unknown=()):
  mp.setattr(honda_carcontroller, "Params", lambda: ValueParams(values, raise_unknown))
  return CarController(DBC[CP.carFingerprint], CP)


class V298AngleController:
  """Dom 2712e1336's CarController._angle_hold_allowed + _update_angle, VERBATIM except that the three override
  constants (ANGLE_OVERRIDE_ON / OFF / LEAD_S = 600 / 500 / 0.06 there) are arguments.  With on = off = inf the O1
  branch can never fire, which is V299's code by construction; with 600 / 500 / 0.06 it is the code that flew route 79."""
  def __init__(self, CP, VM, on, off, lead):
    self.CP, self.VM, self.params = CP, VM, CarControllerParams(CP)
    self.on, self.off, self.lead = on, off, lead
    self.apply_angle_last = None
    self.angle_override = False

  def _angle_hold_allowed(self, CC, CS) -> bool:
    aol = bool(self.CP.alternativeExperience & ALTERNATIVE_EXPERIENCE.ALWAYS_ON_LATERAL)
    return bool(CS.out.cruiseState.available) and (bool(CC.enabled) or aol)

  def _update_angle(self, CC, CS) -> tuple[float, int]:
    p = self.params
    steering_angle = float(CS.out.steeringAngleDeg)
    driver_torque = abs(float(CS.out.steeringTorque))
    was_override = self.angle_override
    if CC.latActive:
      self.angle_override = driver_torque > (self.off if self.angle_override else self.on)
    else:
      self.angle_override = False

    angle_last = self.apply_angle_last
    if angle_last is None or (was_override and not self.angle_override):
      angle_last = steering_angle
    apply_angle = apply_steer_angle_limits_vm(float(CC.actuators.steeringAngleDeg), angle_last, CS.out.vEgoRaw,
                                              steering_angle, CC.latActive, p, self.VM)
    if CC.latActive:
      if self.angle_override:
        apply_angle = steering_angle + float(CS.out.steeringRateDeg) * self.lead
      error_max = float(np.interp(CS.out.vEgoRaw, p.ANGLE_ERROR_MAX_BP, p.ANGLE_ERROR_MAX_V))
      apply_angle = float(np.clip(apply_angle, steering_angle - error_max, steering_angle + error_max))
      apply_angle = float(np.clip(apply_angle, -p.ANGLE_LIMITS.STEER_ANGLE_MAX, p.ANGLE_LIMITS.STEER_ANGLE_MAX))
    self.apply_angle_last = apply_angle

    if CC.latActive:
      raw = math.floor(-10.0 * apply_angle + 0.5)
    elif self._angle_hold_allowed(CC, CS):
      raw = math.floor(-10.0 * steering_angle + 0.5)
    else:
      raw = 0
    return apply_angle, int(np.clip(raw, -p.ANGLE_RAW_MAX, p.ANGLE_RAW_MAX))


# ------------------------------------------------------------------------------------------- the route-79 replay

@pytest.fixture(scope="module")
def r79():
  return dict(np.load(FIXTURE))


@pytest.fixture(scope="module")
def r79_replay(r79):
  """New code (param defaults), the vendored 2712e1336 code with the override disabled (inf) and as flown."""
  with pytest.MonkeyPatch.context() as mp:
    CP = make_cp(mp, FW_A16A, True)
    new = make_controller(mp, CP)
  assert CP.steerControlType == SteerControlType.angle
  cp_vm = r79["cp_vm"]  # mass, wheelbase, centerToFront, tireStiffnessFront, tireStiffnessRear, steerRatio, inertia
  for got, want in zip((CP.mass, CP.wheelbase, CP.centerToFront, CP.tireStiffnessFront, CP.tireStiffnessRear, CP.steerRatio,
                        CP.rotationalInertia), cp_vm, strict=True):
    assert got == pytest.approx(want, rel=1e-6)  # route 79's CarParams (float32 in the log) = this CP's vehicle model
  inf = V298AngleController(CP, new.VM, math.inf, math.inf, 0.06)
  flown = V298AngleController(CP, new.VM, 600, 500, 0.06)

  flags = r79["lim_flags"]
  lat, ena, avail = (flags & 1) > 0, (flags & 2) > 0, (flags & 4) > 0
  k_ang, rate, tq, vraw, des = r79["lim_k_ang"], r79["lim_rate"], r79["lim_tq"], r79["lim_vraw"], r79["lim_des"]
  n = len(flags)
  out = {name: np.zeros(n, np.int64) for name in ("new", "inf", "flown")}
  ang = {name: np.zeros(n) for name in ("new", "inf")}
  status = np.zeros(n, np.int64)
  o1 = np.zeros(n, bool)
  for i in range(n):
    CC = SimpleNamespace(latActive=bool(lat[i]), enabled=bool(ena[i]), actuators=SimpleNamespace(steeringAngleDeg=float(des[i])))
    out_cs = SimpleNamespace(steeringAngleDeg=int(k_ang[i]) * -0.1 + 0.0, steeringRateDeg=float(rate[i]),
                             steeringTorque=float(tq[i]), vEgoRaw=float(vraw[i]),
                             cruiseState=SimpleNamespace(available=bool(avail[i])))
    CS = SimpleNamespace(out=out_cs)
    ang["new"][i], out["new"][i], status[i] = new._update_angle(CC, CS)
    ang["inf"][i], out["inf"][i] = inf._update_angle(CC, CS)
    _, out["flown"][i] = flown._update_angle(CC, CS)
    o1[i] = flown.angle_override
  return dict(lat=lat, out=out, ang=ang, status=status, o1=o1, rec=r79["lim_rec_raw"].astype(np.int64), n=n)


class TestRoute79Replay:
  def test_the_fixture_is_route_79_as_flown(self, r79_replay):
    # the vendored 2712e1336 code with its override reproduces the 0xE4 field the car sent on every latActive frame but
    # 3 of 66,767.  Those 3 (fixture frames 24872, 30533, 40759 = log frames 30596, 36598, 90598, each ~600 frames
    # after a 6000-frame segment rollover) are card-loop pairing artefacts, not code: the car re-sent the previous
    # frame's field there (it ran on the carControl before the one the fixed jc-1 pairing picks), so rec[i] == rec[i-1].
    R = r79_replay
    lat = R["lat"]
    assert lat.sum() == 66767 and R["n"] == 66775
    artefact = lat & (R["out"]["flown"] != R["rec"])
    assert np.flatnonzero(artefact).tolist() == ARTEFACT_FRAMES
    assert all(R["rec"][i] == R["rec"][i - 1] for i in ARTEFACT_FRAMES)
    assert (R["o1"] & lat).sum() > 1000  # the recording exercises V298's override

  def test_identity_with_2712e1336_when_its_override_can_never_fire(self, r79_replay):
    # V299's _update_angle IS V298's with ANGLE_OVERRIDE_ON = OFF = inf: identical raw AND setpoint on 100 % of frames
    R = r79_replay
    assert np.array_equal(R["out"]["new"], R["out"]["inf"])
    assert np.array_equal(R["ang"]["new"], R["ang"]["inf"])

  def test_defaults_replay_route_79_outside_the_o1_episodes(self, r79_replay):
    # V299 at its param defaults vs the 0xE4 the car sent: identical on every latActive frame except (by design) inside
    # V298's O1 episodes and in the convergence window after each O1 release, where V298 restarted its limiter from the
    # wheel and V299 (no override) continues from its own last output until the two agree again.
    R = r79_replay
    lat, o1, new, flown, rec = R["lat"], R["o1"] & R["lat"], R["out"]["new"], R["out"]["flown"], R["rec"]
    differ = lat & (new != flown)  # V299 vs V298-as-flown on the same recorded inputs
    window = np.zeros_like(lat)
    releases = np.flatnonzero(np.r_[False, o1[:-1] & ~o1[1:]] & lat)
    for r in releases:
      k = r
      while k < len(lat) and lat[k] and differ[k] and not o1[k]:
        window[k] = True
        k += 1
    outside = lat & ~o1
    assert not (differ & outside & ~window).any()  # identical everywhere else
    # documented: 3,245 latActive frames outside O1 (5.72 %) differ, every one in a post-release window (415 releases;
    # the kit's refute/rr2_f12_differential.py counts the same 3,245)
    frac = (differ & outside).sum() / outside.sum()
    assert len(releases) == 415
    assert (differ & outside).sum() == 3245 and frac == pytest.approx(0.0572, abs=5e-4)
    # and against the bytes the car actually sent: identical outside O1 and the windows, but for the 3 pairing artefacts
    same_as_sent = outside & ~window
    same_as_sent[ARTEFACT_FRAMES] = False
    assert np.array_equal(new[same_as_sent], rec[same_as_sent])
    assert same_as_sent.sum() == outside.sum() - 3245 - 3

  def test_status_bits_on_route_79(self, r79_replay):
    R = r79_replay
    st, lat = R["status"], R["lat"]
    assert not (st[~lat]).any()  # nothing set while lateral is off (the stale bit is added in update(), not here)
    assert set(np.unique(st)) <= {0, P.ANGLE_STATUS_RATE, P.ANGLE_STATUS_CLIP, P.ANGLE_STATUS_RATE | P.ANGLE_STATUS_CLIP}
    assert (st & P.ANGLE_STATUS_RATE).any() and (st & P.ANGLE_STATUS_CLIP).any()


# ------------------------------------------------------------------------------------------- the firmware family

class TestFirmwareFamily:
  @pytest.mark.parametrize("fw, angle", [
    (b"39990-TVA,A16A", True), (FW_A16A, True), (b"39990-TVA,A16B", True), (FW_A16B, True), (b"39990-TVA,A16Z", True),
    (b"39990-TVA,A160", False), (FW_A160, False),          # V294/V295 torque map: a digit, not a letter
    (b"39990-TVA-A160\x00\x00", False), (b"39990-TVA-A110", False), (b"39990-TVA,A110\x00\x00", False),
    (b"39990-TVA,A16", False), (b"39990-TVA,A16AX", False), (b"39990-TVA,A16a", False), (b"39990-TVA,A16A ", False),
    (b"39990-TVA,A16AB\x00", False), (b"", False), (b"\x00" * 16, False),
  ])
  def test_is_accord_eps_angle_loop_fw(self, fw, angle):
    assert is_accord_eps_angle_loop_fw(fw) == angle

  @pytest.mark.parametrize("fw, switch, mode, flag, missing", [
    (FW_A16A, True, "angle", True, False), (FW_A16B, True, "angle", True, False),
    (FW_A16A, False, "torque", True, False), (FW_A16B, False, "torque", True, False),
    (FW_A160, True, "torque", False, True), (FW_A160, False, "torque", False, False),
  ])
  def test_interface_and_the_missing_firmware_fault(self, monkeypatch, fw, switch, mode, flag, missing):
    CP = make_cp(monkeypatch, fw, switch)
    assert str(CP.steerControlType) == mode
    assert bool(CP.flags & HondaFlags.EPS_ANGLE_LOOP_FW) == flag
    assert bool(CP.flags & HondaFlags.EPS_ANGLE_LOOP_FW_MISSING) == missing


# ----------------------------------------------------------------------------------------------- 0x1AB, the bar

def frame_1ab(s10, counter=0):
  mag = abs(int(s10)) & 0x1FF
  raw10 = mag | (0x200 if s10 < 0 else 0)
  dat = bytearray([0x80 | (raw10 >> 8), raw10 & 0xFF, (counter & 3) << 4])
  dat[2] |= honda_checksum(0x1AB, None, dat)
  return bytes(dat)


class TestEpsTorque1AB:
  def test_recorded_frames_decode(self, r79):
    b = r79["e1ab_b"]
    assert len(b) == 3003
    for row in b:
      dat = bytes(row.tolist())
      raw10 = ((dat[0] & 3) << 8) | dat[1]
      kit_tap = (-1 if raw10 & 0x200 else 1) * (raw10 & 0x1FF)   # the kit's tap decode
      assert accord_eps_torque_s10(dat) == kit_tap                # every recorded frame passes the Honda checksum

  def test_bad_frames_are_rejected(self):
    good = frame_1ab(-123)
    assert accord_eps_torque_s10(good) == -123 and accord_eps_torque_s10(frame_1ab(205)) == 205
    assert accord_eps_torque_s10(bytes([good[0], good[1], good[2] ^ 0x01])) is None  # checksum nibble
    assert accord_eps_torque_s10(good[:2]) is None and accord_eps_torque_s10(good + b"\x00") is None
    assert accord_eps_torque_s10(b"") is None

  def test_parser_listing_only_on_the_angle_firmware(self, monkeypatch):
    for fw, listed in ((FW_A16A, True), (FW_A16B, True), (FW_A160, False)):
      for switch in (True, False):
        CP = make_cp(monkeypatch, fw, switch)
        pt = CarState(CP, SimpleNamespace(flags=0)).get_can_parsers(CP)[Bus.pt]
        assert (0x1AB in pt.message_states) == listed, (fw, switch)
        if listed:
          st = pt.message_states[0x1AB]
          assert st.ignore_alive and st.ignore_checksum and st.ignore_counter

  def run_parser(self, monkeypatch, batches):
    """Feed (t_ns, frames) batches through the real pt parser and CarState's decode; returns per-batch (eps, stale)."""
    CP = make_cp(monkeypatch, FW_A16B, True)
    CS = CarState(CP, SimpleNamespace(flags=0))
    pt = CS.get_can_parsers(CP)[Bus.pt]
    bus = CanBus(CP).pt
    res = []
    for t, frames in batches:
      pt.update([(t, [(a, d, bus) for a, d in frames])])
      res.append((CS.update_accord_eps_torque(pt), CS.accord_eps_torque_stale, pt.can_valid))
    return res

  def test_recorded_segment_with_the_dead_1ab_gap(self, monkeypatch, r79):
    # route 79 segment 0, every bus-1 CAN batch in order: the bar is -8 x s10 of the last good frame, 0 once that frame
    # is > 100 ms old on the parser's clock (the recorded 1.23 s and 0.48 s gaps), never a canValid drop
    t1, b, ev = r79["e1ab_t_ns"], r79["e1ab_b"], r79["e1ab_bus1_ev_ns"]
    by_t = defaultdict(list)
    for t, row in zip(t1.tolist(), b, strict=True):
      by_t[t].append((0x1AB, bytes(row.tolist())))
    assert set(by_t) <= set(ev.tolist())
    batches = [(t, by_t.get(t, []) + [(0x18F, b"\x00" * 5)]) for t in ev.tolist()]
    res = self.run_parser(monkeypatch, batches)
    last_t, last_s10 = None, None
    stale_frames = 0
    for (t, frames), (eps, stale, valid) in zip(batches, res, strict=True):
      for addr, dat in frames:
        if addr == 0x1AB:
          last_t, last_s10 = t, accord_eps_torque_s10(dat)
      expect_stale = last_t is None or t - last_t > P.EPS_TORQUE_STALE_NS
      assert stale == expect_stale
      assert eps == (0.0 if expect_stale else -8.0 * last_s10)
      assert valid
      stale_frames += stale
    # the two recorded gaps (1.23 s, nearly the whole bus silent: 5 bus-1 batches; 0.48 s: 25 batches) zero the bar on
    # every batch more than 100 ms after the last 0x1AB -- 5 + 23 -- and on no other
    gap_ms = np.diff(t1) * 1e-6
    assert (gap_ms > 100).sum() == 2 and stale_frames == 5 + 23

  def test_dead_1ab_with_live_18f_and_bad_checksums(self, monkeypatch):
    t0, dt = 10**9, 10_000_000  # 100 Hz batches, 0x1AB at 50 Hz
    batches = []
    for i in range(200):
      frames = [(0x18F, b"\x00" * 5)]
      if i % 2 == 0 and not 50 <= i < 80:         # 0x1AB dead from 0.50 to 0.80 s (0x18F alive)
        dat = frame_1ab(100 if i < 120 else -50, counter=i * 7)  # counters deliberately wrong
        if 120 <= i < 150:
          dat = bytes([dat[0], dat[1], dat[2] ^ 0x01])            # checksum wrong from 1.20 to 1.50 s
        frames.append((0x1AB, dat))
      batches.append((t0 + i * dt, frames))
    res = self.run_parser(monkeypatch, batches)
    eps = [r[0] for r in res]
    assert all(r[2] for r in res)  # 0x1AB never drops canValid (counter and checksum ignored by the parser)
    assert eps[48] == -800.0 and eps[58] == -800.0  # 0x1AB last at 0.48 s: still fresh at 0.58 s (100 ms)
    assert eps[59] == 0.0 and eps[79] == 0.0 and res[59][1] and not res[58][1]
    assert eps[80] == -800.0 and not res[80][1]
    assert eps[118] == -800.0 and eps[128] == -800.0 and eps[129] == 0.0  # bad checksums: hold 100 ms, then 0
    assert eps[150] == 400.0 and eps[199] == 400.0

  def test_carstate_update_sets_steering_torque_eps(self, monkeypatch):
    from opendbc.car.honda.tests.test_honda_accord_angle_loop import FakeCAN
    for switch in (True, False):
      CP = make_cp(monkeypatch, FW_A16B, switch)
      CS = CarState(CP, SimpleNamespace(flags=0))
      cp = FakeCAN((1, 1, 1))
      cp.vl_raw = {0x1AB: frame_1ab(-150)}
      cp.ts_nanos = {0x1AB: {"MOTOR_TORQUE": 5 * 10**9}}
      cp.last_nonempty_nanos = 5 * 10**9 + 50_000_000
      ret, _ = CS.update({Bus.pt: cp, Bus.cam: FakeCAN((1, 1, 1))}, toggles())
      assert ret.steeringTorqueEps == 1200.0 and not CS.accord_eps_torque_stale
      cp.last_nonempty_nanos = 5 * 10**9 + 150_000_000
      ret, _ = CS.update({Bus.pt: cp, Bus.cam: FakeCAN((1, 1, 1))}, toggles())
      assert ret.steeringTorqueEps == 0.0 and CS.accord_eps_torque_stale

  def test_bar_sign_agrees_with_the_bar_the_car_drew(self, r79):
    # bar = clip(-steeringTorqueEps / 2461, -1, 1) (torque_bar.accord_eps_bar) on the frames where V298's angle-mode bar
    # had a sign: agreement 0.854 (kit rev_bar_sign.py; the opposite sign would agree 0.146)
    b, today = r79["bar_b"], r79["bar_today_sign"]
    eps = np.array([-P.EPS_TORQUE_LSB * accord_eps_torque_s10(bytes(r.tolist())) for r in b], float)
    bar = np.clip(-eps / P.EPS_TORQUE_RAIL, -1, 1)
    agree = np.count_nonzero(np.sign(bar) == today) / len(today)
    assert len(b) == 31987 and agree >= 0.85 and agree == pytest.approx(0.854, abs=1e-3)


# ---------------------------------------------------------------------------------- the torque path is untouched

class TestTorquePathUnchanged:
  @pytest.mark.parametrize("fw, switch", [(FW_A16B, False), (FW_A160, False), (FW_A160, True)])
  def test_replay_is_byte_identical_with_any_params(self, monkeypatch, fw, switch):
    # AccordEpsAngleLoop off (or no angle firmware): the 20d24ab79 golden frames, whatever the new params say
    for values in (None, {"AccordAngleMaxRate": 250.0, "AccordAngleClipScale": 1.6, "AccordAngleBarFromEps": True}):
      for w in load_windows():
        CP = make_cp(monkeypatch, fw, switch)
        assert CP.steerControlType == SteerControlType.torque
        ctrl = make_controller(monkeypatch, CP, values)
        frames = replay_steering_frames(ctrl, w, toggles())
        assert [h for _, h, _ in frames] == w["golden_e4"]["hex"], (w["source"], values)
        assert not hasattr(ctrl, "VM") and ctrl.angle_status == 0


# ------------------------------------------------------------------------------------------------- the params

class TestAngleParams:
  def test_defaults_are_v298(self, monkeypatch):
    CP = make_cp(monkeypatch, FW_A16B, True)
    ctrl = make_controller(monkeypatch, CP)
    assert ctrl.params.ANGLE_LIMITS == P.ANGLE_LIMITS and ctrl.params.ANGLE_LIMITS.MAX_ANGLE_RATE == 1.2
    assert ctrl.params.ANGLE_ERROR_MAX_V == P.ANGLE_ERROR_MAX_V == [17.0, 15.5, 19.5, 17.0, 8.5, 4.5]
    assert ctrl.params.ANGLE_ERROR_MAX_BP == [3.1, 8.0, 10.0, 11.75, 17.5, 26.9]

  @pytest.mark.parametrize("given, rate", [(120.0, 120.0), (250.0, 250.0), (180.0, 180.0), (10.0, 60.0), (1000.0, 250.0),
                                           (math.nan, 120.0), (math.inf, 120.0), (-math.inf, 120.0), ("x", 120.0)])
  def test_max_rate(self, monkeypatch, given, rate):
    CP = make_cp(monkeypatch, FW_A16B, True)
    ctrl = make_controller(monkeypatch, CP, {"AccordAngleMaxRate": given})
    assert ctrl.params.ANGLE_LIMITS.MAX_ANGLE_RATE == pytest.approx(rate / 100.0)
    assert P.ANGLE_LIMITS.MAX_ANGLE_RATE == 1.2  # per instance: the class limits stay V298's
    # one active frame from a settled setpoint at 2 m/s moves by min(jerk limit, rate)
    ctrl.apply_angle_last = 0.0
    CC = SimpleNamespace(latActive=True, enabled=True, actuators=SimpleNamespace(steeringAngleDeg=30.0))
    CS = SimpleNamespace(out=SimpleNamespace(steeringAngleDeg=0.0, steeringRateDeg=0.0, steeringTorque=0.0, vEgoRaw=2.0,
                                             cruiseState=SimpleNamespace(available=True)))
    angle, _, st = ctrl._update_angle(CC, CS)
    assert angle == pytest.approx(min(get_max_angle_delta_vm(2.0, ctrl.VM, ctrl.params), rate / 100.0))
    assert st & P.ANGLE_STATUS_RATE

  @pytest.mark.parametrize("given, scale", [(1.0, 1.0), (1.6, 1.6), (1.3, 1.3), (0.5, 1.0), (3.0, 1.6), (math.nan, 1.0)])
  def test_clip_scale_and_its_band(self, monkeypatch, given, scale):
    CP = make_cp(monkeypatch, FW_A16B, True)
    ctrl = make_controller(monkeypatch, CP, {"AccordAngleClipScale": given})
    assert ctrl.params.ANGLE_ERROR_MAX_V == pytest.approx([17.0 * scale, 15.5 * scale, 19.5 * scale, 17.0 * scale, 8.5, 4.5])
    assert P.ANGLE_ERROR_MAX_V == [17.0, 15.5, 19.5, 17.0, 8.5, 4.5]
    # the band 11.75-17.5 m/s blends 17 x scale to 8.5: at 14 m/s 13.67 deg (x1.0), 19.88 deg (x1.6)
    em = float(np.interp(14.0, ctrl.params.ANGLE_ERROR_MAX_BP, ctrl.params.ANGLE_ERROR_MAX_V))
    assert em == pytest.approx(17.0 * scale + (8.5 - 17.0 * scale) * (14.0 - 11.75) / 5.75)

  def test_unknown_keys_are_defaults(self, monkeypatch):
    CP = make_cp(monkeypatch, FW_A16B, True)
    ctrl = make_controller(monkeypatch, CP, raise_unknown=("AccordAngleMaxRate", "AccordAngleClipScale"))
    assert ctrl.params.ANGLE_LIMITS == P.ANGLE_LIMITS and ctrl.params.ANGLE_ERROR_MAX_V == P.ANGLE_ERROR_MAX_V
    assert read_accord_angle_param(ValueParams(raise_unknown=("k",)), "k", 120.0, 60.0, 250.0) == 120.0

  def test_params_are_read_once(self, monkeypatch):
    reads = []

    class CountingParams(ValueParams):
      def get_float(self, key, **kw):
        reads.append(key)
        return super().get_float(key, **kw)

    CP = make_cp(monkeypatch, FW_A16B, True)
    monkeypatch.setattr(honda_carcontroller, "Params", lambda: CountingParams())
    ctrl = CarController(DBC[CP.carFingerprint], CP)
    angle_reads = [k for k in reads if k.startswith("AccordAngle")]
    assert sorted(angle_reads) == ["AccordAngleClipScale", "AccordAngleMaxRate"]
    CC = SimpleNamespace(latActive=True, enabled=True, actuators=SimpleNamespace(steeringAngleDeg=3.0))
    CS = SimpleNamespace(out=SimpleNamespace(steeringAngleDeg=0.0, steeringRateDeg=0.0, steeringTorque=0.0, vEgoRaw=9.0,
                                             cruiseState=SimpleNamespace(available=True)))
    for _ in range(50):
      ctrl._update_angle(CC, CS)
    assert [k for k in reads if k.startswith("AccordAngle")] == angle_reads


# --------------------------------------------------------------------------------------------- the status word

class TestAngleStatus:
  def run(self, monkeypatch, *, desired, measured, last, v=10.0, lat=True, stale=False):
    CP = make_cp(monkeypatch, FW_A16B, True)
    ctrl = make_controller(monkeypatch, CP)
    ctrl.apply_angle_last = last
    CC = structs.CarControl.new_message()
    CC.enabled = lat
    CC.latActive = lat
    CC.actuators.steeringAngleDeg = desired
    out = SimpleNamespace(vEgo=v, vEgoRaw=v, aEgo=0.0, steeringAngleDeg=measured, steeringRateDeg=0.0, steeringTorque=0.0,
                          steeringPressed=False, gasPressed=False, brakePressed=False, standstill=False,
                          cruiseState=SimpleNamespace(available=True, enabled=False, standstill=False))
    CS = SimpleNamespace(out=out, v_cruise_factor=1.0, is_metric=False, acc_hud=False, lkas_hud=False, accord_eps_torque_stale=stale)
    new_actuators, _ = ctrl.update(CC.as_reader(), CS, 0, toggles())
    return ctrl, new_actuators

  def test_bits(self, monkeypatch):
    ctrl, _ = self.run(monkeypatch, desired=1.0, measured=1.0, last=1.0)
    assert ctrl.angle_status == 0
    ctrl, _ = self.run(monkeypatch, desired=30.0, measured=1.0, last=1.0)
    assert ctrl.angle_status == P.ANGLE_STATUS_RATE
    ctrl, _ = self.run(monkeypatch, desired=1.0, measured=40.0, last=1.0)  # the wheel 39 deg away: the clip binds
    assert ctrl.angle_status == P.ANGLE_STATUS_CLIP
    ctrl, _ = self.run(monkeypatch, desired=-30.0, measured=40.0, last=1.0)
    assert ctrl.angle_status == P.ANGLE_STATUS_RATE | P.ANGLE_STATUS_CLIP
    ctrl, _ = self.run(monkeypatch, desired=30.0, measured=1.0, last=1.0, lat=False, stale=True)
    assert ctrl.angle_status == P.ANGLE_STATUS_EPS_STALE
    ctrl, _ = self.run(monkeypatch, desired=30.0, measured=1.0, last=1.0, stale=True)
    assert ctrl.angle_status == P.ANGLE_STATUS_RATE | P.ANGLE_STATUS_EPS_STALE

  def test_actuators_output_torque_is_never_written(self, monkeypatch):
    # the mici UI reads carOutput.actuatorsOutput.torque: in angle mode it stays 0, the status goes elsewhere
    for desired, measured in ((30.0, 1.0), (1.0, 40.0), (0.0, 0.0)):
      _, new_actuators = self.run(monkeypatch, desired=desired, measured=measured, last=1.0)
      assert new_actuators.torque == 0.0

  def test_status_field_is_in_the_schema(self):
    from cereal import custom
    msg = custom.StarPilotCarState.new_message()
    msg.accordAngleStatus = P.ANGLE_STATUS_RATE | P.ANGLE_STATUS_CLIP | P.ANGLE_STATUS_EPS_STALE
    assert msg.accordAngleStatus == 276
    field = custom.StarPilotCarState.schema.fields["accordAngleStatus"]
    assert field.proto.ordinal.explicit == 31

  def test_card_publishes_the_controller_status(self):
    card = Path(__file__).resolve().parents[5] / "selfdrive" / "car" / "card.py"
    if not card.exists():
      pytest.skip("opendbc outside the openpilot tree")
    src = card.read_text(encoding="utf-8")
    block = src[src.index("def state_publish"):src.index("def controls_update")]
    assert 'getattr(self.CI.CC, "angle_status", 0)' in block and "FPCS.accordAngleStatus" in block
    assert block.index("FPCS.accordAngleStatus") < block.index("fpcs_send.starpilotCarState = FPCS")
    assert "actuatorsOutput.torque" not in block


def test_no_override_anywhere_in_the_controller():
  src = inspect.getsource(honda_carcontroller)
  assert "angle_override" not in src and "ANGLE_OVERRIDE" not in src
