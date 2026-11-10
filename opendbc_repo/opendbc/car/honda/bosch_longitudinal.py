import math
from collections import deque

import numpy as np

from opendbc.car import ACCELERATION_DUE_TO_GRAVITY, structs
from opendbc.car.honda.values import CAR, HondaFlags

LongCtrlState = structs.CarControl.Actuators.LongControlState
BOSCH_BRAKE_FORCE_ON = -0.12
BOSCH_BRAKE_FORCE_RELEASE = -0.02

CLASSIC_BOSCH_CARS = frozenset((
  CAR.HONDA_NBOX_2G,
  CAR.HONDA_ACCORD,
  CAR.HONDA_CIVIC_BOSCH,
  CAR.HONDA_CIVIC_BOSCH_DIESEL,
  CAR.HONDA_CIVIC_2022,
  CAR.HONDA_CRV_5G,
  CAR.HONDA_CRV_HYBRID,
  CAR.HONDA_HRV_3G,
  CAR.HONDA_CITY_7G,
  CAR.ACURA_RDX_3G,
  CAR.ACURA_RDX_3G_MMR,
  CAR.HONDA_INSIGHT,
  CAR.HONDA_E,
  CAR.HONDA_E_ADVANCE,
  CAR.ACURA_MDX_4G,
  CAR.HONDA_ODYSSEY_5G_MMR,
  CAR.ACURA_TLX_2G,
  CAR.HONDA_FIT_4G,
  CAR.ACURA_INTEGRA,
  CAR.ACURA_ADX,
))
HONDA_BOSCH_RADARLESS = frozenset((
  CAR.HONDA_CIVIC_2022,
  CAR.HONDA_HRV_3G,
  CAR.HONDA_CITY_7G,
  CAR.HONDA_FIT_4G,
  CAR.ACURA_INTEGRA,
  CAR.ACURA_ADX,
))

def bosch_brake_force_thresholds(fingerprint) -> tuple[float, float]:
  """(brake-mode ON, RELEASE) force thresholds in m/s^2, per car."""
  if fingerprint == CAR.HONDA_ACCORD:
    # Brake mode on this car has a FLOOR: a request between 0 and -0.25 m/s^2 is delivered at -0.3 to -0.4 with a
    # -0.7 bite on entry (route bb, 2026-10-21, 21 entries in 5 min of no-lead cruise, each a 3-6 mph sag at ~18 s
    # period = the "ping-pong").  With gas reaching zero at force -0.2 and the brake arming at -0.12 the car never
    # coasted at all.  Arm the brake only once the request is at the floor, where it cannot overdeliver, and let it
    # go as soon as the request rises back to what a coast gives, so the band in between is a coast (gas 0, no brake).
    return -0.45, -0.20
  return BOSCH_BRAKE_FORCE_ON, BOSCH_BRAKE_FORCE_RELEASE


def update_honda_bosch_braking(braking: bool, gas_pedal_force: float, stopping: bool, long_active: bool,
                               on: float = BOSCH_BRAKE_FORCE_ON, release: float = BOSCH_BRAKE_FORCE_RELEASE) -> bool:
  if not long_active:
    return False
  if stopping:
    return True
  if braking:
    return gas_pedal_force <= release
  return gas_pedal_force < on


# Platforms whose gas and brake gains are learned as per-speed tables with a memory of recent driving near each
# knot, instead of the single integrating gas factor below.
BOSCH_TABLE_LEARNER_CARS = frozenset((CAR.HONDA_ACCORD,))
# Accord knot defaults: medians of steady-command pid frames over 1,422 logged segments (5.7 h of openpilot
# longitudinal, 2026-11-06).  Gas: table force the sent gas stood for / net force the powertrain produced (aEgo +
# grade + aero), at speeds 1-5 m/s 0.57-0.68, 5-10 0.53, 10-32 0.48-0.52, 32-36 0.37.  Brake: aEgo / commanded accel
# while the Bosch brake was in charge; it tracks vehicle acceleration itself, so grade is NOT added back.
ACCORD_GAS_FACTOR_BP = [0.0, 3.0, 6.0, 10.0, 20.0, 30.0, 35.0]
ACCORD_GAS_FACTOR_V = [0.64, 0.64, 0.55, 0.52, 0.50, 0.49, 0.37]
ACCORD_BRAKE_RATIO_BP = [0.0, 5.0, 15.0, 25.0, 35.0]
ACCORD_BRAKE_RATIO_V = [0.95, 1.08, 1.13, 1.07, 1.10]
TABLE_UPDATE_HZ = 50  # update() runs on every other 100 Hz control frame
TABLE_HORIZON = 30 * 60 * TABLE_UPDATE_HZ  # each knot remembers about the last 30 minutes of driving near it
TABLE_PRIOR_WEIGHT = 60 * TABLE_UPDATE_HZ  # the logged default counts as one minute of fresh evidence
STEADY_FRAMES = TABLE_UPDATE_HZ  # a command held for 1 s is steady: the launch lag of the 2.0T + 10AT is not gain
STEADY_GAS_UNITS = 60.0  # 0.08 m/s^2 of table force
STEADY_BRAKE_MPS2 = 0.1
BRAKE_SCALE_FULL = -2.0  # the learned brake ratio corrects requests down to here ...
BRAKE_SCALE_NONE = -3.0  # ... and fades to a pass-through by here, where the data runs out
GAS_FACTOR_LIMITS = (0.1, 3.0)
# Launch shortfall. 2026-11-09, routes 0000002b-2d, gas-only frames with a command over 0.6 m/s^2: the car delivered
# 58 % of the command under 2.5 m/s, 70 % at 2.5-5 and 77-85 % above 5, so the lead pulled away at 2-3 m/s^2 while
# the car made 0.8 on a 1.4 request and the driver took over with the pedal. The tables are steady-state gain and
# stay that way; this adds force only for the part of the command the car is not yet delivering, only at low speed.
LAUNCH_BOOST_GAIN = 0.6
LAUNCH_BOOST_MAX = 0.5  # m/s^2 of table force
LAUNCH_BOOST_MIN_ACCEL = 0.5  # commands under this are creeping, not launching
LAUNCH_BOOST_FULL_MPS = 4.0
LAUNCH_BOOST_END_MPS = 8.0
BRAKE_RATIO_LIMITS = (0.7, 1.5)


class SpeedTable:
  """Per-speed-knot running estimate whose memory is driving time spent near each knot.

  Every update() frame decays the two bracketing knots by their interpolation weight over TABLE_HORIZON, whether or
  not that frame taught anything, so a knot forgets at the pace the car drives through its speeds.  A learnable
  frame adds its sample with the same weight.  The logged default sits under the estimate as TABLE_PRIOR_WEIGHT of
  evidence, so a fresh install starts on the corpus value and a knot with little data stays near it.
  """

  def __init__(self, bp, defaults, limits, horizon=TABLE_HORIZON, prior_weight=TABLE_PRIOR_WEIGHT):
    self.bp = [float(x) for x in bp]
    self.defaults = [float(x) for x in defaults]
    self.limits = (float(limits[0]), float(limits[1]))
    self.horizon = float(horizon)
    self.prior_weight = float(prior_weight)
    self.sums = [0.0] * len(self.bp)
    self.weights = [0.0] * len(self.bp)

  def _knots(self, v):
    v = float(np.clip(v, self.bp[0], self.bp[-1]))
    i = int(np.searchsorted(self.bp, v, side='right')) - 1
    if i >= len(self.bp) - 1:
      return ((len(self.bp) - 1, 1.0),)
    w = (v - self.bp[i]) / (self.bp[i + 1] - self.bp[i])
    return ((i, 1.0 - w), (i + 1, w))

  def values(self):
    return [float(np.clip((self.prior_weight * d + s) / (self.prior_weight + w), *self.limits))
            for d, s, w in zip(self.defaults, self.sums, self.weights, strict=True)]

  def value(self, v) -> float:
    return float(np.interp(v, self.bp, self.values()))

  def observe_driving(self, v):
    for k, w in self._knots(v):
      keep = 1.0 - w / self.horizon
      self.sums[k] *= keep
      self.weights[k] *= keep

  def learn(self, v, sample):
    if not math.isfinite(sample) or not self.limits[0] <= sample <= self.limits[1]:
      return
    for k, w in self._knots(v):
      self.sums[k] += w * sample
      self.weights[k] += w

  def state(self):
    return {'bp': list(self.bp), 'sums': list(self.sums), 'weights': list(self.weights)}

  def load(self, state) -> bool:
    try:
      if [float(x) for x in state['bp']] != self.bp:
        return False
      sums = [float(x) for x in state['sums']]
      weights = [float(x) for x in state['weights']]
    except (KeyError, TypeError, ValueError):
      return False
    if (len(sums) != len(self.bp) or len(weights) != len(self.bp) or
        not all(math.isfinite(x) and 0.0 <= x <= 10.0 * self.horizon for x in weights) or
        not all(math.isfinite(x) for x in sums)):
      return False
    self.sums, self.weights = sums, weights
    return True


def launch_boost(accel: float, a_ego: float, v_ego: float) -> float:
  """Extra table force for a launch the car is under-delivering: zero once aEgo reaches the command or by 8 m/s."""
  if not (math.isfinite(accel) and math.isfinite(a_ego) and math.isfinite(v_ego)):
    return 0.0
  if accel < LAUNCH_BOOST_MIN_ACCEL or v_ego >= LAUNCH_BOOST_END_MPS:
    return 0.0
  fade = min(1.0, (LAUNCH_BOOST_END_MPS - max(v_ego, 0.0)) / (LAUNCH_BOOST_END_MPS - LAUNCH_BOOST_FULL_MPS))
  return float(np.clip(LAUNCH_BOOST_GAIN * (accel - a_ego), 0.0, LAUNCH_BOOST_MAX)) * fade


def brake_command(accel: float, ratio: float) -> float:
  """Divide a brake request by the learned delivery ratio, fading to pass-through for hard braking."""
  if accel >= 0.0 or ratio <= 0.0:
    return accel
  blend = float(np.clip((accel - BRAKE_SCALE_NONE) / (BRAKE_SCALE_FULL - BRAKE_SCALE_NONE), 0.0, 1.0))
  return accel * (blend / ratio + (1.0 - blend))


def bosch_gas_learn_speed(fingerprint) -> float:
  """Frames of unit accel error x unit force per unit of gas-factor change (50 Hz)."""
  if fingerprint == CAR.HONDA_INSIGHT:
    return 150.0
  if fingerprint in (CAR.ACURA_RDX_3G, CAR.ACURA_RDX_3G_MMR):
    return 300.0
  if fingerprint == CAR.HONDA_ACCORD:
    # 2.0T + 10AT: during a launch the accel error is LAG (turbo spool, then each upshift's torque dip, ~2 s
    # per gear), not gain.  At 50 the factor wound 0.24 -> 1.75 in 3 s and then crashed to the floor at boost,
    # and the gas cut it produced re-armed the next cycle on every upshift (routes b5-b9, 2026-10-21).  A gain
    # estimator has to average over the slowest plant dynamic, so it learns 40x slower: at most ~0.05/s at a
    # 1 m/s^2 error and 2 m/s^2 force, i.e. under 0.1 across a whole launch.
    return 2000.0
  return 50.0


def bosch_gas_learning_allowed(last_gas: float, gas_rail: float) -> bool:
  """No learning against the gas rail: with the command already pinned there, a larger factor cannot change
  the gas, so the error integrates as pure wind-up that is paid back as a cut when the error flips."""
  return last_gas < gas_rail


def get_honda_bosch_wind_brake_mps2(v_ego: float) -> float:
  return float(np.interp(v_ego, [0.0, 13.4, 22.4, 31.3, 40.2], [0.0, 0.049, 0.136, 0.267, 0.441]))

def qualified(cp):
  return (cp.brand == "honda" and cp.carFingerprint in CLASSIC_BOSCH_CARS and cp.flags & HondaFlags.BOSCH and
          not cp.flags & HondaFlags.BOSCH_CANFD and cp.openpilotLongitudinalControl and
          not cp.passive and not cp.dashcamOnly and not cp.notCar and
          bool(cp.flags & HondaFlags.BOSCH_RADARLESS) == (cp.carFingerprint in HONDA_BOSCH_RADARLESS))


class BoschLongitudinal:
  def __init__(self, cp, params):
    if not qualified(cp):
      raise ValueError("Honda road load requires its admitted classic Bosch profile")
    self.CP = cp
    self.params = params
    self.pitch = 0.0
    self.accel = 0.0
    self.gas = 0.0
    self.bosch_braking = False
    self.bosch_last_gas = 0.0
    self.command_accel = 0.0
    self.gas_table = self.brake_table = None
    if cp.carFingerprint in BOSCH_TABLE_LEARNER_CARS:
      self.gas_table = SpeedTable(ACCORD_GAS_FACTOR_BP, ACCORD_GAS_FACTOR_V, GAS_FACTOR_LIMITS)
      self.brake_table = SpeedTable(ACCORD_BRAKE_RATIO_BP, ACCORD_BRAKE_RATIO_V, BRAKE_RATIO_LIMITS)
    self.gas_history = deque(maxlen=STEADY_FRAMES)
    self.brake_history = deque(maxlen=STEADY_FRAMES)
    self.set_factors(1.0, 1.0)

  def tables_state(self):
    if self.gas_table is None:
      return None
    return {'gas': self.gas_table.state(), 'brake': self.brake_table.state()}

  def load_tables(self, state) -> bool:
    if self.gas_table is None or not isinstance(state, dict):
      return False
    return bool(self.gas_table.load(state.get('gas', {})) and self.brake_table.load(state.get('brake', {})))

  def set_factors(self, gas, wind):
    self.bosch_gas_factor = float(gas) if math.isfinite(gas) and 0.01 <= gas <= 3.0 else 1.0
    self.bosch_wind_factor = float(wind) if math.isfinite(wind) and 0.1 <= wind <= 5.0 else 1.0
    self.bosch_wind_factor_before_brake = self.bosch_wind_factor
    self.bosch_gas_factor_before_gasmax = self.bosch_gas_factor
    self.bosch_wind_factor_before_gasmax = self.bosch_wind_factor

  def observe_orientation(self, orientation):
    if len(orientation) == 3 and math.isfinite(orientation[1]):
      self.pitch = orientation[1]

  def update(self, CC, CS, accel):
    actuators = CC.actuators
    hill_brake = math.sin(self.pitch) * ACCELERATION_DUE_TO_GRAVITY
    wind_brake_mps2 = get_honda_bosch_wind_brake_mps2(CS.out.vEgo)
    min_gas = self.params.BOSCH_GAS_LOOKUP_BP[0]
    self.accel = float(np.clip(accel, self.params.BOSCH_ACCEL_MIN, self.params.BOSCH_ACCEL_MAX))
    gas_pedal_force = self.accel + hill_brake
    v_ego = CS.out.vEgo
    if self.gas_table is not None:
      self.gas_table.observe_driving(v_ego)
      self.brake_table.observe_driving(v_ego)
      self.bosch_gas_factor = self.gas_table.value(v_ego)
    if self.CP.carFingerprint not in HONDA_BOSCH_RADARLESS:
      gas_pedal_force += wind_brake_mps2 * self.bosch_wind_factor
      if actuators.longControlState == LongCtrlState.pid and (not CS.out.gasPressed):
        gas_error = self.accel - CS.out.aEgo
        if self.gas_table is not None:
          self._learn_tables(CS, gas_pedal_force, wind_brake_mps2)
        elif (gas_error != 0.0 and gas_pedal_force > min_gas and
            bosch_gas_learning_allowed(self.bosch_last_gas, self.params.BOSCH_GAS_LOOKUP_V[-1])):
          gas_learn_speed = bosch_gas_learn_speed(self.CP.carFingerprint)
          gas_learn_force = gas_pedal_force
          self.bosch_gas_factor = float(np.clip(self.bosch_gas_factor + gas_error / gas_learn_speed * gas_learn_force, 0.1, 3.0))
        if gas_error != 0.0 and (not CS.out.brakePressed) and (CS.out.vEgo > 0.0):
          wind_learn_speed = 100.0 if self.CP.carFingerprint in (CAR.ACURA_RDX_3G, CAR.ACURA_RDX_3G_MMR) else 1000.0
          wind_adjust = 1.0 + wind_brake_mps2 / wind_learn_speed
          if gas_error > 0.0:
            self.bosch_wind_factor = float(np.clip(self.bosch_wind_factor * wind_adjust, 0.1, 3.0))
          else:
            self.bosch_wind_factor = float(np.clip(self.bosch_wind_factor / wind_adjust, 0.1, 3.0))
        if gas_pedal_force <= 0.0:
          self.bosch_wind_factor = max(self.bosch_wind_factor, self.bosch_wind_factor_before_brake)
        else:
          self.bosch_wind_factor_before_brake = self.bosch_wind_factor
        if gas_pedal_force >= self.params.BOSCH_ACCEL_MAX:
          self.bosch_gas_factor = min(self.bosch_gas_factor, self.bosch_gas_factor_before_gasmax)
          self.bosch_wind_factor = min(self.bosch_wind_factor, self.bosch_wind_factor_before_gasmax)
        else:
          self.bosch_gas_factor_before_gasmax = self.bosch_gas_factor
          self.bosch_wind_factor_before_gasmax = self.bosch_wind_factor
    self.launch_boost = 0.0
    if (self.gas_table is not None and CC.longActive and actuators.longControlState == LongCtrlState.pid and
        not CS.out.gasPressed and not CS.out.brakePressed):
      self.launch_boost = launch_boost(self.accel, CS.out.aEgo, v_ego)
    gas_lookup_input = (gas_pedal_force + self.launch_boost) * self.bosch_gas_factor
    self.gas = float(np.interp(gas_lookup_input, self.params.BOSCH_GAS_LOOKUP_BP, self.params.BOSCH_GAS_LOOKUP_V))
    self.gas = min(self.gas, max(60.0, self.bosch_last_gas + 60.0))
    self.bosch_last_gas = self.gas
    stopping = actuators.longControlState == LongCtrlState.stopping
    brake_on, brake_release = bosch_brake_force_thresholds(self.CP.carFingerprint)
    self.bosch_braking = update_honda_bosch_braking(self.bosch_braking, gas_pedal_force, stopping, CC.longActive,
                                                    brake_on, brake_release)
    self.command_accel = self.accel
    if self.brake_table is not None and self.bosch_braking and not stopping:
      self.command_accel = float(np.clip(brake_command(self.accel, self.brake_table.value(v_ego)),
                                         self.params.BOSCH_ACCEL_MIN, self.params.BOSCH_ACCEL_MAX))
    self.gas_history.append(self.gas)
    self.brake_history.append(self.command_accel if self.bosch_braking else 0.0)
    return self.command_accel, self.gas, gas_pedal_force, self.bosch_braking

  def _learn_tables(self, CS, gas_pedal_force, wind_brake_mps2):
    """Teach the per-speed tables from what the last command did, on steady commands only.

    Gas sample: the table force the gas sent one frame ago stood for, over the net force the powertrain produced
    (aEgo plus grade plus aero).  Brake sample: aEgo over the accel sent while the Bosch brake was in charge.
    """
    v_ego = CS.out.vEgo
    if CS.out.brakePressed or v_ego <= 1.0:
      return
    gas_rail = self.params.BOSCH_GAS_LOOKUP_V[-1]
    if (len(self.gas_history) == STEADY_FRAMES and max(self.gas_history) - min(self.gas_history) < STEADY_GAS_UNITS and
        40.0 < self.bosch_last_gas < gas_rail - 50.0 and gas_pedal_force > self.params.BOSCH_GAS_LOOKUP_BP[0]):
      bp, lookup = self.params.BOSCH_GAS_LOOKUP_BP, self.params.BOSCH_GAS_LOOKUP_V
      table_force = float(np.interp(self.bosch_last_gas, lookup, bp))
      net_force = CS.out.aEgo + math.sin(self.pitch) * ACCELERATION_DUE_TO_GRAVITY + wind_brake_mps2 * self.bosch_wind_factor
      if table_force > 0.05 and net_force > 0.15:
        self.gas_table.learn(v_ego, table_force / net_force)
    if (self.bosch_braking and self.bosch_last_gas <= 0.0 and len(self.brake_history) == STEADY_FRAMES and
        max(self.brake_history) < -0.3 and max(self.brake_history) - min(self.brake_history) < STEADY_BRAKE_MPS2):
      self.brake_table.learn(v_ego, CS.out.aEgo / self.command_accel)
