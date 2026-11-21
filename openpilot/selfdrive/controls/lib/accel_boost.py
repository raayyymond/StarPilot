import numpy as np
from openpilot.common.constants import CV
from openpilot.common.realtime import DT_MDL

ACCEL_BOOST_MAX = 0.2
ACCEL_BOOST_RATE = 0.025
ACCEL_BOOST_PER_OVERRIDE = 0.05
ACCEL_BOOST_MIN_SPEED = 10 * CV.MPH_TO_MS

# Lead gate (owner, 2026-11-12; replay only, not driven): no boost toward a lead that is closing or braking.
# Replay u7->u8 (main set, engaged, no gas): the ungated boost was on in 39,476 rows with a lead closing
# (vRel < -0.5) or braking (aLeadK < -0.5); 6,583 of them turned a coast or light decel into accel.
# The applied boost fades out over LEAD_GATE_FADE_OUT_S and back in over LEAD_GATE_FADE_IN_S; total_boost is kept.
LEAD_GATE_ENABLED = True
LEAD_GATE_HEADWAY_S = 4.0  # only leads within this time gap at v_ego
LEAD_GATE_CLOSING_V = -0.5  # m/s, vRel below this is closing
LEAD_GATE_BRAKING_A = -0.5  # m/s^2, aLeadK below this is braking
LEAD_GATE_FADE_OUT_S = 0.5
LEAD_GATE_FADE_IN_S = 2.0


def lead_slowing(sm, v_ego):
  try:
    radar_state = sm['radarState']
  except (KeyError, TypeError):
    return False
  for lead in (getattr(radar_state, 'leadOne', None), getattr(radar_state, 'leadTwo', None)):
    if lead is None or not lead.present:
      continue
    if float(lead.dRel) > LEAD_GATE_HEADWAY_S * max(v_ego, 0.0):
      continue
    if float(lead.vRel) < LEAD_GATE_CLOSING_V or float(lead.aLeadK) < LEAD_GATE_BRAKING_A:
      return True
  return False


class AccelBoost:
  def __init__(self, lead_gate: bool = True):
    # lead_gate: the planner passes bosch_a_toggles.vehicle_supported(CP) (Bosch-A Honda, radar on or vision-only), so
    # every other car runs with the gate off (lead_scale stays 1.0, the pre-gate behavior). The default only reaches
    # direct constructions (tests, replay harnesses) and matches the Bosch-A source.
    self.lead_gate = lead_gate
    self.total_boost = 0.0
    self.boost_this_override = 0.0
    self.boost_eligible = False
    self.lead_scale = 1.0

  def update(self, sm, output_a_target_e2e, output_a_target_mpc, a_cruise):
    enabled = sm['selfdriveState'].enabled
    gas_pressed = sm['carState'].gasPressed
    v_ego = sm['carState'].vEgo
    speed_scale = np.interp(v_ego, [ACCEL_BOOST_MIN_SPEED, 2 * ACCEL_BOOST_MIN_SPEED], [0.0, 1.0])
    boost_scale = speed_scale * np.interp(output_a_target_e2e, [-1.0, -0.5, 5.0], [0.0, 1.0, 1.0], right=0.0)
    if LEAD_GATE_ENABLED and self.lead_gate:
      if lead_slowing(sm, v_ego):
        self.lead_scale = max(0.0, self.lead_scale - DT_MDL / LEAD_GATE_FADE_OUT_S)
      else:
        self.lead_scale = min(1.0, self.lead_scale + DT_MDL / LEAD_GATE_FADE_IN_S)
    model_limited = (sm['selfdriveState'].experimentalMode and
                     output_a_target_e2e + self.total_boost * boost_scale < min(output_a_target_mpc, a_cruise) - 0.1)

    if not enabled or not gas_pressed:
      self.boost_this_override = 0.0
      # Latch eligibility before the override changes the lead plan.
      self.boost_eligible = enabled and model_limited

    if not enabled:
      self.total_boost = 0.0
    elif gas_pressed and self.boost_eligible:
      increase = min(ACCEL_BOOST_RATE * DT_MDL * speed_scale,
                     ACCEL_BOOST_PER_OVERRIDE - self.boost_this_override, ACCEL_BOOST_MAX - self.total_boost)
      self.total_boost += increase
      self.boost_this_override += increase

    return output_a_target_e2e + self.total_boost * boost_scale * self.lead_scale
