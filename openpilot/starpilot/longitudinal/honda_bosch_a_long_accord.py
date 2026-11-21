"""Fork-owned Accord variant of the Honda Bosch A Long Enhancement (honda_bosch_a_long.py stays verbatim from the source)."""
from openpilot.starpilot.longitudinal.honda_bosch_a_long import HondaBoschALongEnhancement


class AccordBoschALongEnhancement(HondaBoschALongEnhancement):
  """HondaBoschALongEnhancement without coast first: owner, the coast-first floor is not ported (COAST_ACCEL = -0.5 sits
  past the Accord's brake arm point of -0.45)."""

  def __init__(self, CP, dt: float):
    super().__init__(CP, dt)
    from opendbc.car.honda.bosch_a_radar import BOSCH_A_RANGE_ONLY_CARS
    if CP.carFingerprint in BOSCH_A_RANGE_ONLY_CARS:
      # The newborn-lead hold and the implausible-drop bound tame the CAN speed's Kalman artefacts. A range-only lead's
      # speed and accel are the vision lead's, and the hold's -1.0 floor would clip a real brake for up to 2 s after
      # every track change. The MPC reads radarState as published.
      self.newborn_hold = None
      self.drop_bound = None

  def _coast_first(self, *, lead_one, lead_two, target: float, v_ego: float, follow_s: float, fcw: bool,
                   reset_state: bool) -> float:
    self.reset_coast()  # coast_floor stays None, so coast_active reads False
    return float(target)
