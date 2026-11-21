"""The Bosch-A longitudinal car gates. There are no owner toggles (owner 2026-11-13: no toggle, default for all Bosch-A cars).

Two gates, both false for None or an unreadable CP:
- vehicle_supported(CP): a Honda in VERIFIED_BOSCH_A_CARS with openpilot longitudinal, radar on or off.
- long_tuning_active(CP): vehicle_supported and the radar on (CP.radarUnavailable False), the same gate radard uses to build
  BoschALeadPolicy (honda_bosch_a_long.active_for).

Need the radar (long_tuning_active; owner 2026-11-13: a vision-only Bosch-A Honda falls back to the stock planner for these):
HondaBoschALongEnhancement (Stock Brake Feel, onset ramp, coast first, need floor, newborn-lead hold, implausible-drop bound),
BoschARelease.coast and BoschAStopGap.
Run vision-only too (vehicle_supported): HumanFollowing and HumanAcceleration (human_following.enabled_for), BoschARelease.smooth
(release limit + dwell), BoschARelease.action_t (plan read-ahead) and the Accel Boost lead gate (AccelBoost(lead_gate=...);
at HEAD it had no car check and ran on every car; owner 2026-11-13: keep it vision-only too - replay showed 53 of 283
vision-only episodes braking later than 0.2 s without it, and row-identical to 786d62d5d2 with it). The NeedFloor 1.5 s min-gap gate and the camera cap hold
(radar_camera_witness, C-012) are always on inside their gates. The old HumanFollowing owner param (default Off) was removed.

Constructor defaults: HondaBoschALongEnhancement(CP, dt), BoschARelease(CP) and BoschAStopGap(CP) compute their gate from CP;
AccelBoost() with no argument keeps its lead gate ON (tests and replay harnesses); the planners pass
lead_gate=vehicle_supported(CP). The planners' `bosch_a=` keyword is ignored (kept for replay harnesses).
"""


def vehicle_supported(CP) -> bool:
  """Verified Bosch-A Honda with openpilot longitudinal; the radar may be on or off."""
  if CP is None:
    return False
  try:
    from opendbc.car.honda.bosch_a_radar import VERIFIED_BOSCH_A_CARS
    return bool(CP.brand == "honda" and CP.carFingerprint in VERIFIED_BOSCH_A_CARS and CP.openpilotLongitudinalControl)
  except Exception:
    return False


def long_tuning_active(CP) -> bool:
  """vehicle_supported and the radar on. A missing radarUnavailable reads as off."""
  return vehicle_supported(CP) and not bool(getattr(CP, 'radarUnavailable', True))
