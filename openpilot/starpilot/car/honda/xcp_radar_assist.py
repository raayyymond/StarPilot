"""Alpha opt-in gate for fusing Bosch-A XCP Array B kinematics into radard."""

from openpilot.starpilot.saved_source import read_saved

KEY = "AlphaHondaXcpRadarAssist"
LABEL = "Alpha: XCP Radar Lag Removal (Bosch-A)"
REASON = (
  "Fuses live 20 Hz Bosch Array B XCP Doppler velocity and acceleration into verified radar tracks to reduce lead lag. "
  "Off keeps XCP strictly read-only for logging."
)


def capability(cp, params=None) -> tuple | None:
  """Return a stable vehicle capability binding when Alpha XCP lag removal is supported, else None."""
  if cp is None:
    return None
  try:
    from opendbc.car.honda.bosch_a_radar import VERIFIED_BOSCH_A_CARS
    if (
      cp.brand != "honda"
      or cp.carFingerprint not in VERIFIED_BOSCH_A_CARS
      or cp.radarUnavailable
      or cp.notCar
      or cp.passive
      or cp.dashcamOnly
    ):
      return None
    if params is not None:
      release, readable = read_saved(params, "IsReleaseBranch", 8)
      if not readable or release not in (None, b"0"):
        return None
    return (
      str(cp.carFingerprint),
      str(cp.brand),
      int(cp.flags),
      int(cp.alternativeExperience),
      bool(cp.openpilotLongitudinalControl),
      bool(cp.pcmCruise),
      bool(cp.radarUnavailable),
    )
  except (AttributeError, KeyError, TypeError, ValueError, OverflowError):
    return None


def enabled(params, cp) -> bool:
  """Fail-closed runtime gate: True only on a supported non-release vehicle with an exact saved b'1'."""
  if params is None or capability(cp, params) is None:
    return False
  try:
    raw, readable = read_saved(params, KEY, 8)
  except (OSError, TypeError, ValueError):
    return False
  return bool(readable and raw == b"1")
