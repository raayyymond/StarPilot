import json
import math
import os
from pathlib import Path

TABLES_FILE = Path("honda") / "bosch_long_tables.json"


def force_stop_distance_qualified(cp) -> bool:
  """The Accord on Bosch alpha long brakes a Force Stop to its distance, like the Ioniq 6 and 5 PE.

  2026-11-23 routes 00000073/74: the default law hands over to the native stop 6.2 m before the model's stop
  point, whatever the speed, and only ever shortens its distance. The car was still at 2.3-3.9 m/s there, so
  it came to rest 1-5 m short (t=425.8: 4 m, the driver crept 4.0 m), and 5-10 m short where the model moved
  its stop point out while the car slowed (t=524.3, crept 6.6 m). Distance braking follows the model's point
  until the last metres and closes the loop on what is left.
  """
  try:
    from opendbc.car.honda.values import CAR
    return bool(cp.brand == 'honda' and cp.carFingerprint == CAR.HONDA_ACCORD and
                cp.openpilotLongitudinalControl and not cp.pcmCruise)
  except (AttributeError, ImportError, TypeError, ValueError):
    return False


def tables_path() -> Path:
  from openpilot.starpilot.storage import starpilot_storage_root
  return starpilot_storage_root() / TABLES_FILE


class BoschLearningParams:
  def __init__(self, params, owner, path: Path | None = None):
    self.params = params
    self.owner = owner
    factors = []
    for key in ("HondaGasFactorParams", "HondaWindFactorParams"):
      try:
        value = float(params.get(key, return_default=True))
        if not math.isfinite(value):
          value = 1.0
      except (TypeError, ValueError, OverflowError, RuntimeError):
        value = 1.0
      factors.append(value)
    owner.set_factors(*factors)
    self.saved = (owner.bosch_gas_factor, owner.bosch_wind_factor)
    # Per-speed tables live in a StarPilot-owned file: the Params key set is compiled into a prebuilt library.
    self.path = None
    self.saved_tables = None
    if getattr(owner, "tables_state", None) is not None and owner.tables_state() is not None:
      try:
        self.path = path if path is not None else tables_path()
        state = json.loads(self.path.read_text())
        if owner.load_tables(state):
          self.saved_tables = json.dumps(owner.tables_state(), sort_keys=True)
      except (OSError, ValueError, TypeError):
        pass

  def persist(self, frame):
    if frame > 0 and frame % 6000 == 0:
      # Learning only moves while openpilot drives the pedals; skip rewriting unchanged values.
      factors = (self.owner.bosch_gas_factor, self.owner.bosch_wind_factor)
      if factors != self.saved:
        self.params.put("HondaGasFactorParams", factors[0])
        self.params.put("HondaWindFactorParams", factors[1])
        self.saved = factors
      if self.path is not None:
        tables = json.dumps(self.owner.tables_state(), sort_keys=True)
        if tables != self.saved_tables:
          try:
            self.path.parent.mkdir(parents=True, exist_ok=True)
            tmp = self.path.with_suffix(".tmp")
            tmp.write_text(tables)
            os.replace(tmp, self.path)
            self.saved_tables = tables
          except OSError:
            pass
