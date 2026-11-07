import json
import math
import os
from pathlib import Path

TABLES_FILE = Path("honda") / "bosch_long_tables.json"


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
