from openpilot.common.params import UnknownKeyName
from openpilot.starpilot.common.safe_mode import (
  SAFE_MODE_BACKUP_PARAM,
  SAFE_MODE_MANAGED_KEYS,
  apply_safe_mode,
  restore_safe_mode,
  _apply_value,
)


class RemovedParamStore:
  def get(self, key):
    raise UnknownKeyName(key)


class FakeParamStore:
  def __init__(self, values=None):
    self.values = dict(values or {})

  def get(self, key):
    return self.values.get(key)

  def get_stock_value(self, key):
    return None

  def put(self, key, value):
    self.values[key] = value

  def put_bool(self, key, value):
    self.values[key] = bool(value)

  def remove(self, key):
    self.values.pop(key, None)


def test_apply_value_ignores_removed_param():
  assert not _apply_value(RemovedParamStore(), "RemovedParam", "stale value")


def test_safe_mode_does_not_manage_manual_fingerprint():
  assert "ForceFingerprint" not in SAFE_MODE_MANAGED_KEYS


def test_safe_mode_migrates_saved_manual_fingerprint_out_of_backup():
  params = FakeParamStore()
  params_raw = FakeParamStore({
    "ForceFingerprint": False,
    SAFE_MODE_BACKUP_PARAM: {
      "ForceFingerprint": {"present": True, "value": True},
    },
  })

  apply_safe_mode(params, params_raw)

  assert params_raw.get("ForceFingerprint") is True
  assert "ForceFingerprint" not in params_raw.get(SAFE_MODE_BACKUP_PARAM)


def test_safe_mode_restore_ignores_stale_manual_fingerprint_backup():
  params_raw = FakeParamStore({
    "ForceFingerprint": True,
    SAFE_MODE_BACKUP_PARAM: {
      "ForceFingerprint": {"present": True, "value": False},
    },
  })

  restore_safe_mode(params_raw)

  assert params_raw.get("ForceFingerprint") is True


class StaleLibraryParamStore(FakeParamStore):
  """Params store whose compiled library predates some keys (raises UnknownKeyName like params_pyx does)."""

  def __init__(self, values=None, unknown=()):
    super().__init__(values)
    self.unknown = set(unknown)

  def _check(self, key):
    if key in self.unknown:
      raise UnknownKeyName(key)

  def get(self, key):
    self._check(key)
    return super().get(key)

  def get_stock_value(self, key):
    self._check(key)
    return super().get_stock_value(key)

  def put(self, key, value):
    self._check(key)
    super().put(key, value)

  def remove(self, key):
    self._check(key)
    super().remove(key)


def test_safe_mode_apply_and_restore_skip_keys_unknown_to_the_compiled_library():
  unknown = ("AccordRatePlantFF",)
  assert unknown[0] in SAFE_MODE_MANAGED_KEYS
  params = StaleLibraryParamStore(unknown=unknown)
  params_raw = StaleLibraryParamStore({"ExperimentalMode": True, "LateralTune": True}, unknown=unknown)

  assert apply_safe_mode(params, params_raw)

  # The known keys were still forced to their safe values and backed up.
  assert params_raw.get("ExperimentalMode") is False
  assert params_raw.get("LateralTune") is None
  backup = params_raw.get(SAFE_MODE_BACKUP_PARAM)
  assert backup["ExperimentalMode"] == {"present": True, "value": True}
  assert backup["LateralTune"] == {"present": True, "value": True}
  assert unknown[0] not in backup
  assert unknown[0] not in params_raw.values

  assert restore_safe_mode(params_raw)

  assert params_raw.get("ExperimentalMode") is True
  assert params_raw.get("LateralTune") is True
  assert params_raw.get(SAFE_MODE_BACKUP_PARAM) is None
  assert unknown[0] not in params_raw.values


def test_safe_mode_restore_survives_backup_entry_for_a_key_this_library_does_not_know():
  unknown = ("AccordEpsGainScale",)
  params_raw = StaleLibraryParamStore({
    "ExperimentalMode": False,
    SAFE_MODE_BACKUP_PARAM: {
      "ExperimentalMode": {"present": True, "value": True},
      unknown[0]: {"present": True, "value": 1.25},
    },
  }, unknown=unknown)

  assert restore_safe_mode(params_raw)

  assert params_raw.get("ExperimentalMode") is True
  assert params_raw.get(SAFE_MODE_BACKUP_PARAM) is None
  assert unknown[0] not in params_raw.values


def test_safe_mode_backs_up_a_key_once_the_library_learns_it():
  key = "AccordRatePlantFF"
  params = StaleLibraryParamStore(unknown=(key,))
  params_raw = StaleLibraryParamStore({key: True}, unknown=(key,))
  apply_safe_mode(params, params_raw)
  assert key not in params_raw.get(SAFE_MODE_BACKUP_PARAM)

  # Simulate the rebuilt library: the key is known now, so the next apply backs it up and forces it.
  params.unknown.clear()
  params_raw.unknown.clear()
  assert apply_safe_mode(params, params_raw)
  assert params_raw.get(SAFE_MODE_BACKUP_PARAM)[key] == {"present": True, "value": True}
  assert params_raw.get(key) is None

  # And with nothing new to back up, a repeat apply is a no-op.
  assert not apply_safe_mode(params, params_raw)
