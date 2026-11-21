"""radard's startup path (setup) on the Accord: a startup exception here costs the car a whole drive."""
from opendbc.car import gen_empty_fingerprint
from opendbc.car.honda.interface import CarInterface
from opendbc.car.honda.values import CAR
from openpilot.selfdrive.controls import radard
from openpilot.starpilot.car.honda.radar import BoschALeadPolicy


class FakeParams:
  """No saved params: every key reads as absent."""
  def __init__(self, root):
    self.root = root

  def get_param_path(self, key: str = "") -> str:
    return str(self.root / key)

  def get(self, key, *args, **kwargs):
    return None

  def get_bool(self, key, *args, **kwargs):
    return False


def accord_cp(radar_unavailable: bool = False):
  cp = CarInterface.get_params(CAR.HONDA_ACCORD, gen_empty_fingerprint(), [], False, False, False)
  cp.radarUnavailable = radar_unavailable
  return cp


def test_setup_builds_a_bosch_a_radard_for_the_accord(tmp_path):
  sm, pm, rd = radard.setup(FakeParams(tmp_path), accord_cp())
  assert isinstance(rd, radard.RadarD)
  assert isinstance(rd.honda_bosch_a, BoschALeadPolicy)
  assert rd.honda_bosch_a.xcp_assist is False  # no saved assist key: the Array B assist stays inert
  assert 'starpilotCarState' in sm.services  # the radar point STATUS carrier, never a validity input
  for ignored in (sm.ignore_alive, sm.ignore_average_freq, sm.ignore_valid):
    assert 'starpilotCarState' in ignored


def test_setup_without_radar_has_no_bosch_a_policy(tmp_path):
  sm, pm, rd = radard.setup(FakeParams(tmp_path), accord_cp(radar_unavailable=True))
  assert rd.honda_bosch_a is None
  assert 'starpilotCarState' not in sm.services
