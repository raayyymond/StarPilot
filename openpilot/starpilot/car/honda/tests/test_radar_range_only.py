"""The Accord seam on top of the ported lead policy: range from the radar track, speed and accel from the vision lead."""
import pytest

from opendbc.car import gen_empty_fingerprint
from opendbc.car.honda.interface import CarInterface
from opendbc.car.honda.values import CAR
from openpilot.cereal import messaging
from openpilot.selfdrive.controls import radard
from openpilot.selfdrive.controls.radard import RadarD
from openpilot.selfdrive.controls.tests.test_radard_setup import FakeParams
from openpilot.starpilot.car.honda.radar import BoschALeadPolicy
from openpilot.starpilot.car.honda.radar_range_only import RangeOnlyLeadPolicy
from openpilot.starpilot.car.honda.tests.test_radar_pipeline import radar_data, source_tick

EGO = 25.0
CAN_CLOSING = -6.0   # the object feed says the lead closes at 6 m/s; the vision lead holds our speed
RANGE = 40.0


def run(range_only: bool, ticks: int = 40):
  sm = messaging.SubMaster(['modelV2', 'carState', 'radarTracks'])
  radar = RadarD(honda_bosch_a_radar=True, honda_bosch_a_range_only=range_only)
  for tick in range(ticks):
    source_tick(sm, tick, radar_data(True, CAN_CLOSING, RANGE), speed=EGO, probability=0.99, distance=RANGE + 1.0)
    radar.update(sm, sm['radarTracks'])
  return sm, radar


def drive(range_only: bool):
  return run(range_only)[1].radar_state.leadOne


def test_range_only_lead_keeps_the_track_range_and_takes_vision_speed():
  lead = drive(range_only=True)
  assert lead.present and lead.radar and lead.radarTrackId == 1
  assert lead.dRel == pytest.approx(RANGE)                # the track's range, not the vision lead's 41 m
  assert lead.vRel == pytest.approx(0.0, abs=1e-3)        # the vision lead holds our speed
  assert lead.vLead == pytest.approx(EGO, abs=1e-3) and lead.vLeadK == pytest.approx(EGO, abs=1e-3)
  assert lead.aLeadK == pytest.approx(0.0, abs=1e-3)


def test_the_ported_policy_alone_publishes_the_can_speed():
  lead = drive(range_only=False)
  assert lead.present and lead.radar and lead.dRel == pytest.approx(RANGE)
  assert lead.vRel < -1.0                                  # what the seam exists to keep away from the planner


def test_a_track_fed_by_array_b_passes_through():
  sm, radar = run(range_only=True)
  radar.tracks[1].xcp_active = True                        # what update_tracks sets when Array B feeds this track
  lead = radar.honda_bosch_a.leads(sm, radar, EGO)[0]
  assert lead['present'] and lead['radar']
  assert lead['vRel'] < -1.0                               # the track's own speed is published, not the vision lead's


def test_setup_gives_the_accord_the_range_only_policy_and_the_civic_the_plain_one(tmp_path):
  accord = CarInterface.get_params(CAR.HONDA_ACCORD, gen_empty_fingerprint(), [], False, False, False)
  civic = CarInterface.get_params(CAR.HONDA_CIVIC_BOSCH, gen_empty_fingerprint(), [], False, False, False)
  _, _, rd_accord = radard.setup(FakeParams(tmp_path), accord)
  _, _, rd_civic = radard.setup(FakeParams(tmp_path), civic)
  assert type(rd_accord.honda_bosch_a) is RangeOnlyLeadPolicy
  assert type(rd_civic.honda_bosch_a) is BoschALeadPolicy
