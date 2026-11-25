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


def _fed(policy, radar, sm_at, point, target, cycles, start=0):
  """Run radard cycles with `target` (or nothing) in the snapshot. Returns whether Array B fed the track, per cycle."""
  from types import SimpleNamespace
  fed = []
  for cycle in range(start, start + cycles):
    now = T0 + cycle * 50_000_000
    listed = [dict(target, kin_updated_ns=now, kin_age_ms=20.0)] if target is not None else []
    policy.set_xcp_snapshot({"mono_ns": now, "targets": listed})
    policy.update_tracks(sm_at(now, cycle + 1), SimpleNamespace(points=[point]), radar)
    fed.append(radar.tracks[point.trackId].xcp_active)
  return fed


T0 = 10_000_000_000
SLOT = dict(slot=33, rtid=403, age=40, stage="measured", ghost=False, flags=0xE840, dRel_m=29.0, yRel_m=0.2, vRel_mps=-2.0,
            ax_mps2=0.0, lat_valid=True, trust_accel=True, op_track_id=22)


def _held_setup():
  from openpilot.starpilot.car.honda.tests.test_xcp_radar_assist import _dummy_radar, _make_point, _make_sm
  policy = RangeOnlyLeadPolicy(xcp_assist=True, xcp_path=None)
  return policy, _dummy_radar(), (lambda now, frame: _make_sm(now, frame=frame)), _make_point(22, 28.0, 0.2, -2.0)


def test_a_fed_track_keeps_its_last_read_while_the_slot_goes_unread():
  from openpilot.starpilot.car.honda import radar_range_only
  policy, radar, sm_at, point = _held_setup()
  assert _fed(policy, radar, sm_at, point, SLOT, 4) == [True] * 4
  speed = radar.tracks[22].vLeadK
  hold = radar_range_only.XCP_HOLD_CYCLES
  assert _fed(policy, radar, sm_at, point, None, hold + 2, start=4) == [True] * hold + [False] * 2   # then it is let go
  policy, radar, sm_at, point = _held_setup()
  _fed(policy, radar, sm_at, point, SLOT, 4)
  assert _fed(policy, radar, sm_at, point, None, 2, start=4) == [True, True]
  assert radar.tracks[22].vLeadK == pytest.approx(speed)                 # held: the filter's last estimate, unchanged
  assert _fed(policy, radar, sm_at, point, SLOT, 3, start=6) == [True] * 3   # read again: no change of source at all
  assert radar.tracks[22].xcp_hold_frames == 0                           # and the next gap gets the full hold again


def test_a_slot_that_is_listed_and_refused_is_not_held():
  policy, radar, sm_at, point = _held_setup()
  _fed(policy, radar, sm_at, point, SLOT, 4)
  assert _fed(policy, radar, sm_at, point, dict(SLOT, dRel_m=45.0), 1, start=4) == [False]      # the slot now says another range
  policy, radar, sm_at, point = _held_setup()
  _fed(policy, radar, sm_at, point, SLOT, 4)
  assert _fed(policy, radar, sm_at, point, dict(SLOT, rtid=404, dRel_m=60.0, op_track_id=None), 1, start=4) == [False]   # another object


def test_a_held_read_is_still_judged_against_the_can_track():
  from openpilot.starpilot.car.honda.tests.test_xcp_radar_assist import _make_point
  policy, radar, sm_at, point = _held_setup()
  _fed(policy, radar, sm_at, point, SLOT, 4)
  moved = _make_point(22, 28.0, 0.2, -7.0)                               # the CAN track now closes 5 m/s faster than the held read
  assert _fed(policy, radar, sm_at, moved, None, 1, start=4) == [False]


def test_the_plain_policy_does_not_hold():
  from openpilot.starpilot.car.honda.tests.test_xcp_radar_assist import _dummy_radar, _make_point, _make_sm
  policy, radar, point = BoschALeadPolicy(xcp_assist=True, xcp_path=None), _dummy_radar(), _make_point(22, 28.0, 0.2, -2.0)
  sm_at = lambda now, frame: _make_sm(now, frame=frame)
  _fed(policy, radar, sm_at, point, SLOT, 4)
  assert _fed(policy, radar, sm_at, point, None, 1, start=4) == [False]
