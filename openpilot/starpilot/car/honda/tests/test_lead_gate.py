"""2026-11-08 route 0000002a--d7d0d36342: the lead marker flickered. Two radard gates, measured over 35,846 frames:
every leadOne drop happened with the filtered probability at 0.35-0.40 against a bare "> 0.35" gate, and at 80-130 m
the model read 20-25 % longer than the radar for the same car, so a fixed 25 m staleness limit evicted the matched track
every few frames and the borrowed range stepped between radar and vision. Rendering only drew what radard published."""

from types import SimpleNamespace

import pytest

from openpilot.common.realtime import DT_MDL
from openpilot.starpilot.car.honda import radar as honda_radar
from openpilot.starpilot.car.honda.radar import RADAR_TO_CAMERA, BoschALeadPolicy, Track

V_EGO = 30.0
# radard.KalmanParams(DT_MDL) without importing radard (its messaging .so is device-only); the gains do not matter here.
KALMAN = SimpleNamespace(A=[[1.0, DT_MDL], [0.0, 1.0]], C=[1.0, 0.0], K=[[0.1988689], [0.28555364]])


def vision_lead(distance: float, prob: float = 0.95, speed: float = V_EGO):
  return SimpleNamespace(x=[distance + RADAR_TO_CAMERA], y=[0.0], v=[speed], a=[0.0], xStd=[max(1.0, distance * 0.1)],
                         yStd=[1.0], vStd=[1.0], prob=prob)


def track(identifier: int, distance: float, frames: int = 5) -> Track:
  t = Track(identifier, V_EGO, KALMAN)
  for _ in range(frames):
    t.update(distance, 0.0, 0.0, V_EGO, True, True)
  return t


def policy(tracks: dict[int, Track]) -> BoschALeadPolicy:
  p = BoschALeadPolicy(range_only=True)
  p.ready, p.v_ego, p.tracks = True, V_EGO, tracks
  return p


def step(p: BoschALeadPolicy, lead, filtered_prob: float):
  sm = {'modelV2': SimpleNamespace(leadsV3=[lead, lead]), 'carState': SimpleNamespace(standstill=False)}
  radar = SimpleNamespace(lead_prob_filters=[SimpleNamespace(x=filtered_prob), SimpleNamespace(x=filtered_prob)])
  return p.leads(sm, radar, V_EGO)[0]


def test_a_present_lead_survives_the_probability_dipping_just_under_the_entry_gate():
  p = policy({})
  lead = vision_lead(120.0)
  assert step(p, lead, 0.30)['present'] is False          # never entered: the entry gate is unchanged
  assert step(p, lead, 0.36)['present'] is True
  for _ in range(honda_radar.LEAD_GATE_HOLD_FRAMES):       # the measured flicker: 0.26-0.34 for a few hundred ms
    assert step(p, lead, 0.30)['present'] is True
  assert step(p, lead, 0.30)['present'] is False           # held long enough: it goes
  assert step(p, lead, 0.36)['present'] is True
  assert step(p, lead, 0.20)['present'] is False           # well under the gate: it goes at once
  assert p.lead_gate_open == [False, False] and p.lead_gate_below == [0, 0]


def test_the_hold_resets_once_the_probability_clears_the_entry_gate_again():
  p = policy({})
  lead = vision_lead(120.0)
  step(p, lead, 0.40)
  for _ in range(honda_radar.LEAD_GATE_HOLD_FRAMES - 1):
    step(p, lead, 0.30)
  step(p, lead, 0.40)                                      # back over the gate: the hold starts fresh
  for _ in range(honda_radar.LEAD_GATE_HOLD_FRAMES):
    assert step(p, lead, 0.30)['present'] is True


def test_a_matched_track_keeps_lending_its_range_through_the_models_long_range_bias():
  # radar 77 m, model 100 m to acquire, then the measured 104 m: old code evicted after 3 cycles (27 m > 25 m) and the
  # lead's range stepped 77 -> 104 -> 77 for 1-10 frames at a time.
  p = policy({7: track(7, 77.0)})
  lead = step(p, vision_lead(100.0), 0.95)
  assert lead['radar'] and lead['dRel'] == pytest.approx(77.0)
  for _ in range(12):
    lead = step(p, vision_lead(104.0), 0.95)
    assert lead['radar'] and lead['radarTrackId'] == 7 and lead['dRel'] == pytest.approx(77.0)
  assert p.preferred_gross_distance_stale_counts[0] == 0


def test_a_track_far_outside_the_scaled_limit_is_still_evicted():
  p = policy({7: track(7, 77.0)})
  step(p, vision_lead(100.0), 0.95)
  for _ in range(honda_radar.HONDA_BOSCH_A_GROSS_DISTANCE_STALE_CYCLES):
    lead = step(p, vision_lead(130.0), 0.95)              # 53 m off at 130 m: more than 0.35 x 130 = 45.5 m
  assert p.prev_lead_track_ids[0] == -1
  lead = step(p, vision_lead(130.0), 0.95)
  assert lead['present'] and not lead['radar'] and lead['dRel'] == pytest.approx(130.0)


def test_the_floor_still_applies_at_short_range():
  # 25 m stays the limit under ~71 m: a 30 m disagreement at 40 m is a different car.
  p = policy({3: track(3, 40.0)})
  step(p, vision_lead(44.0), 0.95)
  for _ in range(honda_radar.HONDA_BOSCH_A_GROSS_DISTANCE_STALE_CYCLES):
    step(p, vision_lead(70.0), 0.95)
  assert p.prev_lead_track_ids[0] == -1
