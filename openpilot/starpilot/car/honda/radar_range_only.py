"""Fork seam: Bosch-A lead policy for cars whose object feed gives a steady range but a stepwise, lagging speed.

radar.py is verbatim from bob-unify-car. On the Accord its lead still took phantom decelerations from the CAN speed:
open-loop replay of route 00000024--6f4943e0a6 (2026-11-21, current decoder) gave 24 episodes of aLeadK <= -1 with a
calm vision lead in 9 min, down to -2.8, 21 of them not in the lead's own range; the range-only lead gave 0.

So here a radar lead that matches the vision lead keeps the track's range and position and takes its speed and
acceleration from the vision lead, as before the port. Everything else is the parent's: association, lead keep, the
post-standstill gate, and the low-speed lead that is not the vision lead's car. A track fed by Array B (xcp_active)
is passed through untouched, so its Doppler speed reaches the planner when the assist is on.
"""
from types import SimpleNamespace

from openpilot.starpilot.car.honda.radar import BoschALeadPolicy, get_RadarState_from_vision, track_matches_vision

VISION_SPEED_FIELDS = ('vRel', 'vLead', 'vLeadK', 'aLeadK', 'aLeadTau')


class RangeOnlyLeadPolicy(BoschALeadPolicy):
  def __init__(self, **kwargs):
    super().__init__(**kwargs)
    self.range_only_vision = [SimpleNamespace(prev_aLeadK=0.0) for _ in range(2)]

  def leads(self, sm, radar, model_v_ego):
    output = super().leads(sm, radar, model_v_ego)
    model_leads = sm['modelV2'].leadsV3
    detection_probability = float(getattr(self.starpilot_toggles, 'lead_detection_probability', 0.35))
    for index, lead in enumerate(output):
      if not (lead.get('present', False) and lead.get('radar', False)):
        continue
      track = self.tracks.get(int(lead.get('radarTrackId', -1)))
      probability = float(radar.lead_prob_filters[index].x)
      if track is None or getattr(track, 'xcp_active', False) or probability <= detection_probability:
        continue
      if not track_matches_vision(track, model_leads[index], self.v_ego, dist_scale=0.25, dist_floor=5.0, vel_limit=10.0,
                                  y_std_scale=1.0, y_floor=1.0):
        continue
      vision = get_RadarState_from_vision(model_leads[index], self.v_ego, model_v_ego, probability,
                                          self.range_only_vision[index])
      for field in VISION_SPEED_FIELDS:
        lead[field] = vision[field]
    return output
