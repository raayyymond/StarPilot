"""Fork seam: Bosch-A lead policy for cars whose object feed gives a steady range but a stepwise, lagging speed.

radar.py is verbatim from bob-unify-car. On the Accord its lead still took phantom decelerations from the CAN speed:
open-loop replay of route 00000024--6f4943e0a6 (2026-11-21, current decoder) gave 24 episodes of aLeadK <= -1 with a
calm vision lead in 9 min, down to -2.8, 21 of them not in the lead's own range; the range-only lead gave 0.

So here a radar lead that matches the vision lead keeps the track's range and position and takes its speed and
acceleration from the vision lead, as before the port. Everything else is the parent's: association, lead keep, the
post-standstill gate, and the low-speed lead that is not the vision lead's car. A track fed by Array B (xcp_active)
is passed through untouched, so its Doppler speed reaches the planner when the assist is on.

Between the two sources the lead's speed steps: the vision lead's speed sits a median 1.0-1.5 m/s rms from the
Doppler speed and 0.4-1.2 s behind it (routes 00000075/77, 2026-11-24, 142 s / 160 s of fed lead), and on those
drives the lead changed source 185 / 54 times. Most changes were not the radar losing the car: 64 of 82 and 14 of 20
gaps in the feed lasted 0.35 s or less, and in them the slot was simply not read again within the 200 ms radard
allows. So a fed track whose slot is missing from the snapshot keeps its last Array B read on offer for XCP_HOLD_CYCLES
more radard cycles. The parent's gate still judges it against the CAN track every cycle, the published speed is the
filter's last estimate, and a slot that is listed but refused, or listed under another object, is never held.
"""
from types import SimpleNamespace

from openpilot.starpilot.car.honda.radar import BoschALeadPolicy, get_RadarState_from_vision, track_matches_vision

VISION_SPEED_FIELDS = ('vRel', 'vLead', 'vLeadK', 'aLeadK', 'aLeadTau')
XCP_HOLD_CYCLES = 5   # 250 ms past the 200 ms a read may be old: three radar renewals


class RangeOnlyLeadPolicy(BoschALeadPolicy):
  def __init__(self, **kwargs):
    super().__init__(**kwargs)
    self.range_only_vision = [SimpleNamespace(prev_aLeadK=0.0) for _ in range(2)]

  def _read_xcp_snapshot(self, now_ns: int, *, used_boot_clock: bool = False):
    snapshot = super()._read_xcp_snapshot(now_ns, used_boot_clock=used_boot_clock)
    if snapshot is None:
      return None
    targets = [target for target in (snapshot.get('targets') or ()) if isinstance(target, dict)]
    listed = {target.get('slot') for target in targets}
    held = []
    for track in self.tracks.values():
      if not track.xcp_active or track.last_xcp_match is None or track.xcp_slot_key is None:
        continue
      if track.xcp_slot_key[0] in listed:
        track.xcp_hold_frames = 0
      elif track.xcp_hold_frames < XCP_HOLD_CYCLES:
        track.xcp_hold_frames += 1
        held.append(track.last_xcp_match)
    return {**snapshot, 'targets': targets + held} if held else snapshot

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
