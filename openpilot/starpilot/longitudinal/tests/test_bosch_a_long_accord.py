"""The Accord variant of the Bosch-A long enhancement drops coast first only. Static tests; no replay or road evidence."""
from types import SimpleNamespace

from openpilot.starpilot.longitudinal import honda_bosch_a_long as hb
from openpilot.starpilot.longitudinal.honda_bosch_a_long_accord import AccordBoschALongEnhancement

ACCORD = SimpleNamespace(brand='honda', carFingerprint='HONDA_ACCORD', openpilotLongitudinalControl=True,
                         radarUnavailable=False)
# A distant radar lead tapping its brakes at matched speed: clear for coast first by every check in _clear().
FAR_LEAD = SimpleNamespace(present=True, radar=True, dRel=80.0, vLead=30.0, vRel=0.0, aLeadK=-1.0)
ABSENT = SimpleNamespace(present=False, radar=False, dRel=0.0, vLead=0.0, vRel=0.0, aLeadK=0.0)


def coast_first(enh, target):
  return enh._coast_first(lead_one=FAR_LEAD, lead_two=ABSENT, target=target, v_ego=30.0, follow_s=1.45, fcw=False,
                          reset_state=False)


def test_coast_first_returns_target_where_parent_floors_it():
  parent = hb.HondaBoschALongEnhancement(ACCORD, 0.05)
  assert parent.coast_enabled
  assert coast_first(parent, -2.0) == hb.COAST_ACCEL  # the premise: the source raises this brake to the floor
  accord = AccordBoschALongEnhancement(ACCORD, 0.05)
  assert accord.coast_enabled and accord.stock_feel
  assert coast_first(accord, -2.0) == -2.0
  assert not accord.coast_active


def test_step_keeps_the_onset_law():
  # Everything but coast first is the parent's: with follow_s given, step() equals the parent's onset output.
  parent, accord = hb.HondaBoschALongEnhancement(ACCORD, 0.05), AccordBoschALongEnhancement(ACCORD, 0.05)
  kwargs = dict(lead_one=FAR_LEAD, lead_two=ABSENT, prev=0.0, target=-2.0, v_ego=30.0, stopping=False, standstill=False,
                reset_state=False, fcw=False)
  assert accord.step(**kwargs, follow_s=1.45) == parent.step(**kwargs)
