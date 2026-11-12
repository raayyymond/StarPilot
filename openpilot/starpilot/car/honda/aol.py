from opendbc.car.structs import car
from opendbc.car.honda.stock_aol import qualified as qualified_stock, BOSCH_WORDS, NIDEC_WORDS, temporary_restriction, ALT_RADAR_STOCK, CLASSIC_BOSCH
from openpilot.starpilot.car.honda.stock_intent import HondaStockCardIntent
from opendbc.car.honda.values import CAR, HondaFlags, HondaSafetyFlags
from openpilot.starpilot.aol.policy import AolVehiclePolicy


CLASSIC_BOSCH_AOL_CARS = frozenset((
  CAR.HONDA_NBOX_2G, CAR.HONDA_ACCORD, CAR.HONDA_CIVIC_BOSCH, CAR.HONDA_CIVIC_BOSCH_DIESEL,
  CAR.HONDA_CRV_5G, CAR.HONDA_CRV_HYBRID, CAR.ACURA_RDX_3G, CAR.HONDA_INSIGHT,
  CAR.HONDA_E, CAR.HONDA_E_ADVANCE,
))
_CLASSIC_BOSCH_AOL_IDS = frozenset(str(candidate) for candidate in CLASSIC_BOSCH_AOL_CARS)
_NON_CLASSIC_BOSCH_FLAGS = HondaFlags.BOSCH_RADARLESS | HondaFlags.BOSCH_CANFD | HondaFlags.BOSCH_ALT_RADAR
_AOL_SAFETY_PARAMS = frozenset((
  int(HondaSafetyFlags.BOSCH_LONG),
  int(HondaSafetyFlags.BOSCH_LONG | HondaSafetyFlags.ALT_BRAKE),
  int(HondaSafetyFlags.BOSCH_LONG | HondaSafetyFlags.AOL_BOSCH_LONG),
  int(HondaSafetyFlags.BOSCH_LONG | HondaSafetyFlags.ALT_BRAKE | HondaSafetyFlags.AOL_BOSCH_LONG),
))


def qualified_honda(CP) -> bool:
  rdx = CP.carFingerprint == CAR.ACURA_RDX_3G
  word = int(CP.safetyConfigs[0].safetyParam) if len(CP.safetyConfigs) == 1 else -1
  return (word in ((131, 163) if rdx else _AOL_SAFETY_PARAMS) and
          str(CP.carFingerprint) in _CLASSIC_BOSCH_AOL_IDS and
          CP.brand == 'honda' and bool(CP.flags & HondaFlags.BOSCH) and
          not bool(CP.flags & _NON_CLASSIC_BOSCH_FLAGS) and
          bool(CP.openpilotLongitudinalControl) and not bool(CP.pcmCruise) and
          not bool(CP.notCar) and not bool(CP.passive) and not bool(CP.dashcamOnly) and
          len(CP.safetyConfigs) == 1 and
          CP.safetyConfigs[0].safetyModel == car.CarParams.SafetyModel.hondaBosch and
          int(CP.safetyConfigs[0].safetyParam) in (_AOL_SAFETY_PARAMS | {131, 163}))


def policy_for(CP) -> AolVehiclePolicy:
  if qualified_stock(CP):
    safety_addition = 256 if CP.carFingerprint == CAR.HONDA_ODYSSEY_TWN else 0
    if CP.flags & HondaFlags.GAS_INTERCEPTOR:
      safety_addition |= int(HondaSafetyFlags.NIDEC_PEDAL_AOL)
    if CP.carFingerprint in CLASSIC_BOSCH and CP.openpilotLongitudinalControl:
      safety_addition |= int(HondaSafetyFlags.CLASSIC_LONG_AOL | HondaSafetyFlags.AOL_BOSCH_LONG)
    if CP.carFingerprint in ALT_RADAR_STOCK:
      if CP.carFingerprint == CAR.ACURA_MDX_4G:
        safety_addition |= int(HondaSafetyFlags.STOCK_TJA)
      if CP.openpilotLongitudinalControl:
        safety_addition |= int(HondaSafetyFlags.ALT_RADAR_AOL | HondaSafetyFlags.AOL_BOSCH_LONG)
    return AolVehiclePolicy(intent_supported=True, settings_supported=True, runtime_supported=True,
                            normal_runtime_supported=True, ordinary_axis_ack_required=qualified_stock(CP, marked_only=True), explicit_latch=True,
                            alternative_experience_addition=32,
                            safety_param_addition=safety_addition)
  qualified = qualified_honda(CP)
  return AolVehiclePolicy(
    intent_supported=qualified, settings_supported=qualified,
    runtime_supported=qualified and bool(int(CP.safetyConfigs[0].safetyParam) & 0x20),
    distance_personality=qualified,
    safety_param_addition=0x20 if qualified else 0,
  )


def native_profile_supported(model: int, param: int) -> bool:
  return ((model == int(car.CarParams.SafetyModel.hondaBosch) and param in BOSCH_WORDS | {34, 35, 163}) or
          (model == int(car.CarParams.SafetyModel.hondaNidec) and param in NIDEC_WORDS))


def native_accepts_cp(CP, model: int, param: int) -> bool:
  # Honda's native receipt checks the safety profile, then the shared exact CP match.
  if qualified_stock(CP, marked_only=True):
    return (native_profile_supported(model, param) and model == int(CP.safetyConfigs[-1].safetyModel.raw) and
            param == CP.safetyConfigs[-1].safetyParam)
  return qualified_honda(CP) and param == CP.safetyConfigs[0].safetyParam and param in (34, 35, 163)


def native_latch_rejected(CP, native):
  return qualified_stock(CP, marked_only=True) and native.requestedLateral and not native.lateralAllowed


def ordinary_axis_request_allowed(CP, CS):
  return qualified_stock(CP, marked_only=True) and CS.canValid and not CS.canTimeout and not CS.steerFaultPermanent


def retain_on_native_denial(CP, native, state):
  return temporary_restriction(CP, state)


def create_intent(CP, settings):
  if qualified_stock(CP):
    safety_addition = 256 if CP.carFingerprint == CAR.HONDA_ODYSSEY_TWN else 0
    if CP.flags & HondaFlags.GAS_INTERCEPTOR:
      safety_addition |= int(HondaSafetyFlags.NIDEC_PEDAL_AOL)
    if CP.carFingerprint in CLASSIC_BOSCH and CP.openpilotLongitudinalControl:
      safety_addition |= int(HondaSafetyFlags.CLASSIC_LONG_AOL | HondaSafetyFlags.AOL_BOSCH_LONG)
    if CP.carFingerprint in ALT_RADAR_STOCK:
      if CP.carFingerprint == CAR.ACURA_MDX_4G:
        safety_addition |= int(HondaSafetyFlags.STOCK_TJA)
      if CP.openpilotLongitudinalControl:
        safety_addition |= int(HondaSafetyFlags.ALT_RADAR_AOL | HondaSafetyFlags.AOL_BOSCH_LONG)
    return HondaStockCardIntent(CP, settings)
  from openpilot.starpilot.aol.intent import AolCardIntent
  return AolCardIntent(settings)


def aol_safety_config(CP):
  return CP.safetyConfigs[-1] if qualified_stock(CP, marked_only=True) else None


def classic_bosch_long_cancel_owner(CP) -> bool:
  if qualified_stock(CP, marked_only=True):
    return bool(CP.carFingerprint in CLASSIC_BOSCH and CP.openpilotLongitudinalControl and not CP.pcmCruise and
                len(CP.safetyConfigs) == 1 and
                CP.safetyConfigs[0].safetyModel == car.CarParams.SafetyModel.hondaBosch)
  return qualified_honda(CP) and not qualified_stock(CP)


def long_mask_proves_controls_denied(CP) -> bool:
  """Stock-AOL classic-Bosch alpha long: lateral granted with longitudinal denied means controls_allowed fell.

  The panda grants longitudinal on controls_allowed alone and keeps lateral on the latch token, and it parses the
  brake and CANCEL before the host does. 2026-11-11 route 00000036 t=231.655 (brake) and t=520.857 (CANCEL): that
  mask reached selfdrived 5 ms before the carState with the cause, while the 10 Hz pandaStates still said allowed
  for another 90 ms, and it was read as a lost permission.
  """
  return bool(qualified_stock(CP, marked_only=True) and classic_bosch_long_cancel_owner(CP))


def held_brake_renews_lateral(CP) -> bool:
  """A held brake in lateral-only does not stop a fresh carState renewing the expired lateral output.

  2026-11-11 route 00000038 t=476.297: one 28 ms CAN gap at 3 m/s, cruise off, brake held. The published output
  expired at 30 ms and the fresh carState could not renew it because the held brake counts as a user disable, so
  always-on lateral ended on a critical fault. Route 00000039 t=261.170, a 26 ms gap with no pedal, renewed.
  """
  return bool(qualified_stock(CP, marked_only=True) and classic_bosch_long_cancel_owner(CP))


def cancel_press_ends_cruise_only(CP, CS) -> bool:
  """A fresh CANCEL press on a classic-Bosch alpha-long owner: the panda drops longitudinal on the same CAN frame.

  2026-11-09 route 0000002f t=790.87: the panda status sampled 5 ms after the press still carried the host's
  longitudinal request, card read that as a transport fault and cleared the lateral latch. Presses whose panda
  sample landed after the host withdrew (t=296.48, t=738.43) kept lateral. The brake pedal has the same race and
  already holds lateral intent through it; CANCEL now uses that hold.
  """
  try:
    return bool(classic_bosch_long_cancel_owner(CP) and
                any(event.type == car.CarState.ButtonEvent.Type.cancel and event.pressed for event in CS.buttonEvents))
  except (AttributeError, IndexError, TypeError, ValueError):
    return False


def physical_cancel_lateral_allowed(CP, CS, intent, native, *, settings, previous,
                                    source_ns, now_ns, axis_session_id) -> bool:
  """CANCEL withdraws longitudinal only while a granted classic-Bosch alpha-long LAT axis remains.

  Covers the classic Bosch alpha-long owners (Accord and kin), where CANCEL has no other job than
  ending cruise, on either AOL contract: the marked stock owner (what the Accord runs: safety word
  2083, HondaStockCardIntent) and the older host-latch owner. A CANCEL the driver has mapped to an
  action keeps that action. 2026-11-09 route 0000002d: the first version excluded the stock owner,
  so the buttonCancel NO_ENTRY dropped the axis and card cleared the latch 15 ms after the press.
  """
  from openpilot.starpilot.aol.intent import ButtonType
  from openpilot.starpilot.aol.runtime import (AXIS_MAX_AGE_NS, INTENT_MAX_AGE_NS, SAFETY_MAX_AGE_NS,
                                               native_inventory_matches_cp)
  try:
    return bool(classic_bosch_long_cancel_owner(CP) and settings is not None and settings.enabled and
      not any(settings.cancel_actions) and
      previous is not None and previous.lateral_active and previous.native_acknowledged and
      intent is not None and intent.settingsQualified and intent.allowedLatch and intent.lateralArmed and
      not intent.pauseLateral and intent.producerSessionId and intent.carStateLogMonoTime == source_ns and
      0 < source_ns <= intent.observedMonoTime <= now_ns <= intent.validUntilMonoTime and
      now_ns - source_ns <= AXIS_MAX_AGE_NS and now_ns - intent.observedMonoTime <= INTENT_MAX_AGE_NS and
      intent.validUntilMonoTime - intent.observedMonoTime <= SAFETY_MAX_AGE_NS and
      CS.canValid and not CS.canTimeout and CS.cruiseState.available and not CS.steerFaultTemporary and
      not CS.steerFaultPermanent and not CS.accFaulted and not CS.doorOpen and
      any(event.type == ButtonType.cancel for event in CS.buttonEvents) and
      native is not None and native.compatible and int(native.protocolVersion) == 1 and
      axis_session_id and native.axisSessionId == axis_session_id and native.requestedLateral and native.lateralAllowed and
      native_accepts_cp(CP, native.safetyModel, native.safetyParam) and native.pandaInventory and
      native_inventory_matches_cp(native, CP) and
      0 < native.observedMonoTime <= now_ns <= native.validUntilMonoTime and
      now_ns - native.observedMonoTime <= SAFETY_MAX_AGE_NS and
      native.validUntilMonoTime - native.observedMonoTime <= SAFETY_MAX_AGE_NS)
  except (AttributeError, KeyError, IndexError, TypeError, ValueError, OverflowError):
    return False
