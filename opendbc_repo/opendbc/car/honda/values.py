from dataclasses import dataclass, field
from enum import Enum, IntFlag

from opendbc.car import Bus, CarSpecs, DbcDict, PlatformConfig, Platforms, structs, uds
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.docs_definitions import CarFootnote, CarHarness, CarDocs, CarParts, Column
from opendbc.car.fw_query_definitions import FwQueryConfig, Request, StdQueries, p16
from opendbc.car.lateral import AngleSteeringLimits, MAX_LATERAL_ACCEL, MAX_LATERAL_JERK

Ecu = structs.CarParams.Ecu
VisualAlert = structs.CarControl.HUDControl.VisualAlert
GearShifter = structs.CarState.GearShifter


class CarControllerParams:
  # Allow small margin below -3.5 m/s^2 from ISO 15622:2018 since we
  # perform the closed loop control, and might need some
  # to apply some more braking if we're on a downhill slope.
  # Our controller should still keep the 2 second average above
  # -3.5 m/s^2 as per planner limits
  NIDEC_ACCEL_MIN = -4.0  # m/s^2
  NIDEC_ACCEL_MAX = 1.6  # m/s^2, lower than 2.0 m/s^2 for tuning reasons

  NIDEC_ACCEL_LOOKUP_BP = [-1., 0., .6]
  NIDEC_ACCEL_LOOKUP_V = [-4.8, 0., 2.0]

  NIDEC_MAX_ACCEL_V = [0.5, 2.4, 1.4, 0.6]
  NIDEC_MAX_ACCEL_BP = [0.0, 4.0, 10., 20.]

  NIDEC_GAS_MAX = 198  # 0xc6
  NIDEC_BRAKE_MAX = 1024 // 4

  BOSCH_ACCEL_MIN = -3.5  # m/s^2
  BOSCH_ACCEL_MAX = 2.0  # m/s^2

  BOSCH_GAS_LOOKUP_BP = [-0.2, 2.0]  # 2m/s^2
  BOSCH_GAS_LOOKUP_V = [0, 1600]

  STEER_STEP = 1  # 100 Hz
  STEER_DELTA_UP = 3  # min/max in 0.33s for all Honda
  STEER_DELTA_DOWN = 3
  STEER_GLOBAL_MIN_SPEED = 3 * CV.MPH_TO_MS

  # ---- Angle mode: the Accord on the angle-loop EPS firmware only (V298 A16A, V299 A16B; interface.py decides; see
  # is_accord_eps_angle_loop_fw).  0xE4 STEER_TORQUE then carries an ANGLE SETPOINT, raw = round(-10 * deg) in the
  # frame of carState.steeringAngleDeg, and the EPS closes a position loop on it at 1 kHz.  The panda bounds NOTHING
  # on 0xE4 for Honda except "bytes 0-1 zero while not allowed" (safety/modes/honda.h, honda_tx_hook, "STEER: safety
  # check"), so every limit below is the fork's.
  ANGLE_LIMITS: AngleSteeringLimits = AngleSteeringLimits(
    400,  # deg; the EPS clamps its setpoint at raw +-4096 (409.6 deg)
    ([], []),
    ([], []),
    MAX_LATERAL_ACCEL=MAX_LATERAL_ACCEL,  # 3.589 m/s^2, the opendbc module constant
    MAX_LATERAL_JERK=MAX_LATERAL_JERK,    # 3.589 m/s^3
    MAX_ANGLE_RATE=1.2,  # deg/frame = 120 deg/s; below the servo's rail-limited rate (467-483 deg/s at <= 10 m/s)
  )
  # Error clip |setpoint - measured angle| <= ANGLE_ERROR_MAX(v).  It bounds the P torque the fork can demand
  # (V298: P = DC stiffness x error) without starving a correctly working setpoint: each value is the error the
  # inner loop carries while the setpoint slews at the fork's own maximum rate,
  #   rate(v) x (0.06 s round trip + tau_inner(v)),  tau_inner = b / (k + s)
  # with s = V298's DC stiffness 52/64/33/25/47/96 lane counts per deg (Kp_eff 515/641/332/245/467/957) at the knots
  # below, b and k the V294 plant nominal, rate = min(120 deg/s, the 3.589 m/s^3 jerk limit).  Demanded-P cap
  # s x clip = 886/987/644/417/386/444 lane counts = 36/40/26/17/16/18 % of the 2461 rail.  Hold error beyond the
  # clip is the EPS integrator's job.  Values are sized on a model, not measured on the car.
  ANGLE_ERROR_MAX_BP = [3.1, 8.0, 10.0, 11.75, 17.5, 26.9]  # m/s, the V298 gain-table knots
  ANGLE_ERROR_MAX_V = [17.0, 15.5, 19.5, 17.0, 8.5, 4.5]   # deg
  ANGLE_RAW_MAX = 4000  # raw clip before packing; the packer clips nothing and wraps past +-32767
  # V299: NO fork-side driver override (operator ruling, 2026-10-02).  The setpoint never follows the hand.  The EPS
  # is the override: its fade, and its integrator freeze at Honda's steeringPressed level (raw 1229 = wire 1200).  The
  # error clip above is the only bound between the setpoint and the wheel.
  # Operator params, each read ONCE when the controller is built (CarController.__init__): unknown key, unreadable or
  # non-finite = the default, then clamped to [min, max].  The defaults are V298's values exactly.
  ANGLE_MAX_RATE_PARAM = ("AccordAngleMaxRate", 120.0, 60.0, 250.0)  # deg/s; ANGLE_LIMITS.MAX_ANGLE_RATE = value / 100
  ANGLE_CLIP_SCALE_PARAM = ("AccordAngleClipScale", 1.0, 1.0, 1.6)   # x the ANGLE_ERROR_MAX_V knots at BP <= 11.75 m/s
  ANGLE_CLIP_SCALE_MAX_BP = 11.75
  # Angle status word, starpilotCarState.accordAngleStatus (card.py copies CarController.angle_status, the frame
  # carOutput carries).  Bits 1, 2, 8, 32, 64, 128 and 512+ are reserved (0).
  ANGLE_STATUS_RATE = 4         # latActive and the rate/jerk limit bound the setpoint this frame
  ANGLE_STATUS_CLIP = 16        # latActive and the error clip bound the setpoint this frame
  ANGLE_STATUS_EPS_STALE = 256  # no good 0x1AB frame within EPS_TORQUE_STALE_NS: steeringTorqueEps (the bar) reads 0
  # 0x1AB STEER_MOTOR_TORQUE on the angle-loop firmware = the EPS lane torque: 10-bit sign-magnitude (bit 9 = sign,
  # + = right), LSB 8 lane counts, rail 2461 counts.  carState.steeringTorqueEps = -8 * s10 (+ = left).
  EPS_TORQUE_LSB = 8
  EPS_TORQUE_RAIL = 2461
  EPS_TORQUE_STALE_NS = 100_000_000  # 0x1AB runs at 50 Hz (route 79: gap p50 20.1 ms, p99.9 30.8 ms)
  # 0xE4 byte 2 bits 3:2 (no DBC signal) = the EPS's gp-0x6803.  V298 runs its lane only when it reads 2, on every
  # frame including request-off ones; the stock camera sends 0 or 1 there, never 2.
  ANGLE_ARM = 2

  def __init__(self, CP):
    self.STEER_MAX = CP.lateralParams.torqueBP[-1]
    # mirror of list (assuming first item is zero) for interp of signed request
    # values and verify that both arrays begin at zero
    assert CP.lateralParams.torqueBP[0] == 0
    assert CP.lateralParams.torqueV[0] == 0
    self.STEER_LOOKUP_BP = [v * -1 for v in CP.lateralParams.torqueBP][1:][::-1] + list(CP.lateralParams.torqueBP)
    self.STEER_LOOKUP_V = [v * -1 for v in CP.lateralParams.torqueV][1:][::-1] + list(CP.lateralParams.torqueV)


class HondaSafetyFlags(IntFlag):
  ALT_BRAKE = 1
  BOSCH_LONG = 2
  NIDEC_ALT = 4
  RADARLESS = 8
  BOSCH_CANFD = 16
  GAS_INTERCEPTOR = 32
  # Accord 11G MVL radar/camera handover messages. Keep the generic CAN-FD
  # safety profile unchanged for other Honda CAN-FD platforms (for example CR-V 6G).
  BOSCH_CANFD_MVL = 64


class HondaFlags(IntFlag):
  # Detected flags
  # Bosch models with alternate set of LKAS_HUD messages
  BOSCH_EXT_HUD = 1
  BOSCH_ALT_BRAKE = 2

  # Static flags
  BOSCH = 4
  BOSCH_RADARLESS = 8

  NIDEC = 16
  NIDEC_ALT_PCM_ACCEL = 32
  NIDEC_ALT_SCM_MESSAGES = 64

  BOSCH_CANFD = 128

  HAS_ALL_DOOR_STATES = 256  # Some Hondas have all door states, others only driver door
  BOSCH_ALT_RADAR = 512
  ALLOW_MANUAL_TRANS = 1024
  HYBRID = 2048
  BOSCH_TJA_CONTROL = 4096
  EPS_MODIFIED = 8192
  # Detected: the EPS reports the Accord angle-loop firmware family (is_accord_eps_angle_loop_fw)
  EPS_ANGLE_LOOP_FW = 16384
  # The angle-interface switch is on but no EPS fwVersion is the angle-loop family (carstate: permanent fault)
  EPS_ANGLE_LOOP_FW_MISSING = 32768


# F181 of the Accord EPS angle-loop firmware FAMILY: 39990-TVA,A16 + exactly ONE capital letter (V298 = A16A,
# V299 = A16B).  The comma keeps EPS_MODIFIED set.  The torque images read 39990-TVA,A160 (V294/V295: a digit, not
# a letter) and stock 39990-TVA-A1x0, so neither matches; a torque fork never meets the family unknowingly.
HONDA_ACCORD_EPS_ANGLE_LOOP_FW_FAMILY = b"39990-TVA,A16"


def is_accord_eps_angle_loop_fw(fw_version: bytes) -> bool:
  # the car pads fwVersion with NULs to 16 bytes (route 79 reads 39990-TVA,A16A + two NULs): strip them first
  v = bytes(fw_version).rstrip(b"\x00")
  family = HONDA_ACCORD_EPS_ANGLE_LOOP_FW_FAMILY
  return len(v) == len(family) + 1 and v.startswith(family) and ord("A") <= v[-1] <= ord("Z")


# Angle mode's vehicle delay (lagd adds the 0.2 s software part): the inner angle loop's lag, ~0.16-0.17 s at
# 15-27 m/s on the V298 gains.  A model value; drive 1 learns it (UseAutoSteerDelay on).
HONDA_ACCORD_ANGLE_STEER_ACTUATOR_DELAY = 0.15


# Car button codes
class CruiseButtons:
  RES_ACCEL = 4
  DECEL_SET = 3
  CANCEL = 2
  MAIN = 1


class CruiseSettings:
  DISTANCE = 3
  LKAS = 1


class HondaStarPilotFlags(IntFlag):
  HAS_CAMERA_MESSAGES = 8


@dataclass
class HondaCarDocs(CarDocs):
  package: str = "Honda Sensing"

  def init_make(self, CP: structs.CarParams):
    if CP.flags & HondaFlags.BOSCH:
      if CP.flags & HondaFlags.BOSCH_CANFD:
        harness = CarHarness.bosch_c
      elif CP.flags & HondaFlags.BOSCH_RADARLESS:
        harness = CarHarness.bosch_b
      else:
        harness = CarHarness.bosch_a
    else:
      harness = CarHarness.nidec

    self.car_parts = CarParts.common([harness])

    if CP.alphaLongitudinalAvailable:
      self.footnotes.append(Footnote.EXP_LONG)


class Footnote(Enum):
  CIVIC_DIESEL = CarFootnote(
    "2019 Honda Civic 1.6L Diesel Sedan does not have ALC below 12mph.",
    Column.FSR_STEERING)
  EXP_LONG = CarFootnote(
    "Enabling longitudinal control (alpha) will disable all CMBS functionality, including AEB and FCW.",
    Column.LONGITUDINAL)


@dataclass
class HondaBoschPlatformConfig(PlatformConfig):
  def init(self):
    self.flags |= HondaFlags.BOSCH


@dataclass
class HondaBoschCANFDPlatformConfig(HondaBoschPlatformConfig):
  dbc_dict: DbcDict = field(default_factory=lambda: {Bus.pt: 'honda_common_canfd_generated'})

  def init(self):
    super().init()
    self.flags |= HondaFlags.BOSCH_CANFD


@dataclass
class HondaNidecPlatformConfig(PlatformConfig):
  def init(self):
    self.flags |= HondaFlags.NIDEC


def radar_dbc_dict(pt_dict):
  return {Bus.pt: pt_dict, Bus.radar: 'acura_ilx_2016_nidec'}


# Certain Hondas have an extra steering sensor at the bottom of the steering rack,
# which improves controls quality as it removes the steering column torsion from feedback.
# Tire stiffness factor fictitiously lower if it includes the steering column torsion effect.
# For modeling details, see p.198-200 in "The Science of Vehicle Dynamics (2014), M. Guiggiani"


class CAR(Platforms):
  # Bosch Cars
  HONDA_NBOX_2G = HondaBoschPlatformConfig(
    [
      HondaCarDocs("Honda N-Box 2018", "All", min_steer_speed=5.),
    ],
    CarSpecs(mass=890., wheelbase=2.520, steerRatio=18.64),
    # Bus.radar = the hand-written 16-slot Bosch-A object bank DBC (see radar_interface.py for the decode).
    # RX-parse only; the decode itself takes no CAN authority, so factory AEB/CMBS/FCW stay live as long as
    # stock longitudinal remains in control. Enabling openpilot longitudinal (alpha) disables all CMBS
    # functionality, including AEB and FCW -- see Footnote.EXP_LONG and CarInterface.init()'s UDS
    # CommunicationControl call, which suppresses the stock radar's longitudinal TX for the same reason.
    {Bus.pt: 'acura_rdx_2020_can_generated', Bus.radar: 'honda_bosch_a_radar'},
  )
  HONDA_ACCORD = HondaBoschPlatformConfig(
    [
      HondaCarDocs("Honda Accord 2018-22", "All", video="https://www.youtube.com/watch?v=mrUwlj3Mi58", min_steer_speed=3. * CV.MPH_TO_MS),
      HondaCarDocs("Honda Inspire 2018", "All", min_steer_speed=3. * CV.MPH_TO_MS),
      HondaCarDocs("Honda Accord Hybrid 2018-22", "All", min_steer_speed=3. * CV.MPH_TO_MS),
    ],
    # steerRatio: 11.82 is spec end-to-end
    CarSpecs(mass=3279 * CV.LB_TO_KG, wheelbase=2.83, steerRatio=16.33, centerToFrontRatio=0.39, tireStiffnessFactor=0.8467),
    # Bus.radar = the hand-written 16-slot Bosch-A object bank DBC (see radar_interface.py for the decode).
    # RX-parse only; the decode itself takes no CAN authority, so factory AEB/CMBS/FCW stay live as long as
    # stock longitudinal remains in control. Enabling openpilot longitudinal (alpha) disables all CMBS
    # functionality, including AEB and FCW -- see Footnote.EXP_LONG and CarInterface.init()'s UDS
    # CommunicationControl call, which suppresses the stock radar's longitudinal TX for the same reason.
    {Bus.pt: 'honda_civic_hatchback_ex_2017_can_generated', Bus.radar: 'honda_bosch_a_radar'},
    flags=HondaFlags.ALLOW_MANUAL_TRANS,
  )
  HONDA_ACCORD_11G = HondaBoschCANFDPlatformConfig(
    [
      HondaCarDocs("Honda Accord 2023-25", "All"),
      HondaCarDocs("Honda Accord Hybrid 2023-25", "All"),
    ],
    CarSpecs(mass=3477 * CV.LB_TO_KG, wheelbase=2.83, steerRatio=16.7, centerToFrontRatio=0.39),
    {Bus.pt: 'honda_common_canfd_generated', Bus.radar: 'honda_common_canfd_generated'},
  )
  HONDA_CIVIC_BOSCH = HondaBoschPlatformConfig(
    [
      HondaCarDocs("Honda Civic 2019-21", "All", video="https://www.youtube.com/watch?v=4Iz1Mz5LGF8",
                   footnotes=[Footnote.CIVIC_DIESEL], min_steer_speed=2. * CV.MPH_TO_MS),
      HondaCarDocs("Honda Civic Hatchback 2017-18", min_steer_speed=12. * CV.MPH_TO_MS),
      HondaCarDocs("Honda Civic Hatchback 2019-21", "All", min_steer_speed=12. * CV.MPH_TO_MS),
    ],
    CarSpecs(mass=1326, wheelbase=2.7, steerRatio=15.38, centerToFrontRatio=0.4),  # steerRatio: 10.93 is end-to-end spec
    # Bus.radar = the hand-written 16-slot Bosch-A object bank DBC (see radar_interface.py for the decode).
    # RX-parse only; the decode itself takes no CAN authority, so factory AEB/CMBS/FCW stay live as long as
    # stock longitudinal remains in control. Enabling openpilot longitudinal (alpha) disables all CMBS
    # functionality, including AEB and FCW -- see Footnote.EXP_LONG and CarInterface.init()'s UDS
    # CommunicationControl call, which suppresses the stock radar's longitudinal TX for the same reason.
    {Bus.pt: 'honda_civic_hatchback_ex_2017_can_generated', Bus.radar: 'honda_bosch_a_radar'},
  )
  HONDA_CIVIC_BOSCH_DIESEL = HondaBoschPlatformConfig(
    [],  # don't show in docs
    HONDA_CIVIC_BOSCH.specs,
    # Bus.radar = the hand-written 16-slot Bosch-A object bank DBC (see radar_interface.py for the decode).
    # RX-parse only; the decode itself takes no CAN authority, so factory AEB/CMBS/FCW stay live as long as
    # stock longitudinal remains in control. Enabling openpilot longitudinal (alpha) disables all CMBS
    # functionality, including AEB and FCW -- see Footnote.EXP_LONG and CarInterface.init()'s UDS
    # CommunicationControl call, which suppresses the stock radar's longitudinal TX for the same reason.
    {Bus.pt: 'honda_civic_hatchback_ex_2017_can_generated', Bus.radar: 'honda_bosch_a_radar'},
  )
  HONDA_CIVIC_2022 = HondaBoschPlatformConfig(
    [
      HondaCarDocs("Honda Civic 2022-24", "All", video="https://youtu.be/ytiOT5lcp6Q"),
      HondaCarDocs("Honda Civic Hybrid 2025-26", "All"),
      HondaCarDocs("Honda Civic Hatchback 2022-24", "All", video="https://youtu.be/ytiOT5lcp6Q"),
      HondaCarDocs("Honda Civic Hatchback Hybrid (Europe only) 2023", "All"),
      # TODO: Confirm 2024
      HondaCarDocs("Honda Civic Hatchback Hybrid 2025-26", "All"),
    ],
    HONDA_CIVIC_BOSCH.specs,
    {Bus.pt: 'honda_bosch_radarless_generated'},
    flags=HondaFlags.BOSCH_RADARLESS | HondaFlags.ALLOW_MANUAL_TRANS
  )
  HONDA_CRV_5G = HondaBoschPlatformConfig(
    [HondaCarDocs("Honda CR-V 2017-22", min_steer_speed=15. * CV.MPH_TO_MS)],
    # steerRatio: 12.3 is spec end-to-end
    CarSpecs(mass=3410 * CV.LB_TO_KG, wheelbase=2.66, steerRatio=16.0, centerToFrontRatio=0.41, tireStiffnessFactor=0.677),
    # Bus.radar = the hand-written 16-slot Bosch-A object bank DBC (see radar_interface.py for the decode).
    # RX-parse only; the decode itself takes no CAN authority, so factory AEB/CMBS/FCW stay live as long as
    # stock longitudinal remains in control. Enabling openpilot longitudinal (alpha) disables all CMBS
    # functionality, including AEB and FCW -- see Footnote.EXP_LONG and CarInterface.init()'s UDS
    # CommunicationControl call, which suppresses the stock radar's longitudinal TX for the same reason.
    {Bus.pt: 'honda_civic_hatchback_ex_2017_can_generated', Bus.body: 'honda_crv_ex_2017_body_generated', Bus.radar: 'honda_bosch_a_radar'},
    flags=HondaFlags.BOSCH_ALT_BRAKE,
  )
  HONDA_CRV_6G = HondaBoschCANFDPlatformConfig(
    [
      HondaCarDocs("Honda CR-V 2023-26", "All"),
      HondaCarDocs("Honda CR-V Hybrid 2023-25", "All"),
    ],
    CarSpecs(mass=1703, wheelbase=2.7, steerRatio=16.2, centerToFrontRatio=0.42),
  )
  HONDA_CRV_HYBRID = HondaBoschPlatformConfig(
    [HondaCarDocs("Honda CR-V Hybrid 2017-22", min_steer_speed=12. * CV.MPH_TO_MS)],
    # mass: mean of 4 models in kg, steerRatio: 12.3 is spec end-to-end
    CarSpecs(mass=1667, wheelbase=2.66, steerRatio=16, centerToFrontRatio=0.41, tireStiffnessFactor=0.677),
    # Bus.radar = the hand-written 16-slot Bosch-A object bank DBC (see radar_interface.py for the decode).
    # RX-parse only; the decode itself takes no CAN authority, so factory AEB/CMBS/FCW stay live as long as
    # stock longitudinal remains in control. Enabling openpilot longitudinal (alpha) disables all CMBS
    # functionality, including AEB and FCW -- see Footnote.EXP_LONG and CarInterface.init()'s UDS
    # CommunicationControl call, which suppresses the stock radar's longitudinal TX for the same reason.
    {Bus.pt: 'honda_civic_hatchback_ex_2017_can_generated', Bus.radar: 'honda_bosch_a_radar'},
  )
  HONDA_HRV_3G = HondaBoschPlatformConfig(
    [HondaCarDocs("Honda HR-V 2023-25", "All")],
    CarSpecs(mass=3125 * CV.LB_TO_KG, wheelbase=2.61, steerRatio=15.2, centerToFrontRatio=0.41, tireStiffnessFactor=0.5),
    {Bus.pt: 'honda_bosch_radarless_generated'},
    flags=HondaFlags.BOSCH_RADARLESS,
  )
  HONDA_CITY_7G = HondaBoschPlatformConfig(
    [HondaCarDocs("Honda City (Brazil only) 2023", "All")],
    CarSpecs(mass=3125 * CV.LB_TO_KG, wheelbase=2.6, steerRatio=19.0, centerToFrontRatio=0.41, minSteerSpeed=23. * CV.KPH_TO_MS),
    {Bus.pt: 'honda_bosch_radarless_generated'},
    flags=HondaFlags.BOSCH_RADARLESS,
  )
  ACURA_RDX_3G = HondaBoschPlatformConfig(
    [HondaCarDocs("Acura RDX 2019-21", "All", min_steer_speed=3. * CV.MPH_TO_MS)],
    CarSpecs(mass=4068 * CV.LB_TO_KG, wheelbase=2.75, steerRatio=11.95, centerToFrontRatio=0.41, tireStiffnessFactor=0.677),  # as spec
    # Bus.radar = the hand-written 16-slot Bosch-A object bank DBC (see radar_interface.py for the decode).
    # RX-parse only; the decode itself takes no CAN authority, so factory AEB/CMBS/FCW stay live as long as
    # stock longitudinal remains in control. Enabling openpilot longitudinal (alpha) disables all CMBS
    # functionality, including AEB and FCW -- see Footnote.EXP_LONG and CarInterface.init()'s UDS
    # CommunicationControl call, which suppresses the stock radar's longitudinal TX for the same reason.
    {Bus.pt: 'acura_rdx_2020_can_generated', Bus.radar: 'honda_bosch_a_radar'},
    flags=HondaFlags.BOSCH_ALT_BRAKE,
  )
  ACURA_RDX_3G_MMR = HondaBoschPlatformConfig(
    [HondaCarDocs("Acura RDX 2022-26", "All", min_steer_speed=70. * CV.KPH_TO_MS)],
    CarSpecs(mass=4079 * CV.LB_TO_KG, wheelbase=2.75, centerToFrontRatio=0.41, steerRatio=16.2),
    {Bus.pt: 'acura_rdx_2020_can_generated'},
    flags=HondaFlags.BOSCH_ALT_BRAKE | HondaFlags.BOSCH_ALT_RADAR,
  )
  HONDA_INSIGHT = HondaBoschPlatformConfig(
    [HondaCarDocs("Honda Insight 2019-22", "All", min_steer_speed=3. * CV.MPH_TO_MS)],
    CarSpecs(mass=2987 * CV.LB_TO_KG, wheelbase=2.7, steerRatio=15.0, centerToFrontRatio=0.39, tireStiffnessFactor=0.82),  # as spec
    # Bus.radar = the hand-written 16-slot Bosch-A object bank DBC (see radar_interface.py for the decode).
    # RX-parse only; the decode itself takes no CAN authority, so factory AEB/CMBS/FCW stay live as long as
    # stock longitudinal remains in control. Enabling openpilot longitudinal (alpha) disables all CMBS
    # functionality, including AEB and FCW -- see Footnote.EXP_LONG and CarInterface.init()'s UDS
    # CommunicationControl call, which suppresses the stock radar's longitudinal TX for the same reason.
    {Bus.pt: 'honda_insight_ex_2019_can_generated', Bus.radar: 'honda_bosch_a_radar'},
  )
  HONDA_E = HondaBoschPlatformConfig(
    [HondaCarDocs("Honda e 2020", "All", min_steer_speed=3. * CV.MPH_TO_MS)],
    CarSpecs(mass=3338.8 * CV.LB_TO_KG, wheelbase=2.5, centerToFrontRatio=0.5, steerRatio=16.71, tireStiffnessFactor=0.82),
    # Bus.radar = the hand-written 16-slot Bosch-A object bank DBC (see radar_interface.py for the decode).
    # RX-parse only; the decode itself takes no CAN authority, so factory AEB/CMBS/FCW stay live as long as
    # stock longitudinal remains in control. Enabling openpilot longitudinal (alpha) disables all CMBS
    # functionality, including AEB and FCW -- see Footnote.EXP_LONG and CarInterface.init()'s UDS
    # CommunicationControl call, which suppresses the stock radar's longitudinal TX for the same reason.
    {Bus.pt: 'acura_rdx_2020_can_generated', Bus.radar: 'honda_bosch_a_radar'},
  )
  HONDA_E_ADVANCE = HondaBoschPlatformConfig(
    [],  # don't show in docs, base trim already in docs
    CarSpecs(mass=1527, wheelbase=2.5, centerToFrontRatio=0.5, steerRatio=16.71, tireStiffnessFactor=0.82),
    # Bus.radar = the hand-written 16-slot Bosch-A object bank DBC (see radar_interface.py for the decode).
    # RX-parse only; the decode itself takes no CAN authority, so factory AEB/CMBS/FCW stay live as long as
    # stock longitudinal remains in control. Enabling openpilot longitudinal (alpha) disables all CMBS
    # functionality, including AEB and FCW -- see Footnote.EXP_LONG and CarInterface.init()'s UDS
    # CommunicationControl call, which suppresses the stock radar's longitudinal TX for the same reason.
    {Bus.pt: 'honda_e_advance_2020_can_generated', Bus.radar: 'honda_bosch_a_radar'},
  )
  HONDA_PILOT_4G = HondaBoschCANFDPlatformConfig(
    [HondaCarDocs("Honda Pilot 2023-25", "All")],
    CarSpecs(mass=4660 * CV.LB_TO_KG, wheelbase=2.89, centerToFrontRatio=0.442, steerRatio=17.5),
  )
  HONDA_PASSPORT_4G = HondaBoschCANFDPlatformConfig(
    [HondaCarDocs("Honda Passport 2026", "All")],
    CarSpecs(mass=4620 * CV.LB_TO_KG, wheelbase=2.89, centerToFrontRatio=0.442, steerRatio=18.5),
  )
  ACURA_MDX_4G = HondaBoschPlatformConfig(
    [HondaCarDocs("Acura MDX 2022-24", "All", min_steer_speed=70. * CV.KPH_TO_MS)],
    CarSpecs(mass=4788 * CV.LB_TO_KG, wheelbase=2.89, steerRatio=15.8, centerToFrontRatio=0.428),
    {Bus.pt: 'honda_common_canfd_generated'},
    flags=HondaFlags.BOSCH_ALT_RADAR | HondaFlags.BOSCH_TJA_CONTROL,
  )
  # mid-model refresh
  ACURA_MDX_4G_MMR = HondaBoschCANFDPlatformConfig(
    [HondaCarDocs("Acura MDX 2025", "All except Type S")],
    CarSpecs(mass=4544 * CV.LB_TO_KG, wheelbase=2.89, centerToFrontRatio=0.428, steerRatio=16.2),
  )
  HONDA_ODYSSEY_5G_MMR = HondaBoschPlatformConfig(
    [HondaCarDocs("Honda Odyssey 2021-26", "All", min_steer_speed=70. * CV.KPH_TO_MS)],
    CarSpecs(mass=4590 * CV.LB_TO_KG, wheelbase=3.00, steerRatio=19.4, centerToFrontRatio=0.41),
    {Bus.pt: 'acura_rdx_2020_can_generated'},
    flags=HondaFlags.BOSCH_ALT_BRAKE | HondaFlags.BOSCH_ALT_RADAR,
  )
  ACURA_TLX_2G = HondaBoschPlatformConfig(
    [HondaCarDocs("Acura TLX 2021", "All")],
    CarSpecs(mass=3982 * CV.LB_TO_KG, wheelbase=2.87, steerRatio=14.0, centerToFrontRatio=0.43),
    {Bus.pt: 'honda_civic_hatchback_ex_2017_can_generated'},
    flags=HondaFlags.BOSCH_ALT_RADAR,
  )
  ACURA_TLX_2G_MMR = HondaBoschCANFDPlatformConfig(
    [HondaCarDocs("Acura TLX 2024-25", "All")],
    CarSpecs(mass=3990 * CV.LB_TO_KG, wheelbase=2.87, centerToFrontRatio=0.43, steerRatio=13.7),
  )
  HONDA_FIT_4G = HondaBoschPlatformConfig(
    [
      HondaCarDocs("Honda Fit (Taiwan) 2021", "All"),
      HondaCarDocs("Honda Fit (Taiwan) 2024-25", "All"),
    ],
    CarSpecs(mass=1229, wheelbase=2.53, steerRatio=19.7, centerToFrontRatio=0.39, minSteerSpeed=23. * CV.KPH_TO_MS),
    {Bus.pt: 'honda_bosch_radarless_generated'},
    flags=HondaFlags.BOSCH_RADARLESS,
  )
  ACURA_INTEGRA = HondaBoschPlatformConfig(
    [
      HondaCarDocs("Acura Integra 2023-26", "All"),
      HondaCarDocs("Honda Prelude 2026", "All"),
    ],
    CarSpecs(mass=3338.8 * CV.LB_TO_KG, wheelbase=2.5, centerToFrontRatio=0.5, steerRatio=16.71, tireStiffnessFactor=0.82),
    {Bus.pt: 'honda_bosch_radarless_generated'},
    flags=HondaFlags.BOSCH_RADARLESS,
  )
  ACURA_ADX = HondaBoschPlatformConfig(
    [HondaCarDocs("Acura ADX 2025-26", "All")],
    CarSpecs(mass=3578 * CV.LB_TO_KG, wheelbase=2.65, steerRatio=16.6, centerToFrontRatio=0.43),
    {Bus.pt: 'honda_bosch_radarless_generated'},
    flags=HondaFlags.BOSCH_RADARLESS,
  )

  # Nidec Cars
  ACURA_ILX = HondaNidecPlatformConfig(
    [
      HondaCarDocs("Acura ILX 2016-18", "Technology Plus Package or AcuraWatch Plus", min_steer_speed=25. * CV.MPH_TO_MS),
      HondaCarDocs("Acura ILX 2019", "All", min_steer_speed=25. * CV.MPH_TO_MS),
    ],
    CarSpecs(mass=3095 * CV.LB_TO_KG, wheelbase=2.67, steerRatio=18.61, centerToFrontRatio=0.37, tireStiffnessFactor=0.72),  # 15.3 is spec end-to-end
    radar_dbc_dict('acura_ilx_2016_can_generated'),
    flags=HondaFlags.NIDEC_ALT_SCM_MESSAGES | HondaFlags.HAS_ALL_DOOR_STATES,
  )
  HONDA_CRV = HondaNidecPlatformConfig(
    [HondaCarDocs("Honda CR-V 2015-16", "Touring Trim", min_steer_speed=12. * CV.MPH_TO_MS)],
    CarSpecs(mass=3572 * CV.LB_TO_KG, wheelbase=2.62, steerRatio=16.89, centerToFrontRatio=0.41, tireStiffnessFactor=0.444),  # as spec
    radar_dbc_dict('honda_crv_touring_2016_can_generated'),
    flags=HondaFlags.NIDEC_ALT_SCM_MESSAGES | HondaFlags.HAS_ALL_DOOR_STATES,
  )
  HONDA_CRV_EU = HondaNidecPlatformConfig(
    [],  # Euro version of CRV Touring, don't show in docs
    HONDA_CRV.specs,
    radar_dbc_dict('honda_crv_touring_2016_can_generated'),
    flags=HondaFlags.NIDEC_ALT_SCM_MESSAGES | HondaFlags.HAS_ALL_DOOR_STATES,
  )
  HONDA_CRV_SA = HondaNidecPlatformConfig(
    [],  # South Africa version of CRV Touring, don't show in docs
    HONDA_CRV.specs,
    radar_dbc_dict('acura_rdx_2018_can_generated'),
    flags=HondaFlags.NIDEC_ALT_SCM_MESSAGES | HondaFlags.HAS_ALL_DOOR_STATES,
  )
  HONDA_FIT = HondaNidecPlatformConfig(
    [HondaCarDocs("Honda Fit 2018-20", min_steer_speed=12. * CV.MPH_TO_MS)],
    CarSpecs(mass=2644 * CV.LB_TO_KG, wheelbase=2.53, steerRatio=13.06, centerToFrontRatio=0.39, tireStiffnessFactor=0.75),
    radar_dbc_dict('acura_ilx_2016_can_generated'),
    flags=HondaFlags.NIDEC_ALT_SCM_MESSAGES,
  )
  HONDA_FREED = HondaNidecPlatformConfig(
    [HondaCarDocs("Honda Freed 2020", min_steer_speed=12. * CV.MPH_TO_MS)],
    CarSpecs(mass=3086. * CV.LB_TO_KG, wheelbase=2.74, steerRatio=13.06, centerToFrontRatio=0.39, tireStiffnessFactor=0.75),  # mostly copied from FIT
    radar_dbc_dict('acura_ilx_2016_can_generated'),
    flags=HondaFlags.NIDEC_ALT_SCM_MESSAGES,
  )
  HONDA_HRV = HondaNidecPlatformConfig(
    [HondaCarDocs("Honda HR-V 2019-22", min_steer_speed=12. * CV.MPH_TO_MS)],
    HONDA_HRV_3G.specs,
    radar_dbc_dict('acura_ilx_2016_can_generated'),
    flags=HondaFlags.NIDEC_ALT_SCM_MESSAGES,
  )
  HONDA_CLARITY = HondaNidecPlatformConfig(
    [HondaCarDocs("Honda Clarity 2018-21", "All", min_steer_speed=3. * CV.MPH_TO_MS)],
    CarSpecs(mass=1838, wheelbase=2.75, centerToFrontRatio=0.4, steerRatio=16.5),
    radar_dbc_dict('honda_clarity_hybrid_2018_can_generated'),
    flags=HondaFlags.HAS_ALL_DOOR_STATES,
  )
  HONDA_ODYSSEY = HondaNidecPlatformConfig(
    [HondaCarDocs("Honda Odyssey 2018-20")],
    CarSpecs(mass=1900, wheelbase=3.0, steerRatio=14.35, centerToFrontRatio=0.41, tireStiffnessFactor=0.82),
    radar_dbc_dict('honda_odyssey_exl_2018_generated'),
    flags=HondaFlags.NIDEC_ALT_PCM_ACCEL | HondaFlags.HAS_ALL_DOOR_STATES,
  )
  HONDA_ODYSSEY_TWN = HondaNidecPlatformConfig(
    [
      HondaCarDocs("Honda Odyssey (Taiwan) 2018-19"),
      HondaCarDocs("Honda Odyssey (Singapore) 2021"),
    ],
    CarSpecs(mass=1865, wheelbase=2.9, steerRatio=14.35, centerToFrontRatio=0.44, tireStiffnessFactor=0.82),
    radar_dbc_dict('honda_odyssey_twn_2018_generated'),
    flags=HondaFlags.NIDEC_ALT_SCM_MESSAGES,
  )
  ACURA_RDX = HondaNidecPlatformConfig(
    [HondaCarDocs("Acura RDX 2016-18", "AcuraWatch Plus or Advance Package", min_steer_speed=12. * CV.MPH_TO_MS)],
    CarSpecs(mass=3925 * CV.LB_TO_KG, wheelbase=2.68, steerRatio=15.0, centerToFrontRatio=0.38, tireStiffnessFactor=0.444),  # as spec
    radar_dbc_dict('acura_rdx_2018_can_generated'),
    flags=HondaFlags.NIDEC_ALT_SCM_MESSAGES | HondaFlags.HAS_ALL_DOOR_STATES,
  )
  HONDA_PILOT = HondaNidecPlatformConfig(
    [
      HondaCarDocs("Honda Pilot 2016-22", min_steer_speed=12. * CV.MPH_TO_MS),
      HondaCarDocs("Honda Passport 2019-25", "All", min_steer_speed=12. * CV.MPH_TO_MS),
    ],
    HONDA_PILOT_4G.specs,
    radar_dbc_dict('acura_ilx_2016_can_generated'),
    flags=HondaFlags.NIDEC_ALT_SCM_MESSAGES | HondaFlags.HAS_ALL_DOOR_STATES,
  )
  HONDA_RIDGELINE = HondaNidecPlatformConfig(
    [HondaCarDocs("Honda Ridgeline 2017-25", min_steer_speed=12. * CV.MPH_TO_MS)],
    CarSpecs(mass=4515 * CV.LB_TO_KG, wheelbase=3.18, centerToFrontRatio=0.41, steerRatio=15.59, tireStiffnessFactor=0.444),  # as spec
    radar_dbc_dict('acura_ilx_2016_can_generated'),
    flags=HondaFlags.NIDEC_ALT_SCM_MESSAGES | HondaFlags.HAS_ALL_DOOR_STATES,
  )
  HONDA_CIVIC = HondaNidecPlatformConfig(
    [HondaCarDocs("Honda Civic 2016-18", min_steer_speed=12. * CV.MPH_TO_MS, video="https://youtu.be/-IkImTe1NYE")],
    CarSpecs(mass=1326, wheelbase=2.70, centerToFrontRatio=0.4, steerRatio=15.38),  # 10.93 is end-to-end spec
    radar_dbc_dict('honda_civic_touring_2016_can_generated'),
    flags=HondaFlags.HAS_ALL_DOOR_STATES
  )
  HONDA_ACCORD_9G = HondaNidecPlatformConfig(
    [
      HondaCarDocs("Honda Accord 2016-17"),
      HondaCarDocs("Honda Accord Hybrid 2017", "All"),
    ],
    CarSpecs(mass=3343 * CV.LB_TO_KG, wheelbase=2.78, steerRatio=17.5, centerToFrontRatio=0.37),
    radar_dbc_dict('honda_accord_2017_can_ext_generated'),
    flags=HondaFlags.NIDEC_ALT_SCM_MESSAGES | HondaFlags.HAS_ALL_DOOR_STATES,
  )
  ACURA_MDX_3G = HondaNidecPlatformConfig(
    [
      HondaCarDocs("Acura MDX 2014-16", "Advance Package"),
      HondaCarDocs("Acura MDX 2017-19", "All"),
      HondaCarDocs("Acura MDX Hybrid 2017-19", "All"),
    ],
    CarSpecs(mass=4215 * CV.LB_TO_KG, wheelbase=2.82, steerRatio=16.8, centerToFrontRatio=0.428),
    radar_dbc_dict('acura_mdx_2017_can_ext_generated'),
    flags=HondaFlags.NIDEC_ALT_SCM_MESSAGES,
  )
  ACURA_MDX_3G_MMR = HondaNidecPlatformConfig(
    [
      HondaCarDocs("Acura MDX 2020", "All"),
      HondaCarDocs("Acura MDX Hybrid 2020", "All"),
    ],
    CarSpecs(mass=4215 * CV.LB_TO_KG, wheelbase=2.82, steerRatio=16.8, centerToFrontRatio=0.428),
    radar_dbc_dict('acura_ilx_2016_can_generated'),
    flags=HondaFlags.NIDEC_ALT_SCM_MESSAGES,
  )
  ACURA_TLX_1G = HondaNidecPlatformConfig(
    [
      HondaCarDocs("Acura TLX 2015-17", "Advance Package"),
      HondaCarDocs("Acura TLX 2018-20", "All"),
    ],
    CarSpecs(mass=3680 * CV.LB_TO_KG, wheelbase=2.78, steerRatio=17.0, centerToFrontRatio=0.40, tireStiffnessFactor=0.18),
    radar_dbc_dict('acura_mdx_2017_can_ext_generated'),
    flags=HondaFlags.NIDEC_ALT_SCM_MESSAGES | HondaFlags.HAS_ALL_DOOR_STATES,
  )


HONDA_NIDEC_ALT_PCM_ACCEL = CAR.with_flags(HondaFlags.NIDEC_ALT_PCM_ACCEL)
HONDA_NIDEC_ALT_SCM_MESSAGES = CAR.with_flags(HondaFlags.NIDEC_ALT_SCM_MESSAGES)
HONDA_BOSCH = CAR.with_flags(HondaFlags.BOSCH)
HONDA_BOSCH_RADARLESS = CAR.with_flags(HondaFlags.BOSCH_RADARLESS)
HONDA_BOSCH_CANFD = CAR.with_flags(HondaFlags.BOSCH_CANFD)
HONDA_BOSCH_ALT_RADAR = CAR.with_flags(HondaFlags.BOSCH_ALT_RADAR)
# Plain bosch_a harness: not CANFD, not radarless, not alt-radar. These platforms have a firmware-correct
# RadarInterface (16-slot Bosch-A object bank, RX-only, see radar_interface.py) available behind
# HondaBoschARadar. This describes hardware compatibility only; it is deliberately separate from the
# verified set below so a newly supported model cannot start using unvalidated radar data by accident.
HONDA_BOSCH_A = HONDA_BOSCH - HONDA_BOSCH_RADARLESS - HONDA_BOSCH_CANFD - HONDA_BOSCH_ALT_RADAR
HONDA_BOSCH_A_RADAR_VERIFIED = frozenset({CAR.HONDA_CIVIC_BOSCH, CAR.HONDA_CRV_5G})
HONDA_BOSCH_TJA_CONTROL = CAR.with_flags(HondaFlags.BOSCH_TJA_CONTROL)
HONDA_CAMERA_MESSAGE_CARS = {
  CAR.HONDA_ACCORD,
  CAR.HONDA_CIVIC_BOSCH,
  CAR.HONDA_CIVIC_2022,
  CAR.HONDA_CRV_5G,
  CAR.HONDA_CRV_HYBRID,
  CAR.HONDA_HRV_3G,
  CAR.HONDA_INSIGHT,
}


DBC = CAR.create_dbc_map()


STEER_THRESHOLD = {
  # default is 1200, overrides go here
  CAR.ACURA_RDX: 400,
  CAR.HONDA_CRV_EU: 400,
  CAR.HONDA_ACCORD_11G: 600,
  CAR.HONDA_PILOT_4G: 600,
  CAR.HONDA_PASSPORT_4G: 600,
  CAR.ACURA_MDX_4G_MMR: 600,
  CAR.HONDA_CRV_6G: 600,
  CAR.HONDA_CITY_7G: 600,
  CAR.HONDA_NBOX_2G: 600,
  CAR.HONDA_ODYSSEY_5G_MMR: 600,
  CAR.HONDA_ACCORD_9G: 30,
  CAR.ACURA_MDX_3G: 30,
  CAR.ACURA_MDX_3G_MMR: 30,
  CAR.ACURA_TLX_1G: 30,
}


HONDA_ALT_VERSION_REQUEST = bytes([uds.SERVICE_TYPE.READ_DATA_BY_IDENTIFIER]) + \
  p16(0xF112)
HONDA_ALT_VERSION_RESPONSE = bytes([uds.SERVICE_TYPE.READ_DATA_BY_IDENTIFIER + 0x40]) + \
  p16(0xF112)


FW_QUERY_CONFIG = FwQueryConfig(
  requests=[
    # Currently used to fingerprint
    Request(
      [StdQueries.UDS_VERSION_REQUEST],
      [StdQueries.UDS_VERSION_RESPONSE],
      bus=1,
    ),

    # Data collection requests:
    # Log manufacturer-specific identifier for current ECUs
    Request(
      [HONDA_ALT_VERSION_REQUEST],
      [HONDA_ALT_VERSION_RESPONSE],
      bus=1,
      logging=True,
    ),
    # Nidec PT bus
    Request(
      [StdQueries.UDS_VERSION_REQUEST],
      [StdQueries.UDS_VERSION_RESPONSE],
      bus=0,
    ),
    # Bosch PT bus
    Request(
      [StdQueries.UDS_VERSION_REQUEST],
      [StdQueries.UDS_VERSION_RESPONSE],
      bus=1,
      obd_multiplexing=False,
    ),
  ],
  # We lose these ECUs without the comma power on these cars.
  # Note that we still attempt to match with them when they are present
  # This is or'd with (ALL_ECUS - ESSENTIAL_ECUS) from fw_versions.py
  non_essential_ecus={
    Ecu.eps: [CAR.ACURA_RDX_3G, CAR.HONDA_ACCORD, CAR.HONDA_E, CAR.HONDA_E_ADVANCE, CAR.ACURA_MDX_4G, CAR.HONDA_CRV_SA,
              CAR.ACURA_MDX_3G, CAR.HONDA_ACCORD_9G, *HONDA_BOSCH_ALT_RADAR, *HONDA_BOSCH_RADARLESS, *HONDA_BOSCH_CANFD],
    Ecu.vsa: [CAR.ACURA_RDX_3G, CAR.HONDA_ACCORD, CAR.HONDA_CIVIC, CAR.HONDA_CIVIC_BOSCH, CAR.HONDA_CRV_5G, CAR.HONDA_CRV_HYBRID,
              CAR.HONDA_E, CAR.HONDA_E_ADVANCE, CAR.HONDA_INSIGHT, CAR.HONDA_NBOX_2G, CAR.ACURA_MDX_4G, CAR.HONDA_ACCORD_9G,
              *HONDA_BOSCH_ALT_RADAR, *HONDA_BOSCH_RADARLESS, *HONDA_BOSCH_CANFD],
  },
  extra_ecus=[
    (Ecu.combinationMeter, 0x18da60f1, None),
    (Ecu.programmedFuelInjection, 0x18da10f1, None),
    # The only other ECU on PT bus accessible by camera on radarless Civic
    # This is likely a manufacturer-specific sub-address implementation: the camera responds to this and 0x18dab0f1
    # Unclear what the part number refers to: 8S103 is 'Camera Set Mono', while 36160 is 'Camera Monocular - Honda'
    # TODO: add query back, camera does not support querying both in parallel and 0x18dab0f1 often fails to respond
    # (Ecu.unknown, 0x18DAB3F1, None),
  ],
)
