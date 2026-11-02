"""Publish a simhil Scenario as the cereal services and VisionIPC road camera the UI reads.

Live clock: every message carries the host CLOCK_MONOTONIC logMonoTime from
messaging.new_message, and camera frames carry the same stamp, so the UI runs in its
normal (non-replay) mode without a replay display clock.
"""
from __future__ import annotations

import argparse
from dataclasses import dataclass
import threading
import time

from openpilot.cereal import log, messaging
from opendbc.car.structs import car
from openpilot.cereal.visionipc import VisionStreamType
from msgq.visionipc import VisionIpcServer
from openpilot.common.realtime import Ratekeeper
from openpilot.tools.simhil.camera import DEVICE_SENSORS, RoadCameraRenderer, device_camera
from openpilot.tools.simhil.scenario import LEAD_T_IDXS, SCENARIOS, T_IDXS, ScenarioFrame, Simulation

# service -> publish rate in Hz (cereal/services.py rates; carParams also once at start)
RATES: dict[str, float] = {
  "carState": 100., "controlsState": 100., "selfdriveState": 100., "carControl": 100., "carOutput": 100.,
  "starpilotSelfdriveState": 100., "starpilotLateralState": 100.,
  "modelV2": 20., "radarState": 20., "longitudinalPlan": 20., "narrowRoadCameraState": 20.,
  "driverMonitoringState": 20., "driverStateV2": 20.,
  "pandaStates": 10., "starpilotCarState": 10.,
  "extrinsicsCalibration": 4., "lateralDelay": 4.,
  "deviceState": 2., "peripheralState": 2., "managerState": 2.,
  "onroadEvents": 1., "carParams": 0.02,
}
BASE_RATE = 100.
PANDA_TYPES = {"mici": "cuatro", "tizi": "tres", "tici": "tres"}
MANAGED = ("controlsd", "selfdrived", "card", "modeld", "plannerd", "radard", "camerad", "ui")
LANE_CHANGE_STATES = {name: getattr(log.LaneChangeState, name) for name in log.LaneChangeState.schema.enumerants}


def car_params() -> car.CarParams:
  cp = car.CarParams.new_message()
  cp.brand = "mock"
  cp.carFingerprint = "MOCK"
  cp.openpilotLongitudinalControl = True
  cp.steerControlType = car.CarParams.SteerControlType.torque
  cp.wheelbase = 2.7
  cp.steerRatio = 15.0
  cp.mass = 1500.0
  cp.notCar = False
  return cp


def _xyzt(builder, xyz, t) -> None:
  builder.x, builder.y, builder.z = (xyz[0].tolist(), xyz[1].tolist(), xyz[2].tolist())
  builder.t = t


def build_model(frame: ScenarioFrame, frame_id: int, eof_ns: int):
  msg = messaging.new_message("modelV2", valid=True)
  m = msg.modelV2
  m.frameId = m.frameIdExtra = frame_id
  m.timestampEof = eof_ns
  m.modelExecutionTime = 0.01
  t = T_IDXS.tolist()
  _xyzt(m.position, frame.position, t)
  _xyzt(m.velocity, frame.velocity, t)
  _xyzt(m.acceleration, frame.acceleration, t)
  _xyzt(m.orientation, frame.orientation, t)
  _xyzt(m.orientationRate, frame.orientation_rate, t)
  lane_lines = m.init("laneLines", 4)
  for builder, line in zip(lane_lines, frame.lane_lines, strict=True):
    _xyzt(builder, line, t)
  m.laneLineProbs = frame.lane_line_probs.tolist()
  m.laneLineStds = [0.1, 0.05, 0.05, 0.1]
  edges = m.init("roadEdges", 2)
  for builder, edge in zip(edges, frame.road_edges, strict=True):
    _xyzt(builder, edge, t)
  m.roadEdgeStds = frame.road_edge_stds.tolist()
  leads = m.init("leadsV3", 3)
  lead = frame.lead_one
  for builder in leads:
    builder.t = LEAD_T_IDXS.tolist()
    if lead is None:
      builder.prob = 0.0
      builder.x = builder.y = builder.v = builder.a = [0.0] * len(LEAD_T_IDXS)
    else:
      builder.prob = 0.95
      builder.probTime = 0.0
      x = [lead.x + lead.v_rel * tt for tt in LEAD_T_IDXS.tolist()]
      builder.x = [max(v, 0.0) for v in x]
      builder.y = [-lead.y] * len(LEAD_T_IDXS)
      builder.v = [max(lead.v + lead.a * tt, 0.0) for tt in LEAD_T_IDXS.tolist()]
      builder.a = [lead.a] * len(LEAD_T_IDXS)
    builder.xStd = builder.yStd = builder.vStd = builder.aStd = [0.5] * len(LEAD_T_IDXS)
  m.meta.engagedProb = 1.0
  m.meta.laneChangeState = LANE_CHANGE_STATES[frame.lane_change_state]
  m.meta.laneChangeDirection = getattr(log.LaneChangeDirection, frame.lane_change_direction)
  m.confidence = log.ModelDataV2.ConfidenceClass.green
  m.action.desiredCurvature = frame.curvature
  m.action.desiredAcceleration = frame.a_ego
  m.action.shouldStop = frame.v_ego < 0.3 and frame.lead_one is not None
  return msg


def _lead_data(builder, lead, v_ego: float) -> None:
  if lead is None:
    builder.present = False
    return
  builder.present = True
  builder.dRel = lead.d_rel
  builder.yRel = lead.y
  builder.vRel = lead.v_rel
  builder.vLead = builder.vLeadK = lead.v
  builder.aLeadK = lead.a
  builder.aLeadTau = 1.5
  builder.modelProb = 0.95
  builder.radar = True
  builder.radarTrackId = 1


@dataclass
class PublisherConfig:
  scenario: str = "s_curve"
  seed: int = 0
  speed: float = 25.0
  duration: float = 60.0
  device_type: str = "mici"
  realtime: float = 1.0


class SimPublisher:
  def __init__(self, config: PublisherConfig):
    if config.device_type not in DEVICE_SENSORS:
      raise ValueError(f"device type must be one of {sorted(DEVICE_SENSORS)}")
    self.config = config
    self.sim = Simulation(config.scenario, seed=config.seed, speed=config.speed, duration=config.duration)
    self.pm = messaging.PubMaster([name for name in RATES if name not in ("modelV2", "narrowRoadCameraState")])
    self.camera_pm = messaging.PubMaster(["modelV2", "narrowRoadCameraState"])
    self.camera = RoadCameraRenderer(device_camera(config.device_type), self.sim.frame(0).height)
    self.vipc = VisionIpcServer("camerad")
    self.vipc.create_buffers_with_sizes(VisionStreamType.VISION_STREAM_NARROW_ROAD, 5, self.camera.width, self.camera.height,
                                        self.camera.size, self.camera.stride, self.camera.stride * self.camera.y_height)
    self.vipc.start_listener()
    self.start = time.monotonic()
    self.stop_event = threading.Event()
    self.frames_sent = 0

  def sim_time(self) -> float:
    return (time.monotonic() - self.start) * self.config.realtime

  # --- camera + model, 20 Hz, own thread so numpy rendering never stalls 100 Hz state ---
  def camera_loop(self) -> None:
    rk = Ratekeeper(20., print_delay_threshold=None)
    sensor = getattr(log.FrameData.ImageSensor, DEVICE_SENSORS[self.config.device_type])
    frame_id = 0
    while not self.stop_event.is_set():
      frame = self.sim.frame(self.sim_time())
      yuv = self.camera.render_nv12(frame)
      state = messaging.new_message("narrowRoadCameraState", valid=True)
      stamp = state.logMonoTime
      self.vipc.send(VisionStreamType.VISION_STREAM_NARROW_ROAD, yuv, frame_id, stamp, stamp)
      cs = state.narrowRoadCameraState
      cs.frameId = frame_id
      cs.timestampSof = cs.timestampEof = stamp
      cs.sensor = sensor
      cs.transform = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
      cs.exposureValPercent = 50.0
      self.camera_pm.send("narrowRoadCameraState", state)
      self.camera_pm.send("modelV2", build_model(frame, frame_id, stamp))
      frame_id += 1
      self.frames_sent = frame_id
      rk.keep_time()

  # --- everything else on a 100 Hz base tick ---
  def publish_tick(self, tick: int) -> None:
    frame = self.sim.frame(self.sim_time())
    due = [name for name, rate in RATES.items() if name not in ("modelV2", "narrowRoadCameraState")
           and (tick == 0 or tick % max(1, round(BASE_RATE / rate)) == 0)]
    for name in due:
      self.pm.send(name, getattr(self, "_" + name)(frame))

  def run(self, max_seconds: float | None = None) -> None:
    worker = threading.Thread(target=self.camera_loop, name="simhil-camera", daemon=True)
    worker.start()
    rk = Ratekeeper(BASE_RATE, print_delay_threshold=None)
    try:
      while not self.stop_event.is_set():
        self.publish_tick(rk.frame)
        if max_seconds is not None and time.monotonic() - self.start >= max_seconds:
          break
        rk.keep_time()
    finally:
      self.stop_event.set()
      worker.join(timeout=1.0)

  # --- message builders (one per service in RATES) ---
  def _carState(self, f: ScenarioFrame):
    msg = messaging.new_message("carState", valid=True)
    cs = msg.carState
    cs.vEgo = cs.vEgoRaw = cs.vEgoCluster = f.v_ego
    cs.aEgo = f.a_ego
    cs.yawRate = f.yaw_rate
    cs.standstill = f.v_ego < 0.05
    cs.steeringAngleDeg = f.steering_angle_deg
    cs.gearShifter = car.CarState.GearShifter.drive
    cs.leftBlinker, cs.rightBlinker = f.left_blinker, f.right_blinker
    cs.cruiseState.enabled = cs.cruiseState.available = True
    cs.cruiseState.speed = cs.cruiseState.speedCluster = self.config.speed
    cs.vCruise = cs.vCruiseCluster = self.config.speed * 3.6  # km/h
    cs.canValid = True
    return msg

  def _controlsState(self, f: ScenarioFrame):
    msg = messaging.new_message("controlsState", valid=True)
    msg.controlsState.curvature = f.curvature
    msg.controlsState.desiredCurvature = f.curvature
    return msg

  def _selfdriveState(self, f: ScenarioFrame):
    msg = messaging.new_message("selfdriveState", valid=True)
    ss = msg.selfdriveState
    ss.state = log.SelfdriveState.OpenpilotState.enabled
    ss.enabled = ss.active = ss.engageable = True
    ss.alertStatus = log.SelfdriveState.AlertStatus.normal
    ss.alertSize = log.SelfdriveState.AlertSize.none
    ss.personality = log.LongitudinalPersonality.standard
    return msg

  def _carControl(self, f: ScenarioFrame):
    msg = messaging.new_message("carControl", valid=True)
    cc = msg.carControl
    cc.enabled = cc.latActive = cc.longActive = True
    cc.actuators.curvature = f.curvature
    cc.actuators.steeringAngleDeg = f.steering_angle_deg
    cc.actuators.accel = f.a_ego
    cc.hudControl.setSpeed = self.config.speed
    cc.hudControl.leadVisible = f.lead_one is not None
    return msg

  def _carOutput(self, f: ScenarioFrame):
    msg = messaging.new_message("carOutput", valid=True)
    msg.carOutput.actuatorsOutput.curvature = f.curvature
    msg.carOutput.actuatorsOutput.steeringAngleDeg = f.steering_angle_deg
    msg.carOutput.actuatorsOutput.torque = max(-1.0, min(1.0, f.curvature * 150.0))
    msg.carOutput.actuatorsOutput.accel = f.a_ego
    return msg

  def _starpilotSelfdriveState(self, f: ScenarioFrame):
    return messaging.new_message("starpilotSelfdriveState", valid=True)

  def _starpilotLateralState(self, f: ScenarioFrame):
    return messaging.new_message("starpilotLateralState", valid=True)

  def _starpilotCarState(self, f: ScenarioFrame):
    return messaging.new_message("starpilotCarState", valid=True)

  def _radarState(self, f: ScenarioFrame):
    msg = messaging.new_message("radarState", valid=True)
    _lead_data(msg.radarState.leadOne, f.lead_one, f.v_ego)
    _lead_data(msg.radarState.leadTwo, f.lead_two, f.v_ego)
    return msg

  def _longitudinalPlan(self, f: ScenarioFrame):
    msg = messaging.new_message("longitudinalPlan", valid=True)
    lp = msg.longitudinalPlan
    lp.hasLead = f.lead_one is not None
    lp.longitudinalPlanSource = log.LongitudinalPlan.LongitudinalPlanSource.cruise
    lp.speeds = [max(0.0, f.v_ego + f.a_ego * float(t)) for t in T_IDXS[:17]]
    lp.accels = [f.a_ego] * 17
    lp.aTarget = f.a_ego
    lp.shouldStop = f.v_ego < 0.3 and lp.hasLead
    lp.allowThrottle = lp.allowBrake = True
    return msg

  def _driverMonitoringState(self, f: ScenarioFrame):
    msg = messaging.new_message("driverMonitoringState", valid=True)
    dm = msg.driverMonitoringState
    dm.alertLevel = log.DriverMonitoringState.AlertLevel.none
    dm.activePolicy = log.DriverMonitoringState.MonitoringPolicy.vision
    dm.visionPolicyState.faceDetected = True
    dm.visionPolicyState.awarenessPercent = 100
    dm.isRHD = False
    return msg

  def _driverStateV2(self, f: ScenarioFrame):
    msg = messaging.new_message("driverStateV2", valid=True)
    for side in (msg.driverStateV2.leftDriverData, msg.driverStateV2.rightDriverData):
      side.faceOrientation = [0.0, 0.0, 0.0]
      side.faceProb = 1.0
    return msg

  def _pandaStates(self, f: ScenarioFrame):
    msg = messaging.new_message("pandaStates", 1, valid=True)
    ps = msg.pandaStates[0]
    ps.ignitionLine = True
    ps.pandaType = getattr(log.PandaState.PandaType, PANDA_TYPES[self.config.device_type])
    ps.controlsAllowed = True
    ps.safetyModel = car.CarParams.SafetyModel.allOutput
    ps.voltage = 12000
    return msg

  def _extrinsicsCalibration(self, f: ScenarioFrame):
    msg = messaging.new_message("extrinsicsCalibration", valid=True)
    cal = msg.extrinsicsCalibration
    cal.calStatus = log.ExtrinsicsCalibration.Status.calibrated
    cal.calPerc = 100
    cal.validBlocks = 20
    cal.rpyCalib = list(f.rpy_calib)
    cal.rpyCalibSpread = [0.0, 0.0, 0.0]
    cal.wideFromDeviceEuler = [0.0, 0.0, 0.0]
    cal.height = [f.height]
    return msg

  def _lateralDelay(self, f: ScenarioFrame):
    msg = messaging.new_message("lateralDelay", valid=True)
    msg.lateralDelay.lateralDelay = msg.lateralDelay.lateralDelayEstimate = 0.15
    return msg

  def _deviceState(self, f: ScenarioFrame):
    msg = messaging.new_message("deviceState", valid=True)
    ds = msg.deviceState
    ds.started = True
    ds.startedMonoTime = int(self.start * 1e9)
    ds.deviceType = getattr(log.InitData.DeviceType, self.config.device_type)
    ds.networkType = log.DeviceState.NetworkType.wifi
    ds.thermalStatus = log.DeviceState.ThermalStatus.ok
    ds.freeSpacePercent = 80.0
    ds.memoryUsagePercent = 30
    ds.cpuTempC = [45.0]
    ds.maxTempC = 45.0
    return msg

  def _peripheralState(self, f: ScenarioFrame):
    msg = messaging.new_message("peripheralState", valid=True)
    msg.peripheralState.pandaType = getattr(log.PandaState.PandaType, PANDA_TYPES[self.config.device_type])
    msg.peripheralState.voltage = 12000
    msg.peripheralState.current = 1000
    return msg

  def _managerState(self, f: ScenarioFrame):
    msg = messaging.new_message("managerState", valid=True)
    procs = msg.managerState.init("processes", len(MANAGED))
    for proc, name in zip(procs, MANAGED, strict=True):
      proc.name = name
      proc.running = proc.shouldBeRunning = True
    return msg

  def _onroadEvents(self, f: ScenarioFrame):
    return messaging.new_message("onroadEvents", 0, valid=True)

  def _carParams(self, f: ScenarioFrame):
    msg = messaging.new_message("carParams", valid=True)
    msg.carParams = car_params()
    return msg


def main(argv: list[str] | None = None) -> int:
  parser = argparse.ArgumentParser(description="Publish a simhil scenario on the current OPENPILOT_PREFIX")
  parser.add_argument("--scenario", choices=SCENARIOS, default="s_curve")
  parser.add_argument("--seed", type=int, default=0)
  parser.add_argument("--speed", type=float, default=25.0)
  parser.add_argument("--duration", type=float, default=60.0, help="scenario length in seconds; live runs loop it")
  parser.add_argument("--device-type", choices=sorted(DEVICE_SENSORS), default="mici")
  parser.add_argument("--max-seconds", type=float, default=None, help="exit after this many wall seconds")
  args = parser.parse_args(argv)
  publisher = SimPublisher(PublisherConfig(args.scenario, args.seed, args.speed, args.duration, args.device_type))
  print(f"simhil: publishing {args.scenario} (seed {args.seed}, {args.speed} m/s, {args.device_type})", flush=True)
  try:
    publisher.run(args.max_seconds)
  except KeyboardInterrupt:
    pass
  return 0


if __name__ == "__main__":
  raise SystemExit(main())
