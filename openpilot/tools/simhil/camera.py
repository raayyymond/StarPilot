"""Synthetic road-camera image (sky, asphalt, painted lines, lead car) as NV12, pure numpy.

Rendered at half resolution (the NV12 chroma grid) with a flat-ground pinhole model
using the device's narrow-road intrinsics and the identity calibration published by the
simulation, so the UI's model overlay lines up with the painted lines.
"""
from __future__ import annotations

import numpy as np

from openpilot.common.transformations.camera import DEVICE_CAMERAS, CameraConfig
from openpilot.system.camerad.cameras.nv12_info import get_nv12_info
from openpilot.tools.simhil.scenario import PAINTED_LINES, ROAD_EDGE, ScenarioFrame

LINE_HALF_WIDTH = 0.075
DASH_PERIOD, DASH_LENGTH = 12.0, 3.0
SKY_TOP = np.array([96.0, 136.0, 196.0])
SKY_HORIZON = np.array([196.0, 206.0, 216.0])
ASPHALT = np.array([62.0, 62.0, 66.0])
GRASS = np.array([66.0, 92.0, 52.0])
WHITE = np.array([228.0, 228.0, 224.0])
YELLOW = np.array([226.0, 186.0, 60.0])
CAR_BODY = np.array([28.0, 30.0, 36.0])
TAILLIGHT = np.array([200.0, 24.0, 24.0])
DEVICE_SENSORS = {"mici": "os04c10", "tizi": "ox03c10", "tici": "ar0231"}


def device_camera(device_type: str) -> CameraConfig:
  return DEVICE_CAMERAS[(device_type, DEVICE_SENSORS[device_type])].narrow_road


class RoadCameraRenderer:
  def __init__(self, config: CameraConfig, height: float):
    if config.width % 2 or config.height % 2:
      raise ValueError("NV12 needs even frame dimensions")
    self.width, self.height, self.focal = config.width, config.height, config.focal_length
    self.cam_height = height
    w2, h2 = self.width // 2, self.height // 2
    cx, cy = self.width / 2.0, self.height / 2.0
    self.a = ((np.arange(w2) * 2.0 + 1.0) - cx) / self.focal  # right per unit depth, per column
    b = ((np.arange(h2) * 2.0 + 1.0) - cy) / self.focal  # down per unit depth, per row
    self.ground_rows = np.nonzero(b > 1e-4)[0]
    self.row_depth = height / b[self.ground_rows]  # forward distance of each ground row
    self.row_footprint = 2.0 * self.row_depth / self.focal  # meters per half-res pixel
    sky_t = np.clip((np.arange(h2) / max(cy / 2.0, 1.0))[:, None], 0.0, 1.0)
    self.sky = SKY_TOP * (1 - sky_t) + SKY_HORIZON * sky_t  # (h2, 3)
    self.stride, self.y_height, self.uv_height, self.size = get_nv12_info(self.width, self.height)

  def render_rgb(self, frame: ScenarioFrame) -> np.ndarray:
    """Half-resolution RGB float image (H/2, W/2, 3)."""
    h2, w2 = self.height // 2, self.width // 2
    img = np.empty((h2, w2, 3), dtype=np.float32)
    img[:] = self.sky[:, None, :]
    rows, depth, fp = self.ground_rows, self.row_depth, self.row_footprint
    order = np.argsort(frame.centerline_x)
    cl_x = frame.centerline_x[order]
    yc = np.interp(depth, cl_x, frame.centerline_y[order])
    hdg = np.interp(depth, cl_x, frame.centerline_heading[order])
    s_row = np.interp(depth, cl_x, frame.centerline_u[order]) + frame.s_ego
    y_left = -self.a[None, :] * depth[:, None]
    offset = (y_left - yc[:, None]) * np.cos(hdg)[:, None]
    fpc = fp[:, None]

    road = np.clip((ROAD_EDGE - np.abs(offset)) / fpc + 0.5, 0.0, 1.0)[..., None]
    ground = GRASS + (ASPHALT - GRASS) * road
    dash_on = ((s_row % DASH_PERIOD) < DASH_LENGTH)[:, None]
    for line_offset, dashed in PAINTED_LINES:
      cov = np.clip((LINE_HALF_WIDTH - np.abs(offset - line_offset)) / fpc + 0.5, 0.0, 1.0)
      if dashed:
        cov = cov * dash_on
      color = YELLOW if line_offset > 0 and not dashed else WHITE
      ground = ground + (color - ground) * cov[..., None]
    fog = (1.0 - np.exp(-depth / 450.0))[:, None, None]
    img[rows] = ground * (1 - fog) + SKY_HORIZON * fog

    for lead in frame.leads:
      self._draw_lead(img, lead.x, lead.y)
    return img

  def _draw_lead(self, img: np.ndarray, x: float, y: float) -> None:
    if x < 3.0:
      return
    h2, w2 = img.shape[:2]
    scale = self.focal / x / 2.0
    cx, cy = self.width / 4.0, self.height / 4.0
    u0, u1 = cx + (-y - 0.9) * scale, cx + (-y + 0.9) * scale
    v0, v1 = cy + (self.cam_height - 1.45) * scale, cy + (self.cam_height - 0.15) * scale
    c0, c1 = int(np.clip(u0, 0, w2)), int(np.clip(u1, 0, w2))
    r0, r1 = int(np.clip(v0, 0, h2)), int(np.clip(v1, 0, h2))
    if c1 <= c0 or r1 <= r0:
      return
    img[r0:r1, c0:c1] = CAR_BODY
    lamp_w, lamp_r = max(1, int(0.3 * scale)), int(np.clip(cy + (self.cam_height - 0.95) * scale, 0, h2))
    lamp_r1 = int(np.clip(lamp_r + max(1, int(0.18 * scale)), 0, h2))
    img[lamp_r:lamp_r1, c0:min(c0 + lamp_w, c1)] = TAILLIGHT
    img[lamp_r:lamp_r1, max(c1 - lamp_w, c0):c1] = TAILLIGHT

  def render_nv12(self, frame: ScenarioFrame) -> bytes:
    rgb = self.render_rgb(frame)
    return rgb_half_to_nv12(rgb, self.width, self.height, self.stride, self.y_height, self.size)


def rgb_half_to_nv12(rgb: np.ndarray, width: int, height: int, stride: int, y_height: int, size: int) -> bytes:
  """BT.601 video-range NV12 (same coefficients as tools/sim camerad) from a half-res RGB image."""
  r, g, b = rgb[..., 0], rgb[..., 1], rgb[..., 2]
  y = np.clip((b * 13 + g * 65 + r * 33) / 128.0 + 16.0, 0, 255).astype(np.uint8)
  u = np.clip((b * 56 - g * 37 - r * 19) / 256.0 + 128.5, 0, 255).astype(np.uint8)
  v = np.clip((r * 56 - g * 47 - b * 9) / 256.0 + 128.5, 0, 255).astype(np.uint8)
  buf = np.zeros(size, dtype=np.uint8)
  planes = buf[:stride * (y_height + height // 2)].reshape(-1, stride)  # Y rows, then interleaved UV rows
  planes[:height, :width] = np.repeat(np.repeat(y, 2, axis=0), 2, axis=1)
  planes[y_height:y_height + height // 2, :width] = np.stack((u, v), axis=-1).reshape(height // 2, width)
  return buf.tobytes()
