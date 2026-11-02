# Desktop UI in WSL2 (software or host-GPU rendering)

The native Raylib UI runs in WSL2 through WSLg. There is no `/dev/dri` and no comma
hardware; everything below uses the isolated host runtime (`./dev`).

## Rendering backends

| Backend | How | Notes |
| --- | --- | --- |
| Mesa llvmpipe (default) | nothing to set | Software GL. Works for `./c4`, `./c3`, `./onroad`, `./simui`; the compact UI renders ~30-60 FPS. |
| Host GPU via D3D12 | `GALLIUM_DRIVER=d3d12`, or `./simui --gpu` | Uses the Windows GPU driver through WSLg. |
| Hidden window | `HIDDEN_WINDOW=1` (desktop only) | Creates the GL context without showing a window; combine with `RECORD=1 RECORD_OUTPUT=out.mp4`. |

`xvfb-run` also works when no WSLg display is available.

## Commands

- `./c4` / `./c3`: compact / large UI with parked (offroad) state.
- `./onroad --c4 --demo`: replays the demo route (downloads it; needs network access).
- `./simui --c4 --scenario s_curve`: synthetic drive, no route or hardware needed. Opens
  the compact UI onroad and engaged, with a synthetic road camera, lane lines, path and
  leads. Scenarios: `straight`, `s_curve`, `lane_change`, `lead_stop`, `merge`. Options:
  `--c3`, `--seed N`, `--speed M/S`, `--duration S` (live runs loop), `--record out.mp4`,
  `--gpu`. Stop with Ctrl+C.
- Headless PNG frames for checks:
  `./simui --c4 --scenario lead_stop --headless --frames 5 --seconds 15 --out /tmp/simui`
  writes `sim.mp4` and `frame_000.png`... into the output directory.

`./simui` publishes into a private `replay-sim...` messaging prefix with disposable Params
and runs the UI in its live clock mode (no replay display clock). The publisher lives in
`openpilot/tools/simhil/` (`publisher.py` lists every service and rate).

## Android Auto stream preview (`./aapreview`)

`./aapreview` runs the real projection renderer (`starport/current_car_ui.py`) on the PC
through a surfaceless Mesa EGL context and shows the frames the head unit would receive.

- `./aapreview` opens a 1280x720@30 window; `--res 480p|720p|1080p`, `--width/--height`,
  `--fps N` and `--margins WxH` set the requested geometry.
- `--encode [--mp4 out.mp4]` pushes every frame through the software H.264 encoder and
  decodes it back, so the window shows the encoded picture.
- `--headless --frames N --out DIR` dumps PNGs without a window.
- `--fake-head-unit --resolution 1080p [--offer 480p,720p,1080p]` negotiates a real
  `ProjectionSession` against the test `FakeHeadUnit` on localhost.
- To see driving data, start `./simui --c4 --scenario s_curve --prefix replay-aa` (or
  `./onroad --replay-only --prefix replay-aa <route>`) in one terminal, then
  `./aapreview --prefix replay-aa` in another. Keys: K keyframe, S save PNG, Esc quit.

On the device the output size is still negotiated with the head unit. The starport config
(`/data/starport/config.json`) gains `"resolution": "auto"|"480p"|"720p"|"1080p"` to
reorder the preference, and `"hw_encoder_max_height": 720` (set 1080 to opt in). 1080p on
the comma 4 hardware encoder is unverified and needs `libstarport_encoder.so` rebuilt.

## Birds-eye view in the stream

The projection layout has a `birds_eye` widget (enabled by default, 520x600 panel at the
right edge) drawing lane lines, road edges, the planned path, ego and leads top-down.
Tapping it switches between the panel and a full-frame view. Preview it alone with
`./dev python -m openpilot.starpilot.ui.preview_birds_eye --output be.png [--show]`.
