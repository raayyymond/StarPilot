# Pull the Android Auto stream logs after a drive

Everything the stream writes on the comma lives under `/data/starport/logs/`. Nothing here touches the
car; copy the files while parked.

```sh
# from the PC, with `ssh comma` working
mkdir -p ~/starport-logs/$(date +%F) && cd ~/starport-logs/$(date +%F)
scp 'comma:/data/starport/logs/session-*.jsonl' 'comma:/data/starport/logs/render_profile*.txt' \
    'comma:/data/starport/logs/car_ui.log' 'comma:/data/starport/config.json' .
```

What each file says:

| File | Written by | Look for |
|---|---|---|
| `session-NNNNNN-<timestamp>.jsonl` | starportd (supervisor), one per Android Auto session | `projection_ready` (negotiated size/fps), `encoder` (backend `qcom-v4l2` or `libx264`), `frame_pipeline` (nv12, async readback, target fps), `stats` every 30 s (`fps`, `encode_ms`, `encode_peak_ms`, `frame_age_p95_ms`, `frames_from_view`, `load1`, `temp_max_c`, `cpu0_mhz`, `cpu4_mhz`, `wifi_dbm`), `stream_stall` |
| `render_profile.txt` (and `.1.txt`) | the car-view renderer's sampling profiler, a block every few seconds | `render_stats` line: `fps`, `frame_ms`, the per-phase means (`update_ms`, `prepare_ms`, `draw_ms`, `submit_ms`, `readback_wait_ms`), `content` (`birds_eye` or `full`), `paced` (`camera` or `interval`), `dropped`; below it the hottest Python functions and lines |
| `car_ui.log` | the renderer's stdout | `car view pipeline:` and `car view content:` at start, `dropped a frame`, `Async readback unavailable`, tracebacks |
| `config.json` | the settings in force | `stream_content`, `comma_screen_while_streaming`, `resolution`, `hw_encoder_max_height`, `fps`, `encoder` |

A first read of a slow stream: if `encoder` says `libx264`, the hardware encoder was refused and the
software path caps at 15 fps on a comma; if `stats.fps` is low while `render_stats.fps` is near 30, the
encoder or the Wi-Fi link is the limit (`encode_ms`, `frame_age_p95_ms`, `wifi_dbm`); if
`render_stats.fps` itself is low, the phase means say which part of the renderer is slow.
