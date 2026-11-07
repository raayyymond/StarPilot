# Accord durable facts (fork side)

Short, stable facts an agent needs before touching the Accord code. Evidence for each lives in the
kit repo `~/Desktop/Projects/accord-eps-torque-mod`: `docs/STATE.md` (current state),
`docs/BUILD-LINEAGE*.md` (every flashed firmware and what it did), `docs/handoffs/<YYYY-MM>/`
(session narratives). Values below are read from the fork code on `dev` (2026-10-27); the code is
the source of truth if they drift.

## Angle-loop EPS contract (`values.py` `CarControllerParams`, `carcontroller.py`)

- EPS fw `39990-TVA,A16<letter>` (on the car: `A16C`, the kit's V300) → `steerControlType = angle`.
- 0xE4 `STEER_TORQUE` carries the angle setpoint: `raw = round(-10 * deg)`, half-up, left-positive
  degrees negative. The EPS closes a 1 kHz angle loop on it.
- 0xE4 `STEER_CONTROL_ARM = 2` on every frame; the EPS lane runs only while it reads 2.
- Idle hold: when not lat-active, the setpoint is held at the measured wheel angle, but only when
  `enabled` and ACC main is on (the subset every panda policy allows); otherwise 0.
- Error clip |setpoint − measured| ≤ 35.4 / 23.4 / 45 / 61 / 25 / 12.2 deg at
  3.1 / 8 / 10 / 11.75 / 17.5 / 26.9 m/s (the firmware gain-table knots). A pure bound on what is
  sent, never written back into the limiter state.
- Rate limit 250 deg/s (2.5 deg/frame); lateral accel and jerk limits 3.589.
- Actuator delay 0.15 s (model value; lagd learns the rest).

## Bosch longitudinal (`bosch_longitudinal.py`)

- Gas and brake gains are per-speed tables (`SpeedTable`, Accord only; other Bosch cars keep the single
  integrating factor). Gas knots 0/3/6/10/20/30/35 m/s = 0.64/0.64/0.55/0.52/0.50/0.49/0.37, brake
  ratio knots 0/5/15/25/35 m/s = 0.95/1.08/1.13/1.07/1.10: medians of steady-command pid frames over
  1,422 segments (5.7 h, 2026-11-06). The Bosch brake tracks vehicle accel itself, so grade is not
  added back on the brake side; the gas side is open loop, so it is.
- Each knot remembers ~30 min of driving near its speed (decay on every 50 Hz frame near the knot,
  learning only on commands held steady 1 s: launch lag is not gain). The corpus default sits under
  the estimate as one minute of evidence. Brake requests are divided by the ratio down to −2.0 m/s²,
  fading to pass-through by −3.0. State lives in `/data/starpilot/honda/bosch_long_tables.json`
  (the Params key set is a prebuilt library), written every 60 s when changed. Undriven.
- Brake arms at force −0.45, releases at −0.20 (stock −0.12 / −0.02 was the set-speed ping-pong: the
  brake floor fired on tiny requests and the car never coasted).

## Following distance

- Follow-floor 0.4 s on every surface (`profile_*.py`, pinned by `test_follow_floor.py`).

## Open item — check on the car

Under the 7.0 always-on-lateral policy the panda's lateral grant is a session the carcontroller
cannot see, so the idle hold now sends 0 in the AOL-idle state (the old tree held there). **Check
whether engagement from AOL-idle steps the wheel.** BELIEF until a drive says otherwise.

## Never from this folder

No CAN sends, no flashing. Firmware is the kit's domain.
