# StarPilot fork — 2020 Honda Accord — Agent Context

Operator's fork of `firestar5683/StarPilot`. **Fork-only car work happens here.** Firmware work, and
anything that needs firmware + fork together, happens in the kit repo
`C:/Users/dudei/Desktop/Projects/accord-eps-torque-mod` (read-only from here). This file is an INDEX.
Keep it short; durable facts go in `docs/ACCORD.md`, not here.

## 🛑 Load `openpilot-iteration` before any control change

`.claude/skills/openpilot-iteration/SKILL.md` — the operator's architecture, what a good solution
means, what not to break, how to run the work. **Mandatory before proposing, simulating or shipping
any fork-side control change.** Fork-side experiments ship as a Galaxy toggle config when existing
params can express them, code otherwise. Mark every decision-bearing claim EVIDENCE or BELIEF.
Never record the operator's two openpilot colleagues by name ("colleague 1" / "colleague 2").

## Branches

- **`dev`** — work here. Upstream `Dom` 7.0 "Domathon" + the Accord commits (ported by hand
  2026-10-27, `e9256c574e..46dfc39dbe`).
- `dev-dom6` — the pre-rework line (old `Dom` + Accord), kept for reference.
- `accord-PTM` — the frozen V300 release record.
- Upstream `Dom` was force-pushed to the 7.0 history on 2026-10-27; its merge-base with the old line is
  gone. Remotes: `origin` = `git@github.com:raayyymond/StarPilot.git`, `upstream` =
  `https://github.com/firestar5683/StarPilot.git`.

Git rules: commit on `dev`, push to `origin dev`, **never force-push**, never commit the operator's
dirty tree wholesale — stage only the files you changed.

## The car

2020 Honda Accord, EPS firmware `39990-TVA,A16C` = the kit's V300 angle-loop firmware.
`steerControlType = angle`, detected from the EPS fw string alone; **no params rebuild and no toggle
config needed.** Contract and numbers: `docs/ACCORD.md`.

## Where the Accord code lives (7.0 tree)

- `opendbc_repo/opendbc/car/honda/{carcontroller,carstate,hondacan,interface,values,bosch_longitudinal}.py`
- `opendbc_repo/opendbc/dbc/generator/honda/_bosch_2018.dbc`
- `openpilot/selfdrive/controls/controlsd.py`, `openpilot/selfdrive/controls/lib/latcontrol_angle.py`
- `openpilot/selfdrive/locationd/lagd.py`
- `openpilot/selfdrive/ui/mici/onroad/torque_bar.py`
- `openpilot/starpilot/longitudinal/profile_*.py`

Tests that pin it:
- `opendbc_repo/opendbc/car/honda/tests/{test_honda_accord_angle,test_bosch_longitudinal,test_honda}.py`
- `openpilot/selfdrive/controls/tests/{test_latcontrol_angle_accord_saturation,test_lane_change_turn_gate}.py`
- `openpilot/selfdrive/locationd/test/test_lagd.py`
- `openpilot/starpilot/longitudinal/tests/test_follow_floor.py`

## Python and tests

- Python = `C:/Users/dudei/anaconda3/envs/bin_decompile/python.exe`. **Never `python3`** (Windows
  Store stub).
- Local tests need, in Windows path form:
  `PYTHONPATH="<root>;<root>/opendbc_repo;<root>/msgq_repo;<root>/rednose_repo;<root>/panda"`.
  The opendbc and latcontrol tests run locally.
- Anything that imports `openpilot.common.params` cannot run on Windows (`fcntl`, compiled `msgq`).
  Run it on the comma 4: `ssh comma`, python `/usr/local/venv/bin/python`, pytest via
  `PYTHONPATH=/data/tools/pytest`, side worktree `/data/dev-wt`.
  🛑 **Never touch `/data/openpilot` while the car may be driving.**

## rlogs

Routes live on the device under `/data/media/0/realdata`. The decoders (`rlog-tools/`) and the
`fetch-rlogs` skill live in the kit repo, not here — go there to read a drive.

## Safety

- **Never send a CAN message and never flash anything from this folder.** The firmware kit owns both.
- Firmware questions: kit repo `CLAUDE.md`, `docs/STATE.md`, `docs/BUILD-LINEAGE*.md`.
