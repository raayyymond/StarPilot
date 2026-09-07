"""Rebuild the compiled params library (common/params_pyx.so) on the device.

The Dom branch is prebuilt: the valid param keys are compiled into common/params_pyx.so from
common/params_keys.h, and the device never runs scons on boot. After a header change the library
has to be rebuilt on the device (aarch64, AGNOS venv toolchain) or Galaxy reports new keys as
"not editable" and Params().get(key) raises UnknownKeyName. See
docs/how-to/rebuild-params-on-device.md for the manual procedure this module automates.

Used by the Galaxy Software page (/api/update/rebuild_params) and the on-device Software panel.
"""
from __future__ import annotations

import os
import re
import shutil
import subprocess
import tempfile
import time
from collections.abc import Callable
from pathlib import Path

AGNOS_VENV_BIN = "/usr/local/venv/bin"          # scons, cython, numpy, capnpc live here on AGNOS
SCONS_TARGET = "common/params_pyx.so"
BUILD_TIMEOUT_S = 900
VERIFY_TIMEOUT_S = 60
# scons regenerates these as a side effect; they are tracked, so revert them to keep the tree clean.
SIDE_EFFECT_FILES = ("panda/board/obj/gitversion.h", "panda/board/obj/version")
ARTIFACTS = ("common/params_pyx.so", "common/libcommon.a", "common/params_pyx.cpp")

ProgressCallback = Callable[[int, str, float, str], None]


class RebuildParamsError(RuntimeError):
  pass


def _noop_progress(step: int, label: str, percent: float, detail: str) -> None:
  pass


def _build_env() -> dict[str, str]:
  env = dict(os.environ)
  env["PATH"] = f"{AGNOS_VENV_BIN}:{env.get('PATH', '')}" if Path(AGNOS_VENV_BIN).is_dir() else env.get("PATH", "")
  env.setdefault("PYTHONPATH", "")
  return env


def header_keys(repo_path: str | Path) -> list[str]:
  """Every key declared in common/params_keys.h."""
  text = (Path(repo_path) / "common/params_keys.h").read_text(encoding="utf-8", errors="ignore")
  return re.findall(r'^\s*\{"([A-Za-z0-9_]+)",\s*\{', text, flags=re.M)


def compiled_keys(repo_path: str | Path, env: dict[str, str] | None = None) -> list[str]:
  """Keys known to the compiled library, read from a FRESH interpreter (the running one has the old .so loaded)."""
  code = (
    "from openpilot.common.params import Params\n"
    "print('\\n'.join(sorted(k.decode() for k in Params().all_keys())))\n"
  )
  env = dict(env or _build_env())
  env["PYTHONPATH"] = f"{repo_path}:{env.get('PYTHONPATH', '')}"
  python = f"{AGNOS_VENV_BIN}/python3" if Path(f"{AGNOS_VENV_BIN}/python3").exists() else "python3"
  result = subprocess.run([python, "-c", code], cwd=str(repo_path), capture_output=True, text=True,
                          timeout=VERIFY_TIMEOUT_S, env=env)
  if result.returncode != 0:
    raise RebuildParamsError(result.stderr.strip()[-800:] or "could not load the rebuilt params library")
  return [line.strip() for line in result.stdout.splitlines() if line.strip()]


def missing_keys(repo_path: str | Path, env: dict[str, str] | None = None) -> list[str]:
  """Header keys the compiled library does not know. Empty means the library is current."""
  known = set(compiled_keys(repo_path, env))
  return [k for k in header_keys(repo_path) if k not in known]


def _revert_side_effects(repo_path: Path) -> None:
  """Revert the tracked files scons touches as a side effect. Idempotent; never raises."""
  try:
    subprocess.run(["git", "checkout", "--", *SIDE_EFFECT_FILES], cwd=str(repo_path), capture_output=True, text=True, timeout=30)
  except (OSError, subprocess.SubprocessError):
    pass


def _snapshot_artifacts(repo_path: Path) -> Path:
  """Copy the current artifacts aside so a failed build can put them back (scons deletes the target first)."""
  snapshot = Path(tempfile.mkdtemp(prefix="rebuild_params_"))
  for rel in ARTIFACTS:
    src = repo_path / rel
    if src.is_file():
      dst = snapshot / rel
      dst.parent.mkdir(parents=True, exist_ok=True)
      shutil.copy2(src, dst)
  return snapshot


def _restore_artifacts(repo_path: Path, snapshot: Path) -> str:
  """Put the pre-build artifacts back; fall back to the committed copy for anything the snapshot lacks.

  Returns a short human-readable note. Raises RebuildParamsError if the .so is still missing afterwards,
  because that means nothing imports at the next boot.
  """
  restored, checked_out, failed = [], [], []
  for rel in ARTIFACTS:
    src = snapshot / rel
    dst = repo_path / rel
    try:
      if src.is_file():
        dst.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(src, dst)
        restored.append(rel)
        continue
    except OSError:
      pass
    try:
      result = subprocess.run(["git", "checkout", "--", rel], cwd=str(repo_path), capture_output=True, text=True, timeout=60)
    except (OSError, subprocess.SubprocessError):
      result = None
    if result is not None and result.returncode == 0 and dst.is_file():
      checked_out.append(rel)
    else:
      failed.append(rel)

  if not (repo_path / SCONS_TARGET).is_file():
    detail = f"snapshot: {', '.join(restored) or 'none'}; git: {', '.join(checked_out) or 'none'}"
    fix = f"Run `git checkout -- {' '.join(ARTIFACTS)}` in the checkout before rebooting."
    raise RebuildParamsError(f"{SCONS_TARGET} is missing after the failed rebuild and could not be restored ({detail}). {fix}")

  note = f"previous library restored ({len(restored)} file(s) from the pre-build copy"
  if checked_out:
    note += f", {len(checked_out)} from git"
  note += ")"
  if failed:
    note += "; could not restore " + ", ".join(failed)
  return note


def rebuild_params(repo_path: str | Path, progress: ProgressCallback | None = None) -> dict:
  """Run the build, revert side-effect files and verify. Raises RebuildParamsError on failure.

  The artifacts (common/params_pyx.so and friends) are copied aside before scons runs and put back if the
  build, the timeout or the verification fails, so a failed rebuild leaves the device with the library it
  had before instead of a missing or half-linked .so that nothing can import at the next boot.

  Returns a summary dict: seconds, artifacts (path -> bytes), missing (header keys still unknown).
  The caller decides about rebooting; every process that imported the old .so needs a restart.
  """
  progress = progress or _noop_progress
  repo_path = Path(repo_path)
  env = _build_env()
  started = time.time()

  progress(1, "Preparing", 10.0, "Checking the build toolchain...")
  scons = Path(AGNOS_VENV_BIN) / "scons"
  if not scons.exists():
    raise RebuildParamsError(f"{scons} not found: the AGNOS venv toolchain is required to rebuild params")
  if not (repo_path / "common/params_keys.h").is_file():
    raise RebuildParamsError(f"{repo_path} does not look like an openpilot checkout")
  before = missing_keys(repo_path, env)
  progress(1, "Preparing", 60.0, "Saving a copy of the current params library...")
  snapshot = _snapshot_artifacts(repo_path)
  progress(1, "Preparing", 100.0, f"{len(before)} header key(s) missing from the compiled library" if before else "Compiled library already matches the header; rebuilding anyway")

  try:
    progress(2, "Building", 5.0, f"scons -j4 {SCONS_TARGET} (about a minute)...")
    result = subprocess.run([str(scons), "-j4", SCONS_TARGET], cwd=str(repo_path), capture_output=True, text=True,
                            timeout=BUILD_TIMEOUT_S, env=env)
    if result.returncode != 0:
      tail = (result.stderr or result.stdout).strip().splitlines()[-12:]
      raise RebuildParamsError("scons failed:\n" + "\n".join(tail))
    progress(2, "Building", 100.0, "scons: done building targets.")

    progress(3, "Cleaning up", 50.0, "Reverting files scons regenerates as a side effect...")
    _revert_side_effects(repo_path)
    progress(3, "Cleaning up", 100.0, "Done")

    progress(4, "Verifying", 20.0, "Loading the rebuilt library in a fresh interpreter...")
    after = missing_keys(repo_path, env)
    if after:
      raise RebuildParamsError("rebuilt library still does not know: " + ", ".join(after[:10]))
    artifacts = {a: (repo_path / a).stat().st_size for a in ARTIFACTS if (repo_path / a).exists()}
    progress(4, "Verifying", 100.0, f"All {len(header_keys(repo_path))} header keys resolve")
  except BaseException as exc:
    progress(3, "Cleaning up", 50.0, "Rebuild failed; restoring the previous params library...")
    note = _restore_artifacts(repo_path, snapshot)  # raises RebuildParamsError if the .so is still missing
    progress(3, "Cleaning up", 100.0, note)
    if isinstance(exc, subprocess.TimeoutExpired):
      what = "scons" if exc.cmd and str(exc.cmd[0]).endswith("scons") else "verification"
      raise RebuildParamsError(f"{what} timed out after {int(exc.timeout)} s; {note}") from exc
    if isinstance(exc, RebuildParamsError):
      raise RebuildParamsError(f"{exc}\n({note})") from exc
    raise
  finally:
    _revert_side_effects(repo_path)
    shutil.rmtree(snapshot, ignore_errors=True)

  return {"seconds": round(time.time() - started, 1), "artifacts": artifacts, "missing_before": before, "missing": after}
