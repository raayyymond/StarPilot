import subprocess
from pathlib import Path

import pytest

from openpilot.starpilot.common import rebuild_params as rp


OLD_SO = b"old params_pyx.so"
OLD_A = b"old libcommon.a"
OLD_CPP = b"old params_pyx.cpp"


@pytest.fixture
def repo(tmp_path, monkeypatch):
  """A fake checkout with the three artifacts, plus a fake AGNOS venv so the toolchain check passes."""
  repo = tmp_path / "openpilot"
  (repo / "common").mkdir(parents=True)
  (repo / "common/params_keys.h").write_text('{"SomeKey", {PERSISTENT}},\n')
  (repo / "common/params_pyx.so").write_bytes(OLD_SO)
  (repo / "common/libcommon.a").write_bytes(OLD_A)
  (repo / "common/params_pyx.cpp").write_bytes(OLD_CPP)
  venv = tmp_path / "venv"
  venv.mkdir()
  (venv / "scons").write_text("")
  monkeypatch.setattr(rp, "AGNOS_VENV_BIN", str(venv))
  monkeypatch.setattr(rp, "missing_keys", lambda *_a, **_k: [])
  return repo


def _artifacts(repo):
  return {a: (repo / a).read_bytes() if (repo / a).is_file() else None for a in rp.ARTIFACTS}


def _clobber(repo):
  """What scons does before linking: the target is gone, the intermediates are half-written."""
  (repo / "common/params_pyx.so").unlink()
  (repo / "common/libcommon.a").write_bytes(b"partial")


def _fake_run(repo, scons_behaviour, calls):
  def run(cmd, **kwargs):
    calls.append(list(cmd))
    if str(cmd[0]).endswith("scons"):
      return scons_behaviour(cmd, kwargs)
    return subprocess.CompletedProcess(cmd, 0, "", "")
  return run


def test_failed_scons_restores_previous_artifacts(repo, monkeypatch):
  calls = []

  def scons(cmd, kwargs):
    _clobber(repo)
    return subprocess.CompletedProcess(cmd, 2, "", "ld: undefined reference to foo\nscons: *** [common/params_pyx.so] Error 1\n")

  monkeypatch.setattr(rp.subprocess, "run", _fake_run(repo, scons, calls))
  monkeypatch.setattr(rp.tempfile, "tempdir", str(repo.parent))
  progress = []

  with pytest.raises(rp.RebuildParamsError) as excinfo:
    rp.rebuild_params(repo, progress=lambda *a: progress.append(a))

  assert "scons failed" in str(excinfo.value)
  assert "previous library restored" in str(excinfo.value)
  assert _artifacts(repo) == {"common/params_pyx.so": OLD_SO, "common/libcommon.a": OLD_A, "common/params_pyx.cpp": OLD_CPP}
  # side-effect revert still happens on failure
  assert any(c[:3] == ["git", "checkout", "--"] and list(rp.SIDE_EFFECT_FILES)[0] in c for c in calls)
  # progress contract unchanged: (step, label, percent, detail)
  assert all(len(p) == 4 and isinstance(p[0], int) and isinstance(p[2], float) for p in progress)
  assert not list(Path(repo.parent).glob("rebuild_params_*"))


def test_scons_timeout_becomes_rebuild_error_and_restores(repo, monkeypatch):
  def scons(cmd, kwargs):
    _clobber(repo)
    raise subprocess.TimeoutExpired(cmd, kwargs.get("timeout", rp.BUILD_TIMEOUT_S))

  monkeypatch.setattr(rp.subprocess, "run", _fake_run(repo, scons, []))

  with pytest.raises(rp.RebuildParamsError) as excinfo:
    rp.rebuild_params(repo)

  assert "timed out" in str(excinfo.value)
  assert str(rp.BUILD_TIMEOUT_S) in str(excinfo.value)
  assert _artifacts(repo)["common/params_pyx.so"] == OLD_SO
  assert _artifacts(repo)["common/libcommon.a"] == OLD_A


def test_verification_failure_restores_previous_library(repo, monkeypatch):
  def scons(cmd, kwargs):
    (repo / "common/params_pyx.so").write_bytes(b"new but broken .so")
    return subprocess.CompletedProcess(cmd, 0, "scons: done building targets.", "")

  monkeypatch.setattr(rp.subprocess, "run", _fake_run(repo, scons, []))
  monkeypatch.setattr(rp, "missing_keys", lambda *_a, **_k: ["AccordRatePlantFF"])

  with pytest.raises(rp.RebuildParamsError) as excinfo:
    rp.rebuild_params(repo)

  assert "still does not know" in str(excinfo.value)
  assert _artifacts(repo)["common/params_pyx.so"] == OLD_SO


def test_restore_falls_back_to_git_when_snapshot_is_missing(repo, monkeypatch, tmp_path):
  calls = []

  def scons(cmd, kwargs):
    _clobber(repo)
    return subprocess.CompletedProcess(cmd, 1, "", "boom")

  def run(cmd, **kwargs):
    calls.append(list(cmd))
    if str(cmd[0]).endswith("scons"):
      return scons(cmd, kwargs)
    if cmd[:3] == ["git", "checkout", "--"] and cmd[3] in rp.ARTIFACTS:
      (repo / cmd[3]).write_bytes(b"committed " + cmd[3].encode())
    return subprocess.CompletedProcess(cmd, 0, "", "")

  monkeypatch.setattr(rp.subprocess, "run", run)
  # Snapshot has nothing in it (e.g. the copy failed), so git is the only way back.
  monkeypatch.setattr(rp, "_snapshot_artifacts", lambda _repo: tmp_path / "empty_snapshot")

  with pytest.raises(rp.RebuildParamsError) as excinfo:
    rp.rebuild_params(repo)

  assert "from git" in str(excinfo.value)
  assert _artifacts(repo)["common/params_pyx.so"] == b"committed common/params_pyx.so"
  assert any(c == ["git", "checkout", "--", "common/params_pyx.so"] for c in calls)


def test_unrestorable_so_is_reported_as_rebuild_error(repo, monkeypatch, tmp_path):
  def run(cmd, **kwargs):
    if str(cmd[0]).endswith("scons"):
      _clobber(repo)
      return subprocess.CompletedProcess(cmd, 1, "", "boom")
    return subprocess.CompletedProcess(cmd, 128, "", "fatal: not a git repository")

  monkeypatch.setattr(rp.subprocess, "run", run)
  monkeypatch.setattr(rp, "_snapshot_artifacts", lambda _repo: tmp_path / "empty_snapshot")

  with pytest.raises(rp.RebuildParamsError) as excinfo:
    rp.rebuild_params(repo)

  assert "could not be restored" in str(excinfo.value)
  assert "git checkout --" in str(excinfo.value)


def test_successful_rebuild_keeps_new_artifacts_and_cleans_snapshot(repo, monkeypatch, tmp_path):
  def scons(cmd, kwargs):
    (repo / "common/params_pyx.so").write_bytes(b"new .so")
    return subprocess.CompletedProcess(cmd, 0, "scons: done building targets.", "")

  monkeypatch.setattr(rp.subprocess, "run", _fake_run(repo, scons, []))
  monkeypatch.setattr(rp.tempfile, "tempdir", str(tmp_path))

  summary = rp.rebuild_params(repo)

  assert summary["missing"] == []
  assert _artifacts(repo)["common/params_pyx.so"] == b"new .so"
  assert summary["artifacts"]["common/params_pyx.so"] == len(b"new .so")
  assert not list(tmp_path.glob("rebuild_params_*"))
