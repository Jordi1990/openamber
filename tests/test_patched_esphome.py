"""A cached firmware dependency must match both its base commit and patch."""
import hashlib
import importlib.util
from pathlib import Path

import pytest

PATH = Path(__file__).resolve().parents[1] / "tools/esphome_patched.py"
spec = importlib.util.spec_from_file_location("esphome_patched", PATH)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


@pytest.fixture
def dependency(tmp_path, monkeypatch):
    repository = tmp_path / "repository"
    repository.mkdir()
    module.git(repository, "init", "--quiet")
    (repository / "source.txt").write_text("before\n")
    module.git(repository, "add", "source.txt")
    module.git(repository, "-c", "user.name=Test", "-c", "user.email=test@example.invalid",
               "commit", "--quiet", "-m", "baseline")
    commit = module.git(repository, "rev-parse", "HEAD").decode().strip()
    (repository / "source.txt").write_text("after\n")
    patch = tmp_path / "change.patch"
    patch.write_bytes(module.git(repository, "diff"))
    manifest = {"repository": str(repository), "commit": commit,
                "patch_sha256": hashlib.sha256(patch.read_bytes()).hexdigest()}
    monkeypatch.setenv("XDG_CACHE_HOME", str(tmp_path / "cache"))
    return manifest, patch


def test_pinned_source_is_patched_and_reused(dependency):
    manifest, patch = dependency
    source = module.prepare_source(manifest, patch)
    assert (source / "source.txt").read_text() == "after\n"
    assert module.git(source, "rev-parse", "HEAD").decode().strip() == manifest["commit"]
    assert module.prepare_source(manifest, patch) == source


def test_local_source_changes_are_rejected(dependency):
    manifest, patch = dependency
    source = module.prepare_source(manifest, patch)
    (source / "source.txt").write_text("unexpected\n")
    with pytest.raises(RuntimeError, match="source was modified"):
        module.prepare_source(manifest, patch)


def test_untracked_source_is_rejected(dependency):
    manifest, patch = dependency
    source = module.prepare_source(manifest, patch)
    (source / "extra.py").write_text("unexpected\n")
    with pytest.raises(RuntimeError, match="untracked files"):
        module.prepare_source(manifest, patch)


def test_changed_patch_is_rejected(dependency):
    manifest, patch = dependency
    patch.write_text("unexpected\n")
    with pytest.raises(RuntimeError, match="recorded checksum"):
        module.prepare_source(manifest, patch)
