#!/usr/bin/env python3
"""Run the pinned ESPHome source with the reviewed LVGL startup patch."""
import fcntl
import hashlib
import importlib.metadata
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[1]


def git(source, *args):
    return subprocess.check_output(["git", "-C", str(source), *args])


def prepare_source(manifest, patch):
    digest = hashlib.sha256(patch.read_bytes()).hexdigest()
    if digest != manifest["patch_sha256"]:
        raise RuntimeError("ESPHome patch differs from its recorded checksum")
    cache = Path(os.environ.get("XDG_CACHE_HOME", Path.home() / ".cache")) / "openamber/esphome"
    cache.mkdir(parents=True, exist_ok=True)
    source = cache / f"{manifest['commit'][:12]}-{digest[:16]}"
    record = source.with_suffix(".json")
    with (cache / ".lock").open("a") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        if not source.exists():
            with tempfile.TemporaryDirectory(dir=cache) as temporary:
                checkout = Path(temporary) / "source"
                checkout.mkdir()
                git(checkout, "init", "--quiet")
                git(checkout, "fetch", "--quiet", "--depth=1", manifest["repository"], manifest["commit"])
                git(checkout, "checkout", "--quiet", "--detach", "FETCH_HEAD")
                git(checkout, "apply", "--index", str(patch))
                diff_hash = hashlib.sha256(git(checkout, "diff", "--binary", "HEAD")).hexdigest()
                record.write_text(json.dumps({"diff_sha256": diff_hash}) + "\n")
                checkout.rename(source)
        if git(source, "rev-parse", "HEAD").decode().strip() != manifest["commit"]:
            raise RuntimeError("Cached ESPHome source is on a different commit")
        expected = json.loads(record.read_text())["diff_sha256"]
        if hashlib.sha256(git(source, "diff", "--binary", "HEAD")).hexdigest() != expected:
            raise RuntimeError("Cached ESPHome source was modified; refusing an unverified build")
        if git(source, "ls-files", "--others", "--exclude-standard").strip():
            raise RuntimeError("Cached ESPHome source contains untracked files")
    return source


def main():
    manifest = json.loads((ROOT / "patches/esphome.json").read_text())
    if importlib.metadata.version("esphome") != manifest["version"]:
        raise RuntimeError(f"Install esphome=={manifest['version']} in this Python environment first")
    source = prepare_source(manifest, ROOT / "patches" / manifest["patch"])
    env = os.environ.copy()
    env["PYTHONPATH"] = str(source)
    os.execve(sys.executable, [sys.executable, "-m", "esphome", *sys.argv[1:]], env)


if __name__ == "__main__":
    main()
