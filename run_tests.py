#!/usr/bin/env python3
"""OpenAmber Test Automation Runner."""

import os
import subprocess
import sys

PROJECT_ROOT = os.path.dirname(os.path.abspath(__file__))
TESTS_DIR = os.path.join(PROJECT_ROOT, "tests")
VENV_PYTHON_WIN = os.path.join(PROJECT_ROOT, ".venv", "Scripts", "python.exe")
VENV_PYTHON_UNIX = os.path.join(PROJECT_ROOT, ".venv", "bin", "python")


def get_python_binary():
    """Locate the appropriate Python interpreter."""
    if os.path.exists(VENV_PYTHON_WIN):
        return VENV_PYTHON_WIN
    if os.path.exists(VENV_PYTHON_UNIX):
        return VENV_PYTHON_UNIX
    return sys.executable


def main():
    print("=" * 70)
    print("   OpenAmber UI & System Test Automation Runner")
    print("=" * 70)

    python_bin = get_python_binary()

    # Pass through any additional CLI arguments (e.g. specific test files, -k expressions, -s)
    extra_args = sys.argv[1:]
    target_args = [TESTS_DIR] if (not extra_args or extra_args[0].startswith("-")) else []
    cmd = [python_bin, "-m", "pytest", "-v"] + extra_args + target_args
    print(f"Running: {' '.join(cmd)}\n")

    env = os.environ.copy()
    env.setdefault("SDL_VIDEODRIVER", "dummy")
    env.setdefault("SDL_AUDIODRIVER", "dummy")

    res = subprocess.run(cmd, cwd=PROJECT_ROOT, env=env)
    sys.exit(res.returncode)


if __name__ == "__main__":
    main()

