#!/usr/bin/env python3

from __future__ import annotations

import argparse
import stat
import subprocess
import sys
from pathlib import Path


baseDir = Path(__file__).resolve().parent

scriptMap = {
    "r": baseDir / "updateRosPackage.sh",
    "a": baseDir / "updateArduinoSketch.sh",
    "g": baseDir / "updateGuiLauncher.sh",
}


def parseArgs() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Apply execute permission to selected shell scripts and run them. "
            "If no option is given, all scripts are run."
        )
    )
    parser.add_argument("-r", action="store_true", help="Run script updateRosPackage.sh")
    parser.add_argument("-a", action="store_true", help="Run script updateArduinoSketch.sh")
    parser.add_argument("-g", action="store_true", help="Run script updateGuiLauncher.sh")
    return parser.parse_args()


def selectedKeys(args: argparse.Namespace) -> list[str]:
    keys = [key for key in scriptMap if getattr(args, key)]
    return keys or list(scriptMap.keys())


def ensureExecutable(scriptPath: Path) -> None:
    currentMode = scriptPath.stat().st_mode
    scriptPath.chmod(currentMode | stat.S_IXUSR | stat.S_IXGRP | stat.S_IXOTH)


def runScript(scriptPath: Path) -> int:
    if not scriptPath.exists():
        print(f"Error: script not found: {scriptPath}", file=sys.stderr)
        return 1

    ensureExecutable(scriptPath)
    print(f"Running: {scriptPath.name}")

    completed = subprocess.run(
        ["bash", str(scriptPath)],
        cwd=baseDir,
        check=False,
    )
    return completed.returncode


def main() -> int:
    args = parseArgs()
    targets = [scriptMap[key] for key in selectedKeys(args)]

    exitCode = 0
    for scriptPath in targets:
        result = runScript(scriptPath)
        if result != 0:
            exitCode = result

    return exitCode


if __name__ == "__main__":
    raise SystemExit(main())
