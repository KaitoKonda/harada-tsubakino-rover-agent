#!/bin/bash
set -e

scriptDir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
targetDir="$HOME/catkin_ws/src/harada-tsubakino"

mkdir -p "$targetDir"
cp -a "$scriptDir/harada-tsubakino/." "$targetDir/"
chmod +x "$targetDir"/scripts/*.py
cd "$HOME/catkin_ws"
catkin build harada-tsubakino
