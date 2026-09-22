#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TARGET_DIR="$HOME/catkin_ws/src/harada-tsubakino"

mkdir -p "$TARGET_DIR"
cp -a "$SCRIPT_DIR/harada-tsubakino/." "$TARGET_DIR/"
chmod +x "$TARGET_DIR"/scripts/*.py
cd "$HOME/catkin_ws"
catkin build harada-tsubakino
