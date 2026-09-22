#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

source /opt/ros/melodic/setup.bash
source "$HOME/catkin_ws/devel/setup.bash"
cd "$SCRIPT_DIR"
exec python3 "$SCRIPT_DIR/ROSGUILauncher.py"
