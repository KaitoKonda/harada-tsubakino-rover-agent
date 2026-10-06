#!/bin/bash
set -e

scriptDir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

source /opt/ros/melodic/setup.bash
source "$HOME/catkin_ws/devel/setup.bash"
cd "$scriptDir"
exec python3 "$scriptDir/rosGuiLauncher.py"
