#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
INSTALL_DIR="$HOME/ROSGUILauncher"
CONFIG_FILE="$INSTALL_DIR/launcherConfig.yaml"

mkdir -p "$INSTALL_DIR" "$HOME/Desktop"
cp -f "$SCRIPT_DIR/ROSGUILauncher.py" "$INSTALL_DIR/"
cp -f "$SCRIPT_DIR/ROSGUILauncher.sh" "$INSTALL_DIR/"
if [ ! -f "$CONFIG_FILE" ]; then
  cp -f "$SCRIPT_DIR/launcherConfig.yaml" "$CONFIG_FILE"
else
  # Keep a valid rover-specific configuration.  If an interrupted/manual edit
  # left invalid YAML, preserve the ROS master URI when possible, back up the
  # broken file, and rebuild the rest from the repository template.
  python3 - "$CONFIG_FILE" "$SCRIPT_DIR/launcherConfig.yaml" <<'PY'
import datetime
import re
import shutil
import sys
from pathlib import Path

import yaml

config_path = Path(sys.argv[1])
template_path = Path(sys.argv[2])

try:
    loaded = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    if not isinstance(loaded, dict):
        raise ValueError("the top-level YAML value is not a mapping")
except (OSError, UnicodeError, ValueError, yaml.YAMLError) as exc:
    broken_text = config_path.read_text(encoding="utf-8", errors="replace")
    template = yaml.safe_load(template_path.read_text(encoding="utf-8"))

    uri_match = re.search(
        r'(?m)^ros_master_uri\s*:\s*["\']?([^\s"\']+)', broken_text
    )
    if uri_match:
        template["ros_master_uri"] = uri_match.group(1)

    timestamp = datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
    backup_path = config_path.with_name(f"{config_path.name}.invalid-{timestamp}")
    shutil.copy2(config_path, backup_path)
    config_path.write_text(
        yaml.safe_dump(template, sort_keys=False), encoding="utf-8"
    )
    print(f"Repaired invalid launcher config: {exc}")
    print(f"Backup saved to: {backup_path}")
PY
fi
sed "s|@HOME@|$HOME|g" "$SCRIPT_DIR/ROSGUILauncher.desktop" \
  > "$HOME/Desktop/ROSGUILauncher.desktop"
chmod +x "$INSTALL_DIR/ROSGUILauncher.sh" "$HOME/Desktop/ROSGUILauncher.desktop"
