#!/bin/bash
set -e

scriptDir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
installDir="$HOME/ROSGUILauncher"
configFile="$installDir/launcherConfig.yaml"

mkdir -p "$installDir" "$HOME/Desktop"
cp -f "$scriptDir/rosGuiLauncher.py" "$installDir/"
cp -f "$scriptDir/rosGuiLauncher.sh" "$installDir/"
if [ ! -f "$configFile" ]; then
  cp -f "$scriptDir/launcherConfig.yaml" "$configFile"
else
  # Keep a valid rover-specific configuration.  If an interrupted/manual edit
  # left invalid YAML, preserve the ROS master URI when possible, back up the
  # broken file, and rebuild the rest from the repository template.
  python3 - "$configFile" "$scriptDir/launcherConfig.yaml" <<'PY'
import datetime
import re
import shutil
import sys
from pathlib import Path

import yaml

configPath = Path(sys.argv[1])
templatePath = Path(sys.argv[2])

try:
    loaded = yaml.safe_load(configPath.read_text(encoding="utf-8"))
    if not isinstance(loaded, dict):
        raise ValueError("the top-level YAML value is not a mapping")
    if loaded.get("args") is not None and not isinstance(loaded["args"], dict):
        raise ValueError("args is not a mapping or null")
except (OSError, UnicodeError, ValueError, yaml.YAMLError) as exc:
    brokenText = configPath.read_text(encoding="utf-8", errors="replace")
    template = yaml.safe_load(templatePath.read_text(encoding="utf-8"))

    uriMatch = re.search(
        r'(?m)^ros_master_uri\s*:\s*["\']?([^\s"\']+)', brokenText
    )
    if uriMatch:
        template["ros_master_uri"] = uriMatch.group(1)

    timestamp = datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
    backupPath = configPath.with_name(f"{configPath.name}.invalid-{timestamp}")
    shutil.copy2(configPath, backupPath)
    configPath.write_text(
        yaml.safe_dump(template, sort_keys=False), encoding="utf-8"
    )
    print(f"Repaired invalid launcher config: {exc}")
    print(f"Backup saved to: {backupPath}")
else:
    if loaded.get("launch_file") == "haradaTsubakino.launch":
        loaded["launch_file"] = "harada-tsubakino.launch"
        configPath.write_text(
            yaml.safe_dump(loaded, sort_keys=False), encoding="utf-8"
        )
PY
fi
oldDesktop="$HOME/Desktop/ROSGUILauncher.desktop"
if [ -f "$oldDesktop" ] && grep -Fqx 'Name=ROS GUI Launcher' "$oldDesktop"; then
  if grep -Fqx "Exec=$HOME/ROSGUILauncher/ROSGUILauncher.sh" "$oldDesktop" || \
     grep -Fqx "Exec=$HOME/ROSGUILauncher/rosGuiLauncher.sh" "$oldDesktop"; then
    rm -- "$oldDesktop"
  fi
fi
sed "s|@HOME@|$HOME|g" "$scriptDir/rosGuiLauncher.desktop" \
  > "$HOME/Desktop/rosGuiLauncher.desktop"
chmod +x "$installDir/rosGuiLauncher.sh" "$HOME/Desktop/rosGuiLauncher.desktop"
