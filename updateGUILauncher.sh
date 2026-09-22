#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
INSTALL_DIR="$HOME/ROSGUILauncher"

mkdir -p "$INSTALL_DIR" "$HOME/Desktop"
cp -f "$SCRIPT_DIR/ROSGUILauncher.py" "$INSTALL_DIR/"
cp -f "$SCRIPT_DIR/ROSGUILauncher.sh" "$INSTALL_DIR/"
if [ ! -f "$INSTALL_DIR/launcherConfig.yaml" ]; then
  cp -f "$SCRIPT_DIR/launcherConfig.yaml" "$INSTALL_DIR/"
fi
sed "s|@HOME@|$HOME|g" "$SCRIPT_DIR/ROSGUILauncher.desktop" \
  > "$HOME/Desktop/ROSGUILauncher.desktop"
chmod +x "$INSTALL_DIR/ROSGUILauncher.sh" "$HOME/Desktop/ROSGUILauncher.desktop"
