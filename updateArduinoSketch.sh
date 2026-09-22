#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ARDUINO_PORT="${ARDUINO_PORT:-/dev/ttyACM0}"

arduino-cli compile --upload --port "$ARDUINO_PORT" \
  --fqbn arduino:esp32:nano_nora \
  "$SCRIPT_DIR/OtosHmcSerialSender"
