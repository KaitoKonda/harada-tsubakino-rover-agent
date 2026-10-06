#!/bin/bash
set -e

scriptDir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
arduinoPort="${ARDUINO_PORT:-/dev/ttyACM0}"

arduino-cli compile --upload --port "$arduinoPort" \
  --fqbn arduino:esp32:nano_nora \
  "$scriptDir/otosHmcSerialSender"
