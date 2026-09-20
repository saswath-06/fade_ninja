#!/usr/bin/env bash
# Compile and upload the 3-DOF servo firmware.
#
#   tools/flash_firmware.sh                 # compile only — safe, changes nothing
#   tools/flash_firmware.sh --upload        # OVERWRITES whatever is on the board
#
# Uses the arduino-cli bundled inside Arduino IDE, so nothing extra to install.
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
SKETCH="$ROOT/firmware/fade_ninja_arm3"
FQBN="${FQBN:-arduino:renesas_uno:unor4wifi}"
CLI="${ARDUINO_CLI:-/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli}"

if [ ! -x "$CLI" ]; then
  echo "arduino-cli not found at:"
  echo "  $CLI"
  echo "Install Arduino IDE, or set ARDUINO_CLI=/path/to/arduino-cli"
  exit 1
fi

"$CLI" lib install Servo >/dev/null 2>&1 || true
echo "Compiling for $FQBN ..."
"$CLI" compile --fqbn "$FQBN" "$SKETCH"

if [ "${1:-}" != "--upload" ]; then
  echo
  echo "Compiled only. Re-run with --upload to write it to the board."
  echo "That ERASES the sketch currently on the Arduino."
  exit 0
fi

PORT="${2:-$(cd "$ROOT" && uv run python -c '
from fade_ninja.hardware_arm import find_serial_port
print(find_serial_port())')}"

echo "Uploading to $PORT ..."
"$CLI" upload -p "$PORT" --fqbn "$FQBN" "$SKETCH"
echo
echo "Done. Check it answers:"
echo "  uv run python arm3dof_server.py --servo-port --dry-run"
