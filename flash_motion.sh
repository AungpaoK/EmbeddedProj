#!/usr/bin/env bash
# Compile and manually upload the Motion Arduino firmware.
set -euo pipefail

DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ENV_PATH="${ENV_FILE:-${DIR}/.env}"
if [[ -f "$ENV_PATH" ]]; then
    set -a
    # Keep serial configuration consistent with the SLAM launchers.
    # shellcheck disable=SC1090
    source "$ENV_PATH"
    set +a
fi

SKETCH="${DIR}/src/Arduino_1_Motion/Arduino_1_Motion.ino"
FQBN="${ARDUINO_FQBN:-arduino:avr:uno}"

if [[ ! -f "$SKETCH" ]]; then
    echo "Motion sketch not found: $SKETCH" >&2
    exit 1
fi
if ! command -v arduino >/dev/null 2>&1; then
    echo "Arduino IDE CLI ('arduino') is required. Install Arduino IDE 1.8.x first." >&2
    exit 1
fi
if ! command -v readlink >/dev/null 2>&1; then
    echo "readlink is required to resolve the Arduino by-id port." >&2
    exit 1
fi

PORT="${MOTION_PORT:-}"
if [[ -z "$PORT" ]]; then
    shopt -s nullglob
    candidates=(/dev/serial/by-id/*Arduino*)
    shopt -u nullglob
    if (( ${#candidates[@]} != 1 )); then
        echo "Set MOTION_PORT in .env to the Arduino /dev/serial/by-id path." >&2
        echo "Expected exactly one Arduino by-id device; found ${#candidates[@]}." >&2
        exit 1
    fi
    PORT="${candidates[0]}"
fi

if [[ "$PORT" != /dev/serial/by-id/* || "$(basename -- "$PORT")" != *Arduino* ]]; then
    echo "Refusing non-Arduino port: $PORT" >&2
    echo "Set MOTION_PORT to the Arduino symlink under /dev/serial/by-id." >&2
    exit 1
fi

DEVICE="$(readlink -f -- "$PORT" || true)"
if [[ -z "$DEVICE" || ! -c "$DEVICE" ]]; then
    echo "Arduino serial device is unavailable: $PORT" >&2
    exit 1
fi
if [[ "$DEVICE" != /dev/ttyACM* ]]; then
    echo "Refusing unexpected Arduino device target: $PORT -> $DEVICE" >&2
    echo "This project expects the Uno on a /dev/ttyACM* device." >&2
    exit 1
fi

check_port_idle() {
    if command -v fuser >/dev/null 2>&1 && fuser -s "$DEVICE" 2>/dev/null; then
        echo "Serial port is in use: $PORT ($DEVICE). Stop the SLAM Bridge before flashing." >&2
        exit 1
    fi
}

echo "Motion firmware target: Arduino Uno ($FQBN)"
echo "Serial port: $PORT -> $DEVICE"
echo "Compiling $SKETCH ..."
arduino --verify --board "$FQBN" "$SKETCH"

if [[ ! -t 0 ]]; then
    echo "Refusing unattended upload; run this script from an interactive terminal." >&2
    exit 1
fi

check_port_idle
echo
echo "Stop SLAM/Bridge before flashing. For the first test, keep the wheels clear or disconnect motor-driver power."
read -r -p "Type FLASH to upload this sketch to the Arduino above: " CONFIRM
if [[ "$CONFIRM" != "FLASH" ]]; then
    echo "Upload cancelled."
    exit 1
fi

# Check again because the serial port could have been opened during compilation.
check_port_idle
arduino --upload --board "$FQBN" --port "$PORT" "$SKETCH"
echo "Upload complete. Run ./start_slam.sh and confirm the Bridge receives valid ENCODER frames."
