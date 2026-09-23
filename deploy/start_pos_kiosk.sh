#!/usr/bin/env bash
# Keep the POS browser alive across slow desktop startup or browser exits.
set -uo pipefail

STARTUP_DELAY="${POS_STARTUP_DELAY:-5}"
RESTART_DELAY="${POS_RESTART_DELAY:-3}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOG_FILE="${POS_KIOSK_LOG:-/tmp/pos_kiosk_autostart.log}"
LOCK_FILE="${POS_KIOSK_LOCK:-/tmp/food_delivery_pos_kiosk.lock}"

log() {
    printf '[%s] %s\n' "$(date '+%Y-%m-%d %H:%M:%S')" "$*" >> "$LOG_FILE"
}

trap 'log "Kiosk supervisor stopped"; exit 0' INT TERM

sleep "$STARTUP_DELAY"

cd "$SCRIPT_DIR/.."
exec 200>"$LOCK_FILE"
if ! flock -n 200; then
    log "Another POS kiosk supervisor already holds the lock; exiting"
    exit 0
fi

export POS_URL="${POS_URL:-http://127.0.0.1:8765/}"
export DISPLAY="${DISPLAY:-:0}"
if [[ -z "${WAYLAND_DISPLAY:-}" && -n "${XDG_RUNTIME_DIR:-}" ]]; then
    if [[ -e "$XDG_RUNTIME_DIR/wayland-0" ]]; then
        export WAYLAND_DISPLAY=wayland-0
    elif [[ -e "$XDG_RUNTIME_DIR/wayland-1" ]]; then
        export WAYLAND_DISPLAY=wayland-1
    fi
fi

log "POS kiosk supervisor started (DISPLAY=$DISPLAY WAYLAND_DISPLAY=${WAYLAND_DISPLAY:-unset} URL=$POS_URL)"
while true; do
    log "Launching POS browser"
    /bin/bash "$SCRIPT_DIR/pos_kiosk.sh" >> "$LOG_FILE" 2>&1
    exit_code=$?
    log "POS browser exited with status $exit_code; retrying in ${RESTART_DELAY}s"
    sleep "$RESTART_DELAY"
done
