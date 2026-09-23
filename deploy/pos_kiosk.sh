#!/usr/bin/env bash
set -euo pipefail

POS_URL="${POS_URL:-http://127.0.0.1:8765/}"
HEALTH_URL="${POS_URL%/}/api/health"
WAIT_SECONDS="${POS_WAIT_SECONDS:-90}"
DEADLINE=$((SECONDS + WAIT_SECONDS))

while ! python3 -c 'import sys, urllib.request; urllib.request.urlopen(sys.argv[1], timeout=1)' "$HEALTH_URL" >/dev/null 2>&1; do
    if (( SECONDS >= DEADLINE )); then
        echo "POS controller did not become ready at $HEALTH_URL" >&2
        exit 1
    fi
    sleep 1
done

if [[ -n "${POS_BROWSER:-}" ]]; then
    BROWSER="$POS_BROWSER"
elif command -v chromium >/dev/null 2>&1; then
    BROWSER="chromium"
elif command -v chromium-browser >/dev/null 2>&1; then
    BROWSER="chromium-browser"
elif command -v firefox >/dev/null 2>&1; then
    BROWSER="firefox"
else
    echo "No supported browser was found. Set POS_BROWSER to its executable path." >&2
    exit 1
fi

if ! command -v "$BROWSER" >/dev/null 2>&1; then
    echo "Browser executable not found: $BROWSER" >&2
    exit 1
fi

case "$(basename "$BROWSER")" in
    firefox|firefox-esr)
        exec "$BROWSER" --kiosk "$POS_URL"
        ;;
    *)
        exec "$BROWSER" \
            --kiosk \
            --no-first-run \
            --noerrdialogs \
            --disable-session-crashed-bubble \
            "$POS_URL"
        ;;
esac
