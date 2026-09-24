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
        if [[ -n "${POS_FIREFOX_PROFILE:-}" ]]; then
            FIREFOX_PROFILE="$POS_FIREFOX_PROFILE"
            mkdir -p "$FIREFOX_PROFILE"
        else
            # Give each kiosk launch a fresh private profile. This avoids
            # Firefox's "already running, not responding" lock when a prior
            # kiosk process crashed or is still closing in the background.
            if [[ -d "$HOME/snap/firefox/common" ]]; then
                PROFILE_ROOT="$HOME/snap/firefox/common/pos-kiosk-profiles"
            else
                PROFILE_ROOT="${XDG_DATA_HOME:-$HOME/.local/share}/food-delivery-pos/firefox-profiles"
            fi
            mkdir -p "$PROFILE_ROOT"
            FIREFOX_PROFILE="$(mktemp -d "$PROFILE_ROOT/pos.XXXXXXXX")"
            TEMP_FIREFOX_PROFILE="$FIREFOX_PROFILE"
            trap 'if [[ -n "${TEMP_FIREFOX_PROFILE:-}" ]]; then rm -rf -- "$TEMP_FIREFOX_PROFILE"; fi' EXIT
        fi
        python3 -c 'import json, pathlib, sys; pathlib.Path(sys.argv[1]).write_text("user_pref(\"browser.startup.homepage\", " + json.dumps(sys.argv[2]) + ");\nuser_pref(\"browser.startup.page\", 1);\n", encoding="utf-8")' "$FIREFOX_PROFILE/user.js" "$POS_URL"
        if [[ -n "${WAYLAND_DISPLAY:-}" ]]; then
            export MOZ_ENABLE_WAYLAND="${MOZ_ENABLE_WAYLAND:-1}"
        fi
        echo "Launching Firefox kiosk with profile $FIREFOX_PROFILE"
        "$BROWSER" --no-remote --profile "$FIREFOX_PROFILE" --kiosk --private-window "$POS_URL"
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
