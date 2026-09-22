#!/usr/bin/env bash

# Resolve RPLiDAR's current serial device. Prefer udev's stable by-id symlink
# so a change between ttyUSB0/ttyUSB1 after reboot or replug does not break launch.
resolve_lidar_port() {
    if [[ -n "${LIDAR_PORT:-}" ]]; then
        if [[ -e "$LIDAR_PORT" ]]; then
            printf '%s\n' "$LIDAR_PORT"
            return 0
        fi
        echo "LIDAR_PORT is set but does not exist: $LIDAR_PORT" >&2
        return 1
    fi

    local candidate
    local -a identified_ports=()
    local -a usb_ports=()

    for candidate in \
        /dev/serial/by-id/*CP2102* \
        /dev/serial/by-id/*RPLIDAR* \
        /dev/serial/by-id/*Slamtec*; do
        [[ -e "$candidate" ]] && identified_ports+=("$candidate")
    done

    if (( ${#identified_ports[@]} == 1 )); then
        printf '%s\n' "${identified_ports[0]}"
        return 0
    elif (( ${#identified_ports[@]} > 1 )); then
        echo "Multiple likely LiDAR serial devices found:" >&2
        printf '  %s\n' "${identified_ports[@]}" >&2
        echo "Set LIDAR_PORT to the correct device and retry." >&2
        return 1
    fi

    for candidate in /dev/ttyUSB*; do
        [[ -e "$candidate" ]] && usb_ports+=("$candidate")
    done

    if (( ${#usb_ports[@]} == 1 )); then
        printf '%s\n' "${usb_ports[0]}"
        return 0
    elif (( ${#usb_ports[@]} > 1 )); then
        echo "Could not identify the LiDAR among multiple /dev/ttyUSB* devices:" >&2
        printf '  %s\n' "${usb_ports[@]}" >&2
        echo "Set LIDAR_PORT to the correct device and retry." >&2
        return 1
    fi

    echo "No LiDAR serial device found (checked /dev/serial/by-id and /dev/ttyUSB*)." >&2
    return 1
}
