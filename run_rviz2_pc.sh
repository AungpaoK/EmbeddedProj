#!/usr/bin/env bash
# ==============================================================================
# run_rviz2_pc.sh — รัน RViz2 บนเครื่อง PC ผ่าน Docker เพื่อดึงภาพ Map/Scan จาก Raspberry Pi
# ==============================================================================
set -e

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ENV_PATH="${ENV_FILE:-${DIR}/.env}"
if [[ ! -f "$ENV_PATH" ]]; then
    echo "ไม่พบไฟล์ environment: $ENV_PATH" >&2
    echo "สร้างจาก .env.example แล้วกำหนด ip_address ก่อนรัน" >&2
    exit 1
fi

set -a
# shellcheck disable=SC1090
source "$ENV_PATH"
set +a

RVIZ_CONFIG="${1:-${DIR}/src/slam_view.rviz}"
if [[ "$RVIZ_CONFIG" != /* ]]; then
    RVIZ_CONFIG="/workspace/${RVIZ_CONFIG}"
elif [[ "$RVIZ_CONFIG" == "${DIR}"* ]]; then
    RVIZ_CONFIG="/workspace/${RVIZ_CONFIG#"${DIR}/"}"
fi

# .env is the source of truth. Keep uppercase/legacy names as aliases, but
# never silently fall back to the old hardcoded address.
PI_IP="${ip_address:-${IP_ADDRESS:-${PI_IP:-}}}"
if [[ -z "$PI_IP" ]]; then
    echo "ไม่พบ ip_address ใน $ENV_PATH" >&2
    exit 1
fi

# Use a fresh file per launch so an existing RViz/container can never reuse a
# peer list generated from an older .env value.
PEERS_FILE="$(mktemp /tmp/fastdds_peers.XXXXXX.xml)"
cleanup() {
    rm -f "$PEERS_FILE"
}
trap cleanup EXIT

cat <<EOF > "$PEERS_FILE"
<?xml version="1.0" encoding="UTF-8" ?>
<dds xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
    <profiles>
        <participant profile_name="default_profile" is_default_profile="true">
            <rtps>
                <builtin>
                    <initialPeersList>
                        <locator>
                            <udpv4>
                                <address>${PI_IP}</address>
                            </udpv4>
                        </locator>
                    </initialPeersList>
                </builtin>
            </rtps>
        </participant>
    </profiles>
</dds>
EOF

echo "=== [Garvis] เริ่มต้น RViz2 (Unicast เชื่อมตรงไปยัง Raspberry Pi $PI_IP) ==="

xhost +local:root > /dev/null 2>&1 || true

GPU_ARGS=()
if [ -d "/dev/dri" ]; then
    GPU_ARGS=(--device /dev/dri)
fi

docker run -it --rm \
  --net=host \
  --ipc=host \
  --privileged \
  "${GPU_ARGS[@]}" \
  -e DISPLAY="${DISPLAY:-:0}" \
  -e WAYLAND_DISPLAY="${WAYLAND_DISPLAY}" \
  -e XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR}" \
  -e ROS_DOMAIN_ID=0 \
  -e FASTRTPS_DEFAULT_PROFILES_FILE=/tmp/fastdds_peers.xml \
  -v "$PEERS_FILE:/tmp/fastdds_peers.xml:ro" \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v "$HOME/.rviz2:/root/.rviz2:rw" \
  -v "${DIR}:/workspace" \
  osrf/ros:jazzy-desktop \
  bash -c "source /opt/ros/jazzy/setup.bash && rviz2 -d ${RVIZ_CONFIG}"
