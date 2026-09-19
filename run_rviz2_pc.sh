#!/usr/bin/env bash
# ==============================================================================
# run_rviz2_pc.sh — รัน RViz2 บนเครื่อง PC ผ่าน Docker เพื่อดึงภาพ Map/Scan จาก Raspberry Pi
# ==============================================================================
set -e

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RVIZ_CONFIG="${1:-${DIR}/src/slam_view.rviz}"
if [[ "$RVIZ_CONFIG" != /* ]]; then
    RVIZ_CONFIG="/workspace/${RVIZ_CONFIG}"
elif [[ "$RVIZ_CONFIG" == "${DIR}"* ]]; then
    RVIZ_CONFIG="/workspace/${RVIZ_CONFIG#"${DIR}/"}"
fi
PI_IP="${PI_IP:-172.30.81.226}"
PEERS_FILE="$HOME/.fastdds_peers.xml"

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
  -e FASTRTPS_DEFAULT_PROFILES_FILE="$PEERS_FILE" \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v "$HOME/.rviz2:/root/.rviz2:rw" \
  -v "${DIR}:/workspace" \
  osrf/ros:jazzy-desktop \
  bash -c "source /opt/ros/jazzy/setup.bash && rviz2 -d ${RVIZ_CONFIG}"
