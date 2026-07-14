#!/usr/bin/env bash
#
# start_rosbag_logger.sh
# ----------------------
# Manually record a rosbag of the Livox + FAST_LIO topics for long-run
# stability / odometry-drift evaluation. Ctrl-C stops it cleanly (rosbag2
# finalises the bag + metadata on SIGINT).
#
# Default profile (Standard + IMU) records the compact, diagnostic topics:
#   /Odometry  /tf  /tf_static  /livox/imu
# It deliberately does NOT record:
#   * dense clouds (/cloud_registered, /Laser_map, ...) — huge on a long run
#   * /path — a nav_msgs/Path that re-sends the WHOLE growing trajectory every
#     message (FAST_LIO's publish_path push_backs without a cap), so it balloons
#     to MB/msg over hours. The trajectory is fully reconstructable from
#     /Odometry — use analyze_bag.py --plot for the "which way did it drift" view.
#
# Flags:
#   --cloud        also record /cloud_registered (FAST_LIO registered PointCloud2,
#                  for RViz playback later; ~0.6 MB/s ~= 2 GB/hour raw)
#   --lidar        also record raw /livox/lidar (enables offline FAST_LIO re-runs;
#                  LARGE — use for short targeted repro runs, not multi-day)
#   --minimal      drop /livox/imu (record only /Odometry /tf /tf_static)
#   --no-compress  disable zstd file compression
#   --split-mb N   split size in MB (default 512)
#   --topic T      add an extra topic (repeatable)
#   -h, --help     show this help
#
set -uo pipefail

ROS_DISTRO="humble"
WS_LIVOX="${HOME}/livox_ws"
WS_FASTLIO="${HOME}/fast_lio_ws"
BAG_ROOT="${HOME}/ros2_bags"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
QOS_FILE="${SCRIPT_DIR}/rosbag_record_qos.yaml"

TOPICS=(/Odometry /tf /tf_static /livox/imu)
EXTRA_TOPICS=()
WITH_LIDAR=0
WITH_CLOUD=0
COMPRESS=1
SPLIT_MB=512

usage() { sed -n '2,40p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'; }

while [ $# -gt 0 ]; do
	case "$1" in
		--cloud)       WITH_CLOUD=1 ;;
		--lidar)       WITH_LIDAR=1 ;;
		--minimal)     TOPICS=(/Odometry /tf /tf_static) ;;
		--no-compress) COMPRESS=0 ;;
		--split-mb)    shift; SPLIT_MB="${1:?--split-mb needs a number}" ;;
		--topic)       shift; EXTRA_TOPICS+=("${1:?--topic needs a name}") ;;
		-h|--help)     usage; exit 0 ;;
		*) echo "unknown arg: $1 (see --help)" >&2; exit 1 ;;
	esac
	shift
done
[ "${WITH_CLOUD}" -eq 1 ] && TOPICS+=(/cloud_registered)
[ "${WITH_LIDAR}" -eq 1 ] && TOPICS+=(/livox/lidar)
[ "${#EXTRA_TOPICS[@]}" -gt 0 ] && TOPICS+=("${EXTRA_TOPICS[@]}")

# --- ROS env (guard set -u; ROS setup.bash references unbound vars) ---
set +u
source "/opt/ros/${ROS_DISTRO}/setup.bash"
source "${WS_LIVOX}/install/setup.bash"
source "${WS_FASTLIO}/install/setup.bash"
set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"

mkdir -p "${BAG_ROOT}"
BAG_DIR="${BAG_ROOT}/fastlio_$(date +%Y%m%d_%H%M%S)"

REC_ARGS=(--output "${BAG_DIR}" --max-bag-size "$((SPLIT_MB*1024*1024))")
[ -f "${QOS_FILE}" ] && REC_ARGS+=(--qos-profile-overrides-path "${QOS_FILE}")
[ "${COMPRESS}" -eq 1 ] && REC_ARGS+=(--compression-mode file --compression-format zstd)

echo "=================================================================="
echo " rosbag logger"
echo "   topics     : ${TOPICS[*]}"
echo "   output     : ${BAG_DIR}"
echo "   split      : ${SPLIT_MB} MB   compress: $([ "${COMPRESS}" -eq 1 ] && echo zstd || echo none)"
echo "   domain     : ${ROS_DOMAIN_ID}   rmw: ${RMW_IMPLEMENTATION}"
echo "   wrapper PID: $$"
echo "   Stop cleanly with ANY of:"
echo "     - ./stop_rosbag_logger.sh"
echo "     - kill ${$}          (SIGTERM is trapped -> clean finalise)"
echo "     - Ctrl-C             (if running in the foreground)"
echo "   Do NOT use 'kill -9' — it skips finalise and can corrupt the bag."
echo "=================================================================="

# Run the recorder as a child (NOT exec) so this wrapper can translate the
# default SIGTERM from 'kill <pid>' into the SIGINT that rosbag2 needs to
# finalise the bag (write metadata.yaml). Without this, 'kill' leaves the bag
# unindexed and unreadable until 'ros2 bag reindex'.
REC_PID=""
finalize() { [ -n "${REC_PID}" ] && kill -INT "${REC_PID}" 2>/dev/null; }
trap finalize INT TERM

ros2 bag record "${REC_ARGS[@]}" "${TOPICS[@]}" &
REC_PID=$!
echo "${REC_PID}" > "${BAG_DIR}.recpid" 2>/dev/null || true

wait "${REC_PID}"               # returns early when a trapped signal arrives
wait "${REC_PID}" 2>/dev/null   # block until the recorder has fully finalised
rm -f "${BAG_DIR}.recpid" 2>/dev/null || true
echo "rosbag logger stopped; bag finalised at: ${BAG_DIR}"
