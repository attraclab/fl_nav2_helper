#!/usr/bin/env bash
#
# stop_rosbag_logger.sh
# ---------------------
# Cleanly stop a running rosbag logger started by start_rosbag_logger.sh.
# Sends SIGINT (never SIGKILL) to the `ros2 bag record` process so rosbag2
# finalises the bag (writes metadata.yaml). Then verifies the newest bag is
# readable and, if a hard kill previously left it unindexed, reindexes it.
#
set -uo pipefail
ROS_DISTRO="humble"
BAG_ROOT="${HOME}/ros2_bags"

pids="$(pgrep -f 'ros2 bag record' || true)"
if [ -z "${pids}" ]; then
	echo "No 'ros2 bag record' process is running."
else
	echo "Sending SIGINT (clean finalise) to: ${pids}"
	# shellcheck disable=SC2086
	kill -INT ${pids}
	# wait up to ~15 s for a clean exit
	for _ in $(seq 1 30); do
		pgrep -f 'ros2 bag record' >/dev/null || break
		sleep 0.5
	done
	if pgrep -f 'ros2 bag record' >/dev/null; then
		echo "Recorder still running after 15 s — check manually (do NOT kill -9)."
	else
		echo "Recorder stopped cleanly."
	fi
fi

# Verify the newest bag finalised; recover it if a previous hard kill didn't.
BAG="$(ls -dt "${BAG_ROOT}"/fastlio_* 2>/dev/null | head -1 || true)"
if [ -n "${BAG}" ]; then
	if [ -f "${BAG}/metadata.yaml" ]; then
		echo "Latest bag OK: ${BAG}"
	else
		echo "metadata.yaml missing in ${BAG} — attempting 'ros2 bag reindex'..."
		set +u; source "/opt/ros/${ROS_DISTRO}/setup.bash"; set -u
		if ros2 bag reindex "${BAG}" >/dev/null 2>&1 && [ -f "${BAG}/metadata.yaml" ]; then
			echo "Reindexed OK: ${BAG}"
		else
			echo "Reindex failed — the bag may be incomplete."
		fi
	fi
fi
