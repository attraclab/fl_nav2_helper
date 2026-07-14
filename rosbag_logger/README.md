# rosbag logger — Livox + FAST_LIO stability / drift evaluation

A manual rosbag recorder plus an analyzer, for checking that the Livox driver and
FAST_LIO stay stable over a long run and for measuring odometry drift after the
fact. Tuned for the Jetson setup (user `nvidia`, apt ROS Humble, `~/livox_ws` +
`~/fast_lio_ws`, `ROS_DOMAIN_ID=0`, CycloneDDS).

## Files

| File | Purpose |
|------|---------|
| `start_rosbag_logger.sh` | Record the diagnostic topics to `~/ros2_bags/fastlio_<timestamp>`. |
| `stop_rosbag_logger.sh` | Cleanly stop the recorder (SIGINT) and verify/reindex the bag. |
| `rosbag_record_qos.yaml` | **Required** QoS overrides — `/Odometry` and `/livox/*` are best-effort; without these the recorder captures nothing on them. |
| `analyze_bag.py` | Per-topic rate + **max-gap** (stall) report, and `/Odometry` drift summary + optional trajectory plot. |

## Record

Start the Livox driver and FAST_LIO first (your autostart / systemd units), then:

```sh
~/fast_lio/fl_nav2_ws/src/fl_nav2_helper/rosbag_logger/start_rosbag_logger.sh
```

Default profile records `/Odometry /tf /tf_static /livox/imu`, split into 512 MB
zstd-compressed files. Options:

- `--cloud` — also record `/cloud_registered` (FAST_LIO registered `PointCloud2`)
  so you can replay/visualize the map in RViz later. Measured ~0.6 MB/s
  (~30 KB/msg @ ~20 Hz) → **~2 GB/hour raw, ~1 GB/hour zstd** — fine for
  multi-hour runs, but it dominates the bag, so watch disk on long ones.
- `--lidar` — also record raw `/livox/lidar` (enables offline FAST_LIO re-runs to
  debug drift; **large**, use for short repro runs only)
- `--minimal` — drop `/livox/imu` (record only `/Odometry /tf /tf_static`)
- `--no-compress`, `--split-mb N`, `--topic T` (repeatable), `-h`

To capture the cloud for later visualization:

```sh
~/fast_lio/fl_nav2_ws/src/fl_nav2_helper/rosbag_logger/start_rosbag_logger.sh --cloud
```

### Visualize the recorded cloud later

```sh
source /opt/ros/humble/setup.bash
ros2 bag play ~/ros2_bags/fastlio_<timestamp>          # add --loop to repeat
rviz2                                                  # Fixed Frame: camera_init
#   then Add -> By topic -> /cloud_registered (PointCloud2)
```

`/cloud_registered` is already in the fixed `camera_init` frame, so it renders
without TF; the recorded `/tf` also lets you show the moving `body` frame.

### Running it in the background and stopping it

`ros2 bag record` only finalises the bag (writes `metadata.yaml`) on **SIGINT**.
The wrapper traps SIGTERM too, so any of these stop it cleanly:

```sh
# start detached (e.g. over ssh from a laptop):
nohup ~/fast_lio/fl_nav2_ws/src/fl_nav2_helper/rosbag_logger/start_rosbag_logger.sh \
      > ~/rosbag_logger.out 2>&1 &

# ...later, stop it CLEANLY with any one of:
~/fast_lio/fl_nav2_ws/src/fl_nav2_helper/rosbag_logger/stop_rosbag_logger.sh   # easiest
kill <wrapper_pid>        # SIGTERM is trapped -> forwarded as SIGINT -> clean finalise
```

**Never `kill -9`** the logger — SIGKILL can't be trapped, so the bag is left
unindexed. If that happens, recover it with `ros2 bag reindex ~/ros2_bags/fastlio_<ts>`
(`stop_rosbag_logger.sh` does this automatically).

## Analyze

```sh
source /opt/ros/humble/setup.bash
source ~/fast_lio_ws/install/setup.bash
python3 ~/fast_lio/fl_nav2_ws/src/fl_nav2_helper/rosbag_logger/analyze_bag.py \
    ~/ros2_bags/fastlio_<timestamp> --plot ~/traj.png
```

Reading it:
- **`rate(Hz)`** — expect `/Odometry` ~10, `/livox/imu` ~200, `/tf` ~10. A low
  rate means the pipeline was starved.
- **`maxgap(s)`** — the largest pause between two messages. A row flagged
  `<-- STALL` (gap > 1 s) is the "data stopped sending" failure you saw before;
  correlate its timestamp across topics to see whether the driver or FAST_LIO
  dropped out.
- **Odometry drift summary** — with the sensor stationary, `net displacement` and
  `max dist from start` should stay near 0. Growing values = drift; the X/Y/Z
  spans show which axis. The `--plot` PNG shows the XY path coloured by time
  (green = start, red = end) so you can see the direction it wandered.

## Why `/path` is not recorded

FAST_LIO's `publish_path()` does `path.poses.push_back(...)` with no cap and
re-publishes the entire growing `nav_msgs/Path` every cycle, so on a multi-hour
run each message grows into the MB range and the bag explodes. `/path` is just the
accumulated `/Odometry`, so `analyze_bag.py --plot` gives the same trajectory view
from the compact `/Odometry` stream instead.

## Notes

- Bags land in `~/ros2_bags/`; each run is a fresh timestamped directory (no
  clobber). rosbag2 does **not** auto-delete old splits — prune `~/ros2_bags/`
  yourself between long tests to reclaim space (16 GB free on the Jetson's SD).
- To autostart logging alongside the nodes later, wrap this script in a systemd
  unit like your existing `autostart_scripts/*.service` (User=nvidia, ExecStart
  pointing here, `Restart=on-failure`). Kept manual for now per request.
