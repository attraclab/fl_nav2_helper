# Livox + FAST_LIO install notes (Jetson Orin Nano / Intel NUC)

Companion to [`install_livox_fastlio.sh`](./install_livox_fastlio.sh). Target: a
fresh device that **already has ROS 2 Humble from apt** at `/opt/ros/humble`.
Same script works on both Jetson Orin Nano (aarch64) and Intel NUC (x86_64) —
everything below is built from source, so there is nothing arch-specific.

## Running it

```sh
./install_livox_fastlio.sh --dry-run   # print the plan, verify patch files, make NO changes
./install_livox_fastlio.sh             # the real run (calls sudo when needed)
./install_livox_fastlio.sh --help      # usage
```

**`--dry-run` (alias `--check`)** does no apt / sudo / compiling. It prints every
version + repo it would install, the toggle states (`INSTALL_EIGEN_FROM_SOURCE`,
`PCL_FROM_SOURCE`), and verifies the three patched source files resolve on disk —
then exits. It's safe to run on a dev PC without ROS or the lidar attached (a
missing `/opt/ros/humble` is only a warning in dry-run; missing patch files are
still a hard error). Use it as a pre-flight on the Jetson/NUC right before the
real run to confirm ROS is present and the patches are in place.

## What the script installs

| Component | Source | Installed to | Notes |
|-----------|--------|--------------|-------|
| Eigen `3.4.0` | GitLab release, from source | `/usr/local` | The drift fix. `pkg_compare` confirms GitLab==working "lab"; GitHub Eigen is a different snapshot. `find_package(Eigen3)` prefers `/usr/local`. |
| PCL `1.10.0` | source, `-DWITH_VTK=OFF` | `/usr/local/share/pcl-1.10` | Matches your working machines (`pkg_compare/pcl`). Apt on Jammy is 1.12, so we build 1.10. Long build. |
| pcl_conversions / pcl_ros | `ros-perception/perception_pcl` (`humble`) | `~/fast_lio_ws` | Built from source **against PCL 1.10** so nothing links apt 1.12. |
| Livox-SDK2 | `github.com/Livox-SDK/Livox-SDK2` | `/usr/local` (system lib) | Required by the MID360 driver. `ldconfig` run after install. |
| livox_ros_driver2 | `github.com/Livox-SDK/livox_ros_driver2` | `~/livox_ws` | **Your** best-effort `lddc.cpp` / `lddc.h` copied in before build. |
| FAST_LIO (ROS 2) | `github.com/Ericsii/FAST_LIO` (`ros2`) | `~/fast_lio_ws` | **Your** best-effort `laserMapping.cpp` copied in. Built `Release`, forced `-DPCL_DIR=/usr/local/share/pcl-1.10`. |
| apt deps | apt | system | `colcon-common-extensions`, `rosdep`, `rmw-cyclonedds-cpp`, `tf2*`, `pcl-msgs`, `message-filters`, `tf2-eigen/-sensor-msgs`, `libboost-all-dev`, `libflann-dev`, `libqhull-dev`, `python3-dev`, build tools. **No apt PCL.** (Jetson Orin shipped without colcon.) |

Workspace layout matches your existing `simple_start/*.sh` (`~/livox_ws`,
`~/fast_lio_ws`, CycloneDDS, `ROS_DOMAIN_ID=0`).

## The QoS patch — and whether best-effort is the right call

Your instinct is correct and matches ROS 2 convention:

- High-rate sensor streams (raw pointclouds, LIO odometry) should use
  **best-effort**. ROS 2 even ships `rclcpp::SensorDataQoS()` for exactly this —
  which is what your patched `laserMapping.cpp` already uses. Your `lddc.cpp`
  does the equivalent with `custom_qos.best_effort().durability_volatile()`.
- Reliable QoS on a 10 Hz dense pointcloud forces retransmit/ACK bookkeeping;
  under DDS load that is where you saw data "stop sending". Best-effort drops a
  late sample instead of stalling the pipeline — the right trade for LIO, which
  only cares about the freshest scan.

Two additions worth considering on top of best-effort (bigger wins than QoS
alone, optional — not done by the script):

1. **Keep depth shallow.** `SensorDataQoS` is depth 5; for the densest topics
   depth 1 avoids latency build-up if a consumer briefly lags.
2. **Publish less.** FAST_LIO can emit `/cloud_registered`, `/cloud_registered_body`,
   `/cloud_effected`, `/Laser_map`. If you don't consume a topic, disable it in
   the config — not publishing a dense cloud beats any QoS tuning.
3. **CycloneDDS shared memory (Iceoryx)** for the same-host driver→LIO hop moves
   large pointclouds without serializing over the network stack. This is the
   heavier-lift option if DDS traffic is still a bottleneck after the above.

## PCL 1.10 from source — and the version-mixing trap

Your working machines run **PCL 1.10** (`pkg_compare/pcl/note.txt`), but Ubuntu
22.04 apt ships **1.12**. The catch: apt `ros-humble-pcl-conversions` /
`pcl-ros` are compiled against 1.12 and depend on `libpcl-dev`, so installing
them would pull 1.12 back onto the system alongside your 1.10 — exactly the kind
of mixed-version state your comparison work is trying to avoid.

So the script keeps a **single PCL version**:

- Builds PCL `1.10.0` from source to `/usr/local` with `-DWITH_VTK=OFF`
  (FAST_LIO uses only voxel filter / PCD IO / point types — no VTK needed).
- Does **not** install any apt PCL package.
- Clones `perception_pcl` (`humble`) into `~/fast_lio_ws/src` and builds
  `pcl_conversions` / `pcl_ros` **in the same colcon invocation as FAST_LIO**,
  with `-DPCL_DIR=/usr/local/share/pcl-1.10`, so every consumer compiles and
  links against 1.10.

Toggle in the script: `PCL_FROM_SOURCE=0` reverts to the simpler apt PCL 1.12 +
apt `pcl_conversions`/`pcl_ros` path if you ever want it.

**Not compile-tested here** (no ROS/hardware on my side): building `humble`
`perception_pcl` against PCL 1.10 is known-good on your source setup, but if a
minor API mismatch appears on 22.04, the fastest fallback is
`--packages-select pcl_conversions` (header-only, always compatible) since
FAST_LIO's actual include is only `pcl_conversions/pcl_conversions.h`.

## Post-install configuration

Set these three things for your robot before running.

### 1. Lidar + host IP in `MID360_config.json`

Edit the MID360 network config so it matches your network — the **lidar's IP** and
**this host computer's IP** on the lidar network:

```
~/livox_ws/src/livox_ros_driver2/config/MID360_config.json
```
```jsonc
"host_net_info" : {
    "cmd_data_ip"  : "192.168.1.5",   // <- THIS computer's IP (all *_ip fields)
    ...
},
"lidar_configs" : [
    { "ip" : "192.168.1.170", ... }   // <- the MID360's IP
]
```

> The node reads the **installed** copy
> (`~/livox_ws/install/livox_ros_driver2/share/livox_ros_driver2/config/MID360_config.json`),
> which livox's `build.sh` *copies* (not symlinks). So edit `src` **and rebuild the
> driver**, or edit the installed copy directly. A wrong/stale IP shows up as
> `Can not get index`, `Storage point data failed`,
> `found lidar not defined in the user-defined config` and **no `/livox/*` topics**
> (see gotcha #1).

### 2. `map_file_path` in `mid360.yaml`

If you save/load a PCD map (`pcd_save_en: true`), point `map_file_path` at the
correct absolute path:

```
~/fast_lio_ws/src/FAST_LIO/config/mid360.yaml
```
```yaml
common:
    map_file_path: "/home/ubuntu/pcl_gps_map/3dmap.pcd"   # <- your path
```

### 3. Disable RViz in `mapping.launch.py` (headless robots)

On a headless robot you usually don't want FAST_LIO to also launch RViz. Edit

```
~/fast_lio_ws/src/FAST_LIO/launch/mapping.launch.py
```

and comment out the three RViz `add_action` lines at the bottom of
`generate_launch_description()` so only the FAST_LIO node starts:

```python
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_config_path_cmd)
    ld.add_action(declare_config_file_cmd)
    #ld.add_action(declare_rviz_cmd)
    #ld.add_action(declare_rviz_config_path_cmd)

    ld.add_action(fast_lio_node)
    #ld.add_action(rviz_node)

    return ld
```

Leave the `rviz_node` / `declare_rviz_*` definitions in place — just don't add
them to the `LaunchDescription`. FAST_LIO is built `--symlink-install`, so editing
the `src` launch file takes effect immediately (the installed copy is a symlink).

> Note: the installer already auto-fixes a *broken* upstream variant of this file
> (see gotcha #2). This step is the separate, intentional "don't start RViz"
> change — do it if RViz still opens after install.

## After install

```sh
# configure MID360_config.json / mid360.yaml / mapping.launch.py first
# (see "Post-install configuration" above)

source /opt/ros/humble/setup.bash
source ~/livox_ws/install/setup.bash
source ~/fast_lio_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=0

ros2 launch livox_ros_driver2 msg_MID360_launch.py
ros2 launch fast_lio mapping.launch.py config_file:=mid360.yaml
```

Verified end-to-end on a Jetson Orin Nano (MID360): driver publishes `/livox/lidar`
(~4.7 Hz accumulated) + `/livox/imu` (200 Hz); FAST_LIO publishes `/Odometry` at a
steady 10 Hz (frames `camera_init`→`body`) plus `/cloud_registered`, `/path`, `/tf`.

## Runtime gotchas (hit on real hardware)

1. **The driver reads the *installed* `MID360_config.json`, not the `src` one.**
   livox's `build.sh` *copies* the config into
   `~/livox_ws/install/livox_ros_driver2/share/livox_ros_driver2/config/` (no
   symlink), so editing only the `src` copy has no effect until you rebuild.
   Symptoms of a wrong/stale lidar IP: `Can not get index`, `Storage point data
   failed`, `found lidar not defined in the user-defined config, ip: …`, and **no
   `/livox/*` topics**. Fix: put the real lidar IP + this host's IP in the config
   and either rebuild the driver *or* edit the installed copy directly.
2. **Upstream `mapping.launch.py` can ship broken.** On the `ros2` branch, rviz
   was hand-disabled but a continuation line (`package_path, 'rviz', 'fastlio.rviz')`)
   was left dangling → `IndentationError: unexpected indent`. The installer now
   auto-detects this (parses the file; only patches if it fails) and comments the
   line. FAST_LIO is built `--symlink-install`, so the installed launch file is a
   symlink to `src` — fixing one fixes both.

## Re-running / recovery

- The script is re-runnable: existing clones are reused, and upstream
  `lddc.cpp` / `lddc.h` / `laserMapping.cpp` are backed up once as `*.orig`
  before your versions overwrite them.
- Run `--dry-run` first to confirm the plan and that the three patch files
  resolve before committing to the long PCL build.
- If the driver fails to compile because upstream `livox_ros_driver2` changed
  since you made your patch, restore `src/lddc.cpp.orig` and re-apply the
  `best_effort().durability_volatile()` change to the current file.
- `FASTLIO_BRANCH` / repo URLs / `INSTALL_EIGEN_FROM_SOURCE` / `JOBS` are
  variables at the top of the script. On an 8 GB Jetson, drop `JOBS` (e.g. `2`)
  if the FAST_LIO/PCL build runs out of memory.
