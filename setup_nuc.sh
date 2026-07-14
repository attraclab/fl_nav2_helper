#!/usr/bin/env bash
#
# setup_nuc.sh
# ------------
# One-shot provisioning for a fresh Ubuntu 22.04 (x86_64) machine (e.g. Intel
# NUC) to the full robot-compute stack:
#
#   1. ROS 2 Humble desktop + colcon + rosdep + CycloneDDS rmw
#   2. GStreamer + FFmpeg, dialout group, Chromium (snap)
#   3. open3d (system pip) + numpy/transforms3d pin so nothing conflicts
#   4. OpenCV from source (core only, GStreamer+FFmpeg, Python) -> /usr/local
#   5. Livox-SDK2 + livox_ros_driver2 + FAST_LIO   (via install_livox_fastlio.sh)
#   6. micro_ros_agent in ~/micro_ros_ws
#   7. (optional) colcon build of ~/dev_ws if present
#
# Run as your normal user on the machine (it calls sudo when needed):
#   ./setup_nuc.sh
#
# Flags:
#   --jobs N          parallel build jobs (default: nproc)
#   --skip-opencv     skip the OpenCV-from-source build (step 4)
#   --skip-fastlio    skip the Livox + FAST_LIO installer (step 5)
#   --skip-microros   skip micro_ros_agent (step 6)
#   -h, --help        show this help
#
# Lessons baked in (see the NUC bring-up notes):
#   * open3d pip pulls numpy 2.x which breaks scipy 1.8 + ROS -> we pin numpy<2.
#   * numpy>=1.24 removes np.float, breaking apt python3-transforms3d 0.3.1 ->
#     we install transforms3d>=0.4.1 (used by tf_transformations / zmoab nodes).
#   * micro_ros build's rosdep wants libasio-dev -> we pre-install it.
#
set -euo pipefail

ROS_DISTRO="humble"
OPENCV_VER="4.10.0"
JOBS="$(nproc)"
SKIP_OPENCV=0
SKIP_FASTLIO=0
SKIP_MICROROS=0

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

log()  { echo -e "\n\033[1;32m==> $*\033[0m"; }
warn() { echo -e "\033[1;33m[warn] $*\033[0m"; }
die()  { echo -e "\033[1;31m[error] $*\033[0m" >&2; exit 1; }
source_ros() { set +u; # ROS setup.bash references unbound vars
	# shellcheck disable=SC1090
	source "$1"; set -u; }

while [ $# -gt 0 ]; do
	case "$1" in
		--jobs) shift; JOBS="${1:?--jobs needs a number}" ;;
		--jobs=*) JOBS="${1#*=}" ;;
		--skip-opencv) SKIP_OPENCV=1 ;;
		--skip-fastlio) SKIP_FASTLIO=1 ;;
		--skip-microros) SKIP_MICROROS=1 ;;
		-h|--help) sed -n '2,40p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'; exit 0 ;;
		*) die "unknown argument: $1 (see --help)" ;;
	esac
	shift
done
case "${JOBS}" in ''|*[!0-9]*) die "--jobs must be a positive integer" ;; esac

export DEBIAN_FRONTEND=noninteractive
export NEEDRESTART_MODE=a

########################################################################
# 1. ROS 2 Humble desktop
########################################################################
log "1/7  Base tools + ROS 2 ${ROS_DISTRO} apt repo"
sudo apt-get update
sudo apt-get install -y software-properties-common curl gnupg lsb-release \
	build-essential git wget cmake pkg-config locales
sudo add-apt-repository -y universe
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
if [ ! -f /usr/share/keyrings/ros-archive-keyring.gpg ]; then
	sudo curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
		-o /usr/share/keyrings/ros-archive-keyring.gpg
fi
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo "$UBUNTU_CODENAME") main" \
	| sudo tee /etc/apt/sources.list.d/ros2.list >/dev/null
sudo apt-get update

log "1/7  Installing ros-${ROS_DISTRO}-desktop + tools + CycloneDDS"
sudo apt-get install -y \
	ros-${ROS_DISTRO}-desktop \
	ros-dev-tools python3-colcon-common-extensions python3-rosdep python3-pip \
	ros-${ROS_DISTRO}-rmw-cyclonedds-cpp
sudo rosdep init 2>/dev/null || true
rosdep update

########################################################################
# 2. GStreamer, FFmpeg, dialout, Chromium
########################################################################
log "2/7  GStreamer + FFmpeg"
sudo apt-get install -y \
	libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev libgstreamer-plugins-bad1.0-dev \
	gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-bad \
	gstreamer1.0-plugins-ugly gstreamer1.0-libav gstreamer1.0-tools gstreamer1.0-x \
	gstreamer1.0-alsa gstreamer1.0-gl gstreamer1.0-gtk3 \
	ffmpeg libavcodec-dev libavformat-dev libavutil-dev libswscale-dev \
	libswresample-dev libavdevice-dev

log "2/7  dialout group for '${USER}' (serial port access)"
sudo usermod -aG dialout "${USER}"
warn "log out/in (or reboot) for the dialout group to take effect"

log "2/7  Chromium (snap)"
sudo apt-get install -y snapd
sudo snap install chromium || warn "chromium snap failed (check snapd)"

########################################################################
# 3. open3d + numpy/transforms3d pin (conflict-free Python stack)
########################################################################
log "3/7  open3d (system pip) + numpy<2 + transforms3d>=0.4.1 + simple_pid"
# open3d pulls numpy 2.x; the whole ROS/scientific stack needs numpy 1.x, so we
# pin it back. transforms3d>=0.4.1 is required because numpy>=1.24 removed the
# aliases the apt python3-transforms3d 0.3.1 uses.
sudo pip3 install open3d
sudo pip3 install "numpy<2"
sudo pip3 install -U "transforms3d>=0.4.1" simple_pid "numpy<2"
python3 -c "import numpy,open3d,transforms3d; print('numpy',numpy.__version__,'open3d',open3d.__version__,'transforms3d',transforms3d.__version__)"

########################################################################
# 4. OpenCV from source (core only, GStreamer + FFmpeg, Python)
########################################################################
if [ "${SKIP_OPENCV}" -eq 0 ]; then
	log "4/7  OpenCV ${OPENCV_VER} from source (core only) -> /usr/local"
	sudo apt-get install -y \
		libgtk-3-dev libavcodec-dev libavformat-dev libswscale-dev libv4l-dev \
		libxvidcore-dev libx264-dev libjpeg-dev libpng-dev libtiff-dev libwebp-dev \
		libtbb-dev libeigen3-dev libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
		python3-dev python3-numpy
	cd "${HOME}"
	[ -d opencv ] || git clone --depth 1 -b "${OPENCV_VER}" https://github.com/opencv/opencv.git
	rm -rf "${HOME}/opencv/build"; mkdir -p "${HOME}/opencv/build"; cd "${HOME}/opencv/build"
	cmake .. \
		-DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local \
		-DWITH_GSTREAMER=ON -DWITH_FFMPEG=ON -DWITH_V4L=ON \
		-DWITH_TBB=ON -DWITH_OPENGL=ON -DWITH_GTK=ON \
		-DBUILD_opencv_python3=ON -DPYTHON3_EXECUTABLE="$(which python3)" \
		-DINSTALL_PYTHON_EXAMPLES=OFF -DINSTALL_C_EXAMPLES=OFF \
		-DBUILD_EXAMPLES=OFF -DBUILD_TESTS=OFF -DBUILD_PERF_TESTS=OFF \
		-DBUILD_DOCS=OFF -DOPENCV_GENERATE_PKGCONFIG=ON
	make -j"${JOBS}"
	sudo make install
	sudo ldconfig
	# NOTE: python cv2 is now the /usr/local build; ROS cv_bridge keeps apt OpenCV.
	python3 -c "import cv2; print('cv2', cv2.__version__)"
else
	warn "skipping OpenCV build (--skip-opencv)"
fi

########################################################################
# 5. Livox + FAST_LIO
########################################################################
if [ "${SKIP_FASTLIO}" -eq 0 ]; then
	log "5/7  Livox-SDK2 + livox_ros_driver2 + FAST_LIO (install_livox_fastlio.sh)"
	[ -x "${SCRIPT_DIR}/install_livox_fastlio.sh" ] \
		|| die "install_livox_fastlio.sh not found beside this script"
	"${SCRIPT_DIR}/install_livox_fastlio.sh" --jobs "${JOBS}"
else
	warn "skipping Livox + FAST_LIO (--skip-fastlio)"
fi

########################################################################
# 6. micro_ros_agent
########################################################################
if [ "${SKIP_MICROROS}" -eq 0 ]; then
	log "6/7  micro_ros_agent in ~/micro_ros_ws"
	sudo apt-get install -y libasio-dev   # rosdep for the agent (avoids a mid-build stall)
	mkdir -p "${HOME}/micro_ros_ws/src"
	cd "${HOME}/micro_ros_ws"
	[ -d src/micro_ros_setup ] || git clone -b "${ROS_DISTRO}" \
		https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup
	source_ros "/opt/ros/${ROS_DISTRO}/setup.bash"
	rosdep install --from-paths src --ignore-src -y || true
	colcon build
	source_ros "${HOME}/micro_ros_ws/install/local_setup.bash"
	ros2 run micro_ros_setup create_agent_ws.sh
	ros2 run micro_ros_setup build_agent.sh
	source_ros "${HOME}/micro_ros_ws/install/local_setup.bash"
	ros2 pkg executables micro_ros_agent || warn "micro_ros_agent executable not found"
else
	warn "skipping micro_ros_agent (--skip-microros)"
fi

########################################################################
# 7. Optional: build ~/dev_ws if present
########################################################################
if [ -d "${HOME}/dev_ws/src" ]; then
	log "7/7  Building ~/dev_ws (colcon --symlink-install)"
	cd "${HOME}/dev_ws"
	source_ros "/opt/ros/${ROS_DISTRO}/setup.bash"
	rosdep install --from-paths src --ignore-src -y || warn "some rosdeps failed"
	colcon build --symlink-install
else
	warn "no ~/dev_ws/src -> skipping dev_ws build"
fi

########################################################################
log "Setup complete"
cat <<EOF

Next steps:
  * source /opt/ros/${ROS_DISTRO}/setup.bash
    source ~/livox_ws/install/setup.bash
    source ~/fast_lio_ws/install/setup.bash
    (and ~/micro_ros_ws/install/local_setup.bash, ~/dev_ws/install/setup.bash)
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ; export ROS_DOMAIN_ID=0
  * Edit ~/livox_ws/src/livox_ros_driver2/config/MID360_config.json for your IPs.
  * dialout group needs a re-login to take effect.
EOF
