#!/usr/bin/env bash
#
# install_livox_fastlio.sh
# -------------------------
# Install Livox-SDK2 + livox_ros_driver2 + FAST_LIO (ROS 2 Humble port)
# on a fresh Jetson Orin Nano or Intel NUC that already has ROS 2 Humble
# installed from apt at /opt/ros/humble.
#
# It also:
#   - installs Eigen 3.4.0 from source (apt/GitHub versions caused odom drift),
#   - copies YOUR best-effort-QoS patched files over the upstream sources:
#       fl_nav2_helper/livox_note/lddc.cpp   -> livox_ros_driver2/src/lddc.cpp
#       fl_nav2_helper/livox_note/lddc.h     -> livox_ros_driver2/src/lddc.h
#       fl_nav2_helper/fast_lio_note/laserMapping.cpp -> FAST_LIO/src/laserMapping.cpp
#
# Run it as your normal user (it will call sudo when needed):
#   ./install_livox_fastlio.sh
#
set -euo pipefail

############################################
# Config — edit here if needed
############################################
ROS_DISTRO="humble"
EIGEN_VERSION="3.4.0"
INSTALL_EIGEN_FROM_SOURCE=1          # set 0 to skip and use apt libeigen3-dev

# PCL: your working machines run 1.10 (pkg_compare/pcl/note.txt). Ubuntu 22.04
# apt ships 1.12, so we build 1.10.0 from source (VTK OFF) plus a matching
# pcl_conversions/pcl_ros from source, and DO NOT install apt PCL. This keeps a
# single, consistent PCL version like your reference setup.
PCL_FROM_SOURCE=1                    # set 0 to use apt PCL 1.12 + apt pcl_conversions instead
PCL_VERSION="1.10.0"
PCL_SRC="${HOME}/pcl-${PCL_VERSION}"
PERCEPTION_PCL_REPO="https://github.com/ros-perception/perception_pcl.git"
PERCEPTION_PCL_BRANCH="humble"

WS_LIVOX="${HOME}/livox_ws"          # matches your simple_start/*.sh scripts
WS_FASTLIO="${HOME}/fast_lio_ws"
SDK2_SRC="${HOME}/Livox-SDK2"

LIVOX_SDK2_REPO="https://github.com/Livox-SDK/Livox-SDK2.git"
LIVOX_DRIVER_REPO="https://github.com/Livox-SDK/livox_ros_driver2.git"
FASTLIO_REPO="https://github.com/Ericsii/FAST_LIO.git"   # ROS 2 port (mapping.launch.py + mid360.yaml)
FASTLIO_BRANCH="ros2"

JOBS="$(nproc)"                      # lower this (e.g. 2) on an 8 GB Jetson if the build OOMs

DRY_RUN=0                            # set by --dry-run/--check; prints plan + verifies patches, no changes

############################################
# Paths to YOUR modified files (this script lives INSIDE the fl_nav2_helper
# package, so the patched sources sit in subdirs right beside it)
############################################
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
HELPER_DIR="${SCRIPT_DIR}"
MOD_LDDC_CPP="${HELPER_DIR}/livox_note/lddc.cpp"
MOD_LDDC_H="${HELPER_DIR}/livox_note/lddc.h"
MOD_LASERMAPPING="${HELPER_DIR}/fast_lio_note/laserMapping.cpp"

############################################
# Helpers
############################################
log()  { echo -e "\n\033[1;32m==> $*\033[0m"; }
warn() { echo -e "\033[1;33m[warn] $*\033[0m"; }
die()  { echo -e "\033[1;31m[error] $*\033[0m" >&2; exit 1; }

source_ros() {
	# Source a ROS/colcon setup file safely. Their setup.bash references unset
	# vars (e.g. AMENT_TRACE_SETUP_FILES), which trips this script's `set -u`,
	# so we disable nounset just around the source.
	local f="$1"
	set +u
	# shellcheck disable=SC1090
	source "$f"
	set -u
}

backup_then_copy() {
	# backup_then_copy <src_modified> <dst_upstream>
	local src="$1" dst="$2"
	[ -f "$src" ] || die "modified file not found: $src"
	[ -f "$dst" ] || die "upstream target not found (repo layout changed?): $dst"
	if [ ! -f "${dst}.orig" ]; then
		cp -v "$dst" "${dst}.orig"
	fi
	cp -v "$src" "$dst"
}

############################################
# Argument parsing
############################################
usage() {
	cat <<EOF
Usage: ${0##*/} [--dry-run|--check] [--jobs N] [-h|--help]

  --dry-run, --check   Print the versions/repos this script would install,
                       report the toggle states, and verify the three patched
                       source files resolve on disk. Makes NO changes: no apt,
                       no sudo, no compiling. Safe to run on a dev PC without
                       ROS or the lidar attached.
  --jobs N, --jobs=N   Parallel build jobs (default: nproc = ${JOBS}). Lower it
                       (e.g. 2) on an 8 GB Jetson so the PCL/FAST_LIO build does
                       not OOM.
  -h, --help           Show this help and exit.
EOF
}

while [ $# -gt 0 ]; do
	case "$1" in
		--dry-run|--check) DRY_RUN=1 ;;
		--jobs) shift; [ $# -gt 0 ] || die "--jobs needs a number"; JOBS="$1" ;;
		--jobs=*) JOBS="${1#*=}" ;;
		-h|--help) usage; exit 0 ;;
		*) die "unknown argument: $1 (see --help)" ;;
	esac
	shift
done

case "${JOBS}" in
	''|*[!0-9]*) die "--jobs must be a positive integer, got: ${JOBS}" ;;
	0) die "--jobs must be >= 1" ;;
esac

############################################
# 0. Sanity checks
############################################
log "Checking prerequisites"
if [ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
	echo "ROS 2 ${ROS_DISTRO} : found at /opt/ros/${ROS_DISTRO}"
elif [ "${DRY_RUN}" -eq 1 ]; then
	warn "ROS 2 ${ROS_DISTRO} not found at /opt/ros/${ROS_DISTRO} — OK for --dry-run, but required for a real run"
else
	die "ROS 2 ${ROS_DISTRO} not found at /opt/ros/${ROS_DISTRO}"
fi

# The three patched source files must resolve regardless of mode.
missing=0
for f in "$MOD_LDDC_CPP" "$MOD_LDDC_H" "$MOD_LASERMAPPING"; do
	if [ -f "$f" ]; then
		echo "patch file OK    : $f"
	else
		warn "patch file MISSING: $f"
		missing=1
	fi
done
[ "${missing}" -eq 0 ] || die "one or more patched source files are missing (run this script from inside the fl_nav2_helper package)"

echo "ROS distro   : ${ROS_DISTRO}"
echo "Livox ws     : ${WS_LIVOX}"
echo "FAST_LIO ws  : ${WS_FASTLIO}"
echo "Parallel jobs: ${JOBS}"

############################################
# 0.5 Dry run — report the plan and exit before touching anything
############################################
if [ "${DRY_RUN}" -eq 1 ]; then
	log "DRY RUN — a real run would install (no changes made):"
	cat <<EOF
  Eigen             : ${EIGEN_VERSION}  (from source: ${INSTALL_EIGEN_FROM_SOURCE})
                      https://gitlab.com/libeigen/eigen/-/archive/${EIGEN_VERSION}/eigen-${EIGEN_VERSION}.tar.gz
  PCL               : ${PCL_VERSION}  (from source: ${PCL_FROM_SOURCE}, VTK OFF)
                      https://github.com/PointCloudLibrary/pcl (tag pcl-${PCL_VERSION})
  perception_pcl    : ${PERCEPTION_PCL_REPO} (branch ${PERCEPTION_PCL_BRANCH})  [only when PCL_FROM_SOURCE=1]
  Livox-SDK2        : ${LIVOX_SDK2_REPO}
  livox_ros_driver2 : ${LIVOX_DRIVER_REPO}
  FAST_LIO          : ${FASTLIO_REPO} (branch ${FASTLIO_BRANCH})

  Workspaces        : livox=${WS_LIVOX}  fastlio=${WS_FASTLIO}  sdk=${SDK2_SRC}

  Patch files (verified above) would overwrite upstream, backing up as *.orig:
    ${MOD_LDDC_CPP}
        -> ${WS_LIVOX}/src/livox_ros_driver2/src/lddc.cpp
    ${MOD_LDDC_H}
        -> ${WS_LIVOX}/src/livox_ros_driver2/src/lddc.h
    ${MOD_LASERMAPPING}
        -> ${WS_FASTLIO}/src/FAST_LIO/src/laserMapping.cpp

Re-run without --dry-run to install.
EOF
	exit 0
fi

############################################
# 1. apt dependencies
############################################
log "Installing apt dependencies"
sudo apt-get update
sudo apt-get install -y \
	build-essential cmake git wget \
	python3-dev python3-numpy \
	python3-colcon-common-extensions python3-rosdep \
	ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
	ros-${ROS_DISTRO}-tf2 \
	ros-${ROS_DISTRO}-tf2-ros \
	ros-${ROS_DISTRO}-tf2-geometry-msgs

if [ "${PCL_FROM_SOURCE}" -eq 1 ]; then
	# Build deps for PCL 1.10 (VTK/viz off) + deps to build pcl_conversions/pcl_ros
	# from source. Deliberately NOT installing libpcl-dev / ros-*-pcl-conversions
	# / ros-*-pcl-ros so apt PCL 1.12 never lands on the system.
	sudo apt-get install -y \
		libboost-all-dev libflann-dev libqhull-dev \
		ros-${ROS_DISTRO}-pcl-msgs \
		ros-${ROS_DISTRO}-message-filters \
		ros-${ROS_DISTRO}-tf2-eigen \
		ros-${ROS_DISTRO}-tf2-sensor-msgs
else
	warn "PCL_FROM_SOURCE=0 -> installing apt PCL 1.12 + pcl_conversions/pcl_ros"
	sudo apt-get install -y \
		libpcl-dev \
		ros-${ROS_DISTRO}-pcl-conversions \
		ros-${ROS_DISTRO}-pcl-ros
fi

############################################
# 2. Eigen 3.4.0 from source (the drift fix)
#    Installs to /usr/local, which CMake's find_package(Eigen3)
#    prefers over the apt copy in /usr.
############################################
if [ "${INSTALL_EIGEN_FROM_SOURCE}" -eq 1 ]; then
	log "Installing Eigen ${EIGEN_VERSION} from source"
	TMP_EIGEN="$(mktemp -d)"
	wget -qO "${TMP_EIGEN}/eigen.tar.gz" \
		"https://gitlab.com/libeigen/eigen/-/archive/${EIGEN_VERSION}/eigen-${EIGEN_VERSION}.tar.gz"
	tar -xf "${TMP_EIGEN}/eigen.tar.gz" -C "${TMP_EIGEN}"
	mkdir -p "${TMP_EIGEN}/eigen-${EIGEN_VERSION}/build"
	cd "${TMP_EIGEN}/eigen-${EIGEN_VERSION}/build"
	cmake .. -DCMAKE_INSTALL_PREFIX=/usr/local >/dev/null
	sudo make install >/dev/null      # header-only: just installs headers + Eigen3Config.cmake
	rm -rf "${TMP_EIGEN}"
	echo "Eigen ${EIGEN_VERSION} installed to /usr/local/include/eigen3"
else
	warn "Skipping source Eigen; using apt libeigen3-dev"
	sudo apt-get install -y libeigen3-dev
fi

############################################
# 2.5 PCL 1.10.0 from source (match your working machines; VTK OFF)
#     Installs to /usr/local/share/pcl-1.10. FAST_LIO and the source
#     pcl_conversions/pcl_ros are pointed at this via PCL_DIR later.
#     NOTE: this is a long build (~20-40 min).
############################################
if [ "${PCL_FROM_SOURCE}" -eq 1 ]; then
	if [ -d "/usr/local/share/pcl-1.10" ]; then
		log "PCL 1.10 already installed at /usr/local/share/pcl-1.10 (skipping build)"
	else
		log "Building PCL ${PCL_VERSION} from source (VTK OFF)"
		if [ ! -d "${PCL_SRC}" ]; then
			wget -qO "/tmp/pcl-${PCL_VERSION}.tar.gz" \
				"https://github.com/PointCloudLibrary/pcl/archive/refs/tags/pcl-${PCL_VERSION}.tar.gz"
			tar -xf "/tmp/pcl-${PCL_VERSION}.tar.gz" -C "${HOME}"
			# archive extracts as pcl-pcl-1.10.0
			mv "${HOME}/pcl-pcl-${PCL_VERSION}" "${PCL_SRC}"
			rm -f "/tmp/pcl-${PCL_VERSION}.tar.gz"
		fi
		mkdir -p "${PCL_SRC}/build"
		cd "${PCL_SRC}/build"
		cmake .. \
			-DCMAKE_BUILD_TYPE=Release \
			-DCMAKE_INSTALL_PREFIX=/usr/local \
			-DWITH_VTK=OFF \
			-DBUILD_visualization=OFF \
			-DBUILD_apps=OFF \
			-DBUILD_examples=OFF \
			-DBUILD_tools=OFF \
			-DBUILD_GPU=OFF \
			-DBUILD_CUDA=OFF \
			-DWITH_OPENNI=OFF \
			-DWITH_OPENNI2=OFF \
			-DWITH_QT=OFF \
			-DWITH_PCAP=OFF
		make -j"${JOBS}"
		sudo make install
		sudo ldconfig
		echo "PCL ${PCL_VERSION} installed to /usr/local (config: /usr/local/share/pcl-1.10)"
	fi
fi

############################################
# 3. Livox-SDK2 (required by livox_ros_driver2 for MID360)
############################################
log "Building Livox-SDK2"
if [ ! -d "${SDK2_SRC}" ]; then
	git clone "${LIVOX_SDK2_REPO}" "${SDK2_SRC}"
else
	echo "Livox-SDK2 already cloned at ${SDK2_SRC}"
fi
mkdir -p "${SDK2_SRC}/build"
cd "${SDK2_SRC}/build"
cmake .. >/dev/null
make -j"${JOBS}"
sudo make install
sudo ldconfig

############################################
# 4. livox_ros_driver2  (+ your best-effort QoS patch)
############################################
log "Building livox_ros_driver2 with your best-effort QoS patch"
mkdir -p "${WS_LIVOX}/src"
if [ ! -d "${WS_LIVOX}/src/livox_ros_driver2" ]; then
	git clone "${LIVOX_DRIVER_REPO}" "${WS_LIVOX}/src/livox_ros_driver2"
else
	echo "livox_ros_driver2 already cloned"
fi

backup_then_copy "${MOD_LDDC_CPP}" "${WS_LIVOX}/src/livox_ros_driver2/src/lddc.cpp"
backup_then_copy "${MOD_LDDC_H}"   "${WS_LIVOX}/src/livox_ros_driver2/src/lddc.h"

source_ros "/opt/ros/${ROS_DISTRO}/setup.bash"
cd "${WS_LIVOX}/src/livox_ros_driver2"
./build.sh "${ROS_DISTRO}"           # copies package_ROS2.xml -> package.xml and colcon builds ${WS_LIVOX}

############################################
# 5. FAST_LIO (ROS 2)  (+ your best-effort QoS laserMapping)
############################################
log "Building FAST_LIO (ROS 2) with your best-effort QoS laserMapping"
mkdir -p "${WS_FASTLIO}/src"
if [ ! -d "${WS_FASTLIO}/src/FAST_LIO" ]; then
	git clone --recursive -b "${FASTLIO_BRANCH}" "${FASTLIO_REPO}" "${WS_FASTLIO}/src/FAST_LIO"
else
	echo "FAST_LIO already cloned"
fi

backup_then_copy "${MOD_LASERMAPPING}" "${WS_FASTLIO}/src/FAST_LIO/src/laserMapping.cpp"

# The Ericsii ros2 branch ships a broken launch/mapping.launch.py: rviz was
# hand-disabled but the continuation line (`package_path, 'rviz', 'fastlio.rviz')`)
# was left dangling -> IndentationError at launch time. Only patch if the file
# actually fails to parse (so a fixed upstream is left untouched); the fix just
# comments that dangling rviz line, and is idempotent.
FASTLIO_LAUNCH="${WS_FASTLIO}/src/FAST_LIO/launch/mapping.launch.py"
parses() { python3 -c "import ast,sys; ast.parse(open(sys.argv[1]).read())" "$1" 2>/dev/null; }
if [ -f "${FASTLIO_LAUNCH}" ] && ! parses "${FASTLIO_LAUNCH}"; then
	warn "mapping.launch.py fails to parse; commenting the dangling rviz line"
	[ -f "${FASTLIO_LAUNCH}.orig" ] || cp "${FASTLIO_LAUNCH}" "${FASTLIO_LAUNCH}.orig"
	sed -i "/fastlio\.rviz')/ { /^[[:space:]]*#/! s/^/#/ }" "${FASTLIO_LAUNCH}"
	if parses "${FASTLIO_LAUNCH}"; then
		echo "mapping.launch.py patched and now parses OK"
	else
		warn "mapping.launch.py still does not parse after fix — inspect manually"
	fi
fi

# When building PCL from source, also build pcl_conversions/pcl_ros from source
# in the SAME workspace so they compile against PCL 1.10 (not apt 1.12).
CMAKE_ARGS=(-DCMAKE_BUILD_TYPE=Release)
if [ "${PCL_FROM_SOURCE}" -eq 1 ]; then
	if [ ! -d "${WS_FASTLIO}/src/perception_pcl" ]; then
		git clone -b "${PERCEPTION_PCL_BRANCH}" "${PERCEPTION_PCL_REPO}" \
			"${WS_FASTLIO}/src/perception_pcl"
	else
		echo "perception_pcl already cloned"
	fi
	CMAKE_ARGS+=(-DPCL_DIR=/usr/local/share/pcl-1.10)
	export CMAKE_PREFIX_PATH="/usr/local:${CMAKE_PREFIX_PATH:-}"
fi

# FAST_LIO needs the livox CustomMsg interface -> source the livox workspace first.
source_ros "/opt/ros/${ROS_DISTRO}/setup.bash"
source_ros "${WS_LIVOX}/install/setup.bash"
cd "${WS_FASTLIO}"
colcon build --symlink-install --parallel-workers "${JOBS}" \
	--cmake-args "${CMAKE_ARGS[@]}"

############################################
# 6. Done
############################################
log "Install complete"
cat <<EOF

Installed:
  * Eigen ${EIGEN_VERSION}           -> /usr/local (GitLab release; find_package prefers this over apt)
  * PCL ${PCL_VERSION} (VTK OFF)   -> /usr/local  [PCL_FROM_SOURCE=${PCL_FROM_SOURCE}]
  * pcl_conversions/pcl_ros    -> built from source in ${WS_FASTLIO} against PCL 1.10 (when PCL_FROM_SOURCE=1)
  * Livox-SDK2                 -> /usr/local (system lib), source at ${SDK2_SRC}
  * livox_ros_driver2 (patched)-> ${WS_LIVOX}   (best-effort QoS, upstream backed up as *.orig)
  * FAST_LIO ROS2 (patched)    -> ${WS_FASTLIO} (best-effort QoS, upstream backed up as *.orig)
  * apt: rmw-cyclonedds-cpp, tf2*, pcl-msgs, message-filters, boost/flann/qhull

Before running:
  1. Edit the MID360 network config for THIS machine's IP + the lidar IP:
       ${WS_LIVOX}/src/livox_ros_driver2/config/MID360_config.json
  2. Source in this order (as in your simple_start/ scripts):
       source /opt/ros/${ROS_DISTRO}/setup.bash
       source ${WS_LIVOX}/install/setup.bash
       source ${WS_FASTLIO}/install/setup.bash
     and (optionally) export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ; export ROS_DOMAIN_ID=0
  3. Launch:
       ros2 launch livox_ros_driver2 msg_MID360_launch.py
       ros2 launch fast_lio mapping.launch.py config_file:=mid360.yaml

If livox_ros_driver2 fails to compile because upstream changed since you patched
lddc.cpp/lddc.h, restore the backup (mv src/lddc.cpp.orig src/lddc.cpp) and
re-apply the best_effort().durability_volatile() change to the current version.
EOF
