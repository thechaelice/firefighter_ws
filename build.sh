#!/usr/bin/env bash
#
# build.sh - memory/IO-safe colcon build wrapper for firefighter_ws (Raspberry Pi 4)
#
# ---------------------------------------------------------------------------
# WHY THIS EXISTS  (see README "Build" section and run.md for the full story)
# ---------------------------------------------------------------------------
# A plain `colcon build --symlink-install` on this machine is what locks the Pi
# up. Two things combine badly:
#
#   1) colcon_cmake/task/cmake/build.py::_get_make_arguments() runs each package
#      as `cmake --build <dir> -- -j<N> -l<N>` where N = os.cpu_count() (= 4 on a
#      Pi 4). It only skips its own -j/-l if MAKEFLAGS already contains them.
#   2) colcon's default executor builds packages in parallel, so both packages
#      here (sllidar_ros2 + rf2o_laser_odometry) compile at the same time.
#
# Result: up to 2 packages x 4 compile jobs = 8 concurrent g++ processes on a
# 4-core / 3.7 GB board that also has no swap and is already running the VS Code
# remote server (~1.7 GB resident). That causes sustained memory pressure, whose
# writeback floods the SD card and wedges the MMC/SDHCI controller
# ("task ... blocked for more than 122 seconds", workqueue events_freezable
# mmc_rescan, stuck in __mmc_claim_host).
#
# Because the root filesystem is on that same SD card AND the brcmfmac Wi-Fi
# radio hangs off the SDIO/MMC bus, the stall kills disk I/O *and* the network
# together: SSH drops and cannot reconnect, and the Pi has to be power-cycled.
#
# This wrapper caps compiler parallelism (through MAKEFLAGS, which colcon_cmake
# honours and therefore stops passing its own -j4/-l4) and builds one package at
# a time.
#
# ---------------------------------------------------------------------------
# USAGE
#   ./build.sh                 # safest: 1 compiler job, one package at a time
#   JOBS=2 ./build.sh          # a bit faster if the Pi is otherwise idle
# ---------------------------------------------------------------------------
#
# NOTE: `set -u` is deliberately avoided around the ROS sourcing below. The
# generated /opt/ros/<distro>/setup.bash and install/setup.bash reference
# variables they never initialise (e.g. AMENT_TRACE_SETUP_FILES, *_CURRENT_PREFIX)
# and abort under `set -u`. We still want -e and pipefail for our own commands.
# ---------------------------------------------------------------------------
set -eo pipefail

cd "$(dirname "$(readlink -f "$0")")"

JOBS="${JOBS:-1}"
export MAKEFLAGS="-j${JOBS} -l${JOBS}"

echo ">> Building with MAKEFLAGS='${MAKEFLAGS}' and --executor sequential"
echo ">> (caps the build to ${JOBS} compiler job(s) to avoid wedging the SD/MMC bus)"

# ROS setup scripts are not clean under `set -u`; relax it just for sourcing.
set +u
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
set -u

colcon build \
  --symlink-install \
  --executor sequential

set +u
# shellcheck disable=SC1091
source install/setup.bash
set -u
echo ">> Build complete. Environment sourced from install/setup.bash"
