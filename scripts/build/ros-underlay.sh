#!/usr/bin/env bash
set -euo pipefail

# Builds the PX4 ROS underlay -- px4_msgs and nothing else -- that the ROS adapter
# links against. This lived in ditto-autonomy-testing until 2026-09-22; the source
# is now pinned in dependencies.repos and checked out under deps/.
#
# px4_msgs must match the PX4 version this simulator runs. See dependencies.repos.
source "$(dirname "${BASH_SOURCE[0]}")/../lib.sh"
sim_init
[[ -f "$SIM_ROOT/deps/px4_msgs/package.xml" ]] ||
  die "px4_msgs is missing: run 'pixi run -e ros vcs import deps < dependencies.repos'"
mkdir -p "$SIM_ROOT/build/ros-underlay"
# pixi's bundled linker cannot parse the current CommandLineTools SDK (arm64e.x1
# slice), so force Apple clang -- the same override build/ros-adapter.sh uses.
exec pixi run -e ros --manifest-path "$SIM_ROOT/pixi.toml" bash -lc \
  'export CC=/usr/bin/cc CXX=/usr/bin/c++ && cd "$1/build/ros-underlay" && colcon build --symlink-install --base-paths "$1/deps/px4_msgs" --packages-select px4_msgs' \
  bash "$SIM_ROOT"
