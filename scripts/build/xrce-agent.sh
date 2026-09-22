#!/usr/bin/env bash
set -euo pipefail

# Builds the Micro XRCE-DDS Agent, which PX4's uXRCE-DDS client talks to on the
# ROS path. Lived in ditto-autonomy-testing until 2026-09-22; the source is now
# pinned in dependencies.repos and checked out under deps/.
#
# The agent installed at build/micro-xrce-dds-agent is linked
# @rpath-relative with no baked LC_RPATH, so it relocates freely -- which is why
# process/xrce-agent.sh names its lib directory and the ros environment
# explicitly. Rebuilding is only needed after changing the patch or the pin.
source "$(dirname "${BASH_SOURCE[0]}")/../lib.sh"
sim_init

source_dir="$SIM_ROOT/deps/Micro-XRCE-DDS-Agent"
build_dir="$SIM_ROOT/build/agent"
install_dir="$SIM_XRCE_ROOT"
patch_file="$SIM_ROOT/patches/micro-xrce-dds-agent-apple-clang.patch"
pixi_prefix="$SIM_ROOT/.pixi/envs/ros"

# Check for the CMakeLists rather than the directory. macOS is case-insensitive,
# so a bare -d test on deps/Micro-XRCE-DDS-Agent silently matched an unrelated
# directory differing only in case, and cmake was handed the wrong tree.
[[ -f "$source_dir/CMakeLists.txt" ]] || die "agent source is missing: run 'pixi run -e ros vcs import deps < dependencies.repos'"
[[ -d "$pixi_prefix" ]] || die "the ros environment is not installed: run 'pixi install -e ros'"

# Apply the Apple-clang fix if the tree is clean; accept a tree that already
# carries it; refuse anything else rather than building something unknown.
if git -C "$source_dir" apply --check "$patch_file" >/dev/null 2>&1; then
  git -C "$source_dir" apply "$patch_file"
elif ! git -C "$source_dir" apply --reverse --check "$patch_file" >/dev/null 2>&1; then
  die "the Apple-clang patch neither applies nor is already applied: $source_dir"
fi

# Fast DDS, Fast CDR and foonathan_memory come from the ros environment, so the
# agent links the same runtime the ROS adapter and PX4's client do.
# Run inside the ros environment, like scripts/build/ros-underlay.sh: cmake and
# ninja themselves come from there, not from whatever is on PATH. Fast DDS, Fast
# CDR and foonathan_memory are found through the same prefix, so the agent links
# the runtime the ROS adapter and PX4's client already use.
exec pixi run -e ros --manifest-path "$SIM_ROOT/pixi.toml" bash -lc '
  set -euo pipefail
  cmake -S "$1" -B "$2" -G Ninja \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX="$3" \
    -DCMAKE_PREFIX_PATH="$4" \
    -DUAGENT_SUPERBUILD=OFF \
    -DUAGENT_USE_SYSTEM_FASTDDS=ON \
    -DUAGENT_USE_SYSTEM_FASTCDR=ON \
    -DUAGENT_LOGGER_PROFILE=OFF \
    -DUAGENT_BUILD_TESTS=OFF \
    -DUAGENT_P2P_PROFILE=OFF \
    -DUAGENT_DISCOVERY_PROFILE=OFF \
    -DUAGENT_SOCKETCAN_PROFILE=OFF
  cmake --build "$2" --parallel
  cmake --install "$2"
' bash "$source_dir" "$build_dir" "$install_dir" "$pixi_prefix"
