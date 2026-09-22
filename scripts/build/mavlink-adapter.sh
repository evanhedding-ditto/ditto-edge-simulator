#!/usr/bin/env bash
set -euo pipefail

source "$(dirname "${BASH_SOURCE[0]}")/../lib.sh"
sim_init
# The adapter is C++ now, and takes its MAVLink headers from the PX4 build this
# simulator already requires rather than fetching a dialect of its own.
cmake -S "$SIM_EDGE_ADAPTERS_ROOT/adapters/mavlink/px4_ditto_bridge" \
  -B "$SIM_ROOT/build/mavlink-adapter" \
  -DCMAKE_PREFIX_PATH="$SIM_PIXI_ENV" \
  -DDITTO_MAVLINK_INCLUDE_DIR="$SIM_PX4_ROOT/build/px4_sitl_sih/mavlink"
exec cmake --build "$SIM_ROOT/build/mavlink-adapter"
