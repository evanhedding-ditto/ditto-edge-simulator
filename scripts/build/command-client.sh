#!/usr/bin/env bash
set -euo pipefail

source "$(dirname "${BASH_SOURCE[0]}")/../lib.sh"
sim_init
cmake -S "$SIM_ROOT" -B "$SIM_ROOT/build/cmake" \
  -DCMAKE_PREFIX_PATH="$SIM_PIXI_ENV" \
  -DDITTO_EDGE_ADAPTERS_ROOT="$SIM_EDGE_ADAPTERS_ROOT" \
  -DPX4_ROOT="$SIM_PX4_ROOT"
cmake --build "$SIM_ROOT/build/cmake" --target ditto_fleet_command
