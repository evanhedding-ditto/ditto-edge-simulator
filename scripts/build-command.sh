#!/usr/bin/env bash
set -euo pipefail

source "$(dirname "$0")/lib.sh"
sim_init
cmake -S "$SIM_ROOT" -B "$SIM_ROOT/build/cmake" \
  -DDITTO_EDGE_ADAPTERS_ROOT="$SIM_EDGE_ADAPTERS_ROOT"
cmake --build "$SIM_ROOT/build/cmake" --target ditto_fleet_command
