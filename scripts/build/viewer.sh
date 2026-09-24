#!/usr/bin/env bash
set -euo pipefail

source "$(dirname "${BASH_SOURCE[0]}")/../lib.sh"
sim_init
cmake -S "$SIM_ROOT" -B "$SIM_ROOT/build/cmake" \
  -DCMAKE_PREFIX_PATH="$SIM_PIXI_ENV" \
  -DDITTO_EDGE_ADAPTERS_ROOT="$SIM_EDGE_ADAPTERS_ROOT" \
  -DPX4_ROOT="$SIM_PX4_ROOT"
cmake --build "$SIM_ROOT/build/cmake" --target ditto_fleet_viewer ditto_px4_telemetry_ready ditto_network_relay \
  ditto_synthetic_fleet ditto_factory_fleet ditto_gcs_relay
