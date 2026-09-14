#!/usr/bin/env bash
set -euo pipefail

SIM_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export SIM_SCENARIO_FILE="${SIM_SCENARIO_FILE:-$SIM_ROOT/scenarios/mvp-twenty-mixed.env}"
source "$SIM_ROOT/scripts/lib.sh"
sim_init

[[ "${SIM_VEHICLE_COUNT:-}" == 20 ]] || die "demo-twenty requires the twenty-vehicle scenario"
command="$SIM_ROOT/build/cmake/ditto_fleet_command"
[[ -x "$command" ]] || die "build the command client first"

submit() {
  printf '%-6s ' "$1"
  shift
  "$command" --socket "$(node_socket operator)" "$@"
}

# ROS 2/PX4 fleet: distributed destinations.
submit px4_0 goto px4_0 43 -60 20
submit px4_1 goto px4_1 43 -60 20
submit px4_2 goto px4_2 43 -60 20
submit px4_3 goto px4_3 43 -60 20
submit px4_4 goto px4_4 43 -60 20
submit px4_5 goto px4_5 43 -60 20
submit px4_6 goto px4_6 43 -60 20
submit px4_7 goto px4_7 43 -60 20
submit px4_8 goto px4_8 43 -60 20
submit px4_9 goto px4_9 43 -60 20

# Native MAVLink fleet: separated alternating orbits.
submit px4_10 goto px4_10 43 -60 20
submit px4_11 goto px4_11 43 -60 20
submit px4_12 goto px4_12 43 -60 20
submit px4_13 goto px4_13 43 -60 20
submit px4_14 goto px4_14 43 -60 20
submit px4_15 goto px4_15 43 -60 20
submit px4_16 goto px4_16 43 -60 20
submit px4_17 goto px4_17 43 -60 20
submit px4_18 goto px4_18 43 -60 20
submit px4_19 goto px4_19 43 -60 20
