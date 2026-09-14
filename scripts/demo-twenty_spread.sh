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
submit px4_0 goto px4_0 25 -40 6
submit px4_1 goto px4_1 25 -20 6
submit px4_2 goto px4_2 25 0 6
submit px4_3 goto px4_3 25 20 6
submit px4_4 goto px4_4 25 40 6
submit px4_5 goto px4_5 -25 -40 7
submit px4_6 goto px4_6 -25 -20 7
submit px4_7 goto px4_7 -25 0 7
submit px4_8 goto px4_8 -25 20 7
submit px4_9 goto px4_9 -25 40 7

# Native MAVLink fleet: separated alternating orbits.
submit px4_10 orbit px4_10 60 -45 8 7 3 true
submit px4_11 orbit px4_11 60 -15 8 7 3 false
submit px4_12 orbit px4_12 60 15 8 7 3 true
submit px4_13 orbit px4_13 60 45 8 7 3 false
submit px4_14 orbit px4_14 20 -30 9 7 3 true
submit px4_15 orbit px4_15 20 30 9 7 3 false
submit px4_16 orbit px4_16 -20 -30 10 7 3 true
submit px4_17 orbit px4_17 -20 30 10 7 3 false
submit px4_18 orbit px4_18 -60 -15 11 7 3 true
submit px4_19 orbit px4_19 -60 15 11 7 3 false
