#!/usr/bin/env bash
set -euo pipefail

SIM_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
export SIM_SCENARIO_FILE="${SIM_SCENARIO_FILE:-$SIM_ROOT/scenarios/mvp-four-mixed.env}"
source "$SIM_ROOT/scripts/lib.sh"
sim_init

[[ "${SIM_VEHICLE_COUNT:-}" == 4 ]] || die "demo-four requires the four-vehicle scenario"
command="$SIM_ROOT/build/cmake/ditto_fleet_command"
[[ -x "$command" ]] || die "build the command client first"

submit() {
  printf '%-6s ' "$1"
  shift
  "$command" --socket "$(node_socket operator)" "$@"
}

submit px4_0 goto px4_0 18 0 5
submit px4_1 orbit px4_1 0 18 6 6 2 true
submit px4_2 goto px4_2 -18 0 7
submit px4_3 orbit px4_3 0 -18 8 8 3 false
