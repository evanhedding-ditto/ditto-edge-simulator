#!/usr/bin/env bash
set -euo pipefail

SIM_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
export SIM_SCENARIO_FILE="${SIM_SCENARIO_FILE:-$SIM_ROOT/scenarios/mvp-twenty-mixed.env}"
source "$SIM_ROOT/scripts/lib.sh"
sim_init

[[ "${SIM_VEHICLE_COUNT:-}" == 20 ]] || die "demo-twenty requires the twenty-vehicle scenario"
command="$SIM_ROOT/build/cmake/ditto_fleet_command"
[[ -x "$command" ]] || die "build the command client first"

# Every vehicle to one point: the mesh must carry twenty commands at once.
#
# One `batch` write instead of twenty `goto` invocations. The previous loop ran
# the client once per vehicle, and each run was a read-modify-write of the one
# shared fleet-current document: twenty connections, twenty document versions,
# and twenty independently-stamped 30 s TTLs for what is meant to be a single
# order. It also had to stay serial, since UPDATE_LOCAL_DIFF would let two
# overlapping writers clobber each other's slot.
"$command" --socket "$(node_socket operator)" batch <<'COMMANDS'
px4_0 goto 43 -60 20
px4_1 goto 43 -60 20
px4_2 goto 43 -60 20
px4_3 goto 43 -60 20
px4_4 goto 43 -60 20
px4_5 goto 43 -60 20
px4_6 goto 43 -60 20
px4_7 goto 43 -60 20
px4_8 goto 43 -60 20
px4_9 goto 43 -60 20
px4_10 goto 43 -60 20
px4_11 goto 43 -60 20
px4_12 goto 43 -60 20
px4_13 goto 43 -60 20
px4_14 goto 43 -60 20
px4_15 goto 43 -60 20
px4_16 goto 43 -60 20
px4_17 goto 43 -60 20
px4_18 goto 43 -60 20
px4_19 goto 43 -60 20
COMMANDS
