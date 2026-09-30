#!/usr/bin/env bash
set -euo pipefail

SIM_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
export SIM_SCENARIO_FILE="${SIM_SCENARIO_FILE:-$SIM_ROOT/scenarios/park-mgm-cesium-10.env}"
source "$SIM_ROOT/scripts/lib.sh"
sim_init

[[ "$SIM_SCENARIO_ID" == park-mgm-cesium-10 && "$SIM_VEHICLE_COUNT" == 11 &&
  "$SIM_SYNTHETIC_VEHICLE_COUNT" == 10 ]] ||
  die "demo requires the park-mgm-cesium-10 scenario"
command="$SIM_ROOT/build/cmake/ditto_fleet_command"
[[ -x "$command" ]] || die "build the command client first: scripts/build/command-client.sh"
[[ -S "$(node_socket operator)" ]] || die "start it first: pixi run sim park-mgm-cesium-10"

"$command" --socket "$(node_socket operator)" batch <<'COMMANDS'
# Two rows across the Park MGM map; all coordinates are relative to px4_0.
px4_1  orbit   80  -170  14  12  3.0 true  0.00
px4_2  orbit   80   -85  18  14  4.0 false 2.00
px4_3  orbit   80     0  22  10  3.5 true  4.00
px4_4  orbit   80    85  16  16  2.5 false 1.00
px4_5  orbit   80   170  24  12  3.5 true  5.00
px4_6  orbit  -80  -170  26  13  3.2 false 3.00
px4_7  orbit  -80   -85  20  11  4.0 true  1.50
px4_8  orbit  -80     0  12  15  2.8 false 4.50
px4_9  orbit  -80    85  28  10  3.6 true  0.50
px4_10 orbit  -80   170  17  14  3.0 false 2.80
COMMANDS
