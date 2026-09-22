#!/usr/bin/env bash
set -euo pipefail

SIM_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
export SIM_SCENARIO_FILE="${SIM_SCENARIO_FILE:-$SIM_ROOT/scenarios/mvp-twenty-mixed.env}"
source "$SIM_ROOT/scripts/lib.sh"
sim_init

[[ "${SIM_VEHICLE_COUNT:-}" == 20 ]] || die "demo-twenty requires the twenty-vehicle scenario"
command="$SIM_ROOT/build/cmake/ditto_fleet_command"
[[ -x "$command" ]] || die "build the command client first"

# Twenty orbits, spread across the map: every vehicle circles its own centre on
# a 5 x 4 grid at 45 m north and 50 m east spacing, reaching +/-90 north and
# +/-75 east. Radii run 6 to 18 m, so the widest orbits are still well clear of
# their neighbours, and altitude, speed, direction and start angle all vary --
# twenty vehicles turning in step at one radius reads as one animation rather
# than twenty independent commands.
#
# One `batch` write instead of twenty invocations. The previous loop ran the
# client once per vehicle, and each run was a read-modify-write of the one
# shared fleet-current document: twenty connections, twenty document versions,
# and twenty independently-stamped 30 s TTLs for what is meant to be a single
# order. It also had to stay serial, since UPDATE_LOCAL_DIFF would let two
# overlapping writers clobber each other's slot.
"$command" --socket "$(node_socket operator)" batch <<'COMMANDS'
# North +90 m.
px4_0  orbit   90  -75  12   8  3.0 true  0.00
px4_1  orbit   90  -25  20  14  4.0 false 2.40
px4_2  orbit   90   25   8   6  2.5 true  4.80
px4_3  orbit   90   75  24  18  3.5 false 0.92

# North +45 m.
px4_4  orbit   45  -75  18  16  3.5 true  3.32
px4_5  orbit   45  -25  10   7  2.5 false 5.72
px4_6  orbit   45   25  26  12  4.0 true  1.83
px4_7  orbit   45   75  14   9  3.0 false 4.23

# North +0 m.
px4_8  orbit    0  -75   9   6  2.5 true  0.35
px4_9  orbit    0  -25  22  18  3.5 false 2.75
px4_10 orbit    0   25  16  15  3.0 true  5.15
px4_11 orbit    0   75  11  11  4.0 false 1.27

# North -45 m.
px4_12 orbit  -45  -75  25  13  4.0 true  3.67
px4_13 orbit  -45  -25  13   9  3.0 false 6.07
px4_14 orbit  -45   25  19  17  3.5 true  2.18
px4_15 orbit  -45   75  10   7  2.5 false 4.58

# North -90 m.
px4_16 orbit  -90  -75  15  10  3.0 true  0.70
px4_17 orbit  -90  -25  23  16  4.0 false 3.10
px4_18 orbit  -90   25  12   8  2.5 true  5.50
px4_19 orbit  -90   75  21  14  3.5 false 1.62
COMMANDS
