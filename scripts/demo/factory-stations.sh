#!/usr/bin/env bash
set -euo pipefail

SIM_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
# PINNED, not defaulted. These coordinates name rooms in the factory building
# and mean nothing in any other scenario, so there is no useful case for this
# following an inherited SIM_SCENARIO_FILE -- and a stale one left in a shell
# silently redirected the write to whatever scenario it named, with a socket
# path nobody was looking at as the only symptom. Pass a scenario file as the
# first argument to aim a phase somewhere else deliberately.
export SIM_SCENARIO_FILE="${1:-$SIM_ROOT/scenarios/factory-twenty.env}"
source "$SIM_ROOT/scripts/lib.sh"
sim_init

[[ "${SIM_VEHICLE_COUNT:-}" == 20 ]] || die "the factory phases require the twenty-robot scenario"
command="$SIM_ROOT/build/cmake/ditto_fleet_command"
[[ -x "$command" ]] || die "build the command client first"

# Which scenario this will command, before it tries. SIM_SCENARIO_FILE is
# honoured when already set -- that is what lets a phase be aimed at another
# scenario -- but a stale export left in the shell then silently addresses a
# scenario that is not running, and the only symptom is a gRPC error naming a
# socket path nobody was looking at.
operator="$(node_socket operator)"
# Ask whether the scenario is actually UP, not merely whether its socket file
# is on disk: a torn-down run leaves the socket behind, so the file existing
# proves only that this scenario ran at some point.
simulator_running "${SIM_PROCESS_COMPOSE_PORT:-0}" || die \
  "scenario '$SIM_SCENARIO_ID' is not running -- start it with 'pixi run sim-factory'"
echo "commanding $SIM_VEHICLE_COUNT robots in scenario '$SIM_SCENARIO_ID'" >&2

# Phase 1 -- work stations. Every robot to its own post, spread over all three levels.
#
# The widest spread of the three: no two robots share a room where the
# floor plan allows otherwise, so the fleet occupies as much of the building at
# once as it can. This is the phase to run first if the point is to show the
# building being used rather than the robots being moved.
#
# One of three phases. Run them in whatever order suits the demo; each is a
# complete assignment for all twenty robots, so nothing depends on the last.
# A robot holds where it is sent and works there until the next phase arrives,
# which is what makes the propagation visible: the fleet stops moving, one
# write goes into the mesh, and they set off again.
#
# ALTITUDE is the robot's own level datum -- 0, 4 or 8 -- and is carried for
# honesty rather than guidance. These robots cannot change level; the lift is
# drawn but nothing rides it. A destination on another level has no route and
# is ignored rather than approximated.
#
# One `batch` write, not twenty invocations: the fleet-current document is
# shared, and twenty separate writes would be twenty read-modify-writes of it
# with twenty independently-stamped TTLs for what is meant to be one order.
"$command" --socket "$operator" batch <<'COMMANDS'
# Level 0 -- ground.
px4_0   goto     18   -10 0
px4_1   goto     18    -2 0
px4_2   goto     18    10 0
px4_3   goto    -18   -10 0
px4_4   goto    -18     2 0
px4_5   goto    -18    11 0
px4_6   goto      0   -12 0

# Level 1 -- mezzanine.
px4_7   goto     18    -8 4
px4_8   goto     18     4 4
px4_9   goto     18    12 4
px4_10  goto    -18    -8 4
px4_11  goto    -18     2 4
px4_12  goto    -18    11 4
px4_13  goto      0    13 4

# Level 2 -- upper.
px4_14  goto     18   -10 8
px4_15  goto     18     2 8
px4_16  goto     18    10 8
px4_17  goto    -18    -9 8
px4_18  goto    -18     2 8
px4_19  goto      0   -12 8
COMMANDS
