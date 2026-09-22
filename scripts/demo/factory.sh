#!/usr/bin/env bash
# Factory robots in a multi-level building: fleet plus viewer, and nothing else.
#
# NO NETWORK. No Edge Server, no relay, no adapters, no observer -- this is the
# world and the robots driving around in it, which is all the scenario is meant
# to show at this stage. The viewer draws NO LINK against every robot and that
# is correct, not a fault. Wiring the fleet to Edge Server is the next piece of
# work and needs nothing here to change: the robots already speak the same
# MAVLink an adapter reads from a PX4, on the same ports.
#
# Closing the viewer stops the fleet.
set -euo pipefail

source "$(dirname "${BASH_SOURCE[0]}")/../lib.sh"
# The factory scenario, for its world file and robot count -- NOT for its
# network, none of which this script starts. mvp-two-px4.env stood here while
# factory-twenty.env did not exist yet, and defaulting to a two-PX4 scenario
# meant the world path below was a hardcoded fallback rather than something the
# scenario said. Now the demo and the full stack read the same building.
SIM_SCENARIO_FILE="${SIM_SCENARIO_FILE:-$(dirname "${BASH_SOURCE[0]}")/../../scenarios/factory-twenty.env}"
sim_init

world="${SIM_WORLD_FILE:-$SIM_ROOT/worlds/factory-three-level.json}"
[[ "$world" == /* ]] || world="$SIM_ROOT/$world"
[[ -r "$world" ]] || die "world file not readable: $world"

fleet="$SIM_ROOT/build/cmake/ditto_factory_fleet"
viewer="$SIM_ROOT/build/cmake/ditto_fleet_viewer"
for binary in "$fleet" "$viewer"; do
  [[ -x "$binary" ]] || die "build first: scripts/build/viewer.sh"
done

# This demo shares the display and control port ranges with every other fleet,
# deliberately -- an adapter needs no per-scenario configuration that way -- so
# two fleets cannot be up at once. sim.sh refuses a second session for the same
# reason, but this script does not go through sim.sh, and without this check the
# first symptom is a raw "could not bind UDP port" from whichever process loses
# the race, which says nothing about the cause.
while read -r port; do
  simulator_running "$port" || continue
  die "a simulator session is already running on controller port $port;" \
    "stop it with 'pixi run sim-down' first"
done < <(known_simulator_ports)
if /usr/sbin/lsof -nP -iUDP:"$(synthetic_autopilot_port 0)" >/dev/null 2>&1; then
  die "a fleet is already using the control ports; stop it before starting this demo"
fi

# The fleet size comes from the world file unless the caller overrides it, so
# the building and its population stay described in one place. Read with sed
# and awk rather than a JSON parser: the simulator's scripts are bash, and a run
# that needed Python installed to start would be a new dependency for one line.
count="${SIM_FACTORY_ROBOT_COUNT:-}"
if [[ -z "$count" ]]; then
  count="$(sed -n 's/.*"robots_per_level": *\[\([0-9, ]*\)\].*/\1/p' "$world" |
    tr ',' ' ' | awk '{total = 0; for (i = 1; i <= NF; i++) total += $i; print total; exit}')"
fi
[[ "$count" =~ ^[0-9]+$ && "$count" -gt 0 ]] ||
  die "could not read robots_per_level from $world; set SIM_FACTORY_ROBOT_COUNT"
port_base="${SIM_FACTORY_PORT_BASE:-19410}"

"$fleet" --world "$world" --count "$count" --display-port-base "$port_base" &
fleet_pid=$!
trap 'kill "$fleet_pid" 2>/dev/null || true' EXIT

arguments=()
for ((index = 0; index < count; ++index)); do
  arguments+=(--vehicle "robot_$index" --port "$((port_base + index))")
done
arguments+=(--world "$world")

# Give the fleet a moment to start streaming, so the viewer's first frames have
# something in them rather than opening on an empty building.
sleep 1
"$viewer" "${arguments[@]}"
