#!/usr/bin/env bash
# Process list for a synthetic-fleet scenario, emitted on stdout for
# render-fleet.sh to expand.
#
# The whole point of this scenario is what is NOT here. A PX4 scenario runs one
# SIH autopilot, and often an XRCE agent and an adapter, per vehicle; twenty
# vehicles is 72 processes. Here the entire autopilot and adapter tier is one
# `synthetic-fleet` process, so the process count is N edge servers plus three.
# The Edge Servers are the thing under test and are therefore still real, one
# per vehicle, exactly as in a PX4 run.
set -euo pipefail

count="${SIM_VEHICLE_COUNT:?SIM_VEHICLE_COUNT is required}"

printf 'version: "0.5"\n\nprocesses:\n'
printf '  network: {command: ./scripts/run-network.sh, availability: {restart: always}}\n'

for ((index = 0; index < count; ++index)); do
  printf '  edge-px4-%s: {command: ./scripts/run-edge-server.sh vehicle %s, availability: {restart: always}}\n' \
    "$index" "$index"
done

# The operator waits only on the relay plus the first few vehicle servers.
# Listing all hundred would make process-compose serialise startup on the
# slowest one, and the operator's own mesh peers are a small subset anyway.
printf '  edge-operator:\n    command: ./scripts/run-edge-server.sh operator 0\n    depends_on:\n'
printf '      network: {condition: process_started}\n'
gate="$(( count < 4 ? count : 4 ))"
for ((index = 0; index < gate; ++index)); do
  printf '      edge-px4-%s: {condition: process_started}\n' "$index"
done
printf '    availability: {restart: always}\n'

# One process for every vehicle. It waits for all N sockets itself rather than
# through depends_on, because process_started only means the edge server was
# execed, not that it has bound its socket.
printf '\n  synthetic-fleet:\n    command: ./scripts/run-synthetic-fleet.sh\n    depends_on:\n'
printf '      edge-operator: {condition: process_started}\n'
printf '    availability: {restart: always}\n'

# The network observer. It holds a presence stream open per node and tolerates
# a socket that does not exist yet, so it can start as soon as the servers are
# execed rather than waiting for every one of them to bind.
printf '\n  observer:\n    command: ./scripts/run-observer.sh\n    depends_on:\n'
printf '      edge-operator: {condition: process_started}\n'
printf '    availability: {restart: always}\n'
