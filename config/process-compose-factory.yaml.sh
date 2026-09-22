#!/usr/bin/env bash
# Process list for a factory-robot scenario, emitted on stdout for
# session/render-fleet.sh to expand.
#
# Identical to the synthetic list except that the autopilot tier is
# `factory-fleet` instead of `synthetic-fleet`: ground robots driving a
# multi-level building rather than vehicles flying commanded orbits. Everything
# below the MAVLink boundary -- Edge Servers, the relay, one adapter per robot,
# the observer -- is the same stack, which is the point: the robots are a
# different load on an unchanged network, not a different network.
#
# What is NOT here is the autopilot. A PX4 scenario runs one SIH instance, and
# often an XRCE agent too, per vehicle; twenty vehicles is 72 processes. Here
# every autopilot is one `synthetic-fleet` process emulating them over MAVLink.
#
# The adapters ARE here, one per vehicle, exactly as in a PX4 run, because they
# are part of what is under test: they hold the Edge Server connections and do
# every Ditto write. The process count is 2N plus four.
set -euo pipefail

count="${SIM_VEHICLE_COUNT:?SIM_VEHICLE_COUNT is required}"

printf 'version: "0.5"\n\nprocesses:\n'
printf '  network: {command: ./scripts/process/network.sh, availability: {restart: always}}\n'

for ((index = 0; index < count; ++index)); do
  printf '  edge-px4-%s: {command: ./scripts/process/edge-server.sh vehicle %s, availability: {restart: always}}\n' \
    "$index" "$index"
done

# The operator waits only on the relay plus the first few vehicle servers.
# Listing all hundred would make process-compose serialise startup on the
# slowest one, and the operator's own mesh peers are a small subset anyway.
printf '  edge-operator:\n    command: ./scripts/process/edge-server.sh operator 0\n    depends_on:\n'
printf '      network: {condition: process_started}\n'
gate="$(( count < 4 ? count : 4 ))"
for ((index = 0; index < gate; ++index)); do
  printf '      edge-px4-%s: {condition: process_started}\n' "$index"
done
printf '    availability: {restart: always}\n'

# Every robot in one process, and no dependency on anything: it holds no Edge
# Server connection, and the adapters dial in to it, so it should be first up
# rather than last. REPORT-ONLY today -- the robots run their own patrol and
# discard the adapter's arm and offboard requests, so those commands time out
# by design and the fleet-command path is not exercised in this scenario.
printf '\n  factory-fleet:\n    command: ./scripts/process/factory-fleet.sh\n'
printf '    availability: {restart: always}\n'

# One real adapter per vehicle, the same binary a PX4 vehicle uses. It waits for
# its own Edge socket and for the telemetry marker itself, rather than through
# depends_on, because process_started only means a process was execed.
for ((index = 0; index < count; ++index)); do
  printf '  mavlink-px4-%s:\n    command: ./scripts/process/mavlink-adapter.sh %s\n    depends_on:\n' \
    "$index" "$index"
  printf '      edge-px4-%s: {condition: process_started}\n' "$index"
  printf '      factory-fleet: {condition: process_started}\n'
  printf '    availability: {restart: always}\n'
done

# The network observer. It holds a presence stream open per node and tolerates
# a socket that does not exist yet, so it can start as soon as the servers are
# execed rather than waiting for every one of them to bind.
printf '\n  observer:\n    command: ./scripts/process/observer.sh\n    depends_on:\n'
printf '      edge-operator: {condition: process_started}\n'
printf '    availability: {restart: always}\n'
