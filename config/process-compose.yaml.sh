#!/usr/bin/env bash
# Process list for a PX4 fleet of any size, emitted on stdout for
# session/render-fleet.sh to expand. Each vehicle is an Edge Server, a PX4 SIH,
# and an adapter: the ROS 2 one with its XRCE agent, or the MAVLink one for a
# vehicle in SIM_MAVLINK_VEHICLES. SIM_GCS_TCP_PORT adds the relay LAN ground
# stations fly the fleet through.
set -euo pipefail

count="${SIM_VEHICLE_COUNT:?SIM_VEHICLE_COUNT is required}"
mavlink() { [[ " ${SIM_MAVLINK_VEHICLES:-} " == *" $1 "* ]]; }

printf 'version: "0.5"\n\nprocesses:\n'
printf '  network:\n    command: ./scripts/process/network.sh\n    availability: { restart: always }\n'
for ((index = 0; index < count; ++index)); do
  printf '  edge-px4-%s:\n    command: ./scripts/process/edge-server.sh vehicle %s\n' "$index" "$index"
  printf '    availability: { restart: always }\n'
done
printf '  edge-operator:\n    command: ./scripts/process/edge-server.sh operator 0\n    depends_on:\n'
printf '      network: { condition: process_started }\n'
for ((index = 0; index < count; ++index)); do
  printf '      edge-px4-%s: { condition: process_started }\n' "$index"
done
printf '    availability: { restart: always }\n\n'
for ((index = 0; index < count; ++index)); do
  mavlink "$index" && continue
  printf '  xrce-px4-%s:\n    command: ./scripts/process/xrce-agent.sh %s\n' "$index" "$index"
  printf '    availability: { restart: always }\n'
done
for ((index = 0; index < count; ++index)); do
  # A ROS vehicle's PX4 needs its agent listening; a MAVLink one needs nothing
  # but its Edge Server, which px4.sh waits for itself.
  gate="xrce-px4-$index"
  mavlink "$index" && gate="edge-px4-$index"
  printf '  sih-px4-%s:\n    command: ./scripts/process/px4.sh %s\n    depends_on:\n' "$index" "$index"
  printf '      %s: { condition: process_started }\n    availability: { restart: always }\n' "$gate"
done
for ((index = 0; index < count; ++index)); do
  adapter=ros
  mavlink "$index" && adapter=mavlink
  printf '  %s-px4-%s:\n    command: ./scripts/process/%s-adapter.sh %s\n    depends_on:\n' \
    "$adapter" "$index" "$adapter" "$index"
  printf '      edge-px4-%s: { condition: process_started }\n' "$index"
  printf '      sih-px4-%s: { condition: process_started }\n    availability: { restart: always }\n' "$index"
done
printf '  observer:\n    command: ./scripts/process/observer.sh\n    depends_on:\n'
printf '      edge-operator: { condition: process_started }\n    availability: { restart: always }\n'
if [[ -n "${SIM_GCS_TCP_PORT:-}" ]]; then
  printf '  gcs-relay:\n    command: ./scripts/process/gcs-relay.sh\n    availability: { restart: always }\n'
fi
