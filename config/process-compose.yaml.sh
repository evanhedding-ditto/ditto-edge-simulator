#!/usr/bin/env bash
# Process list for a mixed PX4/synthetic fleet. Every vehicle has an Edge Server
# and adapter; synthetic vehicles share one MAVLink autopilot process.
set -euo pipefail

count="${SIM_VEHICLE_COUNT:?SIM_VEHICLE_COUNT is required}"
mavlink() { [[ " ${SIM_MAVLINK_VEHICLES:-} " == *" $1 "* ]]; }
synthetic() {
  [[ "${SIM_SYNTHETIC_FLEET:-0}" == 1 ]] ||
    (( SIM_SYNTHETIC_VEHICLE_COUNT > 0 && $1 >= SIM_SYNTHETIC_START_INDEX &&
      $1 < SIM_SYNTHETIC_START_INDEX + SIM_SYNTHETIC_VEHICLE_COUNT ))
}

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
  # Only a ROS vehicle talks DDS; synthetic and MAVLink vehicles need no agent.
  synthetic "$index" && continue
  mavlink "$index" && continue
  printf '  xrce-px4-%s:\n    command: ./scripts/process/xrce-agent.sh %s\n' "$index" "$index"
  printf '    availability: { restart: always }\n'
done
for ((index = 0; index < count; ++index)); do
  # A ROS vehicle's PX4 needs its agent listening; a MAVLink one needs nothing
  # but its Edge Server, which px4.sh waits for itself.
  synthetic "$index" && continue
  gate="xrce-px4-$index"
  mavlink "$index" && gate="edge-px4-$index"
  printf '  sih-px4-%s:\n    command: ./scripts/process/px4.sh %s\n    depends_on:\n' "$index" "$index"
  printf '      %s: { condition: process_started }\n    availability: { restart: always }\n' "$gate"
done
if (( SIM_SYNTHETIC_VEHICLE_COUNT > 0 )); then
  printf '  synthetic-fleet:\n    command: ./scripts/process/synthetic-fleet.sh\n    availability: {restart: always}\n'
fi
for ((index = 0; index < count; ++index)); do
  adapter=ros
  upstream="sih-px4-$index"
  if synthetic "$index"; then
    adapter=mavlink
    upstream=synthetic-fleet
  elif mavlink "$index"; then
    adapter=mavlink
  fi
  printf '  %s-px4-%s:\n    command: ./scripts/process/%s-adapter.sh %s\n    depends_on:\n' \
    "$adapter" "$index" "$adapter" "$index"
  printf '      edge-px4-%s: { condition: process_started }\n' "$index"
  printf '      %s: { condition: process_started }\n    availability: { restart: always }\n' "$upstream"
done
printf '  observer:\n    command: ./scripts/process/observer.sh\n    depends_on:\n'
printf '      edge-operator: { condition: process_started }\n    availability: { restart: always }\n'
if [[ -n "${SIM_GCS_TCP_PORT:-}" ]]; then
  printf '  gcs-relay:\n    command: ./scripts/process/gcs-relay.sh\n    availability: { restart: always }\n'
fi
