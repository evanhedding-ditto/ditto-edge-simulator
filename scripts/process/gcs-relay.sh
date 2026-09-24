#!/usr/bin/env bash
set -euo pipefail

source "$(dirname "${BASH_SOURCE[0]}")/../lib.sh"
sim_init
[[ "${SIM_GCS_TCP_PORT:-}" =~ ^[1-9][0-9]*$ ]] || die "SIM_GCS_TCP_PORT must be a port number"
(( SIM_GCS_TCP_PORT + SIM_VEHICLE_COUNT <= 65536 )) || die "SIM_GCS_TCP_PORT leaves no port for every vehicle"
# Vehicle i: TCP SIM_GCS_TCP_PORT + i for ground stations; PX4's GCS link is on
# UDP 18570 + i, PX4's own numbering (udp_gcs_port_local in px4-rc.mavlink).
arguments=()
for ((index = 0; index < SIM_VEHICLE_COUNT; ++index)); do
  arguments+=(--vehicle "px4_$index" --port "$((SIM_GCS_TCP_PORT + index))" --px4-port "$((18570 + index))")
done
exec "$SIM_ROOT/build/cmake/ditto_gcs_relay" "${arguments[@]}"
