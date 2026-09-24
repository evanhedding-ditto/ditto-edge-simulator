#!/usr/bin/env bash
set -euo pipefail

source "$(dirname "${BASH_SOURCE[0]}")/../lib.sh"
sim_init

: "${SIM_NETWORK_LINK_CAPACITY_KBPS:=10000}"
: "${SIM_MESH_PEERS_PER_VEHICLE:=$((SIM_VEHICLE_COUNT > 1))}"
: "${SIM_OPERATOR_MESH_PEERS:=1}"
exec "$SIM_ROOT/build/cmake/ditto_network_relay" --vehicles "$SIM_VEHICLE_COUNT" \
  --mesh-peers "$SIM_MESH_PEERS_PER_VEHICLE" --operator-peers "$SIM_OPERATOR_MESH_PEERS" \
  --capacity-kbps "$SIM_NETWORK_LINK_CAPACITY_KBPS" --metrics "$SIM_RUNTIME_DIR/network-metrics.json"
