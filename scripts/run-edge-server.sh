#!/usr/bin/env bash
set -euo pipefail

source "$(dirname "$0")/lib.sh"
sim_init
role="${1:?usage: run-edge-server.sh <vehicle|operator> <index>}"
index="${2:?usage: run-edge-server.sh <vehicle|operator> <index>}"
[[ "$role" == vehicle || "$role" == operator ]] || die "unknown Edge Server role: $role"
[[ "$index" =~ ^[0-9]+$ ]] || die "Edge Server index must be numeric"
binary="${DITTO_EDGE_SERVER_BIN:-$SIM_EDGE_SERVER_ROOT/ditto-edge-server/target/release/ditto-edge-server}"
[[ -x "$binary" || -n "${DITTO_EDGE_SERVER_BIN:-}" ]] || binary="$SIM_EDGE_SERVER_ROOT/ditto-edge-server/target/debug/ditto-edge-server"
config="$SIM_RUNTIME_DIR/edge-$role-$index.yaml"
[[ -x "$binary" ]] || die "Edge Server binary is not executable: $binary"
[[ -r "$config" ]] || die "render the fleet before starting Edge Server"
[[ "${NO_COLOR:-}" != 1 ]] || export NO_COLOR=true
# The SDK caps TCP connections per peer at mesh_chooser_max_wlan_connections,
# default 6, counting inbound AND outbound. This topology gives every node
# 2 * SIM_MESH_PEERS_PER_VEHICLE links, so anything above 3 peers silently ran
# into AtCapacity rejection, 10s-600s backoff and ~120s churn. Edge Server has
# no config surface for SDK system parameters, but the SDK reads any DITTO_*
# environment variable as a parameter layer ranking above the SDK config.
export DITTO_MESH_CHOOSER_MAX_WLAN_CONNECTIONS="${DITTO_MESH_CHOOSER_MAX_WLAN_CONNECTIONS:-20}"
exec nice -n "${SIM_BACKGROUND_NICE:-10}" "$binary" run --config "$config"
