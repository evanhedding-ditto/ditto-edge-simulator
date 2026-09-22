#!/usr/bin/env bash
set -euo pipefail

# Network observer: reports every transport path the fleet is using and cuts
# individual paths off on command. The viewer is one client of it; a network
# manager driving its own state machine is the other.

source "$(dirname "${BASH_SOURCE[0]}")/../lib.sh"
sim_init

binary="${SIM_OBSERVER_BIN:-$SIM_EDGE_ADAPTERS_ROOT/target/release/ditto-network-observer}"
[[ -x "$binary" ]] || binary="$SIM_EDGE_ADAPTERS_ROOT/target/debug/ditto-network-observer"
[[ -x "$binary" ]] || die "build the observer first: cargo build -p ditto-network-observer"

# One --node per Edge Server. The socket's parent directory names the node, so
# the ids match the viewer's --vehicle ids.
arguments=()
for ((index = 0; index < SIM_VEHICLE_COUNT; ++index)); do
  arguments+=(--node "$(node_socket "px4_$index")")
done
arguments+=(--node "$(node_socket operator)")
arguments+=(--listen "$SIM_OBSERVER_ADDR")

exec "$binary" "${arguments[@]}"
