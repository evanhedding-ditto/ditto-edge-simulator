#!/usr/bin/env bash
set -euo pipefail

source "$(dirname "$0")/lib.sh"
sim_init
index="${1:?usage: run-mavlink-adapter.sh <vehicle-index>}"
[[ "$index" =~ ^[0-9]+$ ]] || die "vehicle index must be numeric"
binary="$SIM_EDGE_ADAPTERS_ROOT/target/debug/px4-mavlink-ditto-bridge"
[[ -x "$binary" ]] || die "build the MAVLink adapter first"
wait_for_socket "$(node_socket "px4_$index")"
log="$(node_dir "px4_$index")/mavlink.log"
: >"$log"
exec nice -n "${SIM_BACKGROUND_NICE:-10}" "$binary" "$(node_socket "px4_$index")" "px4_$index" "udpin:127.0.0.1:$((14540 + index))" >>"$log" 2>&1
