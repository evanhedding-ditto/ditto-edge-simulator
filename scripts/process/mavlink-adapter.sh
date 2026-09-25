#!/usr/bin/env bash
set -euo pipefail

source "$(dirname "${BASH_SOURCE[0]}")/../lib.sh"
sim_init
index="${1:?usage: mavlink-adapter.sh <vehicle-index>}"
[[ "$index" =~ ^[0-9]+$ ]] || die "vehicle index must be numeric"
binary="$(mavlink_adapter_binary)" ||
  die "no MAVLink adapter. Drop one at $SIM_ROOT/bin/px4-mavlink-ditto-bridge, or run scripts/build/mavlink-adapter.sh"
# gRPC and Protobuf come from the pixi environment. Naming them here is what lets
# a binary built on another machine run: its LC_RPATH points at the pixi prefix
# on the machine that built it, which does not exist here.
export DYLD_LIBRARY_PATH="$SIM_PIXI_ENV/lib${DYLD_LIBRARY_PATH:+:$DYLD_LIBRARY_PATH}"
wait_for_socket "$(node_socket "px4_$index")"
# Same gate as process/ros-adapter.sh, for the same reason: this is a forwarding path
# and PX4 state generation takes precedence at startup. Without it, ten adapters
# publish vehicle_state at 10 Hz into a five-peer mesh while PX4s are still
# booting, and those replicated writes are the measured startup bottleneck - the
# fleet is disk-bound, not CPU-bound, during boot.
wait_for_file "$SIM_RUNTIME_DIR/px4-telemetry-ready" "${SIM_TELEMETRY_GATE_WAIT_TIMEOUT_SECONDS:-300}"
# A synthetic vehicle cannot reuse PX4's port pair -- 14540+i and 14580+i
# collide with each other past 40 vehicles -- so the synthetic tier has its own.
if is_synthetic_vehicle "$index"; then
  endpoint="udpin:127.0.0.1:$(synthetic_adapter_port "$index")"
else
  endpoint="udpin:127.0.0.1:$((14540 + index))"
fi
log="$(node_dir "px4_$index")/mavlink.log"
: >"$log"
exec nice -n "${SIM_BACKGROUND_NICE:-10}" "$binary" "$(node_socket "px4_$index")" "px4_$index" "$endpoint" >>"$log" 2>&1
