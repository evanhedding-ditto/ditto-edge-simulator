#!/usr/bin/env bash
set -euo pipefail

source "$(dirname "${BASH_SOURCE[0]}")/../lib.sh"
sim_init

command="$SIM_ROOT/build/cmake/ditto_fleet_command"
[[ -x "$command" ]] || die "build the command client first"
report="$SIM_RUNTIME_DIR/fleet-verification.log"
: >"$report"
printf '[Fleet] Waiting for PX4 telemetry\n'
wait_for_file "$SIM_RUNTIME_DIR/px4-telemetry-ready" "${SIM_TELEMETRY_GATE_WAIT_TIMEOUT_SECONDS:-300}"
printf '[Mesh] Waiting for simulated links\n'
wait_for_mesh_links "${SIM_MESH_READY_TIMEOUT_SECONDS:-120}"
printf '[Fleet] PX4 telemetry ready; verifying state, command receipt, and movement\n'
wait_for_socket "$(node_socket operator)"
if ! "$command" --socket "$(node_socket operator)" verify "$SIM_VEHICLE_COUNT" 2>&1 | tee -a "$report"; then
  die "fleet verification failed; retained evidence: $report"
fi
touch "$SIM_RUNTIME_DIR/fleet-ready"
printf '[Fleet] Ready\n'
