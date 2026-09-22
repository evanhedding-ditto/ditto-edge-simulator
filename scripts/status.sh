#!/usr/bin/env bash
set -euo pipefail

source "$(dirname "$0")/lib.sh"
sim_init
active=false
for scenario in "$SIM_ROOT"/scenarios/*.env; do
  SIM_SCENARIO_FILE="$scenario"
  sim_init
  if ! output="$(sim_compose process list -o wide 2>&1)"
  then
    continue
  fi
  active=true
  printf 'Scenario %s:\n%s\n' "$SIM_SCENARIO_ID" "$output"
  [[ -f "$SIM_RUNTIME_DIR/px4-telemetry-ready" ]] && echo "px4-telemetry-ready: ready" || echo "px4-telemetry-ready: pending"
  # Adapter readiness is not a gate and deliberately not one -- nothing waits on
  # it -- but it is the difference between a fleet that can be commanded and one
  # that only looks alive in the viewer, so it is worth showing.
  ready=0
  for node in "$SIM_RUNTIME_DIR"/nodes/px4_*/mavlink.log; do
    [[ -r "$node" ]] && grep -q 'bridge ready' "$node" && ready=$((ready + 1))
  done
  [[ "$ready" -gt 0 ]] && echo "MAVLink adapters ready: $ready/${SIM_VEHICLE_COUNT:-?}"
  if [[ -s "$SIM_RUNTIME_DIR/telemetry-gate.log" ]]; then
    printf 'last telemetry gate: '
    tail -n 1 "$SIM_RUNTIME_DIR/telemetry-gate.log"
  fi
done
"$active" || echo "No active simulator session."
