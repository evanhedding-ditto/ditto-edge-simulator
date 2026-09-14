#!/usr/bin/env bash
set -euo pipefail

source "$(dirname "$0")/lib.sh"
sim_init

marker="$SIM_RUNTIME_DIR/px4-telemetry-ready"
[[ -f "$marker" ]] && exit 0
report="$SIM_RUNTIME_DIR/telemetry-gate.log"
: >"$report"

probe="$SIM_ROOT/build/cmake/ditto_px4_telemetry_ready"
[[ -x "$probe" ]] || die "build the PX4 telemetry probe first"
arguments=()
for ((index = 0; index < SIM_VEHICLE_COUNT; ++index)); do
  arguments+=(--vehicle "px4_$index" --port "$((19410 + index))")
done

fleet_timeout="${SIM_PX4_FLEET_BOOT_TIMEOUT_SECONDS:-180}"
ready_timeout="${SIM_TELEMETRY_READY_TIMEOUT_SECONDS:-60}"
for value in "$fleet_timeout" "$ready_timeout"; do
  [[ "$value" =~ ^[1-9][0-9]*$ ]] || die "telemetry timeouts must be positive integers"
done
wait_for_fleet_infrastructure "$fleet_timeout"
wait_for_fleet_px4_startup "$fleet_timeout"
wait_for_fleet_px4_direct_streams "$fleet_timeout"
printf '[PX4] Verifying direct position and attitude telemetry\n'
if "$probe" --timeout "$ready_timeout" "${arguments[@]}" 2>&1 | tee -a "$report"; then
  touch "$marker"
  exit 0
fi
die "direct PX4 telemetry failed; retained evidence: $report"
