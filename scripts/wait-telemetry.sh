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

# Twenty PX4s boot at ~2x CPU oversubscription, and rcS is ~200 sequential
# client execs per vehicle: measured 55-181 s per vehicle, mean 116 s. At 180 s
# the slowest healthy vehicle finished with ~1 s to spare, so the budget was
# failing runs that would have passed. A genuine wedge is caught far sooner by
# the stall detector (SIM_PX4_STALL_SECONDS) and the bounded-call abort, not by
# this wall.
fleet_timeout="${SIM_PX4_FLEET_BOOT_TIMEOUT_SECONDS:-300}"
ready_timeout="${SIM_TELEMETRY_READY_TIMEOUT_SECONDS:-60}"
for value in "$fleet_timeout" "$ready_timeout"; do
  [[ "$value" =~ ^[1-9][0-9]*$ ]] || die "telemetry timeouts must be positive integers"
done
if [[ "${SIM_SYNTHETIC_FLEET:-0}" == 1 ]]; then
  # A synthetic fleet has no PX4 processes, so the three PX4 phases above have
  # nothing to read: they gate on px4.log, which no synthetic vehicle writes.
  # Everything after this point is identical, because the synthetic fleet emits
  # the same GLOBAL_POSITION_INT and ATTITUDE on the same ports -- the probe and
  # the marker are unchanged, so the viewer and sim.sh need no special case.
  edge=()
  for ((index = 0; index < SIM_VEHICLE_COUNT; ++index)); do edge+=("px4_$index"); done
  edge+=(operator)
  report_tier Edge "Edge Servers ready" "$fleet_timeout" edge_socket_ready "${edge[@]}"
  printf '[Synthetic] Verifying position and attitude telemetry\n'
else
  wait_for_fleet_infrastructure "$fleet_timeout"
  wait_for_fleet_px4_startup "$fleet_timeout"
  wait_for_fleet_px4_direct_streams "$fleet_timeout"
  printf '[PX4] Verifying direct position and attitude telemetry\n'
fi
if "$probe" --timeout "$ready_timeout" "${arguments[@]}" 2>&1 | tee -a "$report"; then
  touch "$marker"
  exit 0
fi
die "direct PX4 telemetry failed; retained evidence: $report"
