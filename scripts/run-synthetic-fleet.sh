#!/usr/bin/env bash
set -euo pipefail

source "$(dirname "$0")/lib.sh"
sim_init
binary="$SIM_ROOT/build/cmake/ditto_synthetic_fleet"
[[ -x "$binary" ]] || die "build the synthetic fleet first"

# Every vehicle's Edge Server must have bound its socket before the fleet starts
# publishing. This is the synthetic equivalent of the px4-telemetry-ready gate
# both adapters wait on, and it exists for the same measured reason: vehicles
# publishing vehicle_state at 10 Hz into a multi-peer mesh while other nodes are
# still starting is what makes fleet startup disk-bound rather than CPU-bound.
wait_for_fleet_edge_sockets

# Same geodetic origin PX4 is given, so a synthetic run and a PX4 run put the
# fleet in the same place and the viewer needs no reconfiguration.
location_file="$SIM_PROTOTYPE_ROOT/.location.env"
if [[ -r "$location_file" ]]; then
  # shellcheck disable=SC1090
  source "$location_file"
fi

arguments=(--runtime-dir "$SIM_RUNTIME_DIR" --count "$SIM_VEHICLE_COUNT")
[[ -n "${PX4_HOME_LAT:-}" ]] && arguments+=(--origin-lat "$PX4_HOME_LAT")
[[ -n "${PX4_HOME_LON:-}" ]] && arguments+=(--origin-lon "$PX4_HOME_LON")
[[ -n "${PX4_HOME_ALT:-}" ]] && arguments+=(--origin-alt "$PX4_HOME_ALT")
[[ -n "${SIM_SYNTHETIC_STATE_RATE_HZ:-}" ]] &&
  arguments+=(--state-rate-hz "$SIM_SYNTHETIC_STATE_RATE_HZ")

log="$SIM_RUNTIME_DIR/synthetic-fleet.log"
: >"$log"
exec nice -n "${SIM_BACKGROUND_NICE:-10}" "$binary" "${arguments[@]}" >>"$log" 2>&1
