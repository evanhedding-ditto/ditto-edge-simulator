#!/usr/bin/env bash
set -euo pipefail

source "$(dirname "${BASH_SOURCE[0]}")/../lib.sh"
sim_init
binary="$SIM_ROOT/build/cmake/ditto_synthetic_fleet"
[[ -x "$binary" ]] || die "build the synthetic fleet first"

# No Edge Server gate here any more. The fleet holds no Edge connection: it is
# an autopilot, and its adapters do the Ditto writing and wait on their own
# sockets. Starting first is what it should do -- the adapters dial in, so a
# vehicle that is not already streaming cannot be found.

# Same geodetic origin PX4 is given, so a synthetic run and a PX4 run put the
# fleet in the same place and the viewer needs no reconfiguration.

arguments=(
  --count "$SIM_SYNTHETIC_VEHICLE_COUNT"
  --start-index "$SIM_SYNTHETIC_START_INDEX"
  --control-local-port-base "$(synthetic_autopilot_port 0)"
  --control-remote-port-base "$(synthetic_adapter_port 0)"
)
[[ -n "${PX4_HOME_LAT:-}" ]] && arguments+=(--origin-lat "$PX4_HOME_LAT")
[[ -n "${PX4_HOME_LON:-}" ]] && arguments+=(--origin-lon "$PX4_HOME_LON")
[[ -n "${PX4_HOME_ALT:-}" ]] && arguments+=(--origin-alt "$PX4_HOME_ALT")

log="$SIM_RUNTIME_DIR/synthetic-fleet.log"
: >"$log"
exec nice -n "${SIM_BACKGROUND_NICE:-10}" "$binary" "${arguments[@]}" >>"$log" 2>&1
