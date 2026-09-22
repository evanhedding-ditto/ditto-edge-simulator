#!/usr/bin/env bash
set -euo pipefail

source "$(dirname "${BASH_SOURCE[0]}")/../lib.sh"
sim_init
binary="$SIM_ROOT/build/cmake/ditto_factory_fleet"
[[ -x "$binary" ]] || die "build the factory fleet first"

world="${SIM_WORLD_FILE:?SIM_WORLD_FILE is required for a factory scenario}"
[[ "$world" == /* ]] || world="$SIM_ROOT/$world"
[[ -r "$world" ]] || die "world file not readable: $world"

# Same port layout as the synthetic fleet, from the same helpers, so a factory
# run and a synthetic run cannot both be up on overlapping sockets and the
# adapter needs no scenario-specific configuration.
arguments=(
  --world "$world"
  --count "$SIM_VEHICLE_COUNT"
  --control-local-port-base "$(synthetic_autopilot_port 0)"
  --control-remote-port-base "$(synthetic_adapter_port 0)"
)

# The building is authored in the local NED frame, so the geodetic origin only
# decides where on Earth it is placed. Taking PX4's home keeps a factory run and
# a PX4 run in the same place, which is what lets the viewer be reconfigured for
# neither.
[[ -n "${PX4_HOME_LAT:-}" ]] && arguments+=(--origin-lat "$PX4_HOME_LAT")
[[ -n "${PX4_HOME_LON:-}" ]] && arguments+=(--origin-lon "$PX4_HOME_LON")
[[ -n "${PX4_HOME_ALT:-}" ]] && arguments+=(--origin-alt "$PX4_HOME_ALT")

log="$SIM_RUNTIME_DIR/factory-fleet.log"
: >"$log"
exec nice -n "${SIM_BACKGROUND_NICE:-10}" "$binary" "${arguments[@]}" >>"$log" 2>&1
