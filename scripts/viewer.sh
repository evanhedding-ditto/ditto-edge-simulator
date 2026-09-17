#!/usr/bin/env bash
set -euo pipefail

source "$(dirname "$0")/lib.sh"
sim_init
viewer="$SIM_ROOT/build/cmake/ditto_fleet_viewer"
[[ -x "$viewer" ]] || die "build the viewer first"
location_file="$SIM_PROTOTYPE_ROOT/.location.env"
if [[ -r "$location_file" ]]; then
  # shellcheck disable=SC1090
  source "$location_file"
  if [[ -n "${PX4_HOME_LAT:-}" && -n "${PX4_HOME_LON:-}" ]]; then
    export SIM_VIEWER_ORIGIN_LAT="$PX4_HOME_LAT" SIM_VIEWER_ORIGIN_LON="$PX4_HOME_LON"
  fi
fi
arguments=()
for ((index = 0; index < SIM_VEHICLE_COUNT; ++index)); do
  arguments+=(--vehicle "px4_$index" --port "$((19410 + index))")
done
arguments+=(--network-metrics "$SIM_RUNTIME_DIR/network-metrics.json")
# The network observer. Harmless when it is not running: the viewer reports
# CONNECTING and draws no overlay.
arguments+=(--observer "${SIM_OBSERVER_ADDR:-127.0.0.1:50090}")
exec "$viewer" "${arguments[@]}"
