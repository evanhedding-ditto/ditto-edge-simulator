#!/usr/bin/env bash
set -euo pipefail

source "$(dirname "${BASH_SOURCE[0]}")/../lib.sh"
sim_init
viewer="$SIM_ROOT/build/cmake/ditto_fleet_viewer"
if [[ "${SIM_VIEWER:-raylib}" == "cesium-native" ]]; then
  viewer="$SIM_ROOT/build/cmake-cesium/ditto_fleet_viewer_cesium"
  if [[ -z "${CESIUM_ACCESS_TOKEN:-}" && -r "$SIM_ROOT/.env" ]]; then
    # shellcheck disable=SC1091
    source "$SIM_ROOT/.env"
  fi
  [[ -n "${CESIUM_ACCESS_TOKEN:-}" ]] || die "set CESIUM_ACCESS_TOKEN in .env"
  export CESIUM_ACCESS_TOKEN SIM_VIEWER_USE_CESIUM=1
fi
[[ -x "$viewer" ]] || die "build the viewer first"
# The viewer pins its horizontal origin to the fleet's home position, so a
# mid-flight restart does not shift the map.
export SIM_VIEWER_ORIGIN_LAT="$PX4_HOME_LAT" SIM_VIEWER_ORIGIN_LON="$PX4_HOME_LON"
if [[ "${SIM_VIEWER:-raylib}" == "cesium-native" ]]; then
  export SIM_VIEWER_ORIGIN_ALT="$PX4_HOME_ALT"
fi
arguments=()
for ((index = 0; index < SIM_VEHICLE_COUNT; ++index)); do
  arguments+=(--vehicle "px4_$index" --port "$((19410 + index))")
done
# Static scenery, when the scenario names one. Relative paths resolve against
# the repository root so a scenario file can stay short. Drawn by the viewer and
# read by nothing else -- see viewer/src/world.hpp.
if [[ -n "${SIM_WORLD_FILE:-}" ]]; then
  world_file="$SIM_WORLD_FILE"
  [[ "$world_file" == /* ]] || world_file="$SIM_ROOT/$world_file"
  [[ -r "$world_file" ]] || die "world file not readable: $world_file"
  arguments+=(--world "$world_file")
fi
arguments+=(--network-metrics "$SIM_RUNTIME_DIR/network-metrics.json")
# The network observer. Harmless when it is not running: the viewer reports
# CONNECTING and draws no overlay.
arguments+=(--observer "$SIM_OBSERVER_ADDR")
exec "$viewer" "${arguments[@]}"
