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
  if [[ -n "${SIM_GCS_TCP_PORT:-}" ]] && ! is_synthetic_vehicle "$index"; then
    arguments+=(--gcs-port "$((SIM_GCS_TCP_PORT + index))")
  fi
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
if [[ -n "${SIM_ISR_TARGETS_FILE:-}" ]]; then
  targets_file="$SIM_ISR_TARGETS_FILE"
  [[ "$targets_file" == /* ]] || targets_file="$SIM_ROOT/$targets_file"
  [[ -r "$targets_file" ]] || die "ISR targets file not readable: $targets_file"
  arguments+=(--targets "$targets_file")
  arguments+=(--isr-socket "$(node_socket operator)")
  arguments+=(--isr-scenario "$SIM_SCENARIO_ID")
  arguments+=(--isr-run-id "$(cat "$SIM_RUNTIME_DIR/run-started-unix-ms")")
  arguments+=(--isr-reset-script "$SIM_ROOT/scripts/session/reset-isr.sh")
fi
arguments+=(--network-metrics "$SIM_RUNTIME_DIR/network-metrics.json")
# Each vehicle's camera as RTSP for a ground station: rtsp://<host>:PORT/px4_<index>.
[[ -z "${SIM_RTSP_PORT:-}" ]] || arguments+=(--rtsp "$SIM_RTSP_PORT")
# The network observer. Harmless when it is not running: the viewer reports
# CONNECTING and draws no overlay.
arguments+=(--observer "$SIM_OBSERVER_ADDR")
exec "$viewer" "${arguments[@]}"
