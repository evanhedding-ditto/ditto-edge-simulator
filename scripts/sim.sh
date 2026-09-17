#!/usr/bin/env bash
set -euo pipefail

# A scenario may be named as the first argument -- `pixi run sim
# synthetic-twenty` -- which beats having to spell out SIM_SCENARIO_FILE. A
# bare name resolves against scenarios/, and a path is taken as given.
if [[ $# -gt 0 ]]; then
  if [[ -r "$1" ]]; then
    SIM_SCENARIO_FILE="$1"
  else
    SIM_SCENARIO_FILE="$(cd "$(dirname "$0")/.." && pwd)/scenarios/${1%.env}.env"
  fi
  export SIM_SCENARIO_FILE
  shift
fi

source "$(dirname "$0")/lib.sh"
sim_init

while read -r port; do
  simulator_running "$port" || continue
  die "a simulator session is already running on controller port $port; use 'pixi run sim-down' first"
done < <(known_simulator_ports)

[[ -r "$SIM_ROOT/build/ros-gateway-host/install/setup.bash" ]] || "$SIM_ROOT/scripts/build-ros-adapter.sh"
if [[ -n "${SIM_MAVLINK_VEHICLES:-}" ]]; then
  "$SIM_ROOT/scripts/build-mavlink-adapter.sh"
fi
viewer="$SIM_ROOT/build/cmake/ditto_fleet_viewer"
telemetry_probe="$SIM_ROOT/build/cmake/ditto_px4_telemetry_ready"
network_relay="$SIM_ROOT/build/cmake/ditto_network_relay"
if [[ ! -x "$viewer" || ! -x "$telemetry_probe" || ! -x "$network_relay" || "$SIM_ROOT/CMakeLists.txt" -nt "$viewer" ||
  "$SIM_ROOT/viewer/src/main.cpp" -nt "$viewer" || "$SIM_ROOT/network/relay.cpp" -nt "$network_relay" ||
  "$SIM_ROOT/tools/px4_telemetry_ready.cpp" -nt "$telemetry_probe" ]]
then
  "$SIM_ROOT/scripts/build-viewer.sh"
fi
# Always, not only when the binary is missing: cargo is incremental and a
# stale observer is worse than a few seconds of build. Gating on existence
# meant a changed observer was silently ignored in favour of the old binary.
"$SIM_ROOT/scripts/build-observer.sh"
fleet_started=false
cleanup() {
  local exit_code=$?
  trap - EXIT INT TERM HUP
  if [[ "$fleet_started" == true ]]; then
    "$SIM_ROOT/scripts/down.sh" || true
  fi
  exit "$exit_code"
}
trap cleanup EXIT INT TERM HUP

fleet_started=true
"$SIM_ROOT/scripts/up.sh"
"$SIM_ROOT/scripts/wait-telemetry.sh"
"$SIM_ROOT/scripts/viewer.sh"
