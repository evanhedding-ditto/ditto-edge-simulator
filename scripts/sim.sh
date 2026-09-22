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

# Only a scenario with ROS vehicles builds the ROS tier. This used to be
# unconditional, so a synthetic or all-MAVLink fleet built -- and first installed
# ROS 2 Jazzy, colcon and px4_msgs for -- an adapter it never launched.
if scenario_uses_ros; then
  [[ -r "$SIM_ROS_UNDERLAY" ]] || "$SIM_ROOT/scripts/build/ros-underlay.sh"
  ros_adapter_binary >/dev/null || "$SIM_ROOT/scripts/build/ros-adapter.sh"
  [[ -x "$SIM_XRCE_ROOT/bin/MicroXRCEAgent" ]] || "$SIM_ROOT/scripts/build/xrce-agent.sh"
fi
# A synthetic scenario names no MAVLink vehicles -- every vehicle is one -- so
# both conditions have to be checked.
if [[ -n "${SIM_MAVLINK_VEHICLES:-}" || "${SIM_SYNTHETIC_FLEET:-0}" == 1 ]]; then
  # Asking the resolver rather than testing bin/ directly: an override or a
  # drop-in then skips the build, instead of building something launch ignores.
  mavlink_adapter_binary >/dev/null || "$SIM_ROOT/scripts/build/mavlink-adapter.sh"
fi
viewer="$SIM_ROOT/build/cmake/ditto_fleet_viewer"
telemetry_probe="$SIM_ROOT/build/cmake/ditto_px4_telemetry_ready"
network_relay="$SIM_ROOT/build/cmake/ditto_network_relay"
synthetic_fleet="$SIM_ROOT/build/cmake/ditto_synthetic_fleet"
factory_fleet="$SIM_ROOT/build/cmake/ditto_factory_fleet"
# Every source listed here is one whose staleness has bitten before: a binary
# that exists but predates its source is silently wrong, and nothing downstream
# says so.
if [[ ! -x "$viewer" || ! -x "$telemetry_probe" || ! -x "$network_relay" ||
  ! -x "$synthetic_fleet" || ! -x "$factory_fleet" || "$SIM_ROOT/CMakeLists.txt" -nt "$viewer" ||
  "$SIM_ROOT/viewer/src/main.cpp" -nt "$viewer" || "$SIM_ROOT/network/relay.cpp" -nt "$network_relay" ||
  "$SIM_ROOT/tools/px4_telemetry_ready.cpp" -nt "$telemetry_probe" ||
  "$SIM_ROOT/tools/synthetic_fleet.cpp" -nt "$synthetic_fleet" ||
  "$SIM_ROOT/tools/factory_fleet.cpp" -nt "$factory_fleet" ]]
then
  "$SIM_ROOT/scripts/build/viewer.sh"
fi
# Always, not only when the binary is missing: cargo is incremental and a
# stale observer is worse than a few seconds of build. Gating on existence
# meant a changed observer was silently ignored in favour of the old binary.
"$SIM_ROOT/scripts/build/observer.sh"
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
"$SIM_ROOT/scripts/session/up.sh"
"$SIM_ROOT/scripts/session/wait-telemetry.sh"
"$SIM_ROOT/scripts/session/viewer.sh"
