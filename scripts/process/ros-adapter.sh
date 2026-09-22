#!/usr/bin/env bash
set -euo pipefail

source "$(dirname "${BASH_SOURCE[0]}")/../lib.sh"
sim_init
index="${1:?usage: ros-adapter.sh <vehicle-index>}"
[[ "$index" =~ ^[0-9]+$ ]] || die "vehicle index must be numeric"
export SIM_ROS_WORKSPACE="$SIM_ROOT/build/ros-gateway-host"
export SIM_ADAPTER_CONFIG="$SIM_EDGE_ADAPTERS_ROOT/adapters/ros2/px4_ditto_bridge/config/px4_0.yaml"
export SIM_VEHICLE_NAMESPACE="px4_$index"
export SIM_EDGE_SOCKET="$(node_socket "$SIM_VEHICLE_NAMESPACE")"
export ROS_DOMAIN_ID="$index"
binary="$(ros_adapter_binary)" ||
  die "no ROS adapter. Drop one at $SIM_ROOT/bin/px4_ditto_bridge_node, or run scripts/build/ros-adapter.sh"
wait_for_socket "$SIM_EDGE_SOCKET"
# Telemetry readiness is owned by the fleet coordinator.  Binding a second
# receiver here races that exclusive PX4 display port on macOS.
wait_for_file "$SIM_RUNTIME_DIR/px4-telemetry-ready" "${SIM_TELEMETRY_GATE_WAIT_TIMEOUT_SECONDS:-300}"
# This is a forwarding path; PX4 state generation takes precedence at startup.
set +u
source "$SIM_ROS_UNDERLAY"
# Only when the adapter was built here. A dropped binary has no overlay, and
# needs nothing from one beyond the underlay already sourced above.
[[ -r "$SIM_ROS_WORKSPACE/install/setup.bash" ]] && source "$SIM_ROS_WORKSPACE/install/setup.bash"
set -u
# Fast DDS 2.0.2 ignores FASTDDS_BUILTIN_TRANSPORTS. Load an explicit profile:
# never construct the shared-memory transport that can wedge this process.
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export FASTRTPS_DEFAULT_PROFILES_FILE="$SIM_ROOT/config/fastdds-udp.xml"
exec nice -n "${SIM_BACKGROUND_NICE:-10}" "$binary" --ros-args \
  --params-file "$SIM_ADAPTER_CONFIG" \
  -p vehicle_namespace:="$SIM_VEHICLE_NAMESPACE" \
  -p edge_server_socket:="$SIM_EDGE_SOCKET"
