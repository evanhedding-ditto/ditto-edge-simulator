#!/usr/bin/env bash
set -euo pipefail

source "$(dirname "${BASH_SOURCE[0]}")/../lib.sh"
sim_init
workspace="$SIM_ROOT/build/ros-gateway-host"
mkdir -p "$workspace"
source_package="$SIM_EDGE_ADAPTERS_ROOT/adapters/ros2/px4_ditto_bridge"
underlay="$SIM_ROS_UNDERLAY"
: "${SIM_GRPC_DIR:=$SIM_PIXI_ENV_ROS/lib/cmake/grpc}"
: "${SIM_PROTOBUF_DIR:=$SIM_PIXI_ENV_ROS/lib/cmake/protobuf}"
[[ -d "$source_package" ]] || die "ROS adapter source does not exist: $source_package"
[[ -r "$underlay" ]] || die "PX4 ROS underlay is not built: $underlay (run scripts/build/ros-underlay.sh)"
[[ -r "$SIM_GRPC_DIR/gRPCConfig.cmake" ]] || die "set SIM_GRPC_DIR to gRPC's CMake package"
[[ -r "$SIM_PROTOBUF_DIR/protobuf-config.cmake" ]] || die "set SIM_PROTOBUF_DIR to Protobuf's CMake package"
exec pixi run -e ros --manifest-path "$SIM_ROOT/pixi.toml" bash -lc \
  'source "$1" && export CC=/usr/bin/cc CXX=/usr/bin/c++ CMAKE_PREFIX_PATH="$6:${CMAKE_PREFIX_PATH:-}" && cd "$2" && colcon build --symlink-install --base-paths "$3" --packages-select px4_ditto_bridge --cmake-args -DBUILD_TESTING=OFF -DgRPC_DIR="$4" -DProtobuf_DIR="$5"' \
  bash "$underlay" "$workspace" "$source_package" "$SIM_GRPC_DIR" "$SIM_PROTOBUF_DIR" "$SIM_PIXI_ENV_ROS"
