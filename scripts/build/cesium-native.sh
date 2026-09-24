#!/usr/bin/env bash
set -euo pipefail

source "$(dirname "${BASH_SOURCE[0]}")/../lib.sh"
sim_init
build_dir="$SIM_ROOT/build/cmake-cesium"
cmake -S "$SIM_ROOT" -B "$build_dir" \
  -DCMAKE_PREFIX_PATH="$SIM_PIXI_ENV" \
  -DCMAKE_BUILD_TYPE=Release \
  -DDITTO_EDGE_ADAPTERS_ROOT="$SIM_EDGE_ADAPTERS_ROOT" \
  -DPX4_ROOT="$SIM_PX4_ROOT" \
  -DDITTO_ENABLE_CESIUM_NATIVE=ON \
  -DDITTO_CESIUM_VCPKG_DIR="$build_dir/viewer/cesium/vcpkg" \
  -UFETCHCONTENT_SOURCE_DIR_CESIUM_NATIVE  # the pinned fetch, not a stray local checkout
cmake --build "$build_dir" --target ditto_fleet_viewer_cesium --parallel "$(getconf _NPROCESSORS_ONLN)"
