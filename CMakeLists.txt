cmake_minimum_required(VERSION 3.20)
project(ditto_edge_simulator LANGUAGES CXX)

option(DITTO_ENABLE_CESIUM_NATIVE "Build the opt-in Cesium Native map viewer" OFF)

set(DITTO_EDGE_ADAPTERS_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../Ditto-Edge-Server/ditto-edge-adapters" CACHE PATH
  "Ditto Edge Adapters checkout")
set(PX4_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../PX4-Autopilot" CACHE PATH "PX4 checkout")
if(NOT EXISTS "${DITTO_EDGE_ADAPTERS_ROOT}/sdk/cpp/CMakeLists.txt")
  message(FATAL_ERROR "DITTO_EDGE_ADAPTERS_ROOT must contain sdk/cpp/CMakeLists.txt")
endif()
if(NOT EXISTS "${DITTO_EDGE_ADAPTERS_ROOT}/observer/sdk/cpp/CMakeLists.txt")
  message(FATAL_ERROR "DITTO_EDGE_ADAPTERS_ROOT must contain observer/sdk/cpp/CMakeLists.txt")
endif()
if(NOT EXISTS "${PX4_ROOT}/build/px4_sitl_sih/mavlink/common/mavlink.h")
  message(FATAL_ERROR "PX4_ROOT must contain a px4_sitl_sih build with generated MAVLink headers")
endif()

find_package(nlohmann_json CONFIG REQUIRED)
find_package(raylib 6.0 QUIET)
if(NOT raylib_FOUND)
  include(FetchContent)
  FetchContent_Declare(raylib
    # The release tarball, 50 MB; a git clone, even a shallow one, is ~400 MB.
    URL https://github.com/raysan5/raylib/archive/refs/tags/6.0.tar.gz
    URL_HASH SHA256=2b3ee1e2120c7a0796b33062c7e9a694dd8a8caa56a96319ac8c8ecf54a90d0b
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE)
  set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
  FetchContent_MakeAvailable(raylib)
endif()
add_subdirectory("${DITTO_EDGE_ADAPTERS_ROOT}/sdk/cpp" ditto_edge_client)
add_subdirectory("${DITTO_EDGE_ADAPTERS_ROOT}/observer/sdk/cpp" ditto_observer_client)

# Vehicle camera feeds (viewer/src/video.cpp): GL readback, hardware H.264.
set(DITTO_VIDEO_FRAMEWORKS "-framework OpenGL" "-framework VideoToolbox" "-framework CoreMedia"
  "-framework CoreVideo" "-framework CoreFoundation" "-framework Foundation" objc)

add_executable(ditto_fleet_viewer viewer/src/main.cpp viewer/src/video.cpp)
target_compile_features(ditto_fleet_viewer PRIVATE cxx_std_17)
target_compile_options(ditto_fleet_viewer PRIVATE -Wall -Wextra -Wpedantic -Wno-missing-field-initializers)
target_include_directories(ditto_fleet_viewer PRIVATE "${PX4_ROOT}/build/px4_sitl_sih")
target_link_libraries(ditto_fleet_viewer PRIVATE
  raylib nlohmann_json::nlohmann_json DittoObserver::Client DittoEdge::Client ${DITTO_VIDEO_FRAMEWORKS})

if(DITTO_ENABLE_CESIUM_NATIVE)
  add_subdirectory(viewer/cesium)
endif()

add_executable(ditto_px4_telemetry_ready tools/px4_telemetry_ready.cpp)
target_compile_features(ditto_px4_telemetry_ready PRIVATE cxx_std_17)
target_compile_options(ditto_px4_telemetry_ready PRIVATE -Wall -Wextra -Wpedantic -Wno-missing-field-initializers)
target_include_directories(ditto_px4_telemetry_ready PRIVATE "${PX4_ROOT}/build/px4_sitl_sih")

# Ground stations on the LAN reach each PX4's GCS link through this.
add_executable(ditto_gcs_relay tools/gcs_relay.cpp)
target_compile_features(ditto_gcs_relay PRIVATE cxx_std_17)
target_compile_options(ditto_gcs_relay PRIVATE -Wall -Wextra -Wpedantic)

add_executable(ditto_network_relay network/relay.cpp)
target_compile_features(ditto_network_relay PRIVATE cxx_std_17)
target_compile_options(ditto_network_relay PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(ditto_network_relay PRIVATE nlohmann_json::nlohmann_json)

add_executable(ditto_fleet_command tools/fleet_command.cpp)
target_compile_features(ditto_fleet_command PRIVATE cxx_std_17)
target_compile_options(ditto_fleet_command PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(ditto_fleet_command PRIVATE DittoEdge::Client nlohmann_json::nlohmann_json)

# Synthetic vehicle fleet: emulated PX4 autopilots, MAVLink only. It links
# nothing from the adapters repository and holds no Edge Server connection --
# a real adapter process per vehicle does every Ditto write, exactly as it does
# for real PX4. All it needs is the MAVLink dialect PX4 generates.
add_executable(ditto_synthetic_fleet tools/synthetic_fleet.cpp)
target_compile_features(ditto_synthetic_fleet PRIVATE cxx_std_17)
target_compile_options(ditto_synthetic_fleet PRIVATE -Wall -Wextra -Wpedantic -Wno-missing-field-initializers)
target_include_directories(ditto_synthetic_fleet PRIVATE "${PX4_ROOT}/build/px4_sitl_sih")

# Factory robot fleet: ground robots driving a multi-level building. Same shape
# as the synthetic fleet -- MAVLink only, nothing from the adapters repository,
# no Edge Server connection -- and additionally reads the world file, on the
# vehicle side of the MAVLink boundary, to know where it may drive.
add_executable(ditto_factory_fleet tools/factory_fleet.cpp)
target_compile_features(ditto_factory_fleet PRIVATE cxx_std_17)
target_compile_options(ditto_factory_fleet PRIVATE -Wall -Wextra -Wpedantic -Wno-missing-field-initializers)
target_include_directories(ditto_factory_fleet PRIVATE "${PX4_ROOT}/build/px4_sitl_sih")
target_link_libraries(ditto_factory_fleet PRIVATE nlohmann_json::nlohmann_json)
