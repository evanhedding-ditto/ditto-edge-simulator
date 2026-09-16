cmake_minimum_required(VERSION 3.20)
project(ditto_edge_simulator LANGUAGES CXX)

set(DITTO_EDGE_ADAPTERS_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../Ditto-Edge-Server/ditto-edge-adapters" CACHE PATH
  "Ditto Edge Adapters checkout")
set(PX4_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../PX4-Autopilot" CACHE PATH "PX4 checkout")
if(NOT EXISTS "${DITTO_EDGE_ADAPTERS_ROOT}/sdk/cpp/CMakeLists.txt")
  message(FATAL_ERROR "DITTO_EDGE_ADAPTERS_ROOT must contain sdk/cpp/CMakeLists.txt")
endif()
if(NOT EXISTS "${PX4_ROOT}/build/px4_sitl_sih/mavlink/common/mavlink.h")
  message(FATAL_ERROR "PX4_ROOT must contain a px4_sitl_sih build with generated MAVLink headers")
endif()

find_package(nlohmann_json CONFIG REQUIRED)
find_package(raylib 6.0 QUIET)
if(NOT raylib_FOUND)
  include(FetchContent)
  FetchContent_Declare(raylib
    GIT_REPOSITORY https://github.com/raysan5/raylib.git
    GIT_TAG 6.0)
  set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
  FetchContent_MakeAvailable(raylib)
endif()
add_subdirectory("${DITTO_EDGE_ADAPTERS_ROOT}/sdk/cpp" ditto_edge_client)

add_executable(ditto_fleet_viewer viewer/src/main.cpp)
target_compile_features(ditto_fleet_viewer PRIVATE cxx_std_17)
target_compile_options(ditto_fleet_viewer PRIVATE -Wall -Wextra -Wpedantic -Wno-missing-field-initializers)
target_include_directories(ditto_fleet_viewer PRIVATE "${PX4_ROOT}/build/px4_sitl_sih")
target_link_libraries(ditto_fleet_viewer PRIVATE raylib nlohmann_json::nlohmann_json)

add_executable(ditto_px4_telemetry_ready tools/px4_telemetry_ready.cpp)
target_compile_features(ditto_px4_telemetry_ready PRIVATE cxx_std_17)
target_compile_options(ditto_px4_telemetry_ready PRIVATE -Wall -Wextra -Wpedantic -Wno-missing-field-initializers)
target_include_directories(ditto_px4_telemetry_ready PRIVATE "${PX4_ROOT}/build/px4_sitl_sih")

add_executable(ditto_network_relay network/relay.cpp)
target_compile_features(ditto_network_relay PRIVATE cxx_std_17)
target_compile_options(ditto_network_relay PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(ditto_network_relay PRIVATE nlohmann_json::nlohmann_json)

add_executable(ditto_fleet_command tools/fleet_command.cpp)
target_compile_features(ditto_fleet_command PRIVATE cxx_std_17)
target_compile_options(ditto_fleet_command PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(ditto_fleet_command PRIVATE DittoEdge::Client nlohmann_json::nlohmann_json)

