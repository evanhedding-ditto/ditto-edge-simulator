# This directory is deliberately absent from ordinary viewer builds. It owns
# the pinned Cesium source and its separate vcpkg state.
include(FetchContent)

set(CESIUM_USE_EZVCPKG ON CACHE BOOL "" FORCE)
set(CESIUM_TESTS_ENABLED OFF CACHE BOOL "" FORCE)
set(CESIUM_ENABLE_CLANG_TIDY OFF CACHE BOOL "" FORCE)
set(CESIUM_INSTALL_STATIC_LIBS ON CACHE BOOL "" FORCE)
set(CESIUM_INSTALL_HEADERS ON CACHE BOOL "" FORCE)
set(DITTO_CESIUM_VCPKG_DIR "${CMAKE_CURRENT_BINARY_DIR}/vcpkg" CACHE PATH
  "Cesium Native's isolated vcpkg checkout and installed packages")
set(EZVCPKG_BASEDIR "${DITTO_CESIUM_VCPKG_DIR}" CACHE PATH "" FORCE)
# Cesium's static S2 library must resolve Abseil from the same vcpkg install;
# otherwise the simulator's Pixi Abseil can win and fail at link time.
set(_cesium_vcpkg_prefix
  "${DITTO_CESIUM_VCPKG_DIR}/56bb2411609227288b70117ead2c47585ba07713/installed/arm64-osx")
set(CMAKE_PREFIX_PATH "${_cesium_vcpkg_prefix};${CMAKE_PREFIX_PATH}")
set(_cesium_triplet_overlays "${CMAKE_CURRENT_SOURCE_DIR}/vcpkg-triplets")
if(DEFINED ENV{VCPKG_OVERLAY_TRIPLETS})
  list(APPEND _cesium_triplet_overlays $ENV{VCPKG_OVERLAY_TRIPLETS})
endif()
set(ENV{VCPKG_OVERLAY_TRIPLETS} "${_cesium_triplet_overlays}")

FetchContent_Declare(cesium_native
  GIT_REPOSITORY https://github.com/CesiumGS/cesium-native.git
  GIT_TAG 80a22ff4337c5b7057cff53d0055045c15c6d350
  GIT_SHALLOW FALSE)
FetchContent_MakeAvailable(cesium_native)
file(GLOB _cesium_absl_archives "${_cesium_vcpkg_prefix}/lib/libabsl_*.a")

# Cesium Native provides tile selection and streaming. Consumers link through
# this target only from other targets inside the optional module.
add_library(ditto_cesium_native INTERFACE)
target_link_libraries(ditto_cesium_native INTERFACE Cesium3DTilesSelection)
add_library(Ditto::CesiumNative ALIAS ditto_cesium_native)

add_executable(ditto_fleet_viewer_cesium
  ../src/main.cpp
  raylib_tileset.cpp)
set_target_properties(ditto_fleet_viewer_cesium PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")
target_compile_features(ditto_fleet_viewer_cesium PRIVATE cxx_std_17)
target_compile_definitions(ditto_fleet_viewer_cesium PRIVATE DITTO_CESIUM_VIEWER=1)
target_compile_options(ditto_fleet_viewer_cesium PRIVATE -Wall -Wextra -Wpedantic -Wno-missing-field-initializers)
target_include_directories(ditto_fleet_viewer_cesium PRIVATE
  "${PX4_ROOT}/build/px4_sitl_sih"
  "${CMAKE_CURRENT_SOURCE_DIR}")
target_link_libraries(ditto_fleet_viewer_cesium PRIVATE
  raylib nlohmann_json::nlohmann_json DittoObserver::Client
  Ditto::CesiumNative CesiumCurl ${_cesium_absl_archives})
