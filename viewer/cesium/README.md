# Optional Cesium Native SDK

The default simulator build does not enter this directory or fetch Cesium. To
build the optional SDK in an isolated build tree:

```sh
pixi run build-cesium-native
```

This produces `build/cmake-cesium/ditto_fleet_viewer_cesium`. A scenario can
select it with `SIM_VIEWER=cesium-native`; all other scenarios keep using the
ordinary raylib viewer and its existing static world layer.

Cesium Native is pinned to commit `80a22ff4337c5b7057cff53d0055045c15c6d350`.
Its vcpkg checkout, installed packages, and build products stay under
`build/cmake-cesium`; the SDK is exposed only as `Ditto::CesiumNative` to
consumers in this optional CMake configuration. The arm64 macOS triplet selects
an SDK matching the host macOS major version to avoid newer-SDK API checks in
third-party ports.
