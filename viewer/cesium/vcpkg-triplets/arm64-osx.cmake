set(VCPKG_TARGET_ARCHITECTURE arm64)
set(VCPKG_CRT_LINKAGE dynamic)
set(VCPKG_LIBRARY_LINKAGE static)
set(VCPKG_CMAKE_SYSTEM_NAME Darwin)
set(VCPKG_OSX_ARCHITECTURES arm64)

# Avoid compiling against the newest installed SDK, which can mark APIs as
# newer than the deployment target and turn warnings into errors in vcpkg ports.
execute_process(COMMAND sw_vers -productVersion OUTPUT_VARIABLE host_version
  OUTPUT_STRIP_TRAILING_WHITESPACE COMMAND_ERROR_IS_FATAL ANY)
string(REGEX MATCH "^[0-9]+" host_major "${host_version}")
file(GLOB sdk_candidates "/Library/Developer/CommandLineTools/SDKs/MacOSX${host_major}*.sdk")
if(NOT sdk_candidates)
  message(FATAL_ERROR "No macOS ${host_major} SDK installed for Cesium Native")
endif()
list(SORT sdk_candidates COMPARE NATURAL ORDER DESCENDING)
list(GET sdk_candidates 0 VCPKG_OSX_SYSROOT)
set(VCPKG_OSX_DEPLOYMENT_TARGET "${host_major}.0")
