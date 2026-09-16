#!/usr/bin/env bash
set -euo pipefail

source "$(dirname "$0")/lib.sh"
sim_init
index="${1:?usage: run-px4.sh <vehicle-index>}"
[[ "$index" =~ ^[0-9]+$ ]] || die "vehicle index must be numeric"
boot_timeout="${SIM_PX4_BOOT_TIMEOUT_SECONDS:-90}"
[[ "$boot_timeout" =~ ^[1-9][0-9]*$ ]] || die "SIM_PX4_BOOT_TIMEOUT_SECONDS must be a positive integer"

binary="$SIM_PX4_ROOT/build/px4_sitl_sih/bin/px4"
source_data="$SIM_PX4_ROOT/build/px4_sitl_sih/etc"
[[ -x "$binary" ]] || die "PX4 SIH binary is not executable: $binary"
px4_phase "px4_$index" launch
is_mavlink=false
is_mavlink_vehicle "$index" && is_mavlink=true
wait_for_fleet_edge_sockets
wait_for_fleet_xrce_listeners
stagger_px4_boot "$index"
px4_phase "px4_$index" barriers
rootfs="$(node_dir "px4_$index")/rootfs"
if [[ ! -f "$rootfs/.ditto-viewer-profile-v21" ]]; then
  [[ ! -e "$rootfs/etc" ]] || die "stale PX4 rootfs; restart the simulator"
  mkdir -p "$rootfs"
  cp -cR "$source_data" "$rootfs/etc"
  mavlink="$rootfs/etc/init.d-posix/px4-rc.mavlink"
  # The ROS 2 vehicles need no MAVLink control link; MAVLink vehicles need one.
  # Camera, gimbal, and GCS links are unused in this simulator.
  sed -i '' '/# GCS link/,/# API\/Offboard link/{ /# API\/Offboard link/!d; }' "$mavlink"
  sed -i '' '/# API\/Offboard link/d' "$mavlink"
  sed -i '' '/mavlink start -x -u $udp_offboard_port_local/d' "$mavlink"
  sed -i '' '/use the same ports for more than 10 instances/d' "$mavlink"
  sed -i '' '/mavlink start -x -u $udp_onboard_payload_port_local/d' "$mavlink"
  sed -i '' '/mavlink start -x -u $udp_onboard_gimbal_port_local/d' "$mavlink"
  # Use PX4's stable onboard stream set on the viewer link. Its position and
  # attitude streams are established atomically with the link.
  sed -i '' 's/-u $udp_sihsim_port_local -r 400000 -m custom/-u $udp_sihsim_port_local -r 100000 -f -m onboard/' "$mavlink"
  sed -i '' '/mavlink start -x -u $udp_sihsim_port_local/a\
if [ "$PX4_DITTO_MAVLINK" = "1" ]; then\
mavlink start -x -u $udp_offboard_port_local -r 100000 -f -m onboard -o $udp_offboard_port_remote\
fi\
true' "$mavlink"
  sed -i '' '/HIL_ACTUATOR_CONTROLS/d' "$mavlink"
  sed -i '' '/HIL_STATE_QUATERNION/d' "$mavlink"
  # Bound every daemon round-trip rcS makes (~200 per vehicle, no timeout in
  # PX4): a hung px4-<cmd> is killed and retried, and three hangs abort rcS so
  # the vehicle reports failed instead of wedging forever.
  cp "$SIM_ROOT/config/px4-bounded.sh" "$rootfs/etc/init.d-posix/px4-bounded.sh"
  sed -i '' "s|^\. px4-alias\.sh\$|. $rootfs/etc/init.d-posix/px4-bounded.sh|" "$rootfs/etc/init.d-posix/rcS"
  grep -qF "px4-bounded.sh" "$rootfs/etc/init.d-posix/rcS" || die "could not bound PX4 startup calls"
  # Native MAVLink vehicles do not use DDS. Avoid starting ten disconnected
  # XRCE clients during fleet boot.
  #
  # UXRCE_DDS_PTCFG=1 builds the agent-side participant for loopback only: no
  # shared memory, no interface enumeration, no multicast. Everything here is on
  # 127.0.0.1. The default (0, full builtin transports) makes participant
  # creation exceed the 1000 ms the client allows it
  # (uxrce_dds_client.cpp:312), stranding a vehicle with
  # "create entities failed: participant: 255" and no recovery path.
  sed -i '' 's/^uxrce_dds_client start -t udp -p \$uxrce_dds_port \$uxrce_dds_ns$/[ "$PX4_DITTO_UXRCE" = "1" ] \&\& { param set UXRCE_DDS_PTCFG 1; uxrce_dds_client start -t udp -p $uxrce_dds_port $uxrce_dds_ns; }/' "$rootfs/etc/init.d-posix/rcS"
  grep -qF 'param set UXRCE_DDS_PTCFG 1; uxrce_dds_client start' "$rootfs/etc/init.d-posix/rcS" || die "could not gate PX4 XRCE startup"
  touch "$rootfs/.ditto-viewer-profile-v21"
fi
data="$rootfs/etc"
location_file="$SIM_PROTOTYPE_ROOT/.location.env"
if [[ -r "$location_file" ]]; then
  # shellcheck disable=SC1090
  source "$location_file"
fi
export ROS_DOMAIN_ID="$index"
export PX4_DITTO_MAVLINK=0 PX4_DITTO_UXRCE=1
if [[ "$is_mavlink" == true ]]; then
  PX4_DITTO_MAVLINK=1
  PX4_DITTO_UXRCE=0
else
  export PX4_UXRCE_DDS_NS="px4_$index"
  export PX4_UXRCE_DDS_PORT="$((8888 + index))"
fi
export PX4_SIM_MODEL=sihsim_quadx PX4_SIMULATOR=sihsim
export PX4_HOME_LAT PX4_HOME_LON PX4_HOME_ALT
runtime="$(node_dir "px4_$index")/px4"
mkdir -p "$runtime"
cd "$runtime"
: >px4.log
px4_phase "px4_$index" exec
exec "$binary" -i "$index" -d "$data" >>px4.log 2>&1
