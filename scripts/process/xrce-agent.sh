#!/usr/bin/env bash
set -euo pipefail

source "$(dirname "${BASH_SOURCE[0]}")/../lib.sh"
sim_init
index="${1:?usage: xrce-agent.sh <vehicle-index>}"
[[ "$index" =~ ^[0-9]+$ ]] || die "vehicle index must be numeric"
agent="$SIM_XRCE_ROOT/bin/MicroXRCEAgent"
[[ -x "$agent" ]] || die "Micro XRCE-DDS Agent is not executable: $agent"
wait_for_fleet_edge_sockets
# The agent creates this vehicle's Fast DDS participant and every data writer on
# the client's behalf, so it is exposed to the shared-memory participant-creation
# wedge that process/ros-adapter.sh already avoids. Same profile, same reason: never
# construct the shared-memory transport. Its DDS peer is that adapter, which is
# already UDP-only, so the transports still match.
export FASTRTPS_DEFAULT_PROFILES_FILE="$SIM_ROOT/config/fastdds-udp.xml"
# The agent is linked @rpath-relative with no baked LC_RPATH, so its own lib
# directory and a Fast DDS runtime have to be named here. The latter is the ros
# pixi environment, which is also what built it.
export DYLD_LIBRARY_PATH="$SIM_XRCE_ROOT/lib:$SIM_PIXI_ENV_ROS/lib${DYLD_LIBRARY_PATH:+:$DYLD_LIBRARY_PATH}"
exec "$agent" udp4 -p "$((8888 + index))"
