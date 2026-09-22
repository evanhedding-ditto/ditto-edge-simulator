#!/usr/bin/env bash

die() { echo "error: $*" >&2; exit 1; }

# An external checkout, preferring deps/ -- populated from dependencies.repos --
# and falling back to a sibling directory, which is how this tree was laid out
# before 2026-09-22. Both work, so an existing checkout needs no rearranging.
#
# The marker is .git, not the directory: an interrupted clone leaves an empty
# deps/<name> behind, and a bare -d test would prefer that over a working
# sibling checkout and then fail somewhere less obvious. (.git is a file in a
# submodule and a directory otherwise, hence -e.)
sim_dep() {
  if [[ -e "$SIM_ROOT/deps/$1/.git" ]]; then printf '%s/deps/%s\n' "$SIM_ROOT" "$1"
  else printf '%s/../%s\n' "$SIM_ROOT" "$1"; fi
}

sim_init() {
  SIM_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
  : "${SIM_SCENARIO_FILE:=$SIM_ROOT/scenarios/mvp-two-px4.env}"
  [[ -r "$SIM_SCENARIO_FILE" ]] || die "scenario not readable: $SIM_SCENARIO_FILE"
  # shellcheck disable=SC1090
  source "$SIM_SCENARIO_FILE"
  : "${SIM_PX4_ROOT:=$(sim_dep PX4-Autopilot)}"
  # The ROS underlay: px4_msgs, built by scripts/build/ros-underlay.sh. Only
  # scenarios with ROS vehicles need it.
  : "${SIM_ROS_UNDERLAY:=$SIM_ROOT/build/ros-underlay/install/setup.bash}"
  # Micro XRCE-DDS Agent, likewise ROS-only. The BUILT agent, so it lives under
  # build/; deps/ holds only fetched source.
  : "${SIM_XRCE_ROOT:=$SIM_ROOT/build/micro-xrce-dds-agent}"
  # gRPC, Protobuf and nlohmann_json come from the pixi environment, not from
  # Homebrew. Naming the prefix explicitly matters: without it CMake finds
  # /opt/homebrew on its own, and a build would silently link whatever version
  # brew happens to hold rather than the one pinned in pixi.lock.
  : "${SIM_PIXI_ENV:=$SIM_ROOT/.pixi/envs/default}"
  : "${SIM_PIXI_ENV_ROS:=$SIM_ROOT/.pixi/envs/ros}"
  : "${SIM_EDGE_SERVER_ROOT:=$(sim_dep Ditto-Edge-Server)}"
  # Vendored inside the server checkout, so it follows wherever that resolved.
  : "${SIM_EDGE_ADAPTERS_ROOT:=$SIM_EDGE_SERVER_ROOT/ditto-edge-adapters}"
  # Derived from the scenario just sourced, never carried over: down.sh calls
  # sim_init once per scenario in one shell, and an exported :=default would
  # pin the first scenario's directory while SIM_VEHICLE_COUNT changed under it.
  [[ -n "${SIM_SCENARIO_ID:-}" ]] || die "scenario defines no SIM_SCENARIO_ID: $SIM_SCENARIO_FILE"
  # Space-separated indices only. A comma or a stray character would match no
  # vehicle, every vehicle would count as a ROS vehicle, and the punishment is a
  # silent ROS 2 install for a fleet that never launches ROS.
  [[ "${SIM_MAVLINK_VEHICLES:-}" =~ ^[0-9\ ]*$ ]] ||
    die "SIM_MAVLINK_VEHICLES must be space-separated indices: ${SIM_MAVLINK_VEHICLES}"
  SIM_RUNTIME_DIR="$SIM_ROOT/build/runtime/$SIM_SCENARIO_ID"
  : "${DITTO_EDGE_ENV_FILE:=$SIM_ROOT/.env}"
  # The network observer's API. One address shared by the observer process, the
  # viewer and any ad-hoc client, so they cannot disagree. Only one simulator
  # session runs at a time, so a fixed port is safe.
  : "${SIM_OBSERVER_ADDR:=127.0.0.1:50090}"
  export SIM_OBSERVER_ADDR
  # Geodetic origin. This was a .location.env dotfile in ditto-autonomy-testing
  # that four scripts read silently; it is an ordinary default now, and a
  # scenario may override it by setting either value before this runs.
  : "${PX4_HOME_LAT:=36.01883233670948}"
  : "${PX4_HOME_LON:=-78.9684198511774}"
  export PX4_HOME_LAT PX4_HOME_LON
  export SIM_ROOT SIM_SCENARIO_FILE SIM_PX4_ROOT SIM_ROS_UNDERLAY SIM_XRCE_ROOT
  export SIM_PIXI_ENV SIM_PIXI_ENV_ROS
  export SIM_EDGE_ADAPTERS_ROOT SIM_EDGE_SERVER_ROOT SIM_RUNTIME_DIR
}

# Process Compose always runs in this repo's pixi environment and always against
# the running scenario's controller port. Four call sites needed both.
sim_compose() {
  pixi run --manifest-path "$SIM_ROOT/pixi.toml" process-compose \
    -p "$SIM_PROCESS_COMPOSE_PORT" "$@"
}

load_ditto_credentials() {
  [[ -r "$DITTO_EDGE_ENV_FILE" ]] || die "set DITTO_EDGE_ENV_FILE to a readable Ditto credential file"
  set -a
  # shellcheck disable=SC1090
  source "$DITTO_EDGE_ENV_FILE"
  set +a
  : "${DITTO_DB_ID:?DITTO_DB_ID is required}"
  : "${DITTO_AUTH_URL:?DITTO_AUTH_URL is required}"
  : "${DITTO_ACCESS_TOKEN:?DITTO_ACCESS_TOKEN is required}"
}

# Every binary the simulator launches resolves the same way, in order:
#
#   1. the named environment variable   an explicit path, wins over everything
#   2. bin/<name>                       drop a prebuilt build in and go
#   3. the local build output           whatever the build script produces
#
# Step 2 is what lets a teammate run the simulator without building a component;
# bin/ is gitignored. These return 1 rather than dying, so that sim.sh can ask
# "do I need to build this?" with the same code that launch uses, and a caller
# can say what to do about it -- which it knows and this does not.
sim_find_binary() {
  local name="$1" override="$2" candidate; shift 2
  if [[ -n "${!override:-}" ]]; then
    [[ -x "${!override}" ]] || die "$override is not executable: ${!override}"
    printf '%s\n' "${!override}"
    return 0
  fi
  for candidate in "$SIM_ROOT/bin/$name" "$@"; do
    [[ -x "$candidate" ]] || continue
    printf '%s\n' "$candidate"
    return 0
  done
  return 1
}

edge_server_binary() {
  sim_find_binary ditto-edge-server DITTO_EDGE_SERVER_BIN \
    "$SIM_EDGE_SERVER_ROOT/ditto-edge-server/target/release/ditto-edge-server" \
    "$SIM_EDGE_SERVER_ROOT/ditto-edge-server/target/debug/ditto-edge-server"
}

mavlink_adapter_binary() {
  sim_find_binary px4-mavlink-ditto-bridge DITTO_MAVLINK_ADAPTER_BIN \
    "$SIM_ROOT/build/mavlink-adapter/px4-mavlink-ditto-bridge"
}

ros_adapter_binary() {
  sim_find_binary px4_ditto_bridge_node DITTO_ROS_ADAPTER_BIN \
    "$SIM_ROOT/build/ros-gateway-host/install/px4_ditto_bridge/lib/px4_ditto_bridge/px4_ditto_bridge_node"
}

node_dir() { printf '%s/nodes/%s\n' "$SIM_RUNTIME_DIR" "$1"; }
node_socket() { printf '%s/edge.sock\n' "$(node_dir "$1")"; }

# Synthetic control link, by vehicle index. The synthetic vehicle binds the
# autopilot port and streams to the adapter port; the adapter binds the adapter
# port and learns where to reply from what arrives.
#
# PX4's own numbering cannot be reused: its pair is 14540+i and 14580+i, which
# collide with each other at more than 40 vehicles. The 1000-port gap here
# covers the tool's 255-vehicle ceiling, and both ranges sit clear of the relay
# and of 19410-19429, which session/wait-telemetry.sh owns.
synthetic_adapter_port() { printf '%s\n' "$(( ${SIM_SYNTHETIC_ADAPTER_PORT_BASE:-24540} + $1 ))"; }
synthetic_autopilot_port() { printf '%s\n' "$(( ${SIM_SYNTHETIC_AUTOPILOT_PORT_BASE:-25540} + $1 ))"; }

# Startup phase timestamps, one line per event, written outside nodes/ so they
# survive teardown.  Twenty vehicles append concurrently; each line is far under
# PIPE_BUF, so O_APPEND writes do not interleave.
px4_phase() {
  printf '%s\t%s\t%s\n' "$1" "$2" "$(( $(date +%s) * 1000 ))" \
    >>"$SIM_RUNTIME_DIR/px4-phases.tsv" 2>/dev/null || true
}

known_simulator_ports() {
  local scenario port
  {
    printf '%s\n' "$SIM_PROCESS_COMPOSE_PORT"
    for scenario in "$SIM_ROOT"/scenarios/*.env; do
      port="$(bash -c 'source "$1"; printf %s "$SIM_PROCESS_COMPOSE_PORT"' _ "$scenario")"
      [[ "$port" =~ ^[1-9][0-9]*$ ]] && printf '%s\n' "$port"
    done
  } | sort -nu
}

simulator_running() {
  /usr/sbin/lsof -nP -iTCP:"$1" -sTCP:LISTEN >/dev/null 2>&1
}

wait_for_socket() {
  local socket="${1:?usage: wait_for_socket <path>}"
  local deadline="$((SECONDS + ${2:-180}))"
  while [[ ! -S "$socket" ]]; do
    (( SECONDS < deadline )) || die "timed out waiting for socket: $socket"
    sleep 1
  done
}

wait_for_file() {
  local file="${1:?usage: wait_for_file <path>}"
  local deadline="$((SECONDS + ${2:-180}))"
  while [[ ! -f "$file" ]]; do
    (( SECONDS < deadline )) || die "timed out waiting for file: $file"
    sleep 1
  done
}

wait_for_udp_listener() {
  local port="${1:?usage: wait_for_udp_listener <port> [timeout]}"
  local deadline="$((SECONDS + ${2:-60}))"
  while ! /usr/sbin/lsof -nP -iUDP:"$port" >/dev/null 2>&1; do
    (( SECONDS < deadline )) || die "timed out waiting for UDP listener: $port"
    sleep 1
  done
}

wait_for_fleet_edge_sockets() {
  local index
  for ((index = 0; index < SIM_VEHICLE_COUNT; ++index)); do
    wait_for_socket "$(node_socket "px4_$index")"
  done
  wait_for_socket "$(node_socket operator)"
}

wait_for_fleet_xrce_listeners() {
  local index mavlink vehicle
  for ((index = 0; index < SIM_VEHICLE_COUNT; ++index)); do
    mavlink=false
    for vehicle in ${SIM_MAVLINK_VEHICLES:-}; do
      [[ "$vehicle" == "$index" ]] && mavlink=true
    done
    [[ "$mavlink" == true ]] || wait_for_udp_listener "$((8888 + index))"
  done
}

is_mavlink_vehicle() {
  local vehicle
  for vehicle in ${SIM_MAVLINK_VEHICLES:-}; do
    [[ "$vehicle" == "$1" ]] && return 0
  done
  return 1
}

# Does this scenario run any ROS 2 vehicle? A synthetic fleet is MAVLink
# throughout, and a scenario may list every vehicle in SIM_MAVLINK_VEHICLES.
# Nothing in the ROS tier -- the underlay, the adapter, the XRCE agent, or the
# ros pixi environment -- is built or installed when this is false, which is what
# keeps a synthetic-only user from paying for ROS 2 they never run.
scenario_uses_ros() {
  local index
  [[ "${SIM_SYNTHETIC_FLEET:-0}" == 1 ]] && return 1
  for ((index = 0; index < SIM_VEHICLE_COUNT; ++index)); do
    is_mavlink_vehicle "$index" || return 0
  done
  return 1
}

# Launcher-side startup visibility. Each tier reports as it comes up, so a stall
# names the tier and the component instead of appearing as one silent wait
# before the first PX4 line. These reuse the readiness signals the PX4 launcher
# already gates on, so they add no new failure mode.
report_tier() {
  local label="$1" noun="$2" timeout="$3" check="$4"; shift 4
  local -a names state pending
  names=("$@")
  local total="${#names[@]}" deadline="$((SECONDS + timeout))" next=0 previous=-1
  local i ready list
  for ((i = 0; i < total; ++i)); do state[i]=0; done
  while :; do
    ready=0; pending=()
    for ((i = 0; i < total; ++i)); do
      if (( state[i] )) || "$check" "${names[i]}"; then
        state[i]=1; ready="$((ready + 1))"
      else
        pending+=("${names[i]}")
      fi
    done
    if (( ready == total )); then
      printf '[%s] %s/%s %s\n' "$label" "$ready" "$total" "$noun"
      return
    fi
    (( SECONDS < deadline )) || die "timed out waiting for $noun: ${pending[*]}"
    if (( ready != previous )) || (( SECONDS >= next )); then
      list="${pending[*]}"
      (( ${#pending[@]} <= 6 )) || list="${pending[*]:0:6} (+$(( ${#pending[@]} - 6 )) more)"
      printf '[%s] %s/%s %s; waiting for %s\n' "$label" "$ready" "$total" "$noun" "$list"
      previous="$ready"; next="$((SECONDS + 5))"
    fi
    sleep 1
  done
}

# Optional cap on the concurrent PX4 boot burst. Twenty simultaneous execs plus
# twenty rootfs copies drive the load spike that stalls PX4's daemon IPC.
# Disabled by default; set SIM_PX4_BOOT_BATCH to enable.
stagger_px4_boot() {
  local batch="${SIM_PX4_BOOT_BATCH:-0}" delay="${SIM_PX4_BOOT_BATCH_DELAY_SECONDS:-8}" wave
  (( batch > 0 )) || return 0
  wave="$(( $1 / batch ))"
  (( wave == 0 )) || sleep "$(( wave * delay ))"
}

# Terminate a process and everything it spawned. A stalled rcS child (a
# px4-param or px4-uxrce_dds_client call blocked on PX4's untimed daemon
# socket) outlives its parent unless it is killed explicitly.
terminate_process_tree() {
  local pid="$1" child
  for child in $(pgrep -P "$pid" 2>/dev/null); do
    terminate_process_tree "$child"
  done
  kill -TERM "$pid" 2>/dev/null || true
}

# Restart one wedged PX4. Process Compose relaunches it, and its rootfs is
# already prepared, so the retry is cheap.
restart_stalled_px4() {
  local index="$1" attempt="$2" attempts="$3" log pid
  log="$(node_dir "px4_$index")/px4/px4.log"
  (( attempt < attempts )) || die "PX4 px4_$index stalled in startup after $attempts attempts"
  printf '[PX4] px4_%s stalled in startup; restarting (attempt %s/%s)\n' \
    "$index" "$((attempt + 1))" "$attempts"
  cp "$log" "$SIM_RUNTIME_DIR/stalled-px4-$index-attempt$attempt.log" 2>/dev/null || true
  pid="$(pgrep -f "bin/px4 -i $index -d " 2>/dev/null | head -1)"
  [[ -n "$pid" ]] && terminate_process_tree "$pid"
  return 0
}

edge_socket_ready() { [[ -S "$(node_socket "$1")" ]]; }
xrce_agent_ready() { /usr/sbin/lsof -nP -iUDP:"$((8888 + $1))" >/dev/null 2>&1; }
px4_process_ready() { [[ -s "$(node_dir "px4_$1")/px4/px4.log" ]]; }

wait_for_fleet_infrastructure() {
  local timeout="${1:?usage: wait_for_fleet_infrastructure <timeout>}"
  local index edge=() xrce=() px4=() relay_deadline="$((SECONDS + 15))"
  for ((index = 0; index < SIM_VEHICLE_COUNT; ++index)); do
    edge+=("px4_$index")
    px4+=("$index")
    is_mavlink_vehicle "$index" || xrce+=("$index")
  done
  edge+=(operator)
  # The relay publishes metrics on a sampling interval; a late file is not a
  # startup failure, so report it without gating on it.
  while [[ ! -f "$SIM_RUNTIME_DIR/network-metrics.json" ]] && (( SECONDS < relay_deadline )); do
    sleep 1
  done
  if [[ -f "$SIM_RUNTIME_DIR/network-metrics.json" ]]; then
    printf '[Net] relay ready\n'
  else
    printf '[Net] relay has not published metrics yet; continuing\n'
  fi
  report_tier Edge "Edge Servers ready" "$timeout" edge_socket_ready "${edge[@]}"
  ((${#xrce[@]} == 0)) || report_tier XRCE "DDS agents ready" "$timeout" xrce_agent_ready "${xrce[@]}"
  report_tier PX4 "processes launched" "$timeout" px4_process_ready "${px4[@]}"
}

wait_for_px4_sih() {
  local vehicle="${1:?usage: wait_for_px4_sih <vehicle> [timeout]}"
  local directory="$(node_dir "$vehicle")/px4"
  local marker="$directory/sih-ready" log="$directory/px4.log"
  local deadline="$((SECONDS + ${2:-180}))"
  while :; do
    [[ -f "$marker" ]] && return
    if [[ -f "$log" ]] && grep -qF "Simulation loop with" "$log"; then
      touch "$marker"
      return
    fi
    (( SECONDS < deadline )) || die "timed out waiting for PX4 SIH: $vehicle"
    sleep 1
  done
}

wait_for_fleet_px4_sih() {
  local timeout="${1:?usage: wait_for_fleet_px4_sih <timeout>}"
  local deadline="$((SECONDS + timeout))" index next_status
  printf '[PX4] Booting %s vehicles\n' "$SIM_VEHICLE_COUNT"
  for ((index = 0; index < SIM_VEHICLE_COUNT; ++index)); do
    next_status="$SECONDS"
    while :; do
      if [[ -f "$(node_dir "px4_$index")/px4/sih-ready" ]] || grep -qF "Simulation loop with" "$(node_dir "px4_$index")/px4/px4.log" 2>/dev/null; then
        printf '[PX4] %s/%s ready (px4_%s)\n' "$((index + 1))" "$SIM_VEHICLE_COUNT" "$index"
        break
      fi
      (( SECONDS < deadline )) || die "timed out waiting for fleet PX4 SIH"
      if (( SECONDS >= next_status )); then
        printf '[PX4] %s/%s ready; waiting for px4_%s\n' "$index" "$SIM_VEHICLE_COUNT" "$index"
        next_status="$((SECONDS + 15))"
      fi
      sleep 1
    done
  done
}

wait_for_fleet_px4_direct_streams() {
  local timeout="${1:?usage: wait_for_fleet_px4_direct_streams <timeout>}"
  local deadline="$((SECONDS + timeout))" index local_port remote_port next_status
  printf '[PX4] Activating direct telemetry streams\n'
  for ((index = 0; index < SIM_VEHICLE_COUNT; ++index)); do
    local_port="$((19450 + index))"; remote_port="$((19410 + index))"
    next_status="$SECONDS"
    while :; do
      if grep -qF "on udp port $local_port remote port $remote_port" "$(node_dir "px4_$index")/px4/px4.log" 2>/dev/null; then
        printf '[PX4] %s/%s direct streams ready (px4_%s)\n' "$((index + 1))" "$SIM_VEHICLE_COUNT" "$index"
        break
      fi
      (( SECONDS < deadline )) || die "timed out waiting for direct PX4 telemetry stream: px4_$index"
      if (( SECONDS >= next_status )); then
        printf '[PX4] %s/%s direct streams ready; waiting for px4_%s\n' "$index" "$SIM_VEHICLE_COUNT" "$index"
        next_status="$((SECONDS + 5))"
      fi
      sleep 1
    done
  done
}

wait_for_px4_startup() {
  local vehicle="${1:?usage: wait_for_px4_startup <vehicle> [timeout]}"
  local directory="$(node_dir "$vehicle")/px4"
  local log="$directory/px4.log"
  local deadline="$((SECONDS + ${2:-180}))"
  while ! grep -qF "Startup script returned successfully" "$log" 2>/dev/null; do
    (( SECONDS < deadline )) || die "timed out waiting for PX4 startup: $vehicle"
    sleep 1
  done
}

# Wait for every PX4 to finish its startup script.
#
# Vehicles are polled as a set rather than in index order, so progress and
# failures name the vehicles actually outstanding. PX4's daemon IPC has no
# timeout, so an rcS that stalls never recovers on its own: a vehicle whose log
# stops growing while the fleet moves on is restarted rather than waited on.
wait_for_fleet_px4_startup() {
  local timeout="${1:?usage: wait_for_fleet_px4_startup <timeout>}"
  # Silence is a weak progress signal and a dangerous one since the bounded-call
  # wrapper landed: a stuck call now logs "nudging daemon" within 2 s and escalates
  # loudly, so a real wedge is noisy, while a healthy vehicle grinding through
  # hundreds of silent `param set` calls under load can print nothing for minutes.
  # At 120 s this restarted a live px4_9 mid-rcS and cost it the whole budget.
  local stall="${SIM_PX4_STALL_SECONDS:-300}" attempts="${SIM_PX4_BOOT_ATTEMPTS:-3}"
  local deadline="$((SECONDS + timeout))" next=0 previous=-1
  local -a pending done_state last_size last_change tries
  local index log size ready list
  for ((index = 0; index < SIM_VEHICLE_COUNT; ++index)); do
    done_state[index]=0; last_size[index]=-1; last_change[index]="$SECONDS"; tries[index]=1
  done
  printf '[PX4] Waiting for %s startups\n' "$SIM_VEHICLE_COUNT"
  while :; do
    ready=0; pending=()
    for ((index = 0; index < SIM_VEHICLE_COUNT; ++index)); do
      log="$(node_dir "px4_$index")/px4/px4.log"
      if (( done_state[index] == 0 )) &&
        grep -qF "Startup script returned successfully" "$log" 2>/dev/null
      then
        done_state[index]=1; px4_phase "px4_$index" rcs_done
      fi
      if (( done_state[index] )); then ready="$((ready + 1))"; continue; fi
      pending+=("px4_$index")
      # rcS reported failure (the bounded-call wrapper aborts it after three
      # hangs): PX4 is exiting and Process Compose relaunches it. Count the
      # attempt; do not wait for the silence fallback.
      if grep -qF "Startup script returned with return value" "$log" 2>/dev/null; then
        (( tries[index] < attempts )) || die "PX4 px4_$index failed startup $attempts times"
        printf '[PX4] px4_%s startup script failed; relaunching (attempt %s/%s)\n' \
          "$index" "$((tries[index] + 1))" "$attempts"
        cp "$log" "$SIM_RUNTIME_DIR/failed-px4-$index-attempt${tries[index]}.log" 2>/dev/null || true
        tries[index]="$(( tries[index] + 1 ))"
        last_size[index]=-1; last_change[index]="$SECONDS"
        while grep -qF "Startup script returned with return value" "$log" 2>/dev/null; do sleep 1; done
        continue
      fi
      size="$(wc -c <"$log" 2>/dev/null | tr -d '[:space:]')"
      [[ -n "$size" ]] || size=0
      if [[ "$size" != "${last_size[index]}" ]]; then
        last_size[index]="$size"; last_change[index]="$SECONDS"
      elif (( size > 0 )) && (( SECONDS - last_change[index] >= stall )); then
        restart_stalled_px4 "$index" "${tries[index]}" "$attempts"
        tries[index]="$(( tries[index] + 1 ))"
        last_size[index]=-1; last_change[index]="$SECONDS"
      fi
    done
    if (( ready == SIM_VEHICLE_COUNT )); then
      printf '[PX4] %s/%s started\n' "$ready" "$SIM_VEHICLE_COUNT"
      return
    fi
    (( SECONDS < deadline )) || die "timed out waiting for PX4 startup: ${pending[*]}"
    if (( ready != previous )) || (( SECONDS >= next )); then
      list="${pending[*]}"
      (( ${#pending[@]} <= 6 )) || list="${pending[*]:0:6} (+$(( ${#pending[@]} - 6 )) more)"
      printf '[PX4] %s/%s started; waiting for %s\n' "$ready" "$SIM_VEHICLE_COUNT" "$list"
      previous="$ready"; next="$((SECONDS + 5))"
    fi
    sleep 1
  done
}

wait_for_mesh_links() {
  local timeout="${1:?usage: wait_for_mesh_links <timeout>}"
  local metrics="$SIM_RUNTIME_DIR/network-metrics.json"
  # Same defaults as session/render-fleet.sh and process/network.sh, which build the topology
  # this counts. Without them `set -u` aborts on the scenarios that omit both.
  local peers="${SIM_MESH_PEERS_PER_VEHICLE:-1}" operator_peers="${SIM_OPERATOR_MESH_PEERS:-1}"
  local expected="$((SIM_VEHICLE_COUNT * peers + operator_peers))"
  local deadline="$((SECONDS + timeout))" count=0 observed=0 previous=-1 next_status=0
  while :; do
    if [[ -r "$metrics" ]]; then
      read -r count observed < <(jq -r '(.observed_unix_ms // 0) as $observed | ([.links[] | select(.connected)] | length) as $count | "\($count) \($observed)"' "$metrics" 2>/dev/null || printf '0 0\n')
    fi
    if (( count == expected && observed > 0 )); then
      printf '[Mesh] %s/%s relay links ready\n' "$count" "$expected"
      return
    fi
    (( SECONDS < deadline )) || die "timed out waiting for simulated mesh links ($count/$expected ready)"
    if (( count != previous || SECONDS >= next_status )); then
      printf '[Mesh] %s/%s relay links ready; waiting\n' "$count" "$expected"
      previous="$count"; next_status="$((SECONDS + 5))"
    fi
    sleep 1
  done
}

clear_runtime_state() {
  local runtime_root="$SIM_ROOT/build/runtime"
  case "$SIM_RUNTIME_DIR" in
    "$runtime_root"/*) ;;
    *) die "refusing to clear runtime outside $runtime_root" ;;
  esac
  rm -rf -- "$SIM_RUNTIME_DIR/nodes" "$SIM_RUNTIME_DIR/px4-profiles"
  rm -f -- "$SIM_RUNTIME_DIR/px4-telemetry-ready" \
    "$SIM_RUNTIME_DIR/network-metrics.json" "$SIM_RUNTIME_DIR/network-metrics.json.new" \
    "$SIM_RUNTIME_DIR/fleet-verification.log"
  rm -rf -- "$SIM_RUNTIME_DIR/px4-telemetry-ready.lock"
}
