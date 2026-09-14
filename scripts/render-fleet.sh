#!/usr/bin/env bash
set -euo pipefail

source "$(dirname "$0")/lib.sh"
sim_init
load_ditto_credentials

: "${SIM_PROCESS_TEMPLATE:=config/process-compose.yaml.in}"
[[ "$SIM_VEHICLE_COUNT" =~ ^[1-9][0-9]*$ ]] || die "SIM_VEHICLE_COUNT must be positive"
SIM_MESH_PEERS_PER_VEHICLE="${SIM_MESH_PEERS_PER_VEHICLE:-1}"
SIM_OPERATOR_MESH_PEERS="${SIM_OPERATOR_MESH_PEERS:-1}"
[[ "$SIM_MESH_PEERS_PER_VEHICLE" =~ ^[1-9][0-9]*$ ]] || die "SIM_MESH_PEERS_PER_VEHICLE must be positive"
[[ "$SIM_OPERATOR_MESH_PEERS" =~ ^[1-9][0-9]*$ ]] || die "SIM_OPERATOR_MESH_PEERS must be positive"
(( SIM_MESH_PEERS_PER_VEHICLE < SIM_VEHICLE_COUNT )) || die "SIM_MESH_PEERS_PER_VEHICLE must be less than SIM_VEHICLE_COUNT"
(( SIM_OPERATOR_MESH_PEERS <= SIM_VEHICLE_COUNT )) || die "SIM_OPERATOR_MESH_PEERS must not exceed SIM_VEHICLE_COUNT"
[[ -r "$SIM_ROOT/$SIM_PROCESS_TEMPLATE" ]] || die "process template not readable: $SIM_PROCESS_TEMPLATE"
mkdir -p "$SIM_RUNTIME_DIR"

render_edge() {
  local role="$1" index="$2" node_id config known_tcp_servers tcp_port
  local source destination peer
  relay_port() { printf '%s' "$((10000 + source * (SIM_VEHICLE_COUNT + 1) + destination))"; }
  if [[ "$role" == vehicle ]]; then
    node_id="px4_$index"
    source="$index"
    known_tcp_servers='['
    for ((peer = 1; peer <= SIM_MESH_PEERS_PER_VEHICLE; ++peer)); do
      destination="$(((index + peer) % SIM_VEHICLE_COUNT))"
      (( peer == 1 )) || known_tcp_servers+=', '
      known_tcp_servers+="\"127.0.0.1:$(relay_port)\""
    done
    known_tcp_servers+=']'
    tcp_port="$((9100 + index))"
  else
    node_id=operator
    source="$SIM_VEHICLE_COUNT"
    known_tcp_servers='['
    for ((peer = 0; peer < SIM_OPERATOR_MESH_PEERS; ++peer)); do
      destination="$((peer * SIM_VEHICLE_COUNT / SIM_OPERATOR_MESH_PEERS))"
      (( peer == 0 )) || known_tcp_servers+=', '
      known_tcp_servers+="\"127.0.0.1:$(relay_port)\""
    done
    known_tcp_servers+=']'
    tcp_port=9200
  fi
  SIM_DEVICE_NAME="ditto-sim-$node_id"
  SIM_PERSISTENCE_DIR="$(node_dir "$node_id")/ditto"
  SIM_SOCKET_PATH="$(node_socket "$node_id")"
  SIM_TCP_PORT="$tcp_port"
  SIM_KNOWN_TCP_SERVERS="$known_tcp_servers"
  SIM_VEHICLE_ID="$node_id"
  export SIM_DEVICE_NAME SIM_PERSISTENCE_DIR SIM_SOCKET_PATH SIM_TCP_PORT SIM_KNOWN_TCP_SERVERS
  export SIM_VEHICLE_ID
  mkdir -p "$SIM_PERSISTENCE_DIR"
  config="$SIM_RUNTIME_DIR/edge-$role-$index.yaml"
  envsubst '${DITTO_DB_ID} ${DITTO_AUTH_URL} ${DITTO_WEBSOCKET_URL} ${DITTO_ACCESS_TOKEN} ${SIM_DEVICE_NAME} ${SIM_PERSISTENCE_DIR} ${SIM_SOCKET_PATH} ${SIM_TCP_PORT} ${SIM_KNOWN_TCP_SERVERS} ${SIM_VEHICLE_ID}' \
    < "$SIM_ROOT/config/edge-server/$role.yaml.in" > "$config"
}

for ((vehicle = 0; vehicle < SIM_VEHICLE_COUNT; ++vehicle)); do
  render_edge vehicle "$vehicle"
done
render_edge operator 0
envsubst < "$SIM_ROOT/$SIM_PROCESS_TEMPLATE" > "$SIM_RUNTIME_DIR/process-compose.yaml"
printf '%s\n' "$SIM_RUNTIME_DIR"
