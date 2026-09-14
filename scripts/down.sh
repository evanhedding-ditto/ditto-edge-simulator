#!/usr/bin/env bash
set -euo pipefail

source "$(dirname "$0")/lib.sh"

compose_pids() {
  ps -axo pid=,command= | awk -v config="$SIM_RUNTIME_DIR/process-compose.yaml" \
    '$2 ~ /process-compose$/ && $0 ~ config {print $1}'
}

runtime_pids() {
  ps -axo pid=,command= | awk -v runtime="$SIM_RUNTIME_DIR" '$0 ~ runtime {print $1}'
}

xrce_pids() {
  ps -axo pid=,command= | awk -v first=8888 -v last="$((8888 + SIM_VEHICLE_COUNT))" \
    '/MicroXRCEAgent .*udp4 -p [0-9]+$/ && $NF >= first && $NF < last {print $1}'
}

startup_gate_pids() {
  ps -axo pid=,comm=,args= | awk '
    ($2 == "bash" && ($0 ~ /wait-telemetry\.sh/ || $0 ~ /wait-ready\.sh/)) ||
    $2 == "ditto_px4_telemetry_ready" {print $1}'
}

terminate_pids() {
  local signal="$1" pid
  shift
  for pid in "$@"; do
    [[ "$pid" =~ ^[1-9][0-9]*$ ]] || continue
    kill -"$signal" "$pid" 2>/dev/null || true
  done
}

compose_down() {
  local client deadline
  pixi run --manifest-path "$SIM_PROTOTYPE_ROOT/pixi.toml" process-compose \
    -p "$SIM_PROCESS_COMPOSE_PORT" --ordered-shutdown down >/dev/null 2>&1 &
  client=$!
  deadline="$((SECONDS + ${SIM_SHUTDOWN_TIMEOUT_SECONDS:-20}))"
  while kill -0 "$client" 2>/dev/null; do
    if (( SECONDS >= deadline )); then
      echo "warning: Process Compose shutdown timed out for $SIM_SCENARIO_ID" >&2
      kill -TERM "$client" 2>/dev/null || true
      wait "$client" 2>/dev/null || true
      return 1
    fi
    sleep 1
  done
  wait "$client"
}

cleanup_scenario() {
  local -a pids=()
  local log pid
  while read -r pid; do pids+=("$pid"); done < <(startup_gate_pids)
  if ((${#pids[@]})); then
    terminate_pids TERM "${pids[@]}"
    sleep 1
  fi
  pids=()
  if [[ -n "$(compose_pids)" ]]; then
    compose_down || true
  fi
  while read -r pid; do pids+=("$pid"); done < <(runtime_pids)
  while read -r pid; do pids+=("$pid"); done < <(xrce_pids)
  if ((${#pids[@]})); then
    terminate_pids TERM "${pids[@]}"
    sleep 3
    pids=()
    while read -r pid; do pids+=("$pid"); done < <(runtime_pids)
    while read -r pid; do pids+=("$pid"); done < <(xrce_pids)
    ((${#pids[@]} == 0)) || terminate_pids KILL "${pids[@]}"
  fi
  for ((index = 0; index < SIM_VEHICLE_COUNT; ++index)); do
    log="$(node_dir "px4_$index")/px4/px4.log"
    [[ -f "$log" ]] && cp "$log" "$SIM_RUNTIME_DIR/last-px4-$index.log"
    log="$(node_dir "px4_$index")/mavlink.log"
    [[ -f "$log" ]] && cp "$log" "$SIM_RUNTIME_DIR/last-mavlink-$index.log"
  done
  for log in telemetry-gate.log fleet-verification.log; do
    [[ -f "$SIM_RUNTIME_DIR/$log" ]] && cp "$SIM_RUNTIME_DIR/$log" "$SIM_RUNTIME_DIR/last-$log"
  done
  clear_runtime_state
}

sim_init
for scenario in "$SIM_ROOT"/scenarios/*.env; do
  (
    SIM_SCENARIO_FILE="$scenario"
    sim_init
    cleanup_scenario
  )
done
