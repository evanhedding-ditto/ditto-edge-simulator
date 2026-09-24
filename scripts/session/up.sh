#!/usr/bin/env bash
set -euo pipefail

source "$(dirname "${BASH_SOURCE[0]}")/../lib.sh"
sim_init
# macOS caps a Unix socket path at 103 bytes, and an Edge Server that cannot
# bind its socket crash-loops until the readiness gate times out minutes later.
# The operator's is the longest node socket, so refuse up front on it.
socket="$(node_socket operator)"
(( ${#socket} <= 103 )) ||
  die "Edge socket path is ${#socket} bytes, over macOS's 103: $socket (shorten SIM_SCENARIO_ID)"
load_ditto_credentials
if sim_compose process list >/dev/null 2>&1; then
  die "a simulator session is already running; use 'pixi run sim-down' first"
fi
clear_runtime_state
rm -f -- "$SIM_RUNTIME_DIR"/last-*
# Rotate rather than delete: comparing a startup change against the run before
# it is the whole point of the phase data, and last-* is cleared above.
[[ -f "$SIM_RUNTIME_DIR/px4-phases.tsv" ]] &&
  mv -f -- "$SIM_RUNTIME_DIR/px4-phases.tsv" "$SIM_RUNTIME_DIR/prev-px4-phases.tsv"
mkdir -p "$SIM_RUNTIME_DIR"
SIM_RUN_STARTED_UNIX_MS="$(( $(date +%s) * 1000 ))"
export SIM_RUN_STARTED_UNIX_MS
printf '%s\n' "$SIM_RUN_STARTED_UNIX_MS" > "$SIM_RUNTIME_DIR/run-started-unix-ms"
"$SIM_ROOT/scripts/session/render-fleet.sh" >/dev/null
sim_compose -f "$SIM_RUNTIME_DIR/process-compose.yaml" up -D -t=false --ordered-shutdown
