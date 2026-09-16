#!/usr/bin/env bash
set -euo pipefail

source "$(dirname "$0")/lib.sh"
sim_init
load_ditto_credentials
if pixi run --manifest-path "$SIM_PROTOTYPE_ROOT/pixi.toml" process-compose \
  -p "$SIM_PROCESS_COMPOSE_PORT" process list >/dev/null 2>&1
then
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
"$SIM_ROOT/scripts/render-fleet.sh" >/dev/null
cd "$SIM_ROOT"
exec pixi run --manifest-path "$SIM_PROTOTYPE_ROOT/pixi.toml" process-compose \
  -p "$SIM_PROCESS_COMPOSE_PORT" -f "$SIM_RUNTIME_DIR/process-compose.yaml" up -D -t=false --ordered-shutdown
