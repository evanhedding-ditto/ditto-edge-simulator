#!/usr/bin/env bash
set -euo pipefail

source "$(dirname "${BASH_SOURCE[0]}")/../lib.sh"
sim_init
[[ -n "${SIM_ISR_TARGETS_FILE:-}" ]] || die "ISR reset needs a scenario with SIM_ISR_TARGETS_FILE"
exec >"$SIM_RUNTIME_DIR/isr-reset.log" 2>&1

# One shared command write sends the whole fleet home. Keep them at 5 m, ready
# for the next flight; the viewer owns clearing ISR target state separately.
batch="$SIM_RUNTIME_DIR/isr-home.batch"
for ((index = 0; index < SIM_VEHICLE_COUNT; ++index)); do
  printf 'px4_%s goto 0 0 5\n' "$index"
done >"$batch"
"$SIM_ROOT/scripts/command.sh" batch "$batch"
