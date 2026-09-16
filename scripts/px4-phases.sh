#!/usr/bin/env bash
# Report where each vehicle's startup time went, from the phase stamps written
# by run-px4.sh and the fleet startup gate.  Run after a launch; the file
# survives teardown and is reset by up.sh.
#
# usage: px4-phases.sh [scenario-id | path/to/px4-phases.tsv]
set -euo pipefail

SIM_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# A read-only report locates its own data: requiring the caller's scenario
# environment only produces a confusing miss in a fresh shell.  An explicit
# scenario id wins; otherwise take the most recently written run.
if [[ -f "${1:-}" ]]; then
  phases="$1"
elif [[ -n "${1:-}" ]]; then
  phases="$SIM_ROOT/build/runtime/$1/px4-phases.tsv"
else
  phases="$(ls -t "$SIM_ROOT"/build/runtime/*/px4-phases.tsv 2>/dev/null | head -1 || true)"
fi
if [[ -z "$phases" || ! -r "$phases" ]]; then
  printf 'no phase data under %s/build/runtime/*/px4-phases.tsv; launch the fleet first\n' \
    "$SIM_ROOT" >&2
  exit 1
fi
printf 'scenario %s\n\n' "$(basename "$(dirname "$phases")")"

sort -k1,1 -k3,3n "$phases" | awk -v budget="${SIM_PX4_FLEET_BOOT_TIMEOUT_SECONDS:-300}" '
  { at[$1 "/" $2] = $3; if (!($1 in seen)) { seen[$1] = 1; order[n++] = $1 } }
  function span(v, a, b,   x, y) {
    x = at[v "/" a]; y = at[v "/" b]
    return (x == "" || y == "") ? "-" : sprintf("%d", (y - x) / 1000)
  }
  END {
    for (i = 0; i < n; i++) {
      v = order[i]; idx = substr(v, 5) + 0; by[idx] = v
      if (idx > last) last = idx
      if (at[v "/launch"] != "" && (t0 == 0 || at[v "/launch"] < t0)) t0 = at[v "/launch"]
    }
    printf "%-8s %7s %8s %7s %7s %7s\n", "vehicle", "start", "barrier", "rootfs", "rcS", "total"
    for (i = 0; i <= last; i++) {
      if (!(i in by)) continue
      v = by[i]; total = span(v, "launch", "rcs_done")
      printf "%-8s %7d %8s %7s %7s %7s\n", v, (at[v "/launch"] - t0) / 1000,
        span(v, "launch", "barriers"), span(v, "barriers", "exec"),
        span(v, "exec", "rcs_done"), total
      if (total == "-") pending = pending " " v
      else { sum += total; done++; if (total + 0 > worst) { worst = total + 0; slowest = v } }
    }
    if (done) printf "\n%d/%d finished rcS; slowest %s at %ds of a %ds budget (mean %ds)\n",
      done, n, slowest, worst, budget, sum / done
    if (pending != "") printf "never finished rcS:%s\n", pending
  }'
