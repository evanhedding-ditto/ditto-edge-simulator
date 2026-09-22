#!/usr/bin/env bash
set -euo pipefail

source "$(dirname "${BASH_SOURCE[0]}")/../lib.sh"
sim_init
role="${1:?usage: edge-server.sh <vehicle|operator> <index>}"
index="${2:?usage: edge-server.sh <vehicle|operator> <index>}"
[[ "$role" == vehicle || "$role" == operator ]] || die "unknown Edge Server role: $role"
[[ "$index" =~ ^[0-9]+$ ]] || die "Edge Server index must be numeric"
binary="$(edge_server_binary)" ||
  die "no Edge Server binary. Drop one at $SIM_ROOT/bin/ditto-edge-server, or build it in $SIM_EDGE_SERVER_ROOT/ditto-edge-server with: cargo build --release"
config="$SIM_RUNTIME_DIR/edge-$role-$index.yaml"
[[ -r "$config" ]] || die "render the fleet before starting Edge Server"
[[ "${NO_COLOR:-}" != 1 ]] || export NO_COLOR=true
# Raise the mesh connection cap. Each node opens SIM_MESH_PEERS_PER_VEHICLE
# outbound links, and at the default the chooser holds only a few and churns
# between them.
#
# Edge Server now pins dittolive-ditto =5.1.0 (this comment described 4.12.4
# until 2026-09-22, and the simulator set the 4.12.4 name until then too). The
# legacy `mesh_chooser_max_wlan_clients` still worked, but only through a
# deprecated fallback that logged a warning on every node at every start. This
# is the name 5.1.0 actually wants; both were measured to apply the same 20.
#
# TWO THINGS CHANGED WITH THE SDK, both affecting what the number means:
#   - 4.12.4 capped OUTBOUND connections only; 5.1.0 caps inbound and outbound
#     combined. The same 20 bounds a different quantity, so peer-count results
#     do not carry across the upgrade -- re-baseline instead.
#   - 5.1.0 was branch-cut before the fix that shares the budget across a peer's
#     TCP transports, so the inbound gate misses the outbound count and the
#     effective ceiling is roughly twice the configured value. The fix is
#     upstream commit 36acb782da, which landed on main after the 5.1.0 branch
#     was cut, so getting it needs an SDK newer than 5.1.0 or a cherry-pick.
#
# 5.1.0 scans every DITTO_* variable at startup and warns on any it does not
# recognise, so a wrong name here is now loud rather than silent -- which was
# not true on 4.12.4.
export DITTO_MESH_CHOOSER_MAX_WLAN_CONNECTIONS="${DITTO_MESH_CHOOSER_MAX_WLAN_CONNECTIONS:-20}"
exec nice -n "${SIM_BACKGROUND_NICE:-10}" "$binary" run --config "$config"
