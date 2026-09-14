#!/usr/bin/env bash
set -euo pipefail

source "$(dirname "$0")/lib.sh"
sim_init
exec cargo build --manifest-path "$SIM_EDGE_ADAPTERS_ROOT/Cargo.toml" -p px4-mavlink-ditto-bridge
