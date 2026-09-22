#!/usr/bin/env bash
set -euo pipefail

# The network observer is a Rust binary in the adapters workspace, not a CMake
# target, so it is built here rather than by build/viewer.sh.
#
# Run unconditionally. cargo does nothing when nothing changed, and the
# alternative -- skipping when the binary exists -- silently runs a stale
# observer after its source changes.

source "$(dirname "${BASH_SOURCE[0]}")/../lib.sh"
sim_init

cargo build --release --manifest-path "$SIM_EDGE_ADAPTERS_ROOT/Cargo.toml" \
  -p ditto-network-observer
