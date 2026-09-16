# Simulator Debug Handoff

## Status at handoff

- The simulator is stopped: `pixi run sim-status` reported no active session.
- Lifecycle validation is **0/2 passes**. Do not claim the system is robust yet.
- The last source edit is **not built** and has one known compile typo; fix that before the next run.

## Immediate next step

In `tools/fleet_command.cpp`, change:

```cpp
next->second.east - position.second
```

to:

```cpp
next->second.east - position.east
```

Then build `ditto_fleet_command` and begin the required clean two-run validation sequence from a stopped state.

## What was changed

### Relay / `NO LINK`

`network/relay.cpp` was changed so an incoming Edge connection is retained while its destination Edge listener is still starting. The relay retries only the destination every 100 ms, rather than dropping the source connection during cold start.

This addresses the observed `px4_19 NO LINK` case: all ten directed links involving `px4_19` had been disconnected because its outbound sources connected before destination listeners existed. After the relay change, live metrics reached **105/105 connected**, including `px4_19`.

### Startup visibility and gates

- `scripts/lib.sh`: PX4 progress, mesh-link readiness gate, correct metric timestamp parsing, and cleanup of transient metrics/verifier logs.
- `scripts/run-px4.sh`: serial PX4 boot with up to three bounded per-vehicle retries (`SIM_PX4_BOOT_ATTEMPTS=3`) and visible attempt logging.
- `scripts/wait-telemetry.sh`: verifies direct PX4 telemetry after PX4 SIH readiness; output is streamed live.
- `tools/px4_telemetry_ready.cpp`: prints periodic direct-telemetry progress.
- `scripts/wait-ready.sh`: waits for mesh links before application verification and preserves verifier output in `build/runtime/<scenario>/fleet-verification.log`.
- `scripts/sim.sh`: rebuilds changed C++ helper/client binaries before launch.
- `scenarios/mvp-twenty-mixed.env`: sets PX4 retries, 180-second telemetry timeout, and 120-second mesh timeout.

The mesh-readiness jq bug was fixed. It had read `observed_unix_ms` after piping the root JSON into an array, so the timestamp became zero and the gate waited forever despite healthy links. The corrected gate was manually observed passing:

```text
[Mesh] 105/105 relay links ready
```

## Evidence from runs

- Direct PX4 telemetry repeatedly reached `20/20 direct PX4 telemetry ready`.
- PX4 `px4_2` had one intermittent first-boot failure (`PX4 server not running`); its retry reached full PX4 initialization. It later succeeded on the first attempt, so this is intermittent, not solved conclusively.
- One earlier run timed out on direct telemetry for `px4_17`; a focused single-vehicle check passed afterward. This also remains an intermittent risk.
- Relay metrics reached `105/105` after the relay change; the old metrics gate itself was then proven working after its jq correction.

## Current blocker: fleet end-to-end command verification

The most recent full pre-command sequence passed PX4, direct telemetry, and mesh readiness. The fleet verifier then failed:

```text
[Fleet] 10/20 command receipts; 0/20 moving
[Fleet] 14/20 command receipts; 6/20 moving
[Fleet] 17/20 command receipts; 12/20 moving
[Fleet] 17/20 command receipts; 0/20 moving
error: fleet command or movement verification timed out
```

The previous verifier treated any missing or stale vehicle state as failure of the entire state map. That erased movement evidence from vehicles already observed moving, explaining the `12/20` to `0/20` regression. The unbuilt `tools/fleet_command.cpp` change changes this to per-vehicle accumulation of initial positions and observed movement.

Three vehicles had actual failed native-command receipts in that run: `px4_10`, `px4_11`, and `px4_14`. The prior verifier did not retain receipt error text. The unbuilt change includes those error strings in failures; after the compile typo is fixed, rerun and capture the exact errors before choosing a bridge fix.

Likely place to investigate once the receipt error is known:

`/Users/evan/ditto-repos/Ditto-Edge-Server/ditto-edge-adapters/adapters/mavlink/px4_ditto_bridge/src/bridge.rs`

Native vehicles require PX4 heartbeat plus armed/offboard state before accepting movement commands. Possible current errors include offboard timeout, unavailable autopilot, or arm timeout; none has been proven yet.

## Startup flow

```text
pixi run sim
  -> scripts/sim.sh (build helpers as needed)
  -> scripts/up.sh (Process Compose)
  -> scripts/wait-telemetry.sh
       PX4 SIH -> direct telemetry
  -> scripts/wait-ready.sh
       mesh links -> fleet command/receipt/movement verification
  -> scripts/viewer.sh
```

Runtime evidence lives under:

```text
build/runtime/mvp-twenty-mixed/
  network-metrics.json
  fleet-verification.log
  last-px4-*.log (after shutdown)
```

## Required continuation procedure

1. Fix the single C++ typo, build, and run from a clean stopped state.
2. Require: all 20 PX4 ready, 20/20 direct telemetry, 105/105 mesh links, 20 command receipts, and movement for every expected vehicle.
3. If a receipt fails, preserve and inspect its exact error; do not infer the cause.
4. Shut down cleanly, then repeat the same managed `pixi run sim` path a second time.
5. Any failure resets the validation count to zero. Do not optimize startup/scaling until two consecutive full passes complete.
