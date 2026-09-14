# Simulator startup and lifecycle debugging

## Startup contract

The managed launcher has these ordered phases:

```text
Process Compose starts network and Edge Servers
  -> all XRCE-DDS Agents bind their UDP ports
  -> all PX4 instances launch together
  -> every PX4 completes its startup script
  -> one telemetry coordinator verifies all display streams
  -> adapters start after the telemetry marker
  -> viewer starts
```

`process_started` means only that Process Compose launched a shell. It is never
used as proof that PX4, MAVLink, an adapter, or the application is ready.

### Ownership

`scripts/wait-telemetry.sh`, invoked synchronously only by `scripts/sim.sh`, is
the sole readiness verifier for UDP ports 19410 through 19429. It waits for every
PX4 SIH marker, then requires every vehicle to supply both
`GLOBAL_POSITION_INT` and `ATTITUDE`. It writes `px4-telemetry-ready` only after
that succeeds. A failed gate returns directly to `sim.sh`, whose trap immediately
runs managed teardown; it never waits for a marker from a failed background job.

ROS adapters consume `px4-telemetry-ready`; none may probe or bind a display
telemetry port. MAVLink adapters bind their separate PX4 onboard-control ports
as soon as their Edge socket exists, before PX4's first control stream.

`pixi run sim` deliberately does **not** run `wait-ready.sh`, issue commands, or
require command receipts or observed movement. It starts the fleet, verifies
direct PX4 telemetry, and opens the viewer. Operators issue commands manually
while the viewer is running. The demo scripts likewise submit only their stated
commands; they do not run the fleet verifier first.

On failure the coordinator does not restart PX4. Automatic recovery was hiding
the first failure and could restart an otherwise queued serial boot. It records
the final probe result in `build/runtime/<scenario>/telemetry-gate.log`; the
runtime directory retains that report and the final PX4 logs after managed
shutdown.

## Failures observed

- `rg: command not found` occurred only under the Process Compose runtime PATH.
  Runtime readiness detection now uses the system `grep -qF`, not a development
  shell tool.
- A fleet with all 20 SIH markers intermittently produced zero datagrams from
  random PX4 instances (for example `px4_5`, `px4_6`, `px4_11`, and `px4_19`).
  The PX4 logs reported a running `Minimal` MAVLink link, but the direct probe
  received neither packets nor MAVLink frames. Pre-binding the localhost ports
  did not help, so receiver ownership was not the root cause.
- The root cause was reliance on PX4's implicit `minimal` stream profile at 20
  concurrent SIH instances. `run-px4.sh` now uses the `custom` profile and
  explicitly configures the only two display streams: `GLOBAL_POSITION_INT` at
  10 Hz and `ATTITUDE` at 20 Hz. The generated rootfs marker is versioned so the
  revised PX4 MAVLink startup file is always rebuilt.
- The former asynchronous telemetry gate could fail while `sim.sh` waited up to
  five minutes for its marker, leaving a live fleet that contaminated the next
  launch. The gate now executes synchronously under `sim.sh`.
- A Process Compose dependency on `process_started` allowed adapters to start
  before their PX4 transport was ready. Explicit marker waits now provide the
  actual dependency.
- Delaying a MAVLink adapter behind display-telemetry readiness caused an
  observed `px4_offboard_timeout`: PX4 armed but its control stream was not
  sufficiently primed. MAVLink control and display telemetry have distinct UDP
  ports; only the latter is exclusive and centrally coordinated.
- The prior serial PX4 startup, fixed settling delay, staggered ROS adapters,
  and PX4 retry loops were removed. They obscured the ordering problem and did
  not resolve intermittent failures.

## Debugging rules

1. Do not start a second scenario until `pixi run sim-status` and the host
   process table confirm the first is stopped.
2. On a failed launch, preserve `telemetry-gate.log`, `last-px4-*.log`, and the
   Process Compose log before modifying startup code. Do not add retries or
   restarts until the original failure is understood.
3. Treat SIH boot, direct telemetry, fleet application readiness, command
   acceptance, and clean teardown as independent checks.
4. Startup and shutdown code must be validated from a clean stopped state with
   two consecutive executions of the exact managed `pixi run sim` path. Each run
   needs PX4 boot, direct telemetry, application readiness, command/receipt, and
   managed shutdown. A failure resets the sequence.

## Startup reordering — 2026-09-11

The 20-node startup was changed to an event-driven dependency chain:

1. Vehicle and operator Edge Servers start.
2. All XRCE-DDS Agents wait until every required Edge socket exists, then bind.
3. Every PX4 waits until all required XRCE UDP listeners are bound, then the
   PX4 processes launch concurrently.
4. PX4 completion and direct telemetry are verified before adapters and the
   viewer proceed.

No arbitrary five-second stabilization period, serial PX4 sequencing, adapter
stagger, or automatic PX4 retry remains in this path. One clean observation
reached all 20 PX4 startup completions, 20/20 direct telemetry streams, and
105/105 mesh links. This is promising evidence, not the required two-run
lifecycle validation.

## Non-regression rules

- Do not treat any one of these as a substitute for another: Process Compose
  `process_started`, PX4 SIH, display telemetry, PX4 startup-script completion,
  adapter availability, operator state replication, command receipt, and motion.
- The SIH marker occurs before PX4 finishes its `px4-rc.mavlink` script. A
  MAVLink vehicle is not command-ready until the log contains `Startup script
  returned successfully` and the onboard link is present.
- Keep the MAVLink adapter's receive port available before PX4 starts its
  onboard link. Do not delay that adapter behind the display-telemetry marker;
  doing so previously caused `px4_offboard_timeout`.
- Keep display state deterministic and minimal: use PX4 `custom` MAVLink mode
  with only `GLOBAL_POSITION_INT` (10 Hz) and `ATTITUDE` (20 Hz). Do not revert
  to `minimal` merely because a small scenario appears to work.
- The display-port holder experiment was rejected. Binding a receiver before
  PX4 boot neither prevented zero-datagram failures nor fixed the root cause;
  do not reintroduce it.
- Every gate must be owned or awaited by `sim.sh`. A background gate with only a
  marker for completion can fail silently while the launcher waits forever.
- `pixi run sim-status` must enumerate all scenario controller ports. A default
  scenario lookup is not evidence that another scenario is stopped.
- A missing viewer network line is not evidence of a dead PX4 or missing command
  link. In `mvp-twenty-mixed`, the operator directly peers only with `px4_0` and
  `px4_10`; the remaining vehicles use the mesh. Check PX4 startup, onboard
  sockets, receipt status, and fresh vehicle state separately from the directional
  mesh metrics. For example, `px4_16` was armed and moving with fresh state while
  its expected `px4_16 -> px4_{17,18,19}` mesh metrics were disconnected.
- Any change to PX4 startup ordering, MAVLink ports/streams, adapter ordering,
  Process Compose lifecycle, or shutdown resets the complete two-run gate.

## Validation record — 2026-09-10

The failed `minimal`-profile run reset the lifecycle sequence. After the explicit
stream and synchronous-gate fixes, two consecutive clean executions of
`SIM_SCENARIO_FILE="$PWD/scenarios/mvp-twenty-mixed.env" pixi run sim` passed:

1. 20/20 direct PX4 telemetry, 20/20 application readiness, and accepted
   `set_armed` receipt for `px4_19`; viewer exit completed managed shutdown.
2. The same 20/20 telemetry and application checks, with a second accepted
   `set_armed` receipt for `px4_19`; viewer exit again completed managed shutdown.

Both runs used the managed viewer path. No Process Compose controller, PX4, XRCE
agent, telemetry probe, or viewer remained after shutdown.

This historical record predates the 2026-09-11 startup reordering and does not
validate the current source; current lifecycle validation remains 0/2.

## Open control-link regression — 2026-09-10

During a later live 20-node session, `px4_10` received an `orbit_local` command
whose receipt failed as `autopilot_unavailable`. PX4 had reached SIH and its
display MAVLink link, but its log stopped before `Startup script returned
successfully` and never created the separate onboard MAVLink link (expected
local UDP port `14590`; its adapter was correctly listening on `14550`). The
vehicle therefore could display state but could not move.

This was introduced by placing synchronous display-stream configuration before
the MAVLink onboard-link startup. The staged correction starts the onboard link
first and requires every PX4 to log full startup completion before the fleet can
be marked ready or the viewer can open. `sim-status` now also enumerates every
configured scenario, rather than silently checking only the default two-node
scenario. These changes do not alter the active session and are **not yet
validated**; the mandatory two-run lifecycle sequence is reset.

## ROS adapter startup wedge — 2026-09-11

The live `mvp-twenty-mixed` session exposed `px4_1` stuck at its previous
position while `px4_0` and native MAVLink `px4_10` continued to publish fresh
moving state. This was not a viewer mesh issue: the operator state for `px4_1`
was stale, its current command had no new receipt, and its ROS adapter had no
Edge Server socket. PX4 and its XRCE UDP endpoint were both live.

A stack sample of `ros-px4-1` proved it was blocked in the `BridgeNode`
constructor, inside Fast DDS shared-memory participant creation while opening a
shared-memory port. It never reached the bridge-ready log, subscriptions, Edge
link, or command observer. `ros-px4-0` completed construction and moved under
the same PX4/XRCE configuration. Therefore SIH, direct display telemetry, an
XRCE process, and a Process Compose `Running` state are not ROS-control
readiness evidence.

The staged correction forces host ROS adapters to use Fast DDS UDPv4 built-in
transport only; PX4-to-XRCE was already UDP. The fleet-ready barrier now runs a
single fleet-wide verification: it requires fresh state from every expected
vehicle, dispatches a distinct short go-to command to every vehicle, then
requires that command's accepted receipt and at least one metre of fresh
observed displacement for every vehicle. The viewer cannot open until this
passes. These changes deliberately do not alter the live session and remain
**unvalidated** until two clean managed launches pass the full lifecycle gate.

## Validation attempts — 2026-09-11

The lifecycle sequence remains at zero passes. A clean launch first failed its
direct telemetry gate at 18/20 after testing streams as soon as SIH appeared.
The gate now waits for every PX4 startup script before probing. A following run
then exposed `px4_2` not returning from the PX4 startup script before the old
180-second fleet deadline. Serial startup now waits for full predecessor
startup rather than merely SIH, with a 600-second fleet allowance.

Subsequent clean runs reached 20/20 direct telemetry. One exposed a missing
operator socket before the new verifier; that explicit socket barrier is now
restored. Later runs showed the verifier cannot use operator-replicated state
as an immediate proxy for a remote vehicle under the static mesh, so it now
reads freshness and displacement from each vehicle's own Edge socket while
still requiring operator-to-vehicle command propagation and accepted receipts.
The latest evidence also shows an intermittently wedged ROS bridge constructor
without a `bridge ready` log; ROS starts are now staggered. No run has reached
the viewer yet, so no command/motion or managed-viewer-shutdown validation pass
is claimed.
