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

## PX4 daemon lost wakeup and the bounded-call wrapper — 2026-09-14

**Status: partially validated. It is not yet established what is fixed.** One
managed launch on commit `d9dd4db` reached 20/20 PX4 startups. On that same
launch `px4_0` accepted its command, armed, and detected takeoff, but did not
move to the commanded position; that is unexplained (see below). The two-run
lifecycle gate has not been passed. Treat every claim in this section as
evidence-backed for the single-instance experiments and *unconfirmed at fleet
scale* until repeated launches say otherwise.

### Symptom

Every 20-node launch today hung on a random vehicle (`px4_4`; `px4_1`+`px4_10`;
`px4_9`; `px4_0`; `px4_19`; `px4_17`+`px4_18`; `px4_2`+`px4_15`). Eighteen or
nineteen vehicles finished `rcS` in ~40 s; the rest sat in `rcS` for minutes
until the fleet budget expired. The serial in-order gate blamed a later,
healthy vehicle (`px4_18`, `px4_2`) for a hang elsewhere.

### Root cause, as captured

Reproduced on a **single idle PX4 with the pristine `etc/`** (no simulator
scripts involved), then captured with stack samples and `lsof`:

- client (`px4-commander --instance N start`): connected to
  `/tmp/px4-sock-N`, blocked in `read()` at `client.cpp:141`;
- server: daemon thread alive in `poll()` at `server.cpp:150`, **zero accepted
  connections, no `_handle_client` thread, no output**;
- the client's socket peer was the **listen** socket — the connection was in
  the accept backlog, never accepted;
- a second connection woke `poll()`; accept is FIFO, so the stuck command was
  served first and completed once; `rcS` then finished.

So PX4's posix daemon occasionally misses the wakeup for a pending client
connection. Rate ≈ 1 per 2,000–4,000 calls; `rcS` makes ~200 calls per
vehicle (an unloaded `rcS` finishes all of them in ~1 s); a 20-vehicle launch
is ~4,000 calls, hence about one hang per launch. It reproduced at load
average 2; load only widens the timing window. 4,000 iterations of the same
poll/accept/shutdown pattern in a bare loop lost no wakeups, so it is specific
to PX4's daemon loop, not macOS `poll()`. Lost wakeups clustered on the
rapid `px4-mavlink` burst after `mavlink start`. Worth an upstream PX4 report.

Disproved today, each by direct test: lockstep starvation (px4_19 wedged
before SIH started), CPU or memory pressure as the cause, a `POLLHUP`-only fd
leak, `EMFILE` on `accept()`. Do not rebuild PX4 with `CONFIG_BOARD_NOLOCKSTEP`:
SIH's realtime path integrates an unclamped wall-clock `dt` and it disables
`PX4_SIM_SPEED_FACTOR`.

### Fix

`config/px4-bounded.sh` is installed into each rootfs by `run-px4.sh`, which
redirects `rcS` line 11 from `. px4-alias.sh` to it (marker `v19`). It sources
the real aliases, then replaces every `px4-*` alias with a function that runs
the client in the background and polls with exponential backoff from 5 ms.
After `PX4_CALL_NUDGE_MS` (2 s) of silence it opens one extra connection
(`px4-param show SYS_AUTOSTART`), which wakes the daemon; the stuck command
completes exactly once, so there are no duplicate side effects (a retried
`mavlink start` fails with `port already occupied`). Kill-and-retry after
`PX4_CALL_TIMEOUT_S` (30 s, `PX4_CALL_ATTEMPTS` 3) is the fallback; the third
hang exits `rcS` with status 1 so the vehicle is reported failed rather than
left silently misconfigured (`rcS` runs with `set -e` disabled). A completed
call's exit code passes through untouched: `param compare`/`greater` return 1
to mean false inside `rcS` conditionals.

`wait_for_fleet_px4_startup` now polls vehicles as a set and reports the real
pending list, restarts a vehicle only on `Startup script returned with return
value` (preserving its log as `failed-px4-N-attemptK.log`), and keeps a 120 s
silence fallback. `wait_for_fleet_infrastructure` reports Net/Edge/XRCE/PX4
launched tiers before the startup gate.

Two regressions were introduced and fixed the same day: a 200 ms poll floor
(40× slowdown; fleet finished 10/20) and a 45 s silence restart that killed
healthy vehicles. Lesson: test the wrapper against real PX4, not a fake client.

### Evidence

- Single instance, pristine config, no wrapper: 1 hang in ~6 boots (60 s cap).
- Single instance, wrapper: **25/25 boots, 3 lost wakeups, each healed in 2 s,
  0 retries, 0 aborts, 1.5 s per boot (4.3 s when nudged).**
- Fleet: one managed launch reached 20/20 startups. `px4_0` was nudged once
  (`dataman start`) and booted normally. **Correction to the first draft of
  this section:** its *first* command was receipted `failed:
  autopilot_unavailable` — the command was written ~6 s before px4_0's ROS
  adapter started, the adapter observed it on startup before any telemetry
  had arrived (`revision == 0`), and the adapter rejects that case immediately
  instead of holding the command until telemetry arrives or it expires. Its
  later commands executed correctly (it was at its seq-35 target within 2
  minutes). The earlier "did not move" reading came from operator-side
  `vehicle_state` that was 14 s stale; replicated state at the operator lags
  4–14 s, which is a separate observation worth keeping in mind when judging
  motion from the viewer. Evidence retained in
  `build/runtime/mvp-twenty-mixed/px4_0-command-failure-*` (survives teardown).

### Earlier ordering regression, fixed first

`config/process-compose-*-mixed.yaml.in` and `run-px4.sh` were edited on
2026-09-11 at 16:26–16:29 — after the validated session ended (docs written
16:19) — making each MAVLink PX4 depend on its adapter and adding a fleet-wide
`wait_for_fleet_mavlink_listeners` barrier, contradicting the order documented
above. Reverted (`cb85d0c` baseline captures the revert); the first clean
20-node run and both demo scripts followed. That change was not the daemon
bug, but it was real and it was undocumented.

### Still open

- ~~Adapter startup race: do not dequeue while `revision == 0`.~~ **Retracted
  2026-09-14.** That fix would not have helped any failure actually observed;
  see "Adapters could not tell dead telemetry from live" below. The real defect
  was that neither adapter could detect telemetry that had *stopped*, and it is
  now fixed and validated.
- Two-run lifecycle validation: startup passes 2/2 headless (57 s and 60 s);
  `verify 20` still fails on the px4_2 XRCE defect, so the full gate including an
  end-to-end fleet command is not yet met. See the two 2026-09-14 (later) sections.
- ~~A separate intermittent PX4 sensor-init failure (`Preflight Fail: No valid
  data from Baro 0 / Compass 0` → invalid EKF position → blind land).~~
  **Resolved 2026-09-15 as a side effect of the startup fix.** It was a symptom
  of the disk-contention slow boot, not an independent defect: the fault rate
  tracked startup time (10 vehicles affected on a 367 s run, 3 on a 60 s run),
  and the messages were transient EKF convergence noise that cleared —
  `position_valid` was true on all 20 vehicles even when they logged. On a
  post-fix run issuing a `goto` to each vehicle individually: **18/20 accepted,
  armed and reached the target; zero `Preflight Fail`, zero `Arming denied`.**
  The only two failures were the XRCE defect below.
- ~~`wait-ready.sh` dies on the 2- and 4-node scenarios
  (`SIM_MESH_PEERS_PER_VEHICLE` unbound)~~ **Fixed 2026-09-14:**
  `wait_for_mesh_links` now defaults both peer counts to 1, matching
  `render-fleet.sh` and `run-network.sh`, which already did. `lib.sh` still
  carries three uncalled functions (`wait_for_px4_sih`, `wait_for_fleet_px4_sih`,
  `wait_for_fleet_mavlink_listeners`).
- Adapters: both rewritten onto one architecture (ditto-edge-adapters, then a standalone repo,
  `e04109f`); at 20 nodes, accepted receipts went from 17/20 to 20/20. The
  MAVLink adapter's original defect was a biased `select!` that starved command
  intake and awaited gRPC inside the control loop.

## Startup is disk-bound, not CPU-bound — 2026-09-14 (later)

**Status: 20/20 startup in 57-60 s, from >392 s or outright failure. Two
consecutive passing runs, headless (up.sh / wait-telemetry.sh / down.sh), which
is an explicit substitution for the managed `pixi run sim` path. See the run
record below for exactly what is and is not proven.**

### The measurement that changed the diagnosis

The earlier reading in this file — that twenty SIH vehicles saturate the CPU —
is wrong. Measured mid-startup on a twenty-node fleet:

```
CPU usage:  10.9% user, 28.7% sys, 61.5% IDLE
Disk:       ~21,000 tps, 152-173 MB/s, 7.5 KB per transaction
Threads:    25 processes in state U (uninterruptible disk wait), 4 running
Memory:     71% free; the whole fleet is ~2.3 GB
```

Load average ~20 was processes **blocked on disk**, not computing. PX4 across all
twenty instances never exceeded ~0.8 of a core. Sys time at 2.6x user time is the
signature of syscall and I/O load, not computation.

### The feedback loop

Every vehicle that finishes booting starts publishing `vehicle_state` at 10 Hz,
which replicates to five mesh peers plus the operator. Each completion therefore
*adds* disk load for the vehicles still booting. That is why `rcS` time rose
monotonically with boot order in every run before the fix.

`run-ros-adapter.sh` already waited on `px4-telemetry-ready`, with the comment
"this is a forwarding path; PX4 state generation takes precedence at startup".
`run-mavlink-adapter.sh` waited only for its Edge socket, so ten MAVLink adapters
published into the mesh throughout PX4 boot, against that documented intent. It
now takes the same gate.

Spotlight was also indexing `build/runtime`, which each run fills with ~1.8 GB of
Ditto stores and ~500 MB of ULogs and then deletes. `build/.metadata_never_index`
stops that (`build/` is gitignored).

### The silence-based stall detector was inverted

`SIM_PX4_STALL_SECONDS` restarted any vehicle whose log had not grown in 120 s.
Since the bounded-call wrapper landed, a stuck call logs "nudging daemon" within
2 s and escalates loudly — **a real wedge is noisy**. Silence now means a healthy
vehicle grinding through hundreds of quiet `param set` calls. At 120 s this
restarted a live px4_9 mid-`rcS` and cost it the whole budget. Raised to 300 s.

### Run record

| run | config | startup | note |
| --- | --- | --- | --- |
| 1 | no batch | FAIL 367 s, 14/20 | stall detector restarted healthy px4_9 |
| 2 | batch 5 | FAIL 392 s, 15/20 | zero restarts; wave 3 never finished |
| 3 | batch 5 + gate + Spotlight | **PASS 60 s, 20/20** | worst vehicle 56 s |
| 4 | same as 3 | **PASS 57 s, 20/20** | consecutive confirmation |
| 5 | same as 3, after clearing leaked DDS segments | **PASS 53 s, 20/20** | worst vehicle 49 s |

`verify 20` failed on runs 3-5 (183-187 s) for one reason only: px4_2 has no DDS
telemetry, so the fleet never reaches 20/20 fresh state and `verify` never issues
a command. Startup itself is clean.

### Four-node scenario: full lifecycle gate PASSED, 2/2 consecutive

| run | startup | verify | shutdown | total |
| --- | --- | --- | --- | --- |
| gate4A | 8 s | **12 s, 4/4 command-ready and moving** | clean | 36 s |
| gate4B | 9 s | **11 s, 4/4 command-ready and moving** | clean | 35 s |

Two consecutive complete runs from a stopped state, covering PX4 boot, direct
telemetry, application readiness, an end-to-end command receipt with observed
motion, and managed shutdown. This is the headless path (`up.sh` /
`wait-telemetry.sh` / `command.sh verify` / `down.sh`), an explicit substitution
for `pixi run sim`, which opens the viewer.

It exercises both adapter implementations (px4_0-1 ROS, px4_2-3 MAVLink) and so
is the end-to-end validation of the telemetry-staleness change. It also confirms
the twenty-node failures are scale effects, not defects in the vehicle stack:
the same code, four vehicles, passes everything in 35 s.

Runs 1 and 2 had **zero** wrapper nudges, hangs or aborts fleet-wide: the daemon
lost-wakeup bug did not occur at all. Those failures were pure slowness.

Run 3's per-vehicle `rcS` split is floored by the instrument — `rcs_done` is
stamped by the fleet gate, which does not poll until infrastructure is ready
(~54 s with the stagger), so earlier finishers are all stamped on its first pass.
The fleet-level 60 s is real; the per-vehicle split in that run is not.

**Attribution settled by single-variable test (2026-09-15):** with the MAVLink
gate removed and the Spotlight exclusion left in place, startup FAILED at 372 s —
against 53/57/60 s with the gate. The adapter gate is the fix; the Spotlight
exclusion is housekeeping worth keeping but is not what made the difference.

### A second accumulator: leaked Fast DDS segments

`/private/tmp/boost_interprocess` had grown to 2,712 files and 701 MB of Fast DDS
shared-memory segments dating back to 2026-09-10 — one set leaked per participant,
never cleaned. Every participant creation touches that directory. Files older than
24 h (2,115 of them, 545 MB, all from long-dead processes) were removed, leaving
597 files / 152 MB. Note the UDP-only profile does **not** stop these being
created: 170 appeared during today's runs alone. Worth a periodic sweep.

## Adapters could not tell dead telemetry from live — 2026-09-14 (later)

**Status: fixed in both adapters, validated end to end against a real failure.**

### The defect

Neither adapter's `Snapshot` carried a timestamp, so `revision == 0` was the only
liveness test. That detects "no telemetry has ever arrived"; it cannot detect
"telemetry stopped". Once `revision` was non-zero, a frozen snapshot was
indistinguishable from a live one.

Captured on px4_18: the adapter published state until t+1198 s, then stopped.
Commands arriving at t+1558 s were dispatched against that frozen snapshot, which
can never satisfy `done()`, so each burned the full 15 s offboard timeout and was
receipted `px4_offboard_timeout` — blaming PX4 for refusing offboard when in fact
the adapter was blind.

### The fix

`Snapshot.updated` holds the arrival time of the last telemetry, and a command is
refused `autopilot_unavailable` when it is absent or older than 5 s. This replaces
the `revision == 0` test, which it subsumes.

5 s is not an invented number: it matches `kFreshStateMs` in
`tools/fleet_command.cpp`, which refuses to issue against a vehicle whose state is
older than that. That gate is the stricter of the two — the Edge write happens
after the telemetry it carries — so a command the fleet considered safe to issue
is never rejected by the adapter.

All four rejection paths in both adapters were previously **silent**: a vehicle
would sit still with no explanation in any log. They now log.

### Validated

On a live fleet, by hand:

```
px4_2 (XRCE session dead, no telemetry):
  adapter: "command px4_2-... failed: autopilot_unavailable"     <- now logged
  receipt: failed/autopilot_unavailable
px4_0 (healthy):
  adapter: "command px4_0-... accepted"
  receipt: accepted                                              <- no false reject
```

### Retracted hypothesis

The first theory in this file's "Still open" list — hold the command while
`revision == 0` rather than rejecting — would not have fixed any observed failure.
px4_2 had no telemetry for an entire 1778 s run; holding would only have delayed
the identical failure until expiry while blocking the queue behind it.

### XRCE session failure — ROOT-CAUSED AND FIXED (2026-09-15)

```
ERROR [uxrce_dds_client] create entities failed: participant: 255
```

`uxrce_dds_client.cpp:312` gives the agent a **hardcoded 1000 ms** to create the
DDS participant and reply; `255` is the uninitialized status, i.e. no reply
arrived. The retry at `:628` re-enters the same call and hits the same wall.

**Cause:** `UXRCE_DDS_PTCFG` was left at its default of 0, so the agent built the
participant with full builtin transports — shared memory, every interface,
multicast discovery. That exceeds 1000 ms under load. Setting it to **1**
(localhost only: `useBuiltinTransports=false`, one UDPv4 transport whitelisted to
127.0.0.1) matches this simulator, where everything is on loopback.

`run-px4.sh` now injects `param set UXRCE_DDS_PTCFG 1` into `rcS` alongside the
existing XRCE gating (rootfs marker v21). **Result: 2 runs, 10/10 ROS vehicles
with 27 writers, zero failures — baseline was 2 stranded per run.**

**Full twenty-node lifecycle gate now PASSES, 2/2 consecutive:**

| run | startup | verify | shutdown | total |
| --- | --- | --- | --- | --- |
| g20A | 56 s | **50 s — 20/20 command-ready and moving** | clean | 198 s |
| g20B | 98 s | **118 s — 20/20 command-ready and moving** | clean | 310 s |

Eliminated along the way, each by direct test: the UDP-only Fast DDS profile on
the agents; clearing leaked `/private/tmp/boost_interprocess` segments;
restarting the client module; restarting the agent; staggering client start by
2 s per wave position; and `px4-bounded.sh` (failures reproduced with the wrapper
fully disabled). Trimming `dds_topics.yaml` would NOT have helped — the failure
is at the participant, before any writer exists.
