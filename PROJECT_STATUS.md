# Ditto Edge Simulator — Project Status

Last updated: 2026-09-14

## Status — 2026-09-14

**Not yet known what is fixed.** The random per-vehicle startup hang was traced
to PX4's posix daemon missing a `poll()` wakeup for a pending client
connection, and `config/px4-bounded.sh` now nudges the daemon after 2 s of
silence. That is proven on a single instance (25/25 boots) and has produced
exactly one 20/20 fleet startup; on that launch `px4_0`'s first command was
rejected `autopilot_unavailable` (adapter started after the command was
written, before telemetry) and its later commands executed. The two-run lifecycle gate is 0/2. Full record,
disproved theories, and open items: [`DEBUGGING.md`](DEBUGGING.md), section
"PX4 daemon lost wakeup". The repository was first committed today
(`cb85d0c` baseline; fixes in `680869a`, `5c8a22d`, `d9dd4db`).

## Startup reliability update — 2026-09-10

The startup failure record, root cause, and validation evidence are in
[`DEBUGGING.md`](DEBUGGING.md). Direct display telemetry no longer relies on
PX4's nondeterministic `minimal` profile: every PX4 now explicitly streams
`GLOBAL_POSITION_INT` and `ATTITUDE`. The telemetry gate runs synchronously, so
failure triggers immediate managed teardown rather than leaving a stale fleet.

Two consecutive clean managed 20-node runs passed 20/20 direct telemetry,
20/20 application readiness, an accepted `px4_19` command receipt, and managed
viewer shutdown. A later live run exposed a `px4_10` onboard-control startup
regression; its correction is staged and documented in `DEBUGGING.md`, but the
mandatory two-run lifecycle sequence is reset.

The current live run also exposed a ROS adapter startup wedge on `px4_1`:
Fast DDS blocked in shared-memory participant creation before the adapter could
publish state or receive commands. A staged UDP-only ROS transport correction
and fleet-wide command/observed-motion readiness gate are documented in
`DEBUGGING.md`. They have not been applied to the live session and are not yet
validated.

Clean 20-node validation is in progress but has no qualifying pass yet. The
full failure trail and the current staged startup/readiness changes are kept in
`DEBUGGING.md`; do not treat the earlier two-run record as applicable to the
current source.

## Current 20-node startup design — 2026-09-11

Startup is now dependency-driven: Edge Servers, then XRCE-DDS Agents, then all
PX4 instances concurrently, then full PX4 startup/direct telemetry, adapters,
and the viewer. The earlier serial PX4 boot, fixed settling period, adapter
staggering, and retry loops were removed. A clean observation reached 20/20 PX4
startup completions, 20/20 direct telemetry, and 105/105 mesh links; validation
of the changed source is still 0/2 runs.

Normal startup and demo scripts no longer issue automatic commands or require
fleet receipts/movement. Open the viewer with `pixi run sim`, then issue any
commands manually.

## Current goal

The first MVP recreates the proven `ditto-autonomy-testing` two-PX4 slice with the new gRPC Edge
Server and ROS 2 adapter. It deliberately has no world model, autonomy executive, or replay
system yet. The network relay currently applies capacity only; delay, loss, and topology remain
out of scope.

## Decisions held

- An autopilot owns vehicle motion. Commands flow through the vehicle's local Edge Server and ROS
  adapter to PX4; the simulator never moves a drone directly.
- Each vehicle is practically self-contained: PX4 SIH, XRCE-DDS Agent, ROS bridge, vehicle Edge
  Server, and local persistent store. Process Compose only supervises them.
- The viewer initially shows each vehicle's self-published local PX4 state, not simulator truth or
  operator knowledge.
- Operator-to-vehicle Edge TCP synchronization traverses a simulator-owned relay with a scenario
  capacity budget and externally measured per-link TX/RX utilization.
- Replay is intentionally deferred.

## Implemented

- Scenario `mvp-two-px4`: two vehicle Edge Servers, one operator Edge Server, two PX4 SIH
  instances, two XRCE-DDS Agents, and two ROS 2 adapters.
- Rendered, per-node Edge Server configuration: Unix-socket gRPC APIs, loopback TCP peers,
  isolated persistence, vehicle command subscriptions, and operator subscriptions to state and
  command status.
- New gRPC C++ command client. It writes only to the operator Edge Server; replication carries
  `arm`, `disarm`, `goto_local`, and `orbit_local` documents to the vehicle nodes.
- Live raylib viewer over PX4 display telemetry and simulator-owned link metrics. It renders
  procedural quadrotors with PX4 quaternion attitude and time-bounded pose interpolation.
- Managed Process Compose session:
  - `pixi run sim` builds missing artifacts, starts the fleet, opens the viewer, and invokes the
    complete `down.sh` path when the viewer exits.
  - `pixi run sim-status` shows process states; `pixi run sim-down` is an idempotent stop.
- SDK Unix-socket gRPC authority fix in
  `../Ditto-Edge-Adapters/sdk/cpp/src/client.cpp`. It forces `localhost` authority so the Rust
  HTTP/2 server accepts UDS requests.

## Proven live

- All nine supervised processes reached ready state.
- Vehicle state replicated through the operator Edge Server.
- `goto px4_0 15 -10 5` was accepted; PX4 armed, entered offboard mode, and moved toward the
  target.
- `orbit px4_1 0 0 5 10 3` was accepted; PX4 armed and entered offboard orbit flight.
- Operator queries observed the accepted status and changing PX4 local state.
- Adapter, command client, and viewer compile successfully. The existing adapter SDK unit test
  also passed.

## Running the MVP

From this repository:

```bash
pixi run sim
```

Use another terminal for commands while the viewer is open:

```bash
./scripts/command.sh goto px4_0 15 -10 5
./scripts/command.sh orbit px4_1 0 0 5 10 3
./scripts/command.sh status px4_0
```

Closing the viewer ends the managed session. `pixi run sim-down` is available for an explicit or
recovery stop.

## Credentials and configuration

- Credentials live in this repository's ignored `.env`, copied from the former prototype, with
  owner-only permissions.
- `pixi run sim` always re-renders the three Edge Server configurations before startup, so a token
  or auth-URL change requires no source rebuild—end the session and start it again.
- If `DITTO_DB_ID` changes, remove the generated scenario runtime state under
  `build/runtime/mvp-two-px4/` before the next run. This prevents a local persistent Ditto store
  from carrying state across database identities.

## Current working state

No Process Compose simulator session is active as of this update.

The new simulator repository is still an initial uncommitted worktree; all source and configuration
files are untracked. The adapter SDK transport fix is a separate uncommitted modification in
`Ditto-Edge-Adapters`. Do not accidentally include the pre-existing untracked
`Ditto-Edge-Server/CODEBASE_CONTEXT.md` in a commit.

## Deferred next work

1. Replace the fixed two-node shell scenario with a concise scenario declaration and generator.
2. Add world truth, then a viewer mode that contrasts world truth, operator knowledge, and a
   selected vehicle's local knowledge.
3. Insert real network bearer/topology/impairment controls in front of Edge Server traffic.
4. Add the first local autonomy executive and mission scenario.
5. Add recording/metrics only when a real scenario needs them; keep replay out of scope until then.
