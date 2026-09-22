# Ditto Edge Simulator — Project Status

Current state, held decisions, and what is next. Reviewed 2026-09-18.

Dated investigation narratives are **not** kept here. They live in the knowledge
store; `DEBUGGING.md` carries the index. This file should be rewritten in place
when the state changes, never appended to with a new dated section.

## Where the project is

The working target is the **twenty-node fleet**, and it passes end to end. Two
vehicle tiers are interchangeable beneath the same Edge/adapter stack:

- **PX4 SIH** — real autopilots, on PX4 `v1.18.0-rc1` with zero local delta
  (`g7kah`). Ten ROS 2 and ten native MAVLink vehicles in `mvp-twenty-mixed`.
- **Synthetic** — `tools/synthetic_fleet.cpp`, one process emulating N PX4s over
  MAVLink. It replaces the autopilot and nothing else: each vehicle is driven by
  its own real `px4-mavlink-ditto-bridge` process, so a synthetic run exercises
  the shipped adapter in production's process shape. It exists to reach scales no
  machine can run PX4 at.

Edge Server is on Ditto SDK `5.1.0` (`g7n9f`). Commands propagate to all twenty
nodes near-instantly, which they did not on 4.12.4.

**The 100-node ceiling moved, and the reason is not settled.** `g7n9g` recorded
44/100 vehicles holding state fresh within `kFreshStateMs` at 10 Hz, attributed
to the SQLite replication write path — roughly one commit per document applied,
~4,200 applications/sec across 21 stores on one disk. On 2026-09-18, after the
synthetic tier moved its writes out into one adapter process per vehicle,
`synthetic-hundred` verified **100/100 command-ready and moving**.

The publish rate did not change and neither did the coalescing. What changed is
that 200 `EdgeLink` threads and 100 gRPC clients in one address space became 100
independent processes. If the old ceiling were purely disk-bound that should not
have helped at all, so part of the 44/100 was likely the single-process synthetic
harness measuring itself. **One run, on a machine at load average 144 — repeat it
before treating either number as the truth.**

The network observer that was gating this landed on 2026-09-17 (`g7nda`,
`g7ndf`), so scaling past twenty is no longer blocked on it. What the observer
found instead is that the fleet had been replicating over paths nothing was
watching: AWDL and Ditto Cloud carried traffic while the relay reported "21
links, healthy". Both are suppressed by default now. The same blind spot exists
one level down — LAN and relay-proxied links are indistinguishable in presence —
and closing it is the next piece of work.

## Decisions held

- An autopilot owns vehicle motion. Commands flow through the vehicle's local Edge Server and ROS
  adapter to PX4; the simulator never moves a drone directly.
- Each vehicle is practically self-contained: PX4 SIH, XRCE-DDS Agent, ROS bridge, vehicle Edge
  Server, and local persistent store. Process Compose only supervises them.
- A simulation tier removes exactly one cost and names what it stops proving.
  The synthetic tier removes PX4 — which is what forced out `rclcpp` — and keeps
  the adapter, the autopilot transport and the per-vehicle process boundary,
  because none of those had to go with it.
- Adapters are one codebase. Two implementations of one program required to
  behave identically, with nothing enforcing it, drifted in eleven places; the
  guarantee now comes from there being one implementation, not from diligence.
- The viewer initially shows each vehicle's self-published local PX4 state, not simulator truth or
  operator knowledge.
- Operator-to-vehicle Edge TCP synchronization traverses a simulator-owned relay with a scenario
  capacity budget and externally measured per-link TX/RX utilization.
- The network observer is a separate headless service, not part of the viewer. A
  customer needs to drive cut-off from their own state machine; the viewer is one
  client of it, not its owner.
- Cut-off is native, throttling is not. The SDK has no rate limit anywhere, so
  shaping stays in the relay (simulator-only) or in host-level `tc`/`pf`. Do not
  promise a Ditto-side throttle.
- Replay is intentionally deferred.

## Implemented

- **Five scenarios** under `scenarios/`: `mvp-two-px4`, `mvp-four-mixed`,
  `mvp-twenty-mixed`, `synthetic-twenty`, `synthetic-hundred`. `session/render-fleet.sh`
  generates per-node Edge Server configuration from the scenario declaration; a
  `SIM_PROCESS_TEMPLATE` ending in `.sh` is treated as a generator, because 100
  vehicles is 100 Edge Server entries.
- **One adapter codebase**, vendored at `../Ditto-Edge-Server/ditto-edge-adapters/`.
  Schema, guidance, Edge I/O and the command state machine are a shared C++
  library, `adapters/core`; each adapter is one file implementing four methods.
  The Rust MAVLink adapter is gone. Eleven behavioural drifts between the two
  implementations were resolved in the process, one of them a real bug: the two
  returned opposite-signed yaw for the approach leg of every orbit.
- **Adapter tests**, the first this tree has had: 98 golden vectors captured from
  the Rust adapter before deletion, replayed against the C++ core, plus a
  twelve-case command-lifecycle test with a fake autopilot and an injected clock.
- **gRPC C++ command client** (`tools/fleet_command.cpp`). It writes only to the
  operator Edge Server; replication carries `arm`, `disarm`, `goto_local` and
  `orbit_local` to the vehicle nodes.
- **Live raylib viewer** over PX4 display telemetry and simulator-owned link
  metrics, with procedural quadrotors, PX4 quaternion attitude and time-bounded
  pose interpolation. It parses MAVLink off UDP directly and does not read Ditto.
- **Simulator-owned network relay** (`network/relay.cpp`) applying a per-link
  capacity budget and publishing measured TX/RX utilization. Delay, loss and
  topology remain out of scope.
- **Network observer** (`g7ndf`), end to end. Edge Server gained `EdgeTransport`
  (read/write eleven transport kinds at runtime, no restart) and `ObservePresence`
  (streaming). The `ditto-network-observer` crate serves `GetNetwork`,
  `WatchNetwork` and `SetTransports`; a C++ client and `ditto_observer_ctl` sit in
  front of it. The viewer draws one edge per (pair, transport) with the cloud as a
  per-node stalk, and toggles paths per node or fleet-wide. It starts as a
  Process Compose process on `SIM_OBSERVER_ADDR`, so `sim-down` stops it.
- **Managed Process Compose session**: `pixi run sim` builds missing artifacts,
  starts the fleet, opens the viewer and runs the full `down.sh` path on exit;
  `pixi run sim-status` shows process states; `pixi run sim-down` is idempotent.
- **Static world scenery** (`viewer/src/world.hpp`, `worlds/`). One header the
  viewer includes and a JSON declaration per world, behind `SIM_WORLD_FILE`. A
  world that declares an `extent_m` also sizes the ground plane, and the camera
  frames on the span of the objects; both used to be guessed from vehicle count.
- **SDK Unix-socket gRPC authority fix** at `ditto-edge-adapters/sdk/cpp/src/client.cpp:90`,
  forcing `localhost` authority so the Rust HTTP/2 server accepts UDS requests.

## Validated

| scenario | result |
| --- | --- |
| `mvp-four-mixed` | 2/2 full lifecycle, 4/4 command-ready and moving, ~35 s |
| `mvp-twenty-mixed` | 2/2 full lifecycle, 20/20 command-ready and moving, on v1.18.0-rc1 |
| `synthetic-twenty` | 2/2 full lifecycle, 20/20 command-ready and moving in 8-10 s, through real adapters |
| `synthetic-hundred` | 100/100 command-ready and moving; 204 processes, 9.9 GB of 24 GB, verify 61 s |

Validation is per-scenario and resets on any change to PX4 startup ordering,
MAVLink ports or streams, adapter ordering, Process Compose lifecycle, or
shutdown. See `DEBUGGING.md`.

Observer measurements, all on `synthetic-twenty` (`g7nda`, `g7ndf`):

| what | result |
| --- | --- |
| cut-off | 21 nodes reconfigured in 317 ms, no restart; 90 AWDL + 21 cloud sockets gone, all processes alive |
| presence vs `lsof` | matched link for link on both runs, so the graph is measured and not inferred |
| settled fleet | 1 snapshot in 30 s with AWDL and cloud both on — a quiet fleet is genuinely quiet |
| presence resolution | ~1 s, coalesced. Not the sub-10 ms figure, which belonged to the store's Observe RPC |
| LAN discovery on | 21 links to 175 of a possible 210; sockets 42 to 450 |

Two things about isolation that the network manager depends on. Cutting
`tcp_connect` does not isolate a node — peers keep dialing in, so isolation needs
both directions. Big Peer is the exception: it lives only in `connect`, nothing
ever dials in, and clearing `websocket_urls` takes it down live.

### What the synthetic tier proves, and what it does not

Proves, and did not before 2026-09-18: the shipped adapter binary, N of them in N
processes, at N up to 100 — its MAVLink codec, telemetry decode, publish loop,
command queue, receipt path and UDS gRPC client, in production's process shape.
The adapter's command state machine against PX4's real acceptance ordering:
setpoints must stream before offboard is granted, mode and arm repeat at 1 Hz
until taken, and acceptance needs armed and offboard in one heartbeat at PX4's
1 Hz rate. And offboard-loss behaviour — killing an adapter now drops its vehicle
out of offboard and holds it, which was previously invisible.

Does not prove: PX4 itself (EKF2, the controllers, arming checks, geofence,
failsafes, every mode but offboard); PX4's own MAVLink implementation (stream
scheduling, datagram coalescing, `COMMAND_ACK`, mission and parameter protocols,
signing); the ROS 2 and uORB path, which is unchanged and still proven only by
`mvp-*` on real PX4; dynamics beyond a point mass, with roll and pitch always
zero and no wind; estimator error, with `local_valid` and `global_valid` always
true; PX4 boot, rcS and XRCE; and lossy links, since loopback UDP does not drop.

## Next

0. **Confirm the viewer's position source on real PX4.** The viewer now prefers
   `LOCAL_POSITION_NED` per vehicle and falls back to `GLOBAL_POSITION_INT`.
   PX4's `onboard` stream set, which the viewer link uses, streams it at 30 Hz
   where the global stream ran at 50 Hz, so `mvp-*` changed rate and datum and
   has not been run since. The synthetic tier is unaffected: its display link
   carries no local position, so it still takes the global path. Cheap to check
   on the next `mvp-*` run and it gates nothing else.
1. **Merge the relay's per-link `tx_bps`/`rx_bps` into the observer.** This is
   the top of the list because it fixes two things at once. Presence carries peer
   keys and a transport type but no addresses, so a direct LAN link and a
   relay-proxied one both report `access_point` and cannot be told apart. A link
   the relay lists is proxied; one it does not list is direct. The same merge is
   the only route to bandwidth per link. In production there is no relay, so the
   ambiguity is a simulator problem only.
2. Finish the observer's rough edges: aggregate across per-node observers (today
   one observer reads N sockets, which is only correct on a single host), surface
   toggle errors in the UI (chips fire on a detached thread and a failure goes to
   the log), and make `multicast_beta`'s group address, port and interface
   settable.
3. Repeat `synthetic-hundred` and settle whether the old 44/100 was an Edge
   Server limit or the single-process harness. It is one run either way, and the
   number people quote should be the one that is true.
4. Verify whether a node still pushes local writes to Big Peer with zero
   subscriptions registered. Untested, and a one-way-cloud story depends on it.
5. Put the `vehicle_state` publish rate back under a knob. It was
   `--state-rate-hz` on the synthetic fleet and is now the adapter's fixed
   100 ms, so sweeping it — which is how the freshness ceiling was found — needs
   an argument or env var on the adapter.
6. Real network bearer, topology and impairment controls in front of Edge Server
   traffic — the relay applies capacity only today.
7. Per-peer admission control. `presence().set_connection_request_handler` allows
   or denies each inbound request with the peer's metadata in hand, which is finer
   than per-transport and is the natural next phase for the customer's priority
   state machine.
8. World truth. **Static scenery started 2026-09-21**: `sim::world`
   (`viewer/src/world.hpp`) draws boxes and cylinders declared in PX4 local NED,
   with `base_m` for stacked storeys and `solid`/`wire`/`translucent` styles.
   Display only, by design -- nothing publishes it and no vehicle reads it
   through the viewer, so the no-simulator-truth rule costs nothing to hold.
   **The twenty-node scenarios declare no world**: scenery under that fleet was
   tried and read as clutter, so they run on empty ground and the loader is kept
   for worlds that are the scenario. Still to do: objects with physical
   consequence, which belongs on the autopilot side of the MAVLink boundary, and
   the viewer mode contrasting world truth, operator knowledge, and a selected
   vehicle's local knowledge.
9. The first local autonomy executive and mission scenario.
10. Recording and metrics only when a real scenario needs them; replay stays out
    of scope until then.

## Traps

- **A fleet command carries its own deadline, and the adapter enforces it.**
  `expires_unix_ms` is stamped by the command client and checked on arrival at
  `adapters/core/src/bridge.cpp:130`, which rejects a late command with status
  `expired` / `command_expired`. The document replicates either way, so an
  expired one looks exactly like replication having failed. Look for an
  `expired` receipt in `fleet_command_receipts` before suspecting the sync path.
  The TTL is `kDefaultCommandTtl`, 240 s, overridable with `SIM_COMMAND_TTL_S`.
  It was 30 s for `batch` and single commands while `verify` used 240, so the
  same fleet disagreed with itself about when an order went stale; isolating a
  node for more than 30 s and restoring it therefore never applied the pending
  command. One value now.
- **The viewer's NO LINK threshold is coupled to the relay's publish cadence,
  and the margin is measured rather than assumed.** The relay samples every
  second; the viewer re-reads every 250 ms and calls a node dead after
  `kMetricsStaleMs`. Every node inherits the file's single `observed_unix_ms`,
  so once that margin is gone the whole fleet blinks at once. Shortening the
  threshold or lengthening the relay's sample interval reintroduces it.

- `process/edge-server.sh` prefers `target/release/ditto-edge-server` and never
  rebuilds it. After changing Edge Server, run `cargo build --release` or you will
  debug a binary without your code. Left this way on purpose: it is another
  product's artifact and may be pinned deliberately.
- The presence connection type for peer-to-peer Wi-Fi serialises as
  `p2_p_wi_fi`, not `p2p_wifi`. That string is the observer's and the UI's
  per-transport key.
- `sync_group` partitions the mesh but is an optimization, not a security
  control. An explicit connect transport still syncs across groups.
- PX4's generated MAVLink dialect omits the `MAV_CMD` enum entirely — it uses its
  own uORB constants — so `MAV_CMD_DO_SET_MODE` and `MAV_CMD_COMPONENT_ARM_DISARM`
  do not exist as symbols. Both the adapter and the synthetic fleet write the
  wire values (176, 400) out instead.
- Only a process with exactly one MAVLink link may use the global
  `MAVLINK_COMM_0` parser channel. The synthetic fleet and the viewer each decode
  N links and carry per-link state; sharing the channel lets one link's partial
  frame corrupt another's.
