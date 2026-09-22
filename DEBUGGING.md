# Simulator startup and lifecycle debugging

This file holds the startup contract and the standing rules — what must stay
true. Dated investigation narratives are no longer kept here; they live in the
knowledge store, which versions them and records what was disproved:

| investigation | `kb get` |
| --- | --- |
| PX4 posix daemon lost wakeup; the `px4-bounded.sh` wrapper | `g7nd2` |
| Startup is disk-bound, not CPU-bound; the MAVLink telemetry gate | `g7nd3` |
| XRCE participant timeout; adapters blind to stopped telemetry; ROS Fast DDS wedge; the `minimal`→`custom` MAVLink profile | `g7nd4` |
| PX4 HRT callout-timer stall — diagnosis, then resolution | `g286u`, `g47eu` |
| PX4 v1.18.0-rc1 upgrade; adapters vendored into Ditto-Edge-Server | `g7kah` |
| Scenario-file scoping: demo scripts hitting the wrong runtime tree | `g7kai` |
| Mesh connection cap, 4.12.4 vs 5.1.0 | `g7ky6`, `g7ky7` |
| Edge Server SDK 5.1 upgrade and the defects it surfaced | `g7n9f` |
| Synthetic fleet: where per-node CPU actually goes | `g7n9g` |
| Network observer: presence measured against `lsof`; AWDL and the cloud link presence never reports; Big Peer direction | `g7nda` |
| Network observer build: runtime transport control, cut-off numbers, and the five defects testing caught | `g7ndf` |

## Port map

Every port the simulator assigns, by vehicle index `i`. Two ranges that overlap
silently produce a fleet flying one vehicle's setpoints, which is far harder to
recognise than a refusal, so `synthetic_fleet.cpp` validates its three bases
before opening a socket.

| what | ports | proto |
| --- | --- | --- |
| Process Compose controller | 18081-18086, one per scenario | TCP |
| XRCE-DDS agent | 8888+i | UDP |
| PX4 display link | PX4 binds 19450+i, streams to 19410+i | UDP |
| PX4 control link | PX4 binds 14580+i, streams to 14540+i | UDP |
| synthetic display link | ephemeral, streams to 19410+i | UDP |
| synthetic control link | fleet binds 25540+i, streams to 24540+i | UDP |
| Edge Server peer listen | vehicle 9100+i, operator 9200 | TCP |
| relay listeners | 10000 + source*(N+1) + destination | TCP |
| network observer | 127.0.0.1:50090 | TCP |
| Edge Server app API | `<runtime>/nodes/<node>/edge.sock` | UDS |

PX4's own pair cannot be reused by the synthetic tier: `14580+0 == 14540+40`, so
it collides with itself past forty vehicles. The synthetic bases are a thousand
apart, which covers the tool's 255-vehicle ceiling, and both sit clear of the
relay's worst case (10000-20199 at N=100) and of 19410-19429.

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

A synthetic scenario has no autopilot tier to boot, so it skips the middle:

```text
Process Compose starts network, Edge Servers and the synthetic fleet
  -> the fleet streams MAVLink from its first tick, unconditionally
  -> one telemetry coordinator verifies all display streams
  -> one real MAVLink adapter per vehicle starts after the marker
  -> viewer starts
```

`process_started` means only that Process Compose launched a shell. It is never
used as proof that PX4, MAVLink, an adapter, or the application is ready.

### Ownership

`scripts/session/wait-telemetry.sh`, invoked synchronously only by `scripts/sim.sh`, is
the sole readiness verifier for UDP ports 19410+i, i < `SIM_VEHICLE_COUNT`. It waits
for the infrastructure tier, then for every PX4 to report `Startup script
returned successfully`, then requires every vehicle to supply both
`GLOBAL_POSITION_INT` and `ATTITUDE`. It writes `px4-telemetry-ready` only after
that succeeds. A failed gate returns directly to `sim.sh`, whose trap immediately
runs managed teardown; it never waits for a marker from a failed background job.

No adapter of either kind may probe or bind a display telemetry port. Both kinds
consume `px4-telemetry-ready` before starting, and bind only their own control
port.

That gate on the MAVLink adapter was once forbidden here, because an adapter
binding late could miss a finite burst of frames and produce
`px4_offboard_timeout`. It is now required instead, and the earlier rule was
withdrawn: both real PX4 (`-f -m onboard`) and the synthetic fleet stream
continuously and forever, so a late bind costs at most one stream period, while
adapters publishing `vehicle_state` into a multi-peer mesh during boot is the
measured startup bottleneck (`g7nd3`).

`pixi run sim` deliberately does **not** run `session/wait-ready.sh`, issue commands, or
require command receipts or observed movement. It starts the fleet, verifies
direct PX4 telemetry, and opens the viewer. Operators issue commands manually
while the viewer is running. The demo scripts likewise submit only their stated
commands; they do not run the fleet verifier first.

On failure the coordinator does not restart PX4. Automatic recovery was hiding
the first failure and could restart an otherwise queued serial boot. It records
the final probe result in `build/runtime/<scenario>/telemetry-gate.log`; the
runtime directory retains that report and the final PX4 logs after managed
shutdown.

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

## Non-regression rules

- Do not treat any one of these as a substitute for another: Process Compose
  `process_started`, PX4 SIH, display telemetry, PX4 startup-script completion,
  adapter availability, operator state replication, command receipt, and motion.
- The SIH marker occurs before PX4 finishes its `px4-rc.mavlink` script. A
  MAVLink vehicle is not command-ready until the log contains `Startup script
  returned successfully` and the onboard link is present.
- Both adapters wait on `px4-telemetry-ready` before starting. The older rule
  forbidding that for the MAVLink adapter was withdrawn; see Ownership above for
  why, and do not reinstate it without re-measuring startup.
- A synthetic vehicle streams MAVLink unconditionally from its first tick, armed
  or not. The adapter binds and waits to be dialled, so it learns nothing about a
  vehicle until a frame arrives: a vehicle that only spoke once commanded could
  never be commanded at all.
- A synthetic vehicle sends its control traffic from the socket it is bound on.
  The adapter uses `udpin` and learns where to reply from the source address of
  whatever arrived most recently, so sending from a second socket would send its
  replies somewhere else entirely.
- Each MAVLink link carries its own parser state. The synthetic fleet decodes N
  links in one process, and the global `MAVLINK_COMM_0` channel would let one
  link's partial frame corrupt another's. Only a process with exactly one link
  may use the global channel, as each adapter does.
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
  link. In `mvp-twenty-mixed`, `SIM_OPERATOR_MESH_PEERS=5` puts the operator on
  `px4_0`, `px4_4`, `px4_8`, `px4_12` and `px4_16`; the remaining vehicles use the mesh. Check PX4 startup, onboard
  sockets, receipt status, and fresh vehicle state separately from the directional
  mesh metrics. For example, `px4_16` was armed and moving with fresh state while
  its expected `px4_16 -> px4_{17,18,19}` mesh metrics were disconnected.
- The ROS tier is built only when `scenario_uses_ros` is true, and that gate must
  stay. Before 2026-09-22 `sim.sh` built the ROS adapter unconditionally, so a
  synthetic or all-MAVLink fleet installed ROS 2 Jazzy, colcon, `px4_msgs`, and
  the XRCE agent for a process it never launched. A synthetic-only user should
  need none of them, and should not need PX4 either.
- A binary in `bin/` wins over a local build and is never rebuilt over. That is
  deliberate, and it is also a trap: a stale drop-in silently shadows every later
  source change. If an edit appears to have no effect, look in `bin/` first.
- gRPC, Protobuf, nlohmann_json, CMake and Ninja come from the pixi environment,
  never from Homebrew. Every build either passes `-DCMAKE_PREFIX_PATH` or runs
  inside `pixi run -e ros`, which is what puts cmake and ninja themselves on
  PATH; `build/xrce-agent.sh` did neither until 2026-09-22 and silently used
  Homebrew's build tools. Do not
  drop those `-DCMAKE_PREFIX_PATH` arguments: CMake finds `/opt/homebrew` on its
  own, so a build without them silently links whatever version brew holds rather
  than the one in `pixi.lock`. The C++ SDKs select `gRPC::grpc++_unsecure` when a
  distribution builds it and `gRPC::grpc++` when it does not; conda-forge ships
  only the latter.
- PX4 and Ditto-Edge-Server resolve through `sim_dep`: `deps/<name>` first, then
  a sibling `../<name>`. Both layouts are supported on purpose, so do not
  hard-code either one. The marker is `deps/<name>/.git`, not the directory --
  an interrupted clone leaves an empty directory that would otherwise shadow a
  working sibling. `px4_msgs` and the XRCE agent are `deps/`-only, since
  `dependencies.repos` puts them there. Revisions are pinned in `dependencies.repos`, and
  `px4_msgs` is not an independent pin -- it must match the PX4 version.

- Any change to PX4 startup ordering, MAVLink ports/streams, adapter ordering,
  Process Compose lifecycle, or shutdown resets the complete two-run gate.


## Rules distilled from the 2026-09-14/15 defects

- A real wedge is **noisy**, not silent. Since `px4-bounded.sh` landed, a stuck
  daemon call logs `nudging daemon` within 2 s and escalates. Log silence means a
  healthy vehicle grinding through quiet `param set` calls. Do not lower
  `SIM_PX4_STALL_SECONDS` back toward 120 s; at 120 s it restarted a live
  `px4_9` mid-`rcS`. (`g7nd3`)
- Bounded-call nudge count does **not** discriminate healthy from sick. It has
  been 0/20 on failing runs and 20/20 on healthy ones. Never use it as evidence
  either way. (`g7nd2`, `g286u`)
- A `px4_offboard_timeout` receipt does not mean PX4 refused offboard. An adapter
  dispatching against frozen telemetry produces exactly that receipt. Check
  telemetry freshness before blaming the autopilot. (`g7nd4`)
- SIH boot, display telemetry, a live XRCE process, and Process Compose
  `Running` are not ROS-control readiness. Only the bridge-ready log plus an Edge
  socket are. (`g7nd4`)
- Retrying an XRCE `create entities failed: participant: 255` cannot work — the
  retry re-enters the same hardcoded 1000 ms window. Fix the participant config,
  not the retry. (`g7nd4`)
- Sweep `/private/tmp/boost_interprocess` periodically. Fast DDS leaks one
  shared-memory segment set per participant, and the UDP-only profile does not
  stop it. (`g7nd3`)
