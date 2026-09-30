# Ditto Edge Simulator

A test bench for running a fleet of drones on one laptop, each with its own real
Ditto Edge Server, and watching how they replicate to each other over a network
you control.

The point is to put Ditto under realistic load. The autopilots are simulated and
the radio links are a local relay, but the Edge Servers, the adapters, the
replication, and the gRPC command path are the real software. Nothing here fakes
a sync.

## Setting it up

You need macOS on Apple silicon, Xcode Command Line Tools, [pixi](https://pixi.sh),
and a Rust toolchain. Nothing comes from Homebrew: gRPC, Protobuf,
nlohmann_json, CMake and Ninja are all pinned in `pixi.toml`, so every build
links the same versions on every machine.

```bash
pixi install                              # toolchain: process-compose, gRPC, CMake
pixi run -e ros vcs import deps < dependencies.repos   # ROS scenarios only
```

`dependencies.repos` pins the two external source dependencies by exact revision:
`px4_msgs` and the Micro XRCE-DDS Agent. Neither is packaged for this platform,
so both are built from source. Everything fetched lands in the gitignored
`deps/`.

PX4 is cloned separately, because it is submodule-heavy and vcstool handles
submodules poorly:

```bash
git clone --recursive --branch v1.18.0-rc1 \
  https://github.com/PX4/PX4-Autopilot.git deps/PX4-Autopilot
cd deps/PX4-Autopilot && make px4_sitl_sih
```

`v1.18.0-rc1` is the revision this simulator is validated against. Required for
every scenario, synthetic included.

**Edge Server and the adapters do not have to be built.** Drop prebuilt
binaries in `bin/` -- `ditto-edge-server`, `px4-mavlink-ditto-bridge`,
`px4_ditto_bridge_node` -- and the simulator uses them ahead of any local build
and never rebuilds over them. That removes the Edge Server cargo build and the
adapter builds. It does **not** remove the adapters checkout or the Rust
toolchain: the C++ SDKs are compiled from that tree for the viewer and command
client, and the network observer is cargo-built on every run. See
`bin/README.md`.

**Only ROS scenarios pay for ROS.** ROS 2 Jazzy, colcon, `px4_msgs` and the XRCE
agent live in a separate pixi environment and are built only when a scenario
declares ROS vehicles. Synthetic scenarios install none of it.

**Every scenario needs PX4 though**, even the synthetic ones: the viewer, the
relay probe, both fleets and the MAVLink adapter all compile against the MAVLink
headers that `px4_sitl_sih` generates, and the build stops with an error without
them.

```bash
./scripts/build/ros-underlay.sh    # px4_msgs; only for ROS scenarios
```

Credentials go in a gitignored `.env` at the repository root -- `DITTO_DB_ID`,
`DITTO_AUTH_URL`, `DITTO_ACCESS_TOKEN`, plus `DITTO_WEBSOCKET_URL` if you start
with `SIM_ENABLE_CLOUD_SYNC=1`. Point `DITTO_EDGE_ENV_FILE` elsewhere to
override it.

## Running it

```bash
pixi run sim
```

That builds anything missing, starts the fleet under Process Compose, waits for
telemetry, and opens the viewer. Closing the viewer shuts everything down. With
no argument you get `mvp-two-px4`: two PX4 vehicles.

Name a scenario to get a different fleet:

```bash
pixi run sim synthetic-twenty
```

Two shortcuts exist for the ones in regular use:

```bash
pixi run sim-twenty     # synthetic-twenty
pixi run sim-hundred    # synthetic-hundred
```

`pixi run sim-factory` runs `factory-twenty`, twenty ground robots in a
three-level building; `pixi run demo-factory` shows that world with no network
at all. `pixi run sim-status` tells you whether a fleet is up. `pixi run sim-down` stops
one; it is safe to run when nothing is running.

Only one scenario at a time. Starting a second while the first is up is refused.

## Scenarios

| name | vehicles | what it is |
| --- | --- | --- |
| `mvp-two-px4` | 2 PX4 | the default, and the smallest thing that proves the whole path |
| `park-mgm-two-px4` | 2 PX4 | the same stack over a 1 km display-only map around Park MGM |
| `park-mgm-cesium` | 1 PX4 | Park MGM Cesium MVP; fly and watch from UAS Tool |
| `park-mgm-cesium-10` | 1 PX4 + 10 synthetic | The MVP plus ten cloud-connected, commandable synthetic nodes |
| `park-mgm-8x8` | 8 PX4 + 8 synthetic | ISR load test; each PX4 on UAS Tool at TCP `5760+i` |
| `park-mgm-8-px4` | 8 PX4 | each PX4 on UAS Tool at TCP `5760+i`, with its own RTSP feed |
| `mvp-four-mixed` | 4 PX4 | two ROS 2 adapters, two native MAVLink adapters |
| `mvp-twenty-mixed` | 20 PX4 | ten of each adapter; the real-stack ceiling on one machine |
| `synthetic-twenty` | 20 synthetic | same shape as the above, without PX4 itself |
| `synthetic-hundred` | 100 synthetic | 100 Edge Servers in a five-peer mesh |

The synthetic scenarios replace the autopilot, and only the autopilot. One
process emulates N PX4s: a kinematic integrator wearing PX4's MAVLink manners,
streaming the same messages on the same schedule and honouring the same
arm-and-offboard handshake, down to refusing offboard until setpoints are
already flowing.

`park-mgm-cesium-10` builds on the `park-mgm-cesium` MVP: it keeps `px4_0` real
for phone flight and video, and runs `px4_1` through `px4_10` synthetically.
Every vehicle has its own Edge Server and adapter; the scenario connects each
node to Ditto Cloud.

Run the baseline with `pixi run sim park-mgm-cesium`, or the expanded scenario
with `pixi run sim park-mgm-cesium-10`. In the expanded scenario, commands to
`px4_1` through `px4_10` still travel through their Edge Server adapters.

Everything above that is real. Each vehicle gets its own
`px4-mavlink-ditto-bridge` process — the same binary a PX4 vehicle uses — and
that adapter does every Ditto write. So a synthetic run exercises the shipped
adapter, its MAVLink codec, its command state machine, and its Edge Server
client, in production's process shape.

What it costs is the autopilot: no EKF, no controllers, no rcS, no XRCE. Twenty
vehicles is 44 processes instead of 73, and a hundred is reachable at all.

Both tiers are worth running. The PX4 scenarios prove PX4. The synthetic ones
find where Edge Server breaks.

## What a node is

Each vehicle is a self-contained stack with its own persistent store:

```text
PX4 SIH, or an emulated PX4 from the synthetic fleet
    <-> ROS 2 or MAVLink adapter        (MAVLink only, for a synthetic vehicle)
        <-> Ditto Edge Server
            <-> relay-controlled network
```

The adapter is a separate process in both tiers. Only the autopilot differs.

Process Compose only supervises these processes. It does not connect them.

The autopilot owns vehicle motion. Commands reach a vehicle by replicating into
its local Edge Server; nothing in the simulator moves a drone directly.

## Commanding the fleet

The command client writes one shared `fleet_commands/fleet-current` document
through the operator's Edge Server. Each vehicle picks out its own entry, runs
it, and writes back to `fleet_command_receipts`. All three collections
(`vehicle_state`, `fleet_commands`, receipts) are subscribed mesh-wide, so a
command exercises the replication path rather than a side channel.

```bash
./scripts/build/command-client.sh
./scripts/command.sh goto px4_0 15 -10 5
./scripts/command.sh orbit px4_1 0 0 5 10 3
./scripts/command.sh status px4_0
```

`arm`, `disarm`, `goto`, `orbit`, `status`, `ready`, `verify`, and `batch` are
the available verbs; run the client with no arguments for the full argument
lists.

A command carries a deadline, `expires_unix_ms`, and the **adapter** enforces it
on arrival -- a late one is rejected with `command_expired` and a receipt, not
executed. The document still replicates, so an expired command is easy to
mistake for replication having failed. The TTL is 240 s; set `SIM_COMMAND_TTL_S`
to raise it for a partition test meant to outlast that:

```bash
SIM_COMMAND_TTL_S=1800 ./scripts/demo/twenty-spread.sh
```

A fleet does not move until it is commanded. On startup everything sits at the
origin. The demo scripts give you something to look at:

```bash
./scripts/demo/four.sh              # mvp-four-mixed: opposing waypoints and orbits
./scripts/demo/twenty-spread.sh     # mvp-twenty-mixed: twenty orbits across the map
./scripts/demo/twenty-converge.sh   # mvp-twenty-mixed: pull it back in
./scripts/demo/park-mgm-cesium-10.sh # park-mgm-cesium-10: spread ten synthetic nodes
```

Each demo defaults to the scenario it was written for. To drive a synthetic
fleet with the twenty-node demos, point them at the running scenario, or they
will read the wrong runtime tree and appear to do nothing:

```bash
SIM_SCENARIO_FILE="$PWD/scenarios/synthetic-twenty.env" ./scripts/demo/twenty-spread.sh
```

## The network, and seeing it

Every Ditto TCP connection goes through the simulator's own relay
(`network/relay.cpp`). The relay is deliberately dumb: an opaque byte-stream
path that enforces the scenario's per-direction capacity budget and reports
per-link connection, TX, RX, and utilization to `network-metrics.json`. Ditto
still owns peer selection, subscriptions, reconciliation, and encryption.
Keeping the relay outside PX4, the adapters, and Edge Server means watching a
run does not add replication traffic of its own.

Three scenario knobs shape the mesh:

- `SIM_MESH_PEERS_PER_VEHICLE` — how many succeeding vehicles each one lists as
  a known TCP peer, making a directed ring.
- `SIM_OPERATOR_MESH_PEERS` — how many evenly spaced points the operator
  attaches at.
- `SIM_NETWORK_LINK_CAPACITY_KBPS` — the per-direction budget per link.

The relay does not yet model delay, jitter, loss, or range. When it does, it
belongs here, in front of real traffic.

The network observer starts with the fleet as another Process Compose process,
so it comes up and goes down with everything else. It reports which transport
each node is actually using and can cut individual paths on command. The viewer
draws those as links coloured by transport, with a per-node toggle and a
fleet-wide button for the cloud. Every client shares one address,
`SIM_OBSERVER_ADDR`, default `127.0.0.1:50090`. You can query it without the
viewer:

```bash
../Ditto-Edge-Server/ditto-edge-adapters/target/debug/examples/netctl show
build/cmake/ditto_observer_client/ditto_observer_ctl show
```

The Ditto Cloud link is off by default; both Park MGM Cesium scenarios enable
it. The observer draws a stalk on every node holding one. Cloud paths never
show up in a presence graph's connections, so the observer synthesises those
edges from each peer's `is_connected_to_ditto_cloud` flag.

## The world

Scenery, and only scenery. A scenario may name a world file:

```
SIM_WORLD_FILE=worlds/factory-three-level.json
```

**The twenty-node scenarios declare no world and run on empty ground.** That is
deliberate: the fleet is what those scenarios are for, and blocks standing under
it read as clutter rather than context. The loader is here for worlds that earn
their place, like the factory scenario, where the building *is* the scenario.

Objects are declared in the same PX4 local NED frame the commands are spelled
in -- north, east, and metres above home -- so a world file can be written
straight off a demo script's coordinates. `box` and `cylinder` are the shapes;
`base_m` raises an object's underside off the ground for a storey above the
first; `style` is `solid`, `wire`, or `translucent` with an `opacity`.

**The world is drawn and nothing more.** No autopilot integrates against it, no
adapter publishes it, and no Ditto store holds it, so a vehicle flies or drives
through a wall without complaint. That is the ground rule below about simulator
truth being honoured the cheap way: truth a vehicle must not have is truth the
simulator never sends. Giving an object physical consequence belongs on the
autopilot side of the MAVLink boundary, where a vehicle reads the world itself
and constrains its own motion -- one file read by two processes, never a channel
from the viewer to a vehicle.

A world's `extent_m` sizes the ground plane. The camera frames on the objects
instead, because the plane is deliberately wider than what stands on it, and it
points at their centre in all three axes rather than at a fixed point near the
origin. Horizontally it frames on the footprint's diagonal, since the default
view is yawed 45 degrees and the content presents corner-to-corner; height is
compared against that rather than folded into it, so it decides the distance
only for something tall on a small plot.

A missing or malformed world file is not an error; the viewer draws bare ground.
A single malformed object is skipped and the rest of the world still stands. The
vehicle-side reader is stricter about its own section: `factory_fleet` refuses
to start on a malformed `building`, because a fleet with no floor plan has nowhere
to drive, and twenty robots silently stacked at the origin is worse than a
refusal. Same file, two readers, two tolerances.

`pixi run sim park-mgm-two-px4` places two real PX4 vehicles at Park MGM and
loads a 1 km static Strip map. It draws OSM building footprints raised by their
recorded height or floor count, with USGS NAIP Plus aerial imagery on the ground
and roofs, plus five landmark labels. Heights missing from OSM are estimated;
this map is for visual orientation, not obstacle avoidance. These are generated
extrusions, not 3D tiles. `worlds/build-park-mgm.py` regenerates the mesh from a
local OSM XML extract. The world file records the OSM and USGS sources; the
viewer displays both attributions. The imagery is public domain per USGS.

## The viewer

raylib, 3D. It parses PX4's dedicated MAVLink display stream straight off UDP
and reads the relay's link metrics. Ordinary worlds remain display-only. The
Park MGM ISR scenario is the exception: the viewer writes a target document
only after a camera feed recognizes that target.

It pins its horizontal origin to the configured PX4 home position and widens the
map past four vehicles, so restarting it mid-flight does not shift the fleet.

## Flying from a phone

`pixi run sim park-mgm-cesium` opens `px4_0` to a ground station on the same
Wi-Fi, such as UAS Tool. `park-mgm-cesium-10` keeps that endpoint and adds ten
synthetic nodes:

- MAVLink: TCP to this Mac's IP, port `5760` (`SIM_GCS_TCP_PORT`)
- video: `rtsp://<this Mac's IP>:8554/px4_<i>` for any of the eleven vehicles,
  H.264
  Constrained Baseline, 1280x720 at 30 fps, over UDP or TCP

The MAVLink port is `tools/gcs_relay.cpp`, which bridges each connection to
`px4_0`'s GCS link on loopback. PX4's UDP links latch onto the first sender for
good, so a phone reconnecting from a new port would never be answered directly.
Each video is the vehicle's nose camera on a level gimbal, 30 degrees down; the
viewer renders only while someone watches and encodes in hardware
(`viewer/src/video.cpp`); closing the viewer ends it. While the viewer serves
video, macOS neither naps it nor lets the Mac idle-sleep. Ditto keeps its own
control path, and whichever of Ditto or the phone commanded last is in charge.

`park-mgm-8x8` and `park-mgm-8-px4` have four fixed ISR sites in `scenarios/park-mgm-isr-targets.json`.
Cesium samples the surface beneath each site so the 2.2 m red target rests on
the road or roof. In a watched camera feed, a target receives a red bounding
box and `TARGET ACQUIRED` label when it is fully in frame, unobstructed, within
100 m, and at least 18 pixels across. First recognition publishes its position
and detecting vehicle to a per-target `isr_targets` document, subscribed by
every Edge Server. The viewer sidebar shows the fixed coordinates and found
count. `RESET MISSION` clears found state and commands every vehicle to hold
at home, 5 m up, without reloading Cesium tiles. CoT target publication is not
part of this first version.

macOS's firewall has to allow `ditto_gcs_relay` and `ditto_fleet_viewer_cesium`
to accept incoming connections. It asks the first time, and again after a
rebuild. Neither port asks for credentials: anyone on the network can fly the
drone and watch its camera.

## Layout

```text
scenarios/   one .env per fleet: node count, ports, mesh shape, capacity
config/      Edge Server and Process Compose templates
scripts/     lib.sh, and the five things you invoke: sim, down, status,
             command, px4-phases
scripts/build/     one per build artifact
scripts/process/   what Process Compose launches, one per process type
scripts/session/   the steps of a launch: up, render, wait, viewer
scripts/demo/      canned fleet orders
network/     the capacity relay
viewer/      the raylib view
worlds/      static scenery, drawn by the viewer and read by nothing else
tools/       command client, telemetry probe, synthetic fleet, GCS relay
bin/         drop prebuilt binaries here; gitignored
patches/     local fixes to external sources, applied at build time
deps/        gitignored: fetched sources -- PX4, px4_msgs, the XRCE agent
build/       gitignored: everything built, plus per-node stores and logs
```

The adapters and the Edge Server client are not in this repository. They live in
`Ditto-Edge-Server/ditto-edge-adapters/`, and the build also needs PX4 with a
built `px4_sitl_sih`. Each external checkout is looked for in `deps/` first and
then beside this repository, so either layout works; `SIM_PX4_ROOT`,
`SIM_EDGE_SERVER_ROOT`, and `SIM_EDGE_ADAPTERS_ROOT` override individually.

Until 2026-09-22 the ROS underlay, the XRCE agent, the process supervisor, and
the fleet's home position all came from a separate `ditto-autonomy-testing`
checkout. That repository is gone and everything it supplied is declared here.

## Configuration

Credentials come from the gitignored `.env` at the repository root. Point
`DITTO_EDGE_ENV_FILE` somewhere else to override it. You do not need to rebuild
after changing credentials; the launcher re-renders Edge Server configuration on
every start. If `DITTO_DB_ID` changes, delete the runtime tree for that scenario
under `build/runtime/<scenario>/` first.

Everything generated — rendered configuration, per-node stores, logs — lives
under the gitignored `build/`. A managed shutdown clears local Edge persistence
but keeps the logs. The viewer ignores cloud-replicated state left over from an
earlier run until each vehicle publishes something fresh.

## Ground rules

These are the constraints the design is actually built around, and breaking one
invalidates results rather than just being untidy:

- Simulator truth never reaches an autonomy decision. A node acts only on what
  arrives through its own interfaces and its own Ditto store.
- Network disruption has to hit real traffic. Changing a number on a display is
  not a disrupted link.
- Any comparison against DDS or Zenoh measures mission outcome and equivalent
  correctness. A baseline built badly on purpose proves nothing.
- Scaling down to three nodes should be as easy as scaling up to a hundred.

## Where to look next

`PROJECT_STATUS.md` has the current state, the decisions being held, what has
been validated, and what is next. `DEBUGGING.md` has the startup contract, the
non-regression rules, and an index into the knowledge store for past
investigations. Read `DEBUGGING.md` before changing anything about startup
ordering, MAVLink ports, adapter ordering, or shutdown.
