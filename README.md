# Ditto Edge Simulator

## Purpose

Build a reusable simulation and benchmarking platform for demonstrating Ditto in distributed
robotics and autonomy. The platform will support many heterogeneous nodes running realistic
software stacks while a custom simulated world supplies mission conditions, network disruptions,
and 3D visualization.

The first polished scenario will likely be a collaborative search-and-rescue or ISR mission. The
platform must remain reusable for smaller inspection, logistics, industrial, and public-safety
missions.

## Product stories

1. **Connectivity:** Ditto connects aircraft, ground vehicles, people, Android/ATAK users, operator
   stations, and Big Peer across changing local, tactical, 5G, and satellite links.
2. **Distributed autonomy:** Every autonomous node continues planning from its own persistent local
   knowledge while disconnected, then synchronizes and reconciles information later.
3. **Competitive performance:** The same deliberate mission and network conditions can
   eventually compare Ditto with competent DDS- and Zenoh-based implementations.

## Core architecture

Each autonomous vehicle node should run a realistic, isolated stack:

```text
PX4 or ArduPilot simulation
        <-> ROS 2, MAVLink, or another adapter
Local autonomy executive
        <-> local replicated mission state
Ditto Edge Server
        <-> network interfaces controlled by the simulation
```

Android users should run the real Ditto Android SDK and ATAK integration. Operator and server nodes
should use their real Ditto components, including Big Peer where appropriate.

The custom simulator does not control vehicles directly. Autonomy software issues real movement
commands through each vehicle interface, and the autopilot remains responsible for vehicle motion.
PX4 SIH and comparable lightweight autopilot simulation are the initial scale path. Gazebo is out
of scope for the current phase.

## Repository structure

```text
core/       Lifecycle, orchestration, and scenario execution
world/      Terrain, obstacles, targets, hazards, sensor truth, and agent ground truth
network/    Bearers, topology, range/obstruction models, impairments, and traffic measurement
viewer/     Read-only raylib 3D operational, knowledge, autonomy, and network views
recording/  Deferred event capture and metrics
scenarios/  Mission-specific node composition, configuration, events, and autonomy selection
```

The modules may become separate executables, but they should remain in this repository until their
boundaries are proven.

## Scenario responsibility

A scenario declares:

- Which nodes exist and their initial locations.
- Whether each node is an aircraft, ground vehicle, person, operator, relay, or server.
- Its autopilot or runtime: PX4, ArduPilot, Android, or none.
- Its adapter: ROS 2, MAVLink, ATAK, or another supported integration.
- Its capabilities, sensors, network interfaces, and radio profiles.
- The mission-specific autonomy program it runs.
- Terrain, targets, hazards, scheduled events, and success criteria.

Scenarios configure reusable node templates; they should not duplicate the implementations of
autopilots, Edge Server, adapters, networking, or visualization.

## Autonomy boundary

Mission decision-making remains outside the simulator and runs locally on each autonomous node.
Initially, only the repetitive integration pattern should be standardized:

- Observe and query local Ditto state.
- React to locally visible changes.
- Publish observations, tasks, claims, intent, and outcomes.
- Convert decisions into ROS 2, MAVLink, or other vehicle commands.
- Record decisions and execution results for replay.

Actual search, allocation, planning, and replanning policies may be mission-specific. Reusable
autonomy abstractions should emerge from working scenarios rather than being designed prematurely.

## Visualization goals

The raylib viewer will render full 3D position, attitude, trajectories, terrain, agents, targets,
tasks, routes, sensor footprints, and network links. It should eventually provide four focused
presentation modes:

1. Full operational picture.
2. A selected node's local knowledge, including divergence and later convergence.
3. A selected node's autonomy decisions and replanning.
4. Network behavior and controlled benchmark results.

ATAK and WebTAK remain real operator views. The custom viewer is the instrumented view used to
explain global truth, internal knowledge, networking, and autonomy behavior.

## Foundational rules

- Global simulation truth must never leak directly into local autonomy decisions.
- Every node acts only on information available through its local interfaces and local Ditto store.
- Network disruption must affect real traffic rather than merely changing a visual indicator.
- Comparisons must measure mission outcomes and equivalent correctness, not manufacture weak DDS
  or Zenoh baselines.
- The platform should scale down to a few nodes as naturally as it scales up to a large operation.

## Rough path forward

1. Prove the two-vehicle PX4/ROS 2/gRPC Edge Server slice with Process Compose.
2. Generalize node declarations and generate isolated multi-node runtime configurations.
4. Add the world service and connect it to the existing raylib 3D viewer.
5. Add position-dependent network topology and controlled real-traffic impairment.
6. Implement the first real local autonomy executive and collaborative search scenario.
7. Add knowledge-state inspection and metrics if real scenarios need them.
8. Add Android/ATAK, heterogeneous vehicles, multiple bearers, and Big Peer.
9. Build deliberate DDS and Zenoh comparison experiments after Ditto behavior is established.

## MVP: PX4 vehicle fleet

The default executable slice has two PX4 SIH vehicles, one Edge Server per vehicle, a
non-autonomous operator Edge Server, and ROS 2 adapters connected through gRPC over Unix sockets.
The viewer reads PX4's self-published pose and the simulator-owned link metrics, so its display
remains independent of operator knowledge and does not poll vehicle Edge Server APIs.

Start a complete, managed session with one command:

```bash
pixi run sim
```

It builds missing local artifacts, starts the fleet, and opens the viewer. Closing the viewer
always stops every Process Compose node. In another terminal, submit commands with
`./scripts/command.sh ...`. `pixi run sim-status` shows a running fleet; `pixi run sim-down` is
an idempotent emergency stop.

The viewer reads PX4 SIH's dedicated MAVLink display stream directly; it does not use Edge Server,
adapter, or replicated state, so it remains valid during Edge-path fault injection.

The operator command client writes the shared `fleet_commands/fleet-current` intent document
through the operator Edge Server. Each vehicle executes only its own entry and writes an
independent `fleet_command_receipts` document; all three fleet collections (`vehicle_state`,
`fleet_commands`, and receipts) are subscribed mesh-wide. This exercises the Ditto path rather
than talking to PX4 directly:

```bash
./scripts/build-command.sh
./scripts/command.sh goto px4_0 15 -10 5
./scripts/command.sh orbit px4_1 0 0 5 10 3
./scripts/command.sh status px4_0
```

Credentials default to the ignored `.env` at this repository's root. Set
`DITTO_EDGE_ENV_FILE` to use a different compatible file. Runtime state and rendered
configurations stay under ignored `build/`.

Changing credentials does not require a code rebuild: the managed launcher re-renders its Edge
Server configuration on every start. If `DITTO_DB_ID` changes, remove the generated scenario
runtime state under `build/runtime/mvp-two-px4/` before the next run.

Closing a managed simulation stops every process and clears its local Edge persistence. The
viewer ignores cloud-replicated state from earlier runs until each PX4 publishes fresh telemetry.

### Four-node ROS 2 + MAVLink telemetry test

The mixed scenario runs ROS 2 on `px4_0` and `px4_1`, and the native MAVLink telemetry adapter on
`px4_2` and `px4_3`:

```bash
SIM_SCENARIO_FILE="$PWD/scenarios/mvp-four-mixed.env" pixi run sim
```

The MAVLink nodes mirror telemetry into the shared vehicle-state schema and accept the same arm,
go-to, and orbit commands as their ROS 2 peers.

With that fleet running, launch four separated trajectories through the operator with:

```bash
./scripts/demo-four.sh
```

`px4_0` and `px4_2` fly to opposing waypoints; `px4_1` and `px4_3` fly opposing orbits.

### Twenty-node ROS 2 + MAVLink fleet

This scenario runs ROS 2 adapters on `px4_0` through `px4_9` and native MAVLink adapters on
`px4_10` through `px4_19`:

```bash
SIM_SCENARIO_FILE="$PWD/scenarios/mvp-twenty-mixed.env" pixi run sim
```

The launcher waits until all 20 vehicles have published through the operator before opening the
viewer. Run the demo only after the viewer appears.

In a second terminal, command all 20 nodes with:

```bash
./scripts/demo-twenty.sh
```

PX4 normally collapses MAVLink destinations for instances above nine onto one UDP port. The
launcher creates isolated runtime PX4 startup files for `px4_10` through `px4_19`, so each keeps
its own MAVLink stream and is visible to its corresponding adapter. Those streams are capped at
100 KB/s per vehicle so all ten native adapters keep up.

The viewer fixes its horizontal origin to the configured PX4 home position and uses a wider map
for fleets larger than four, so restarting the viewer after vehicles have moved does not displace
the fleet.
