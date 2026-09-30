# Ditto Edge Simulator

Run eight PX4 SIH vehicles at Park MGM on an Apple silicon Mac. Each vehicle has
its own MAVLink adapter and Ditto Edge Server; commands and receipts travel
through Ditto. The Cesium viewer shows the vehicles, camera feeds, and ISR
targets. The `park-mgm-8-px4` scenario uses **no ROS 2, `px4_msgs`, or XRCE agent**.

## Set up `park-mgm-8-px4`

You need macOS on Apple silicon, Xcode Command Line Tools, [Pixi](https://pixi.sh),
Rust/Cargo, access to the private `getditto/Ditto-Edge-Server` repository, the
two supplied binaries below, Ditto credentials, and a Cesium ion access token.
PX4's macOS setup uses Homebrew for its own build tools; the simulator's
dependencies come from Pixi. Install Xcode Command Line Tools with
`xcode-select --install` if needed.

1. Clone the simulator, the Edge Server branch that contains the adapter source,
   and the PX4 revision used for validation:

   ```bash
   git clone git@github.com:evanhedding-ditto/ditto-edge-simulator.git
   cd ditto-edge-simulator
   mkdir -p deps
   git clone --branch evan/robotics-testing \
     git@github.com:getditto/Ditto-Edge-Server.git deps/Ditto-Edge-Server
   git clone --recursive --branch v1.18.0-rc1 \
     https://github.com/PX4/PX4-Autopilot.git deps/PX4-Autopilot
   ```

2. Build PX4 SIH, then install the simulator's pinned tools. On a new Mac,
   [PX4's macOS setup](https://docs.px4.io/main/en/dev_setup/dev_env_mac.html)
   supplies its build prerequisites. Gazebo's optional `--sim-tools` setup is
   unnecessary for SIH.

   ```bash
   (cd deps/PX4-Autopilot && ./Tools/setup/macos.sh)
   source deps/PX4-Autopilot/.venv/bin/activate
   (cd deps/PX4-Autopilot && make px4_sitl_sih)
   deactivate
   pixi install
   ```

3. Copy the supplied Apple silicon binaries into `bin/` under these exact names:

   ```text
   bin/ditto-edge-server
   bin/px4-mavlink-ditto-bridge
   ```

   Make both executable:

   ```bash
   chmod +x bin/ditto-edge-server bin/px4-mavlink-ditto-bridge
   ```

   If downloaded in a browser and blocked by Gatekeeper, see
   [bin/README.md](bin/README.md) for the quarantine step.

4. Create a `.env` file in the simulator root with the shared credentials:

   ```dotenv
   DITTO_DB_ID="..."
   DITTO_AUTH_URL="..."
   DITTO_ACCESS_TOKEN="..."
   DITTO_WEBSOCKET_URL="..."
   CESIUM_ACCESS_TOKEN="..."
   ```

   `.env`, `bin/`, `deps/`, and `build/` are ignored by Git. The WebSocket URL
   is required because this scenario enables Ditto Cloud.

5. Start the fleet:

   ```bash
   pixi run sim park-mgm-8-px4
   ```

   The first run builds the viewer, network observer, and simulator tools from
   source; the Cesium viewer also fetches its pinned dependencies. The supplied
   binaries skip the Edge Server and MAVLink adapter builds, but the adapter
   checkout remains necessary for the viewer's C++ clients and the observer.

## Use and stop

Connect UAS Tool to this Mac's IP at TCP ports `5760` through `5767` for
`px4_0` through `px4_7`. Each camera is at
`rtsp://<this Mac's IP>:8554/px4_<i>`. Allow the macOS firewall prompts for
the GCS relay and viewer. Closing the viewer shuts down the fleet; you can also
use `pixi run sim-status` and `pixi run sim-down`.

The [scenario file](scenarios/park-mgm-8-px4.env) holds fleet settings.
[DEBUGGING.md](DEBUGGING.md) covers startup failures, and
[PROJECT_STATUS.md](PROJECT_STATUS.md) records the broader simulator status.
