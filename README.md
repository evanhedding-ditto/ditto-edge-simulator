# Ditto Edge Simulator

Eight PX4 drones at Park MGM, each with its own Ditto Edge Server, shown in a
Cesium viewer with camera feeds and ISR targets. Runs on an Apple silicon Mac.

## Setup

1. Install the tools (skip any you have):

   ```bash
   xcode-select --install
   /bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"
   curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
   curl -fsSL https://pixi.sh/install.sh | sh
   ```

   Open a new terminal afterwards.

2. Clone into a path without spaces, outside iCloud-synced folders:

   ```bash
   git clone git@github.com:evanhedding-ditto/ditto-edge-simulator.git
   cd ditto-edge-simulator
   git clone --branch evan/robotics-testing \
     git@github.com:getditto/Ditto-Edge-Server.git deps/Ditto-Edge-Server
   git clone --recursive --branch v1.18.0-rc1 \
     https://github.com/PX4/PX4-Autopilot.git deps/PX4-Autopilot
   ```

3. Build PX4 and install the simulator's tools:

   ```bash
   (cd deps/PX4-Autopilot && ./Tools/setup/macos.sh)
   (cd deps/PX4-Autopilot && source .venv/bin/activate && make px4_sitl_sih)
   pixi install
   ```

4. Put the two supplied binaries in `bin/` and make them executable:

   ```bash
   chmod +x bin/ditto-edge-server bin/px4-mavlink-ditto-bridge
   ```

   If macOS blocks them, see [bin/README.md](bin/README.md).

5. Put the supplied `.env` in the repository root. It holds `DITTO_DB_ID`,
   `DITTO_AUTH_URL`, `DITTO_ACCESS_TOKEN`, `DITTO_WEBSOCKET_URL` and
   `CESIUM_ACCESS_TOKEN`.

## Run

```bash
pixi run sim park-mgm-8-px4
```

The first run builds the viewer and tools, which takes a few minutes. Allow the
firewall prompts. Closing the viewer stops everything; `pixi run sim-down` does
too.

UAS Tool connects to this Mac's IP on TCP `5760`–`5767` (`px4_0`–`px4_7`).
Camera feeds are at `rtsp://<this Mac's IP>:8554/px4_<i>`.

Trouble starting? See [DEBUGGING.md](DEBUGGING.md).
