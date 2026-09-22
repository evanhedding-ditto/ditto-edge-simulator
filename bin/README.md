# Drop prebuilt binaries here

Anything placed here is used ahead of a local build, and is never rebuilt over.
That makes this the way to run the simulator without building the component
yourself, and also the way to pin a known-good binary while you work on
something else.

| File | Skips |
| --- | --- |
| `ditto-edge-server` | the Edge Server cargo build |
| `px4-mavlink-ditto-bridge` | the MAVLink adapter build |
| `px4_ditto_bridge_node` | the ROS 2 adapter build |

**What a drop-in does not remove.** The adapters checkout is still required --
its C++ SDKs compile into the viewer and command client -- and so is a Rust
toolchain, because the network observer is cargo-built on every run. There is no
drop-in for the observer; `SIM_OBSERVER_BIN` points at one if you need to.

Each has an environment override that wins over everything:
`DITTO_EDGE_SERVER_BIN`, `DITTO_MAVLINK_ADAPTER_BIN`, `DITTO_ROS_ADAPTER_BIN`.

## After downloading

```bash
chmod +x bin/<file>
xattr -d com.apple.quarantine bin/<file>   # only if a browser fetched it
```

macOS quarantines anything downloaded through a browser and Gatekeeper blocks it
on first run. `curl` and `gh release download` do not set that flag.

## What travels, and what does not

**`ditto-edge-server` is fully portable.** It links only macOS system
frameworks, so it runs on any Apple silicon Mac with nothing installed.

**The adapters need the pixi environment**, because they link gRPC and Protobuf
from it. For the MAVLink adapter `process/mavlink-adapter.sh` sets
`DYLD_LIBRARY_PATH` for exactly this reason: the library path baked into a
binary points at the pixi prefix on the machine that built it, which will not
exist on yours. `pixi install` is enough. The ROS adapter instead inherits its
library paths from the underlay it sources, so a ROS binary built elsewhere is
only as portable as that environment.

**A dropped ROS adapter saves less than it looks.** It still needs the `ros`
pixi environment and the `px4_msgs` underlay, and once those exist the adapter
itself builds in about fifteen seconds. Supported, but rarely worth it.

Everything here except this file is gitignored.
