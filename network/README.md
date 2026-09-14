# Network

Every configured Ditto TCP connection traverses the simulator-owned relay. Ditto owns peer
selection, synchronization, subscriptions, reconciliation, and encrypted payloads; the relay is
only an opaque byte-stream path that enforces the scenario's symmetric per-direction capacity and
publishes per-link connection, TX, RX, and utilization measurements to `network-metrics.json`.
It is intentionally outside PX4, adapters, and Edge Server, so observing a run does not create
replicated health traffic or a custom replication protocol.

`SIM_MESH_PEERS_PER_VEHICLE` creates a directed static-TCP ring/chord graph: every vehicle lists
that many succeeding vehicles as known Ditto TCP peers. `SIM_OPERATOR_MESH_PEERS` attaches the
operator at evenly spaced points. These are standard Edge Server `known_tcp_servers` settings;
the relay has one opaque listener per configured edge and never creates a peer connection itself.
The twenty-node scenario defaults to three vehicle peers and two operator attachments.

`SIM_NETWORK_LINK_CAPACITY_KBPS` declares the per-direction link budget. The viewer aggregates a
vehicle's attached links for display and reports the busiest link's utilization; it is not an
Edge Server health metric. The current relay does not yet model propagation delay, jitter, loss,
or range; those controls belong here, in front of the real traffic path.
