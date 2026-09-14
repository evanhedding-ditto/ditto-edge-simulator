# World

The MVP has no synthetic world. The viewer reads each vehicle's locally published PX4 pose, which
is sufficient to prove the live control and replication slice. A later world service owns terrain,
events, and global truth without exposing them to node autonomy.

