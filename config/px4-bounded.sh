#!/bin/sh
# Bounded PX4 daemon calls. Sourced by rcS in place of px4-alias.sh.
#
# Every px4-<cmd> in rcS is a synchronous round-trip to this instance's daemon
# socket with no timeout on either side, so one hung call wedges the vehicle
# forever. Each call is bounded here; a call that hangs is killed and retried,
# and one that hangs three times aborts rcS so the vehicle is reported failed
# rather than left silently misconfigured (rcS runs with set -e disabled).

. px4-alias.sh

PX4_CALL_TIMEOUT_S=${PX4_CALL_TIMEOUT_S:-30}
PX4_CALL_ATTEMPTS=${PX4_CALL_ATTEMPTS:-3}
PX4_CALL_NUDGE_MS=${PX4_CALL_NUDGE_MS:-2000}

_px4_call() {
  _attempt=1
  while :; do
    "$@" &
    _pid=$!
    _hung=0
    _waited_ms=0
    _nap_ms=5
    _nudge_pid=
    # A normal call returns in ~5 ms, so poll with exponential backoff: a fast
    # call costs one short nap, a hung one settles at 500 ms polls.
    while kill -0 "$_pid" 2>/dev/null; do
      # The daemon's poll() occasionally misses the wakeup for a pending
      # connection, leaving the client in read() with nobody accepting. Any new
      # connection wakes it, and the pending one is served first, so open one.
      if [ -z "$_nudge_pid" ] && [ "$_waited_ms" -ge "$PX4_CALL_NUDGE_MS" ]; then
        echo "px4-bounded: '$*' silent ${PX4_CALL_NUDGE_MS}ms; nudging daemon" >&2
        px4-param --instance "$px4_instance" show SYS_AUTOSTART >/dev/null 2>&1 &
        _nudge_pid=$!
      fi
      if [ "$_waited_ms" -ge $((PX4_CALL_TIMEOUT_S * 1000)) ]; then
        _hung=1
        kill "$_pid" 2>/dev/null
        break
      fi
      sleep "$(printf '0.%03d' "$_nap_ms")"
      _waited_ms=$((_waited_ms + _nap_ms))
      [ "$_nap_ms" -ge 500 ] || _nap_ms=$((_nap_ms * 2))
    done
    wait "$_pid" 2>/dev/null
    _rc=$?
    if [ -n "$_nudge_pid" ]; then kill "$_nudge_pid" 2>/dev/null; wait "$_nudge_pid" 2>/dev/null; fi
    # A real result, including a legitimate non-zero (param compare/greater
    # return 1 to mean "false" inside rcS conditionals): pass it straight through.
    [ "$_hung" -eq 1 ] || return "$_rc"
    echo "px4-bounded: '$*' hung ${PX4_CALL_TIMEOUT_S}s; retrying ($_attempt/$PX4_CALL_ATTEMPTS)" >&2
    if [ "$_attempt" -ge "$PX4_CALL_ATTEMPTS" ]; then
      echo "px4-bounded: '$*' hung $PX4_CALL_ATTEMPTS times; aborting startup" >&2
      exit 1
    fi
    _attempt=$((_attempt + 1))
  done
}

# Replace every px4-* alias with a bounded function. Aliases that are not
# daemon clients (e.g. set) are left alone.
# bash lists aliases as name='...' in POSIX mode and alias name='...' otherwise.
for _cmd in $(alias | sed -n "s/^alias //; s/^\([a-z_0-9]*\)='px4-.*/\1/p"); do
  unalias "$_cmd"
  eval "$_cmd() { _px4_call px4-$_cmd --instance \"\$px4_instance\" \"\$@\"; }"
done
