#!/bin/bash
# Stop ONE parallel SITL instance N>0 (run on the HOST): every process in starling-gz that carries GZ_PARTITION=starling_N
# (px4 -i N, its gz server/GUI, sim_clock_udp.py, truth_sampler.py, cam_grab.py, `gz topic -e` listeners of bench_ctl,
# the origin setter), plus its mavsdk_server (port 50051+N) and missions in starling-flight. Instance 0: stop_sitl.sh.
# Why everything: a leftover subscriber of /world/<w>/clock (sim_clock_udp.py) makes PX4's start script believe the world
# is still running ("gazebo already running world"), so it skips starting gz and the spawn times out (2026-10-28 18:15).
# 2026-10-31 (codex mission-process-cleanup reply-2): the result is now an exit code the caller must check -
#   0 nothing of instance N left | 1 processes left after TERM + KILL | 2 a process query failed (docker exec error / no answer),
#   so "nothing found" is never assumed | 4 instance outside 2..5 (0 / 1 are protected; LEGACY_OK=1 allows 1).
# Before anything is signalled every target is logged (pid, start time, user, command line) to logs/stop_sitl_<N>.log.
# STOP_PROC (tests only, 2026-11-01 codex mission-process-cleanup reply-8): the process table both queries read, default /proc;
# tests/test_stop_instance.sh points it at a made-up table to drive the internal ENOENT / EACCES / pgrep paths.
PROC=${STOP_PROC:-/proc}
N=${1:?instance}
case "$N" in 2|3|4|5) ;; 1) [ "${LEGACY_OK:-0}" = 1 ] || { echo "!! stop_sitl_instance: instance 1 is protected (LEGACY_OK=1 to override)"; exit 4; };;
  *) echo "!! stop_sitl_instance: instance $N not in 2..5"; exit 4;; esac
LOG=${STOP_LOG:-$(cd "$(dirname "$0")/.." && pwd)/logs/stop_sitl_$N.log}
# gz side: every process with GZ_PARTITION=starling_N, plus px4 started with "-i N"; the last output line is "OK <count>" -
# a missing OK line means the query itself failed
part(){ docker exec -i starling-gz python3 - "$N" "$1" "$PROC" <<'PY'
import errno, os, signal, subprocess, sys
n, mode, proc = sys.argv[1], sys.argv[2], sys.argv[3]; tag = ("GZ_PARTITION=starling_%s" % n).encode()
sig = {"TERM": signal.SIGTERM, "KILL": signal.SIGKILL}.get(mode); left = []; errs = []
for pid in os.listdir(proc):
    if not pid.isdigit() or int(pid) == os.getpid():
        continue
    try:
        # a zombie (exited, not reaped: starling-gz's PID 1 is `sleep infinity`, which reaps nothing) has an empty cmdline and a
        # root-owned environ; it cannot be signalled and is not a leftover -> skipped, never counted as a query error
        if open("%s/%s/stat" % (proc, pid)).read().rsplit(")", 1)[1].split()[0] == "Z":
            continue
        env = open("%s/%s/environ" % (proc, pid), "rb").read().split(b"\0")
        cmd = open("%s/%s/cmdline" % (proc, pid), "rb").read().replace(b"\0", b" ").decode("utf-8", "replace").strip()
    except OSError as e:
        if e.errno not in (errno.ENOENT, errno.ESRCH):    # gone while we looked = fine; anything else = we could not look
            errs.append("%s: %s" % (pid, e.strerror))
        continue
    a = cmd.split()
    if tag in env or (a and os.path.basename(a[0]) == "px4" and "-i" in a and a[a.index("-i") + 1:a.index("-i") + 2] == [n]):
        left.append(pid)
        if mode == "LIST":
            st = subprocess.run(["ps", "-o", "user=,lstart=", "-p", pid], stdout=subprocess.PIPE, universal_newlines=True).stdout.strip()
            print("gz  pid %s  %s  %s" % (pid, st, cmd[:160]))
        if sig is not None:
            try:
                os.kill(int(pid), sig)
            except OSError:
                pass
if errs:                                           # 2026-11-01 codex reply-4: an unreadable process is not "none left"
    print("QUERY_ERROR %d: %s" % (len(errs), "; ".join(errs[:5])))
else:
    print("OK %d" % len(left))
PY
}
# flight side: wk_mission / grid_mission / path_flight / mavsdk_server whose "--port|-p 5005N" matches exactly; same OK line
flight(){ docker exec starling-flight bash -c "PROC=$PROC; P=\$(pgrep -f '[w]k_mission.py|[g]rid_mission.py|[p]ath_flight_phase1|[m]avsdk_server'); r=\$?
  [ \$r -gt 1 ] && { echo QUERY_ERROR pgrep rc \$r; exit 0; }      # rc 1 = no match (fine), >1 = pgrep failed
  for p in \$P; do
    # gone while we looked = fine; still there but unreadable = we could not look (2026-11-01 codex reply-6)
    c=\$(tr '\\0' ' ' < \$PROC/\$p/cmdline 2>/dev/null) || { [ -d \$PROC/\$p ] && { echo QUERY_ERROR cmdline of \$p unreadable; exit 0; }; continue; }
    echo \"\$c\" | grep -qE -- '(--port|-p) $((50051 + N))( |\$)' || continue
    case $1 in LIST) echo \"flt pid \$p  \$(ps -o user=,lstart= -p \$p)  \${c:0:160}\";; TERM) kill \$p 2>/dev/null; echo \$p;; KILL) kill -9 \$p 2>/dev/null; echo \$p;; *) echo \$p;; esac
  done; echo OK"; }
count(){ # part|flight -> number, or "ERR" when the query did not answer
  local o; o=$($1 COUNT 2>/dev/null) || { echo ERR; return; }
  case "$(echo "$o" | tail -1)" in "OK "*) echo "$o" | tail -1 | cut -d' ' -f2;; OK) echo "$o" | grep -vc '^OK$';; *) echo ERR;; esac; }
{ echo "== $(date '+%F %T') stop instance $N (caller pid $PPID)"; part LIST 2>&1; flight LIST 2>&1; } >> "$LOG"
flight TERM >/dev/null 2>&1; part TERM >/dev/null 2>&1
for i in $(seq 1 8); do [ "$(count part)" = 0 ] && [ "$(count flight)" = 0 ] && break; sleep 1; done
if [ "$(count part)" != 0 ] || [ "$(count flight)" != 0 ]; then
  { echo "-- still there after TERM, KILL:"; part LIST 2>&1; flight LIST 2>&1; } >> "$LOG"
  flight KILL >/dev/null 2>&1; part KILL >/dev/null 2>&1; sleep 1
fi
G=$(count part); F=$(count flight)
echo "-- left: gz $G, flight $F" >> "$LOG"
echo "instance $N stop: processes left in partition starling_$N: $G, flight-side: $F (log $LOG)"
[ "$G" = ERR ] || [ "$F" = ERR ] && { echo "!! stop_sitl_instance: process query failed - instance $N state unknown"; exit 2; }
[ "$G" = 0 ] && [ "$F" = 0 ] || { echo "!! stop_sitl_instance: instance $N not fully stopped"; exit 1; }
exit 0
