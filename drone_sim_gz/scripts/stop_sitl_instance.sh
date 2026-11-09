#!/bin/bash
# Stop ONE parallel SITL instance N>0 (run on the HOST): every process in starling-gz that carries GZ_PARTITION=starling_N
# (px4 -i N, its gz server/GUI, sim_clock_udp.py, truth_sampler.py, cam_grab.py, `gz topic -e` listeners of bench_ctl,
# the origin setter), plus its mavsdk_server (port 50051+N) and missions in starling-flight. Instance 0: stop_sitl.sh.
# Why everything: a leftover subscriber of /world/<w>/clock (sim_clock_udp.py) makes PX4's start script believe the world
# is still running ("gazebo already running world"), so it skips starting gz and the spawn times out (2026-10-28 18:15).
N=${1:?instance}
kill_part(){ docker exec -i starling-gz python3 - "$N" "$1" <<'PY'
import os, signal, sys
tag = ("GZ_PARTITION=starling_%s" % sys.argv[1]).encode(); sig = signal.SIGKILL if sys.argv[2] == "KILL" else signal.SIGTERM
left = 0
for pid in os.listdir("/proc"):
    if not pid.isdigit() or int(pid) == os.getpid():
        continue
    try:
        env = open("/proc/%s/environ" % pid, "rb").read().split(b"\0")
    except OSError:
        continue
    if tag in env:
        left += 1
        if sys.argv[2] in ("TERM", "KILL"):
            try:
                os.kill(int(pid), sig)
            except OSError:
                pass
print(left)
PY
}
# px4 -i N itself is started without the partition variable only if launched by hand; catch it by its argument too
docker exec starling-gz bash -c "for p in \$(pgrep -x px4); do tr '\0' ' ' < /proc/\$p/cmdline | grep -q -- '-i $N ' && kill \$p; done; true" >/dev/null 2>&1
kill_part TERM >/dev/null
docker exec starling-flight bash -c "pkill -f '[m]avsdk_server -p $((50051 + N)) '; for p in \$(pgrep -f '[g]rid_mission.py|[p]ath_flight_phase1'); do tr '\0' ' ' < /proc/\$p/cmdline | grep -q -- '--port $((50051 + N))' && kill \$p; done; true" >/dev/null 2>&1
for i in $(seq 1 8); do [ "$(kill_part NONE_JUST_COUNT 2>/dev/null | tail -1)" = "0" ] && break; sleep 1; done
kill_part KILL >/dev/null
docker exec starling-gz bash -c "for p in \$(pgrep -x px4); do tr '\0' ' ' < /proc/\$p/cmdline | grep -q -- '-i $N ' && kill -9 \$p; done; true" >/dev/null 2>&1
sleep 1
echo "instance $N stopped: processes left in partition starling_$N: $(kill_part COUNT_ONLY 2>/dev/null | tail -1)"
