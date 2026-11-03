#!/bin/bash
# Stop the MAIN SITL (instance 0) on the host:  stop_sitl.sh        -> instance 0 only (px4 without "-i N", gz/helpers
#                                                                      without GZ_PARTITION, mavsdk_server 50051, its missions)
#                                                 stop_sitl.sh all    -> everything in both containers (old behaviour)
# Parallel instances N>0 (the workbench, bench.sh, and INSTANCE=N runs) live in GZ_PARTITION starling_N and are left alone
# (2026-10-28: grid_fly.sh/fly_gui.sh call this at start; the old version killed every px4/gz in the container).
rm -f "$HOME/starling_sim/logs/.gui_transparent"   # GUI transparency state (grid_fly.sh) dies with the GUI
if [ "${1:-}" = "all" ]; then
  docker exec starling-gz bash -c "pkill -x px4; pkill -x ruby; pkill -x gz; pkill -x tail; pkill -f '[t]ruth_sampler'; pkill -f '[c]am_follow'; pkill -f '[c]am_grab'; pkill -f '[s]im_clock_udp'; true" >/dev/null 2>&1
  docker exec starling-flight bash -c "pkill -x mavsdk_server; pkill -f '[p]ath_flight_phase1'; pkill -f '[g]rid_mission'; true" >/dev/null 2>&1
else
  docker exec -i starling-gz python3 - <<'PY' >/dev/null 2>&1
import os, signal
NAMES = ("px4", "ruby", "gz", "tail", "python3", "bash")
me = os.getpid()
for pid in os.listdir("/proc"):
    if not pid.isdigit() or int(pid) == me:
        continue
    try:
        cmd = open("/proc/%s/cmdline" % pid, "rb").read().split(b"\0")
        env = open("/proc/%s/environ" % pid, "rb").read().split(b"\0")
    except OSError:
        continue
    exe = os.path.basename(cmd[0].decode(errors="ignore")) if cmd and cmd[0] else ""
    line = b" ".join(cmd).decode(errors="ignore")
    if any(e.startswith(b"GZ_PARTITION=starling_") for e in env):
        continue                                    # belongs to a parallel instance / the workbench
    if line.startswith("gz sim") or line.startswith("gz "):
        exe = "gz"                                  # the gz ruby process rewrites argv into ONE string: basename lies
    sim = (exe in ("px4", "ruby", "gz") or (exe == "tail" and "/dev/null" in line)
           or any(k in line for k in ("truth_sampler", "cam_follow", "cam_grab", "sim_clock_udp", "run_sitl.sh", "make px4_sitl")))
    if sim and exe in NAMES or (sim and exe.startswith("px4")):
        try:
            os.kill(int(pid), signal.SIGTERM)
        except OSError:
            pass
PY
  docker exec starling-flight bash -c "pkill -f '[m]avsdk_server -p 50051 '; for p in \$(pgrep -f '[g]rid_mission.py|[p]ath_flight_phase1'); do tr '\0' ' ' < /proc/\$p/cmdline | grep -q -- '--port 5005[2-9]' || kill \$p; done; true" >/dev/null 2>&1
fi
sleep 2
echo "px4: $(docker exec starling-gz pgrep -x px4 | wc -l)  gz: $(docker exec starling-gz pgrep -af 'gz sim' | grep -vc defunct)  mavsdk_server: $(docker exec starling-flight pgrep -x mavsdk_server | wc -l)   (인스턴스 0만 정리; 남은 것은 병렬 인스턴스/작업대. 전부 끄기: stop_sitl.sh all)"
