#!/usr/bin/env python3
"""Workbench controller: run / pause / reset / repeat grid flights from inside the Gazebo window (run on the HOST).
Started by bench.sh. Works on one parallel SITL instance (default N=1: GZ_PARTITION starling_1, PX4 -i 1, mavsdk 50052,
sim clock udp 14601), so it never touches the main SITL used by grid_fly.sh.

Input, from the Gazebo GUI of the workbench:
  topic /echo (the "Bench command" panel: Message  data: "run 2"  then the Publish switch; the panel always uses /echo)
  topic /bench/cmd (gz.msgs.StringMsg) - from a shell:
      docker exec -e GZ_PARTITION=starling_1 starling-gz gz topic -t /bench/cmd -m gz.msgs.StringMsg -p 'data: "run 2"'
  topic /keyboard/keypress (KeyPublisher, keys pressed while the 3D view has focus)
Commands:
  run [k | <route> | survey <spec>]   reset to the route's start cell, then fly it (k = preset number, default = selected)
  select k                            choose preset k without flying
  repeat n                            fly the selected route n times (reset before each run)
  loop on|off                         keep repeating until "loop off" / "stop"
  reset                               stop the flight, force-disarm, teleport to the start cell of the selected route
  stop                                stop the flight (vehicle lands where it is), leave it there
  pause | resume                      pause / resume the whole simulation (same as the play/pause button)
  list                                print the presets to the status panel
Hotkeys (3D view focused): 1-9 run preset, R reset, X stop, L loop on/off, P pause/resume.
Status goes to topic /bench/status (StringMsg) - shown by the "Status" panel - and to logs/bench_ctl.log.
The mission runs on the Gazebo SIM clock (grid_mission.py --sim-clock), so pausing the simulation pauses the mission too.
Each run is judged with ground truth (grid_verify.py) like grid_fly.sh; logs: logs/bench_<ts>_{mission,truth,plan,report}.
"""
import json, os, re, shlex, subprocess, sys, threading, time

N = int(os.environ.get("BENCH_INSTANCE", "1"))
SIM = os.path.expanduser("~/starling_sim"); SC = SIM + "/repo_export/scripts"; LOGS = SIM + "/logs"
PART = "starling_%d" % N; MODEL = "starling2_%d" % N; MAVSDK_PORT = 50051 + N; CLOCK_PORT = 14600 + N
WORLD = "qvix"
PRESETS = [
    ("A통로 동쪽 면", "survey R1A E"), ("B통로 서쪽 면", "survey R1B W"), ("B통로 동쪽 면", "survey R1B E"),
    ("C통로 서쪽 면", "survey R1C W"), ("C통로 동쪽 면", "survey R1C E"), ("D통로 서쪽 면", "survey R1D W"),
    ("D통로 동쪽 면", "survey R1D E"), ("E통로 서쪽 면", "survey R1E W"),
    ("C·D 통로 왕복 (첫 격자 경로)", "R1C10:S R1C1 R1C10:N R1D10:E R1D1:S R1D10:N R1C10:W R1C10:S"),
]
KEYS = {ord(str(i)): ("run", str(i)) for i in range(1, 10)}
KEYS.update({ord("R"): ("reset", ""), ord("X"): ("stop", ""), ord("L"): ("loop", "toggle"), ord("P"): ("pausetoggle", "")})
LOG = open(os.path.join(LOGS, "bench_ctl.log"), "a", buffering=1)
state = {"sel": 0, "proc": None, "loop": False, "repeat_left": 0, "paused": False, "busy": False, "last": "", "run_id": 0}
lock = threading.Lock()


def log(msg):
    line = "[%s] %s" % (time.strftime("%H:%M:%S"), msg)
    print(line, flush=True); LOG.write(line + "\n")


def gz(cmd, timeout=15):
    return subprocess.run(["docker", "exec", "-e", "GZ_PARTITION=" + PART, "starling-gz", "bash", "-c", cmd],
                          capture_output=True, text=True, timeout=timeout).stdout


def px4(args, timeout=15):
    return gz("cd ~/PX4-Autopilot/build/px4_sitl_default && ./bin/px4-%s --instance %d" % (args.split()[0], N)
              + " " + " ".join(args.split()[1:]), timeout)


def status(msg):
    log("STATUS " + msg)
    state["last"] = msg
    safe = msg.replace("\\", "/").replace('"', "'")
    try:
        q = "▶ " + safe.replace("'", "’")
        # /echo too: the GUI TopicEcho panel ignores a configured topic and starts on its default "/echo"
        gz("gz topic -t /bench/status -m gz.msgs.StringMsg -p 'data: \"%s\"'; gz topic -t /echo -m gz.msgs.StringMsg -p 'data: \"%s\"'" % (q, q), 10)
    except subprocess.TimeoutExpired:
        pass


def route_of(sel_or_text):
    """'3' -> preset 3, 'survey R1B W' -> survey, anything else -> literal route. Returns (label, route tokens)."""
    t = sel_or_text.strip()
    if t.isdigit():
        k = int(t) - 1
        if not 0 <= k < len(PRESETS):
            raise ValueError("preset %s does not exist (1-%d)" % (t, len(PRESETS)))
        state["sel"] = k
        label, t = PRESETS[k]
    else:
        label = t
    if t.startswith("survey "):
        out = subprocess.run(["python3", SC + "/grid_mission.py", "--survey", t[7:], "--check"], capture_output=True, text=True)
        m = re.search(r"^survey route: (.*)$", out.stdout, re.M)
        if not m:
            raise ValueError("survey rejected: %s" % (out.stdout + out.stderr).strip().splitlines()[-1])
        return label, m.group(1), True
    return label, t, False


def spawn_pose(route):
    out = subprocess.run(["python3", SC + "/grid_mission.py", "--spawn-pose", "--route", route], capture_output=True, text=True)
    x, y, z, _, _, yaw = [float(v) for v in out.stdout.strip().split(",")]
    return x, y, z, yaw


def stop_mission():
    p = state["proc"]
    subprocess.run(["docker", "exec", "starling-flight", "bash", "-c",
                    "for p in $(pgrep -f '[g]rid_mission.py'); do tr '\\0' ' ' < /proc/$p/cmdline | grep -q -- '--port %d' && kill $p; done; true" % MAVSDK_PORT])
    if p and p.poll() is None:
        try:
            p.wait(timeout=5)
        except subprocess.TimeoutExpired:
            p.kill()
    state["proc"] = None


def current_yaw():
    out = gz("timeout -s KILL 5 gz model -m %s -p 2>/dev/null" % MODEL)
    m = re.findall(r"\[([-\d.e]+) ([-\d.e]+) ([-\d.e]+)\]", out)
    return float(m[1][2]) if len(m) >= 2 else None


def teleport(route):
    """Move the vehicle to the route's start cell KEEPING its current heading: a heading jump (e.g. landed facing S,
    teleported facing N) leaves the EV-only EKF with "Yaw estimate error / heading estimate not stable" for > 90 s and
    arming is denied (2026-10-28 18:10), and the heading reset it causes is the touchdown-spin trigger of 2026-10-27.
    The flight script turns to N (its --yaw 0) during takeoff anyway."""
    x, y, z, yaw = spawn_pose(route)
    cy = current_yaw()
    if cy is not None:
        yaw = cy
    import math
    qz, qw = math.sin(yaw / 2), math.cos(yaw / 2)
    req = "name: \"%s\", position: {x: %.3f, y: %.3f, z: %.3f}, orientation: {x: 0, y: 0, z: %.6f, w: %.6f}" % (MODEL, x, y, z + 0.02, qz, qw)
    for _ in range(3):                                    # repeat: the first call may land on a still-moving body
        gz("gz service -s /world/%s/set_pose --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean --timeout 3000 --req '%s'" % (WORLD, req))
        time.sleep(1.0)
    return x, y


def ekf_xy():
    out = px4("listener vehicle_local_position -n 1")
    mx = re.search(r"\bx: (-?[\d.]+)", out); my = re.search(r"\by: (-?[\d.]+)", out)
    return (float(mx.group(1)), float(my.group(1))) if mx and my else None


def do_reset(route=None):
    if route is None:
        route = route_of(str(state["sel"] + 1))[1]
    status("리셋: 비행 중지, 시동 해제, 시작 셀로 이동 중…")
    stop_mission()
    if state["paused"]:
        set_pause(False)
    px4("commander disarm -f")
    time.sleep(1.0)
    wx, wy = teleport(route)
    # EV-only EKF follows the teleport (vision position resets); wait until the local position matches (NED N = world y)
    ok = False
    for _ in range(20):
        p = ekf_xy()
        if p and abs(p[0] - wy) < 0.15 and abs(p[1] - wx) < 0.15:
            ok = True; break
        time.sleep(0.5)
    px4("commander mode auto:loiter")
    # after a teleport (especially a heading change, e.g. landed facing S -> start cell facing N) the EKF reports
    # "Yaw estimate error / heading estimate not stable" for a while and arming is denied (2026-10-28 17:53).
    ready = False
    for _ in range(90):
        if "Preflight check: OK" in px4("commander check"):
            ready = True; break
        time.sleep(1.0)
    status("리셋 완료: %s 시작 셀 (EKF %s, 시동 %s)" % (route.split()[0], "일치" if ok else "불일치 - 확인 필요",
                                                    "가능" if ready else "불가 - commander check 확인"))
    return ok and ready


def set_pause(on):
    gz("gz service -s /world/%s/control --reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean --timeout 3000 --req 'pause: %s'"
       % (WORLD, "true" if on else "false"))
    state["paused"] = on
    status("일시정지" if on else "재개")


def run_once(label, route, survey):
    state["run_id"] += 1
    if not do_reset(route):
        status("리셋 후 EKF 위치 불일치로 비행 취소 (%s)" % label)
        return False
    ts = time.strftime("%Y%m%d_%H%M%S"); tag = "bench_%s" % ts
    plan, mlog, truth, rep = ["%s/%s_%s" % (LOGS, tag, k) for k in ("plan.json", "mission.log", "truth.txt", "report.txt")]
    extra = "--hover 0" if survey else ""
    chk = subprocess.run("python3 %s/grid_mission.py --check --route %s --plan-json %s %s" % (SC, shlex.quote(route), plan, extra),
                         shell=True, capture_output=True, text=True)
    if chk.returncode != 0:
        status("경로 거부: %s" % (chk.stdout + chk.stderr).strip().splitlines()[-1]); return False
    sampler = subprocess.Popen(["docker", "exec", "-e", "GZ_PARTITION=" + PART, "starling-gz", "python3", SC + "/truth_sampler.py",
                                MODEL, "1.0"], stdout=open(truth, "w"), stderr=subprocess.STDOUT)
    cmd = ("cd ~/workspace && python3 repo_export/scripts/grid_mission.py --route %s %s --sim-clock %d -- --port %d"
           % (shlex.quote(route), extra, CLOCK_PORT, MAVSDK_PORT))
    status("비행 중: %s" % label)
    with open(mlog, "w") as f:
        state["proc"] = subprocess.Popen(["docker", "exec", "-i", "starling-flight", "bash", "-c", cmd], stdin=subprocess.DEVNULL,
                                         stdout=f, stderr=subprocess.STDOUT)
        rc = state["proc"].wait()
    state["proc"] = None
    time.sleep(2)
    px4("commander mode auto:loiter")                    # after action.land() PX4 stays in Offboard -> next arming refused
    sampler.terminate()
    subprocess.run(["docker", "exec", "starling-gz", "pkill", "-f", "truth_sampler.py %s " % MODEL])
    out = subprocess.run(["python3", SC + "/grid_verify.py", plan, mlog, truth, "/dev/null"], capture_output=True, text=True).stdout
    open(rep, "w").write(out)
    verdict = re.search(r"VERDICT: (\w+)", out)
    status("완료: %s - %s (미션 종료 코드 %d, 리포트 %s)" % (label, verdict.group(1) if verdict else "판정 없음", rc, os.path.basename(rep)))
    return bool(verdict and verdict.group(1) == "PASS")


def worker(label, route, survey):
    try:
        while True:
            run_once(label, route, survey)
            with lock:
                if state["loop"]:
                    continue
                if state["repeat_left"] > 1:
                    state["repeat_left"] -= 1; continue
                state["repeat_left"] = 0
            break
    except Exception as e:                                    # keep the controller alive whatever happens in one run
        status("오류: %s" % e)
    finally:
        state["busy"] = False


def start_runs(arg, repeat=1):
    if state["busy"]:
        status("이미 비행 중 - 먼저 stop 또는 reset"); return
    try:
        label, route, survey = route_of(arg or str(state["sel"] + 1))
    except ValueError as e:
        status(str(e)); return
    state["busy"] = True; state["repeat_left"] = repeat
    threading.Thread(target=worker, args=(label, route, survey), daemon=True).start()


def handle(cmd, arg):
    log("CMD %s %s" % (cmd, arg))
    if cmd == "run":
        start_runs(arg)
    elif cmd == "select":
        try:
            route_of(arg); status("선택: %d. %s" % (state["sel"] + 1, PRESETS[state["sel"]][0]))
        except ValueError as e:
            status(str(e))
    elif cmd == "repeat":
        n = int(arg) if arg.isdigit() else 3
        start_runs("", n); status("반복 %d회: %s" % (n, PRESETS[state["sel"]][0]))
    elif cmd == "loop":
        state["loop"] = (not state["loop"]) if arg in ("", "toggle") else (arg == "on")
        status("반복 비행 %s" % ("켬" if state["loop"] else "끔"))
        if state["loop"] and not state["busy"]:
            start_runs("")
    elif cmd == "reset":
        state["loop"] = False; state["repeat_left"] = 0
        threading.Thread(target=lambda: do_reset(), daemon=True).start()
    elif cmd == "stop":
        state["loop"] = False; state["repeat_left"] = 0
        stop_mission(); px4("commander mode auto:land"); status("중지: 제자리 착륙")
    elif cmd == "pause":
        set_pause(True)
    elif cmd == "resume":
        set_pause(False)
    elif cmd == "pausetoggle":
        set_pause(not state["paused"])
    elif cmd == "list":
        status(" | ".join("%d %s" % (i + 1, p[0]) for i, p in enumerate(PRESETS)))
    else:
        status("알 수 없는 명령: %s (run/select/repeat/loop/reset/stop/pause/resume/list)" % cmd)


def listen(topic, parse):
    while True:
        p = subprocess.Popen(["docker", "exec", "-e", "GZ_PARTITION=" + PART, "starling-gz", "gz", "topic", "-e", "-t", topic],
                             stdout=subprocess.PIPE, text=True, bufsize=1)
        for line in p.stdout:
            r = parse(line.strip())
            if r:
                handle(*r)
        time.sleep(1)                                         # echo died (GUI/world restart): resubscribe


COMMANDS = ("run", "select", "repeat", "loop", "reset", "stop", "pause", "resume", "list")
_last = {"cmd": None, "t": 0.0}


def parse_cmd(line):
    """Commands come from /bench/cmd or /echo (the GUI Publisher panel ignores its configured topic and publishes on its
    default /echo, once per second while its switch is on: an identical command arriving < 2.5 s after the previous copy
    is the same press and is ignored). Status lines (prefix "▶") and the panel's default "Hello" are not commands."""
    m = re.match(r'data: "(.*)"$', line)
    if not m:
        return None
    text = m.group(1).strip()
    now = time.time()
    same = text == _last["cmd"] and now - _last["t"] < 2.5
    _last["cmd"], _last["t"] = text, now
    parts = text.split(None, 1)
    if same or not parts or parts[0].lower() not in COMMANDS:
        return None
    return parts[0].lower(), parts[1] if len(parts) > 1 else ""


def parse_key(line):
    m = re.match(r"data: (\d+)$", line)
    return KEYS.get(int(m.group(1))) if m else None


if __name__ == "__main__":
    log("bench_ctl start: instance %d, partition %s, mavsdk %d, clock %d" % (N, PART, MAVSDK_PORT, CLOCK_PORT))
    threading.Thread(target=listen, args=("/keyboard/keypress", parse_key), daemon=True).start()
    time.sleep(1)
    threading.Thread(target=listen, args=("/echo", parse_cmd), daemon=True).start()
    handle("list", "")
    listen("/bench/cmd", parse_cmd)
