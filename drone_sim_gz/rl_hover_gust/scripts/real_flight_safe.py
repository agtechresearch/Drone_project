#!/usr/bin/env python3
"""Run the UNCHANGED real flight script path_flight_phase1_v13.py with the safety layer of safety_patch.py (2026-11-01).
The flight script file is not edited, copied or rewritten: it is imported and its DroneController methods are replaced at
run time; its own arguments pass through unchanged.

  python3 real_flight_safe.py [--v13 <dir of path_flight_phase1_v13.py>] [--allow-kill] [--result FILE]
                              [--own-server CONN [--server-bin PATH] [--server-port P] [--allow-other-server]] [--] <v13 arguments>

  --v13 DIR       where path_flight_phase1_v13.py is (default: this file's directory, then ../..)
  --allow-kill    allow a SOFTWARE motor kill when no land command got through (default: never; the operator's RC kill
                  switch is the kill authority)
  --result FILE   where the final result JSON goes (default: mission_result.json in the current directory)
  --own-server CONN  (2026-11-01, user decision after the 1930fs SITL test: a dead flight script left mavsdk_server re-sending
                  the last setpoint, the vehicle stayed in Offboard 15 s and no failsafe came) start mavsdk_server here for the
                  MAVLink connection CONN (e.g. udp://:14550) and tie it to THIS process: when the launcher ends in any way,
                  kill -9 included, the kernel kills the server (PR_SET_PDEATHSIG SIGKILL), PX4 stops getting setpoints and
                  after COM_OF_LOSS_T runs its Offboard-loss action (COM_OBL_RC_ACT). v13 gets --server-address localhost
                  --port P (refused if they are given too, also abbreviated like --server-add / --por, with or without '=';
                  and v13's own parser must end on localhost:P - checked before the server starts, 2026-11-03). Refused while another mavsdk_server runs (it would keep sending
                  setpoints) unless --allow-other-server. At the normal end the server is stopped after the result is written.
  --server-bin PATH  mavsdk_server binary (default: the one bundled with MAVSDK-Python, then mavsdk_server on PATH)
  --server-port P    its gRPC port (default 50060)
Result (2026-11-01 codex real-mission-result-contract): v13's own "CUSTOM MISSION COMPLETE" / "Done!" lines are NOT the result.
At the end this launcher prints "[result] ..." lines, writes the JSON and exits with
  0  mission complete, touchdown confirmed, disarmed          3  safety stop / STOP / emergency, touchdown confirmed, disarmed
  2  a stage failed (the FIRST failure is kept), landed       4  stopped before arming (no arm RPC sent, no flight)
  1  error in the flight script (landed or nothing armed)     10 touchdown NOT confirmed, disarm not done or arm result unknown:
                                                                 OPERATOR HAND-OVER (wins over every other code)
  21 the result file could not be written (or prepared: then nothing flew); the flight result is in the log
STOP: from the first STOP / emergency landing the flight script's main flow has 15 s to reach its landing step and 70 s to
finish the stop; otherwise it is cancelled and the safety layer's STOP supervisor finishes (touchdown or hand-over, result).
Every landing (normal end, after a failed stage, STOP, emergency) is the same task of safety_patch.py: land command with a 3 s
limit, 3 fast tries, then a retry every 2 s until accepted or touchdown; touchdown = in_air False received after the land
request, within 40 s; disarm only after it. Position / attitude without a new message for 1 s after arming = land.
Deploy on the vehicle next to path_flight_phase1_v13.py: safety_patch.py, real_flight_safe.py (Python 3.6, MAVSDK 0.12).

What happens on each failure path (2026-11-01, codex offboard-fallback-contract reply-2). The right column is what is EXPECTED
from the vehicle parameters, NOT verified on the vehicle: voxl-px4 1.14.0-2.0.133, handoff params COM_OF_LOSS_T 1.0,
COM_OBL_RC_ACT 0 (Position), NAV_RCL_ACT 2 (= Return in upstream PX4 1.14; Land is 3), COM_RC_IN_MODE 3, COM_DISARM_LAND 0.1.
  path                                   this layer                                  then
  normal end / stage failure / STOP /    the one landing; touchdown wait; disarm     PX4 lands (land accepted)
  exception / timeout                    after touchdown; never kill
  land answered COMMAND_DENIED (FC)      3 fast tries, then a retry every 2 s        mavsdk_server keeps re-sending the LAST
  ("fc_denied")                          until accepted or touchdown                 setpoint: the vehicle may keep following
                                                                                     it (not necessarily a hover) -> OPERATOR
  land gets no RPC answer                same retries; the FC state is UNKNOWN (it   if the FC stops getting setpoints:
  ("rpc_timeout") / no FC ACK / no FC    does not prove a link loss)                 offboard loss after COM_OF_LOSS_T ->
                                                                                     COM_OBL_RC_ACT / RC-loss action (to check)
  VOXL pose pipe stops (MAVSDK alive)    land after 1 s (monitor)                    as the rows above
  touchdown not confirmed in 40 s        no disarm, exit 10, retries stop at the     OPERATOR; PX4 auto-disarm on landing
                                         program end (logged)                        (COM_DISARM_LAND)
  this process dies (kill -9)            without --own-server: nothing; a separate   SITL 1930fs t_pyexit: Offboard kept 15 s
                                         mavsdk_server keeps re-sending setpoints    (no failsafe). With --own-server the
                                                                                     server dies too -> Offboard loss ->
                                                                                     COM_OBL_RC_ACT (4 = Land, plans/
                                                                                     2026-11-01_rc_failsafe_params.md)
  the process hangs (alive, stuck)       not covered: the server is alive and keeps sending -> operator (RC mode switch)
Operator hand-over: whenever "land NOT sent", "TOUCHDOWN NOT CONFIRMED" or exit 10 appears the operator flies / lands by RC;
the RC kill switch is the only kill authority unless --allow-kill."""
import os, signal, subprocess, sys, time, traceback

HERE = os.path.dirname(os.path.abspath(__file__))


def server_bin():
    try:
        import mavsdk
        b = os.path.join(os.path.dirname(mavsdk.__file__), "bin", "mavsdk_server")
        if os.path.exists(b):
            return b
    except Exception:
        pass
    for d in os.environ.get("PATH", "").split(os.pathsep):
        if os.access(os.path.join(d, "mavsdk_server"), os.X_OK):
            return os.path.join(d, "mavsdk_server")
    return None


def other_servers(proc="/proc"):
    """pids of running mavsdk_server processes (by the command's file name)"""
    out = []
    for pid in os.listdir(proc):
        if pid.isdigit() and int(pid) != os.getpid():
            try:
                a = open(os.path.join(proc, pid, "cmdline"), "rb").read().split(b"\0")
            except OSError:
                continue
            if a and os.path.basename(a[0]) == b"mavsdk_server":
                out.append(int(pid))
    return out


ENDPOINT_OPTS = ("--server-address", "--port")


def endpoint_tokens(rest):
    """tokens that set v13's endpoint: the full names, any argparse abbreviation of them (allow_abbrev: a unique prefix
    such as --server-add / --por, also ambiguous prefixes, refused here anyway), with or without '=value'
    (2026-11-03 codex owned-server-endpoint-abbreviation). Tokens after a bare '--' are positional for argparse and ignored."""
    bad = []
    for a in rest:
        if a == "--":
            break
        if not a.startswith("--") or len(a) < 3:
            continue
        name = a.split("=", 1)[0]
        if any(o.startswith(name) for o in ENDPOINT_OPTS):
            bad.append(a)
    return bad


def check_final_endpoint(rest, sport, parser_factory=None):
    """parse exactly what v13 will parse (its own build_arg_parser) and require the endpoint to be the owned server.
    Returns an error text or None. parser_factory: for the offline test."""
    if parser_factory is None:
        import path_flight_phase1_v13 as fs
        parser_factory = fs.build_arg_parser
    argv = ["--server-address", "localhost", "--port", str(sport)] + [a for a in rest if a not in ("-h", "--help")]   # no usage print here
    try:
        ns, _ = parser_factory().parse_known_args(argv)
    except SystemExit as e:
        return "v13 would not accept these arguments (argparse exit %s)" % (e.code,)
    if getattr(ns, "server_address", None) != "localhost" or getattr(ns, "port", None) != sport:
        return "v13 would connect to %s:%s, not the owned server localhost:%d" % (getattr(ns, "server_address", "?"), getattr(ns, "port", "?"), sport)
    return None


def start_own_server(conn, binary, port):
    """mavsdk_server -p PORT CONN as a child that the kernel kills when this process ends (PR_SET_PDEATHSIG SIGKILL)"""
    import ctypes
    parent = os.getpid()
    libc = ctypes.CDLL(None, use_errno=True)

    def tie():
        if libc.prctl(1, int(signal.SIGKILL), 0, 0, 0) != 0:      # PR_SET_PDEATHSIG = 1
            os._exit(97)
        if os.getppid() != parent:                                 # the launcher died before prctl took effect
            os._exit(98)
    p = subprocess.Popen([binary, "-p", str(port), conn], preexec_fn=tie, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(1.0)
    if p.poll() is not None:
        raise RuntimeError("mavsdk_server ended at once (rc %s): %s -p %d %s" % (p.returncode, binary, port, conn))
    return p


def main(argv):
    v13 = None; allow_kill = False; result_path = "mission_result.json"; rest = []
    own = None; sbin = None; sport = 50060; allow_other = False
    i = 0
    while i < len(argv):
        if argv[i] == "--v13":
            v13 = argv[i + 1]; i += 2
        elif argv[i] == "--allow-kill":
            allow_kill = True; i += 1
        elif argv[i] == "--result":
            result_path = argv[i + 1]; i += 2
        elif argv[i] == "--own-server":
            own = argv[i + 1]; i += 2
        elif argv[i] == "--server-bin":
            sbin = argv[i + 1]; i += 2
        elif argv[i] == "--server-port":
            sport = int(argv[i + 1]); i += 2
        elif argv[i] == "--allow-other-server":
            allow_other = True; i += 1
        elif argv[i] == "--":
            rest += argv[i + 1:]; break
        else:
            rest.append(argv[i]); i += 1
    for d in ([v13] if v13 else [HERE, os.path.join(HERE, "..", "..")]):
        if os.path.exists(os.path.join(d, "path_flight_phase1_v13.py")):
            sys.path.insert(0, os.path.abspath(d)); break
    else:
        sys.exit("real_flight_safe: path_flight_phase1_v13.py not found (use --v13 <dir>)")
    sys.path.insert(0, HERE)
    server = None
    if own:
        bad = endpoint_tokens(rest)
        if bad:
            sys.exit("real_flight_safe: --own-server sets v13's --server-address/--port itself - do not pass them or an abbreviation "
                     "(%s; nothing started)" % " ".join(bad))
        err = check_final_endpoint(rest, sport)
        if err:
            sys.exit("real_flight_safe: %s (nothing started)" % err)
        others = other_servers()
        if others and not allow_other:
            sys.exit("real_flight_safe: another mavsdk_server runs (pid %s): it would keep sending setpoints after this launcher "
                     "ends - stop it or pass --allow-other-server (nothing started)" % " ".join(map(str, others)))
        sbin = sbin or server_bin()
        if not sbin:
            sys.exit("real_flight_safe: no mavsdk_server binary found (use --server-bin) - nothing started")
        try:
            server = start_own_server(own, sbin, sport)
        except Exception as e:  # noqa: BLE001
            sys.exit("real_flight_safe: %s - nothing started" % e)
        print("[safety] own mavsdk_server pid %d: %s -p %d %s (ends with this launcher)%s" % (
            server.pid, sbin, sport, own, "; other servers %s" % others if others else ""), flush=True)
        rest = ["--server-address", "localhost", "--port", str(sport)] + rest
    import safety_patch as SP
    import path_flight_phase1_v13 as fs
    # kill authority from the command line ONLY (2026-11-01 codex real-launcher-kill-authority): the WK_ALLOW_KILL environment
    # variable of the SITL tools is ignored here, so a leftover setting cannot widen the real-vehicle policy
    SP.ALLOW_KILL = allow_kill
    if os.environ.get("WK_ALLOW_KILL"):
        print("[safety] WK_ALLOW_KILL=%s in the environment is IGNORED by real_flight_safe.py (use --allow-kill)"
              % os.environ["WK_ALLOW_KILL"], flush=True)
    st = SP.SafetyState()
    log = SP.install(fs, st, monitor=True)
    log("[safety] real_flight_safe: %s, safety layer installed (software kill %s); v13 arguments: %s" % (
        fs.__file__, "ALLOWED" if SP.ALLOW_KILL else "never", " ".join(rest) or "(none)"))
    sys.argv = [fs.__file__] + rest
    try:
        code = run(fs, SP, st, result_path)
    finally:
        if server is not None and (not st.sup_owner or st.flow_task is None or st.flow_task.done()):
            stop_server(server, log)
    if st.sup_owner and st.flow_task is not None and not st.flow_task.done():
        # the supervisor finished the stop and stopped the event loop: a flight-script task that ignored its cancel must not
        # keep the process alive
        try:
            sys.stdout.flush(); sys.stderr.flush()
        finally:
            os._exit(code)
    return code


def stop_server(p, log):
    """normal end: after the result is out, stop the own server (TERM, then KILL); the os._exit path ends it by PDEATHSIG"""
    if p.poll() is None:
        p.terminate()
        try:
            p.wait(3)
        except subprocess.TimeoutExpired:
            p.kill(); p.wait(3)
    log("[safety] own mavsdk_server pid %d stopped (rc %s)" % (p.pid, p.returncode))


def run(fs, SP, st, result_path, entry=None):
    """run the flight script's main (or entry()), then end the layer's tasks and report the result -> exit code.
    The result file is prepared BEFORE anything flies (an earlier file is moved aside; if that fails nothing flies, exit 21).
    When the STOP supervisor finished the stop it may have stopped the event loop (a main flow that ignored its cancel):
    the result it made is used and the process ends with os._exit, so a stuck flight-script task can not keep it alive."""
    err = SP.prepare_result(st, result_path)
    if err:
        fs.log("[result] !!! %s - NOT starting (no flight) !!!" % err)
        fs.log("[result] EXIT %d (%s)" % (SP.EXIT["report"], SP.EXIT_TEXT[SP.EXIT["report"]]))
        return SP.EXIT["report"]
    fs.log("[result] run id %s, result file %s" % (st.run_id, os.path.abspath(result_path)))
    try:
        (entry or fs.main)()
    except SystemExit as e:
        if e.code in (0, None) and st.outcome is None and not st.arm_sent and not st.arm_prep:
            return 0                              # --help and the like: no flight, nothing to report
        st.set_outcome("error", st.last_exception or "the flight script exited with %r" % (e.code,))
    except KeyboardInterrupt:
        st.set_outcome("safety_stop", "Ctrl+C outside the flight script's own SIGINT handler")
    except RuntimeError as e:
        if st.final is None:                      # not the supervisor's loop stop
            traceback.print_exc(); st.set_outcome("error", "%s: %s" % (type(e).__name__, e))
    except BaseException as e:  # noqa: BLE001
        traceback.print_exc()
        st.set_outcome("error", "%s: %s" % (type(e).__name__, e))
    return SP.report(st, fs.log, result_path)["exit"]


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
