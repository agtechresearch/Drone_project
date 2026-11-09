#!/usr/bin/env python3
"""Safety layer around the UNCHANGED flight script path_flight_phase1_v13.py (2026-11-01). The flight script file is never
edited: this module only replaces methods of its DroneController class (and its module-level main_async / log) at run time.
Used by
  wk_mission.py           (SITL, together with the wall-keep / localisation layer)
  real_flight_safe.py     (real vehicle: v13's own mission with this layer only)
so the SITL tests of the safety paths test the same code that runs on the real vehicle. Python 3.6 / MAVSDK 0.12 (VOXL2).

What changes against the flight script (codex graceful-stop-kill / localization-task-supervision / yaw-feedback-required /
feedback-pose-freshness / real-mission-result-contract, 2026-10-31..11-01):
  landing (ONE owner)     every landing - the normal one at the mission end, after a stage failure, a STOP, an emergency -
                          goes through the same task: land command time-limited, 3 fast tries, then a retry every 2 s until
                          accepted or the touchdown is seen. v13's normal landing sent land once and went on on a failure.
  touchdown               only an in_air=False RECEIVED AFTER the land request counts (one in_air reader for the whole flight,
                          a second subscription would steal the stream in MAVSDK 0.12). Deadline LAND_CONFIRM_TIMEOUT from
                          the land request; then "TOUCHDOWN NOT CONFIRMED", no disarm, operator hand-over.
  emergency_stop          v13: land, then kill the motors 0.5 s later WHATEVER the height (Ctrl+C = the vehicle drops).
                          here: land, wait for touchdown, disarm only after it, never kill unless ALLOW_KILL.
  land_and_disarm         replaced (same log lines): shared landing, touchdown wait above, disarm only after touchdown.
  mission result          kept here, NOT taken from v13's "CUSTOM MISSION COMPLETE" / "Done!": the FIRST event wins
                          (complete | stage_failed | safety_stop | error; not_started when nothing was armed), the landing
                          (confirmed / NOT confirmed / not needed) and the disarm are kept apart; result() -> dict + exit code.
  wait_until_yaw_reached  same log lines / return values; a stale or missing yaw resets the hold timer; no yaw at all is a
                          failure (v13: assumed success).
  feedback_state          yaw = NaN when the attitude is older than YAW_FRESH_S.
  log                     every log call goes through a logger that cannot raise.
  feedback freshness      receive time per stream: SITL = MAVSDK telemetry generators wrapped at class level; real = the VOXL
                          pipe's own timestamp changing. With monitor=True (real_flight_safe.py) a task lands the vehicle when
                          position or attitude is older than FEED_STALE_S after arming; wk_mission.py runs its own check.
What a failed land command means (2026-11-01 codex offboard-fallback-contract reply-2): the failure kind is logged as seen -
"rpc_timeout" (no answer from mavsdk_server within LAND_RPC_TIMEOUT: the FC state is UNKNOWN, it does not prove a link loss),
"fc_denied" (the FC answered COMMAND_DENIED*), "fc_no_ack" (mavsdk_server got no ACK from the FC), "no_fc" (NO_SYSTEM /
CONNECTION_ERROR at mavsdk_server), "action_error", "error". While mavsdk_server runs it re-sends the LAST setpoint, so a
vehicle whose land is not accepted may keep following that setpoint (not necessarily a hover where it is): the operator
takes over by RC. No PX4 failsafe is claimed here; the vehicle parameters are listed in real_flight_safe.py."""
import asyncio, json, math, os

YAW_FRESH_S = 0.5            # s: an attitude older than this is not a yaw
FEED_STALE_S = 1.0           # s: position / attitude without a new message -> land (monitor task)
LAND_RPC_TIMEOUT, LAND_TRIES = 3.0, 3   # the land command: time limit per try, tries
LAND_RETRY_S = 2.0           # s between land retries after the fast tries failed (until sent or touchdown confirmed)
LAND_CONFIRM_TIMEOUT = 40.0  # s from the land request: touchdown seen, or NOT confirmed (no disarm, operator hand-over)
ALLOW_KILL = os.environ.get("WK_ALLOW_KILL", "0") == "1"   # software motor kill when no land command got through
FINISH_WAIT_S = 5.0          # s: program end - how long one cancelled task may take to end
# STOP supervision (2026-11-01 codex fallback reply-4 / decisions reply-7). Clock: the event loop's time() - wall time on the
# vehicle, the scaled sim clock in SITL. From the FIRST STOP / emergency landing:
STOP_HANDOFF_S = 15.0        # s: the flight script's main flow must have reached its landing step (else: cancelled, supervisor owns)
STOP_TOTAL_S = 70.0          # s: the whole stop - touchdown or NOT confirmed, disarm, result - must be over (else: supervisor owns)
STOP_END_S = 80.0            # s: HARD limit for the result line when the supervisor finishes (2026-11-01 codex decisions reply-9:
                             # the take-over at <= STOP_TOTAL_S leaves STOP_END_S - STOP_TOTAL_S for disarm, its check, the task
                             # clean-up and the result; every one of these steps gets only what is left of it)
END_RESERVE_S = 1.0          # s kept for writing the result file and the [result] lines
DISARM_CONFIRM_S = 2.0       # s: after a disarm ack, the armed telemetry must say False (and stay so) within this time
# exit code of a flight (real_flight_safe.py); "handover" wins over everything, a result file that could not be written ("report")
# over the mission outcome. Layers (2026-11-01 codex decisions reply-7 S3): launcher / mission_result.json "exit" = these codes;
# a wk_fly.sh run's status.json (run_status.sh) has its own codes: 10 mission failed (handover included), 13 safety landing,
# 16 result file not written; batch_expect compares the status.json code.
EXIT = {"complete": 0, "error": 1, "stage_failed": 2, "safety_stop": 3, "not_started": 4, "handover": 10, "report": 21}
EXIT_TEXT = {0: "mission complete, touchdown confirmed, disarmed", 1: "error in the flight script (landed / nothing armed)",
             2: "a stage failed (touchdown confirmed, disarmed)", 3: "safety stop / STOP (touchdown confirmed, disarmed)",
             4: "stopped before arming (no flight)",
             10: "touchdown NOT confirmed, disarm not done or arm result unknown: OPERATOR HAND-OVER",
             21: "flight result as logged above, but the result FILE was not written"}


class SafetyState(object):
    """per-flight safety state (wk_mission.WallKeep inherits it)"""

    def __init__(self):
        self.rx = {}; self.rx_n = {"pos": 0, "att": 0}; self.rx_patched = False; self.rx_start = None; self.voxl_ts_last = None
        # the feedback source is chosen by mode, not by "whatever stamped first" (2026-11-01 codex real-feedback-source-selection:
        # v13's real mode reads MAVSDK position for its own monitor but takes the attitude from the VOXL pipe, so one MAVSDK
        # position sample switched the VOXL check off and the attitude looked stale): "mavsdk" (--sim) | "voxl" (real)
        self.source = None; self.rx_voxl = {}; self.voxl_bad_ts = 0
        self.stopped = False         # a landing / STOP was requested: no arm, no offboard start any more
        self.in_air = None; self.in_air_t = None   # last in_air value seen (one reader for the flight), and when
        self.log = print
        self.land_task = None; self.land_state = None; self.land_reason = None   # the one landing: requested / sent / failed
        self.land_t0 = None          # loop time of the land request (a touchdown counts only from an in_air=False after it)
        self.land_failures = []      # (time, kind, text) per failed land command, kind: see the module docstring
        self.land_retry_task = None  # keeps asking for a landing after the fast tries (2026-11-01 codex offboard-fallback-contract)
        self.inair_task = None       # the one in_air reader
        self.main_task = None        # the task running the mission (execute_mission / after arming)
        self.flow_task = None        # the task running the flight script's main_async (its exception path calls emergency_stop)
        self.task = None             # a task a stop must cancel and collect (wk_mission's correction task / the monitor)
        # mission result (2026-11-01 codex real-mission-result-contract): the FIRST event is kept, a later landing does not erase it
        self.outcome = None; self.outcome_detail = ""; self.stages = []; self.last_exception = None
        # arm (2026-11-01 codex decisions reply-7 S2): preparation entered vs the arm RPC really sent (the guard of Action.arm)
        self.arm_prep = False        # the flight script's arm step began (its 1 s / pipe / 2 s waits come before the RPC)
        self.arm_sent = False        # the arm RPC went out (the FC may be armed even if it never answered)
        self.arm_state = None        # None | sent (no answer yet) | acked | denied (FC said no) | unknown (other error / cancelled)
        self.arm_sent_t = None; self.arm_done_t = None
        self.touchdown = None        # True confirmed | False NOT confirmed within the deadline | None not waited for
        self.disarm_state = None     # done | refused | failed | None (not tried)
        self.disarm_t = None
        self.disarm_check = None     # disarmed (armed telemetry False after the ack) | ARMED AGAIN | no armed telemetry | None
        # STOP supervision / result ownership
        self.stop_t = None           # loop time of the first STOP / emergency landing
        self.landing_entered = None  # loop time the flight script's main flow reached a landing step
        self.sup_task = None; self.sup_owner = False   # the STOP supervisor; True = it (not the main flow) finishes the stop
        self.final = None            # the result dict once reported (the result is made and written ONCE)
        self.result_path = None; self.run_id = None; self.report_error = None
        self.notes = []              # unresolved / resolved states worth keeping in the result (arm unknown, late arm, ...)

    def set_outcome(self, kind, detail=""):
        if self.outcome is None:
            self.outcome, self.outcome_detail = kind, detail
            try:
                self.log("[result] first mission event: %s%s" % (kind, (" - " + detail) if detail else ""))
            except Exception:  # noqa: BLE001
                pass

    def arm_settled(self):
        """the arm RPC is over with a known effect (or was never sent): acked / denied / not sent"""
        return (not self.arm_sent) or self.arm_state in ("acked", "denied")

    def result(self):
        """final result of the flight: mission outcome, landing and disarm kept apart; exit code (EXIT)"""
        outcome = self.outcome or "error"
        detail = self.outcome_detail if self.outcome else "the flight script ended without a mission result"
        if outcome == "safety_stop" and not self.arm_sent:
            outcome = "not_started"
        if not self.arm_sent:
            landing = "not needed (never armed)"
        else:
            landing = {True: "confirmed", False: "NOT confirmed"}.get(self.touchdown, "NOT confirmed (no touchdown wait)")
        notes = list(self.notes)
        # the arm RPC's effect (codex decisions reply-7 S1.4): no answer -> the vehicle may be armed; an answer AFTER the disarm
        # -> the vehicle may be armed again. An unknown arm is resolved only by a disarm acknowledged after it.
        # an unknown arm is resolved only by a disarm ack AFTER it and the armed telemetry then saying False (codex reply-9: an
        # FC effect of the unknown arm may come after the disarm)
        arm_unknown = self.arm_sent and self.arm_state in ("sent", "unknown") and not (
            self.disarm_state == "done" and self.disarm_t is not None and self.arm_done_t is not None and self.disarm_t > self.arm_done_t
            and self.disarm_check == "disarmed")
        rearmed = self.disarm_check == "ARMED AGAIN"
        if rearmed:
            notes.append("armed telemetry said True again after the disarm ack: the vehicle may be armed")
        if self.arm_sent and self.arm_state == "sent":
            notes.append("arm RPC sent at %.1f, no answer: the FC may be armed" % self.arm_sent_t)
        late_arm = (self.arm_state == "acked" and self.disarm_t is not None and self.arm_done_t is not None
                    and self.arm_done_t > self.disarm_t)
        if late_arm:
            notes.append("arm acknowledged AFTER the disarm (%.1f > %.1f): the vehicle may be armed" % (self.arm_done_t, self.disarm_t))
        handover = bool(self.arm_sent and (self.touchdown is not True or self.disarm_state != "done" or arm_unknown or late_arm
                                           or rearmed))
        failed = [s[0] for s in self.stages if not s[1]]
        code = EXIT["handover"] if handover else (EXIT["report"] if self.report_error else EXIT[outcome])
        return {"run_id": self.run_id, "outcome": outcome, "detail": detail, "first_failed_stage": failed[0] if failed else None,
                "stages": ["%s:%s" % (n, "ok" if ok else "FAILED") for n, ok in self.stages],
                "armed_requested": self.arm_sent, "arm_prep": self.arm_prep, "arm_state": self.arm_state,
                "land_request": self.land_state, "land_reason": self.land_reason,
                "land_failures": [{"kind": k, "text": x} for _, k, x in self.land_failures], "landing": landing,
                "disarm": self.disarm_state, "disarm_check": self.disarm_check, "operator_handover": handover,
                "stop_finished_by": ("supervisor" if self.sup_owner else "main flow") if self.stop_t is not None else None,
                "notes": notes, "report_error": self.report_error, "exit": code}

    def feed_age(self, now):
        """seconds since the last position / attitude message of the ACTIVE source (from the watch start when none came yet)"""
        t0 = self.rx_start if self.rx_start is not None else now
        rx = self.rx_voxl if self.source == "voxl" else self.rx
        return {k: now - rx.get(k, t0) for k in ("pos", "att")}

    def poll_voxl(self, ctl, now):
        """real vehicle: a pipe message counts only when its own timestamp moves FORWARD (a repeated or older stamp - restart,
        reader re-emitting - is no new message), and position / attitude only when the values are finite"""
        if self.source != "voxl":
            return
        lat = (getattr(ctl, "voxl_latest", None) or {}).get(getattr(ctl, "reach_pipe", None)) or {}
        ts = lat.get("voxl_ts_ms")
        if ts is None or ts == self.voxl_ts_last:
            return
        if self.voxl_ts_last is not None and ts < self.voxl_ts_last:
            self.voxl_bad_ts += 1; return
        self.voxl_ts_last = ts
        if all(_fin(lat.get(k)) for k in ("pos_x", "pos_y", "pos_z")):
            self.rx_voxl["pos"] = now
        if _fin(lat.get("yaw_deg")):
            self.rx_voxl["att"] = now


def _fin(v):
    return isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)


def guard_mavsdk(st, action_cls, offboard_cls, telemetry_cls, log):
    """the last line of defence, at the MAVSDK class level (whoever calls: the flight script's own code included)
    (2026-11-01 codex graceful-stop-kill reply-6 / preflight-stop-before-arm):
      disarm         only after an in_air=False was seen (after the land request, when there is one) and nothing newer
                     said True; else refused (the flight script's landing step disarmed after 15 s whatever the state)
      arm, start     refused after a STOP / emergency landing (a STOP in preflight must not be followed by arming)
      kill           only with ALLOW_KILL
    in_air values are recorded by wrapping the telemetry generator (no extra subscription)."""
    for cls, name in ((action_cls, "disarm"), (action_cls, "arm"), (action_cls, "kill"), (offboard_cls, "start")):
        if getattr(cls, "_sp_guarded_" + name, False):
            continue
        orig = getattr(cls, name)

        def make(name, orig, cls):
            async def g(self_x, *a, **k):
                t = cls._sp_target
                # disarm: the last in_air must be False and, once a landing was requested, received AFTER the request (an
                # old False from before the take-off does not count; 2026-11-01 codex graceful-stop-kill reply-8)
                if name == "disarm" and (t.in_air is not False or (t.land_t0 is not None and (t.in_air_t is None or t.in_air_t < t.land_t0))):
                    t.log("!!! TOUCHDOWN NOT CONFIRMED: disarm refused by the safety layer (last in_air %s%s) !!!" % (
                        t.in_air, "" if t.in_air is not False else ", received before the land request"))
                    raise RuntimeError("disarm refused: touchdown not confirmed")
                if name in ("arm", "start") and t.stopped:
                    t.log("!!! %s refused by the safety layer: a STOP / emergency landing was requested before !!!" % name.upper())
                    raise RuntimeError("%s refused after STOP" % name)
                if name == "arm":
                    # the RPC really goes out now (S2): sent -> acked | denied (FC said no) | unknown (other error, cancelled)
                    t.arm_sent = True; t.arm_state = "sent"; t.arm_sent_t = asyncio.get_event_loop().time()
                    try:
                        r = await orig(self_x, *a, **k)
                    except asyncio.CancelledError:
                        t.log("!!! arm RPC cancelled before its answer: the FC may be armed (arm result unknown) !!!"); raise
                    except Exception as e:  # noqa: BLE001
                        t.arm_state = "denied" if land_fail_kind(e)[0] == "fc_denied" else "unknown"
                        t.arm_done_t = asyncio.get_event_loop().time(); raise
                    t.arm_state = "acked"; t.arm_done_t = asyncio.get_event_loop().time()
                    if t.stopped:
                        t.log("!!! arm acknowledged AFTER a STOP (%s) !!!" % ("after the disarm" if t.disarm_t is not None else
                                                                               "before the disarm"))
                    return r
                if name == "kill" and not ALLOW_KILL:
                    t.log("!!! KILL refused by the safety layer (no --allow-kill) !!!")
                    raise RuntimeError("kill refused")
                return await orig(self_x, *a, **k)
            return g
        setattr(cls, name, make(name, orig, cls)); setattr(cls, "_sp_guarded_" + name, True)
        cls._sp_target = st
    action_cls._sp_target = st; offboard_cls._sp_target = st
    if not getattr(telemetry_cls, "_sp_inair", False):
        orig_ia = telemetry_cls.in_air

        async def in_air(self_t):
            async for v in orig_ia(self_t):
                w = telemetry_cls._sp_target
                w.in_air = bool(v); w.in_air_t = asyncio.get_event_loop().time()
                yield v
        telemetry_cls.in_air = in_air; telemetry_cls._sp_inair = True
    telemetry_cls._sp_target = st
    st.log = log


def stamp_telemetry(st):
    """wrap MAVSDK's position / attitude generators (class level, once per process) so each message that really arrives
    stamps st.rx; the target is the state of the latest install()"""
    from mavsdk import telemetry as _T

    def _stamped(key, orig):
        async def gen(self_t):
            async for v in orig(self_t):
                w = _T.Telemetry._sp_target
                w.rx[key] = asyncio.get_event_loop().time(); w.rx_n[key] += 1; w.rx_patched = True
                yield v
        return gen
    _T.Telemetry._sp_target = st
    if not getattr(_T.Telemetry, "_sp_stamped", False):
        _T.Telemetry.position_velocity_ned = _stamped("pos", _T.Telemetry.position_velocity_ned)
        _T.Telemetry.attitude_euler = _stamped("att", _T.Telemetry.attitude_euler)
        _T.Telemetry._sp_stamped = True


def land_fail_kind(e):
    """what a failed land command tells (2026-11-01 codex offboard-fallback-contract reply-2: a missing RPC answer does not prove
    that the FC stopped getting setpoints; only an FC answer is a rejection) -> (kind, explanation)"""
    if isinstance(e, asyncio.TimeoutError):
        return "rpc_timeout", "no answer from mavsdk_server within %.0f s - FC state unknown" % LAND_RPC_TIMEOUT
    r = getattr(getattr(e, "_result", None), "result", None)
    if r is not None:
        n = getattr(r, "name", str(r))
        if n.startswith("COMMAND_DENIED"):
            return "fc_denied", "the FC rejected it (%s)" % n
        if n == "TIMEOUT":
            return "fc_no_ack", "mavsdk_server got no ACK from the FC (%s)" % n
        if n in ("NO_SYSTEM", "CONNECTION_ERROR"):
            return "no_fc", "mavsdk_server has no FC connection (%s)" % n
        return "action_error", "MAVSDK action error %s" % n
    return "error", "other error %s" % type(e).__name__


async def finish(st, deadline=None):
    """end of the flight: every task of this layer is stopped HERE and reported, nothing disappears silently with the event loop
    (2026-11-01 codex offboard-fallback-contract reply-2: the land retries do not outlive the program - a land still not
    accepted is an operator hand-over). The calling task (the STOP supervisor when it finishes the stop) is skipped."""
    if st.land_retry_task is not None and not st.land_retry_task.done():
        st.log("!!! program end: the land command is still NOT accepted - land retries stop here, the vehicle may keep "
               "following its last setpoint -> OPERATOR: take over by RC !!!")
    me = asyncio.Task.current_task(); lp = asyncio.get_event_loop()
    todo = [(name, t) for name, t in (("land retry", st.land_retry_task), ("monitor / correction", st.task),
                                      ("in_air reader", st.inair_task), ("landing", st.land_task), ("STOP supervisor", st.sup_task))
            if t is not None and t is not me and not t.done()]
    for i, (name, t) in enumerate(todo):
        if not t.done():
            t.cancel()
            # bounded: a task that does not end after its cancel must not hold the program (1921p14 i4: a swallowed cancel
            # kept the land retries going and finish() waited for ever); with a deadline (the supervisor's STOP_END_S) the
            # tasks share what is left of it
            wait = FINISH_WAIT_S if deadline is None else max(0.05, min(FINISH_WAIT_S, (deadline - lp.time()) / (len(todo) - i)))
            done, _ = await asyncio.wait([t], timeout=wait)
            if not done:
                st.log("!!! program end: the %s task did not end within %.1f s of its cancel - left behind" % (name, wait))
            elif not t.cancelled() and t.exception() is not None:
                st.log("[safety] %s task ended with %r" % (name, t.exception()))
    return st.result()


def prepare_result(st, path):
    """BEFORE the flight (codex decisions reply-7 S3): a result file of an earlier run is moved aside (kept, never deleted), so
    a file that this run fails to write can not be read as this run's result; the run gets an id written into its result.
    -> None, or the reason the file could not be prepared (then nothing may fly)"""
    import time as _t
    st.result_path = path; st.run_id = "%s_pid%d" % (_t.strftime("%Y%m%d_%H%M%S"), os.getpid())
    if not path:
        return None
    try:
        if os.path.lexists(path):
            os.replace(path, "%s.prev_%s" % (path, _t.strftime("%Y%m%d_%H%M%S")))
        d = os.path.dirname(os.path.abspath(path))
        if not os.access(d, os.W_OK):
            return "result directory %s is not writable" % d
    except OSError as e:
        return "earlier result file %s could not be moved aside: %r" % (path, e)
    return None


async def finalize(st, log=None, deadline=None):
    """the ONE end of a flight's result: stop the layer's tasks, write the JSON, then print the "[result]" lines (so a file
    that could not be written is in the EXIT line - no success EXIT after a failed write). A second call returns the first
    result. Called by the STOP supervisor when it finishes a stop, else by report() after the flight script's main."""
    if st.final is not None:
        return st.final
    log = log or st.log
    await finish(st, None if deadline is None else deadline - END_RESERVE_S)
    if st.final is not None:                 # finished meanwhile by the other owner
        return st.final
    res = st.result()
    if st.result_path:
        try:
            with open(st.result_path + ".tmp", "w") as f:
                json.dump(res, f, indent=1)
            os.replace(st.result_path + ".tmp", st.result_path)
        except (OSError, TypeError, ValueError) as e:
            st.report_error = "result file %s not written: %r" % (st.result_path, e)
            res = st.result()                # exit 21, or 10 when a hand-over is due (the hand-over is not hidden)
    st.final = res
    lines = ["=" * 78, "[result] (the flight script's own COMPLETE / Done lines above are not the result)",
             "[result] run %s, stop finished by %s" % (res["run_id"], res["stop_finished_by"] or "-"),
             "[result] MISSION  %s%s" % (res["outcome"], (" - " + res["detail"]) if res["detail"] else ""),
             "[result] LANDING  %s | land request %s%s | disarm %s | arm %s" % (
                 res["landing"], res["land_request"], (" | %d failed land commands (%s)" % (len(res["land_failures"]),
                 "/".join(sorted(set(f["kind"] for f in res["land_failures"]))))) if res["land_failures"] else "", res["disarm"],
                 res["arm_state"])]
    lines += ["[result] NOTE %s" % n for n in res["notes"]]
    if res["report_error"]:
        lines.append("[result] !!! RESULT FILE NOT WRITTEN: %s !!!" % res["report_error"])
    if res["operator_handover"]:
        lines.append("[result] !!! OPERATOR HAND-OVER: touchdown not confirmed, not disarmed or arm result unknown - fly / land / "
                     "disarm by RC !!!")
    lines += ["[result] EXIT %d (%s)" % (res["exit"], EXIT_TEXT[res["exit"]]), "=" * 78]
    for x in lines:
        try:
            log(x)
        except Exception:  # noqa: BLE001
            pass
    return res


def report(st, log, path=None):
    """after the flight script's main returned / exited (outside the event loop): let a running STOP supervisor finish first
    (it is bounded by STOP_TOTAL_S), then finalize() -> result dict"""
    if path and st.result_path is None:
        st.result_path = path
    loop = asyncio.get_event_loop()
    try:
        if st.sup_task is not None and not st.sup_task.done():
            left = (st.stop_t + STOP_END_S - loop.time()) if st.stop_t is not None else STOP_END_S
            loop.run_until_complete(asyncio.wait([st.sup_task], timeout=max(0.1, left + 0.5)))
        return st.final if st.final is not None else loop.run_until_complete(finalize(st, log))
    except BaseException as e:  # noqa: BLE001  (a second Ctrl+C here must still give a result)
        st.set_outcome("error", "finish: %r" % e)
        if st.final is None:
            st.report_error = "the end step was interrupted (%r): no result file" % e; st.final = st.result()
            for x in ("[result] !!! RESULT FILE NOT WRITTEN: %s !!!" % st.report_error,     # the standard line (codex reply-9)
                      "[result] EXIT %d (%s)" % (st.final["exit"], EXIT_TEXT[st.final["exit"]])):
                try:
                    log(x)
                except Exception:  # noqa: BLE001
                    pass
        return st.final


def install(fs, st, monitor=False):
    """patch fs.DroneController (the flight script module fs) in place; returns the safe logger.
    The flight script's own methods stay reachable as C._sp_orig[name]."""
    C = fs.DroneController
    if not hasattr(C, "_sp_orig"):
        C._sp_orig = {n: getattr(C, n, None) for n in ("feedback_state", "wait_until_yaw_reached", "land_and_disarm",
                                                 "trigger_emergency_land", "emergency_stop", "arm_and_start_offboard",
                                                 "go_to_relative", "go_yaw", "execute_mission")}
    O = C._sp_orig
    _raw_log = fs.log if not getattr(fs.log, "_sp_safe", False) else fs.log._sp_raw

    def safe_log(msg):
        try:
            if isinstance(msg, str) and msg.startswith("Exception: "):
                st.last_exception = msg[len("Exception: "):][:300]   # the flight script's main_async exception line
        except Exception:  # noqa: BLE001
            pass
        try:
            _raw_log(msg)
        except Exception:  # noqa: BLE001
            pass
    safe_log._sp_safe = True; safe_log._sp_raw = _raw_log
    fs.log = safe_log                      # v13 calls its module-global log(): this replaces it for every call
    stamp_telemetry(st)
    from mavsdk import action as _A, offboard as _OF, telemetry as _T
    guard_mavsdk(st, _A.Action, _OF.Offboard, _T.Telemetry, safe_log)
    now = lambda: asyncio.get_event_loop().time()  # noqa: E731

    # the flight script's main flow: which task it is (its exception path calls emergency_stop from inside it)
    if hasattr(fs, "main_async"):
        om = getattr(fs, "_sp_main_async", None) or fs.main_async
        fs._sp_main_async = om

        async def main_async(args):
            st.flow_task = asyncio.Task.current_task()
            return await om(args)
        fs.main_async = main_async

    def feedback_state(self):
        fb = O["feedback_state"](self)
        if fb is None:
            return None
        if st.rx_start is not None and st.feed_age(now())["att"] > YAW_FRESH_S:
            fb = dict(fb); fb["yaw_deg"] = float("nan")
        return fb

    async def wait_until_yaw_reached(self, name, target_yaw, timeout):
        fs.log("       [wait_yaw] reaching {} target_yaw={:+.1f} tol={:.1f}deg timeout={:.1f}s".format(
            name, fs.yaw_to_360(target_yaw), self.args.yaw_tol, timeout))
        start = now(); in_tol_since = None; last_print = -1.0; valid_seen = False
        while True:
            if self.aborted:
                return False, "ABORTED", None
            t = now(); elapsed = t - start
            fb = self.feedback_state()
            fb_yaw = fb.get("yaw_deg", float("nan")) if fb else float("nan")
            valid = fb is not None and math.isfinite(fb_yaw)
            err = abs(fs.angle_diff_deg(target_yaw, fb_yaw)) if valid else float("nan")
            if elapsed - last_print >= 0.5:
                last_print = elapsed
                fs.log("       [{} yaw] t={:.1f}s target={:+.1f} fb_yaw={} err={}".format(
                    name, elapsed, fs.yaw_to_360(target_yaw), "n/a" if not valid else "{:+.1f}".format(fb_yaw),
                    "n/a" if not valid else "{:.1f}deg".format(err)))
            if valid:
                valid_seen = True
                if err <= self.args.yaw_tol:
                    if in_tol_since is None:
                        in_tol_since = t
                    elif t - in_tol_since >= self.args.yaw_stable_time:
                        return True, "REACHED", elapsed
                else:
                    in_tol_since = None
            else:
                in_tol_since = None              # a gap in the yaw is not part of the hold
            if (not valid_seen) and elapsed >= min(2.0, timeout):
                fs.log("[safety] %s: no yaw feedback -> turn NOT confirmed (failure)" % name)
                return False, "NO_YAW_FEEDBACK", elapsed
            if elapsed > timeout:
                return False, "TIMEOUT", elapsed
            await asyncio.sleep(0.1)

    # ---- stage results: the first failed stage is kept (v13 only logs it and, without --stop-on-fail, goes on)
    def _stage(nm, name_pos):
        async def w(self, *a, **k):
            ok = await O[nm](self, *a, **k)
            name = a[name_pos] if len(a) > name_pos else k.get("name", nm)
            st.stages.append((str(name), bool(ok)))
            if not ok and not self.aborted:
                st.set_outcome("stage_failed", "%s did not pass" % name)
            return ok
        return w

    async def execute_mission(self):
        st.main_task = asyncio.Task.current_task()
        r = await O["execute_mission"](self)
        if not self.aborted:
            st.set_outcome("complete", "all %d stages passed" % len(st.stages))
        return r

    # ---- the one in_air reader
    def _inair_start(self):
        if st.inair_task is None or st.inair_task.done():
            st.inair_task = asyncio.ensure_future(_inair_read(self))

    async def _inair_read(self):
        said = False
        while True:
            try:
                async for v in self.drone.telemetry.in_air():
                    st.in_air = bool(v); st.in_air_t = now()
            except asyncio.CancelledError:
                raise
            except Exception as e:  # noqa: BLE001
                if not said:
                    fs.log("[safety] in_air stream: %r (re-subscribing)" % e); said = True
            await asyncio.sleep(0.5)

    def _touched_down():
        """an in_air=False received after the land request and after the arm RPC's answer; while an arm RPC is still without an
        answer nothing counts (the FC may arm after the False; codex decisions reply-7 S1.4)"""
        if st.arm_sent and st.arm_state == "sent":
            return False
        ref = max(st.land_t0 if st.land_t0 is not None else float("inf"), st.arm_done_t or 0.0)
        return st.in_air is False and st.in_air_t is not None and st.in_air_t >= ref

    async def _wait_touchdown(self, until=None):
        """until an in_air=False received after the land request, or LAND_CONFIRM_TIMEOUT after the request (or `until`, the
        STOP supervisor's whole-stop limit, when that is earlier)"""
        _inair_start(self)
        t0 = now(); deadline = (st.land_t0 if st.land_t0 is not None else t0) + LAND_CONFIRM_TIMEOUT; last = -9.0
        if until is not None:
            deadline = min(deadline, until)
        while True:
            t = now()
            if _touched_down():
                st.touchdown = True
                fs.log("       *** LANDED *** (in_air false %.1f s after the land request)" % (st.in_air_t - st.land_t0))
                return True
            if t >= deadline:
                st.touchdown = False
                return False
            if t - last >= 1.0:
                last = t
                try:
                    fb = self.feedback_state()
                    fs.log("       [land] t={:.1f}s in_air={}{}".format(t - (st.land_t0 or t0), st.in_air,
                           " fb_rel=(N{:+.2f},E{:+.2f},U{:+.2f})".format(fb["rel_n"], fb["rel_e"], fb["rel_u"]) if fb else ""))
                except Exception:  # noqa: BLE001
                    pass
            await asyncio.sleep(0.1)

    async def _disarm(self, deadline=None):
        """disarm (time-limited; with a deadline only what is left of it), then the armed telemetry must say False after the ack
        (codex decisions reply-9: an arm of unknown result can take effect after the disarm)"""
        lim = LAND_RPC_TIMEOUT if deadline is None else max(0.05, min(LAND_RPC_TIMEOUT, deadline - now()))
        try:
            await asyncio.wait_for(self.drone.action.disarm(), lim)
            st.disarm_state = "done"; st.disarm_t = now(); fs.log("       Disarmed.")
        except asyncio.CancelledError:   # Python 3.6: a subclass of Exception - never swallow a cancel (1921p14 i4)
            raise
        except Exception as e:  # noqa: BLE001
            st.disarm_state = "refused" if "refused" in str(e) else "failed"
            fs.log("       Disarm note: {!r}".format(e)); return
        lim = DISARM_CONFIRM_S if deadline is None else max(0.0, min(DISARM_CONFIRM_S, deadline - now()))
        st.disarm_check = await _armed_check(self, lim)
        fs.log("       disarm check (armed telemetry, %.1f s): %s" % (lim, st.disarm_check))

    async def _armed_check(self, lim):
        """armed telemetry for `lim` s after the disarm ack: disarmed (False seen, no True after it) | ARMED AGAIN | no armed telemetry
        (v13 never reads this stream, so this is its only subscriber)"""
        seen = []

        async def read():
            async for a in self.drone.telemetry.armed():
                seen.append(bool(a))
        t = asyncio.ensure_future(read())
        try:
            await asyncio.wait([t], timeout=lim)
        finally:
            t.cancel()
        if t.done() and not t.cancelled() and t.exception() is not None:
            fs.log("       disarm check: armed telemetry %r" % t.exception())
        if any(seen) and (False not in seen or seen[-1] or True in seen[seen.index(False):]):
            return "ARMED AGAIN"
        return "disarmed" if seen else "no armed telemetry"

    # ---- the one landing (normal end, stage failure, STOP, emergency)
    async def _request_landing(self, reason, emergency):
        st.stopped = True                    # no arm / offboard start after a landing request
        if st.land_task is None:
            self.streaming = False
            if emergency:
                self._emergency_triggered = True
                self.aborted = True
            st.land_reason = reason
            st.land_task = asyncio.ensure_future(_land_procedure(self, reason, emergency))
        await asyncio.shield(st.land_task)

    async def _land_procedure(self, reason, emergency):
        st.land_state = "requested"; st.land_t0 = now(); _inair_start(self)
        if emergency:
            fs.log("!!! EMERGENCY LAND TRIGGERED: {} !!!".format(reason))
        else:
            fs.log("       land requested: {}".format(reason))
        await asyncio.sleep(0.2)
        for i in range(LAND_TRIES):
            try:
                await asyncio.wait_for(self.drone.action.land(), LAND_RPC_TIMEOUT)
                st.land_state = "sent"
                fs.log("       Land command sent.")
                return
            except asyncio.CancelledError:   # Python 3.6: a subclass of Exception - never swallow a cancel (1921p14 i4)
                raise
            except Exception as e:  # noqa: BLE001
                _land_failed(e, "%d/%d" % (i + 1, LAND_TRIES))
                await asyncio.sleep(0.5)
        st.land_state = "failed"
        st.land_retry_task = asyncio.ensure_future(_land_retry(self))
        if ALLOW_KILL:
            try:
                await asyncio.wait_for(self.drone.action.kill(), LAND_RPC_TIMEOUT)
                fs.log("       Kill command sent (--allow-kill).")
            except asyncio.CancelledError:   # Python 3.6: a subclass of Exception - never swallow a cancel (1921p14 i4)
                raise
            except Exception as ke:  # noqa: BLE001
                fs.log("       Kill also failed: {!r}".format(ke))
        else:
            fs.log("       land NOT sent after {} tries ({}) - motors NOT killed (no --allow-kill); retrying every {:.0f} s until "
                   "accepted or touchdown. While mavsdk_server runs it re-sends the LAST setpoint: the vehicle may keep "
                   "following it -> OPERATOR: take over by RC".format(
                       LAND_TRIES, "/".join(sorted(set(f[1] for f in st.land_failures))), LAND_RETRY_S))

    def _land_failed(e, what):
        kind, why = land_fail_kind(e)
        st.land_failures.append((now(), kind, repr(e)[:120]))
        fs.log("       land command failed ({}, {}: {}): {!r}".format(what, kind, why, e))

    async def _land_retry(self):
        k = 0
        while not _touched_down():
            await asyncio.sleep(LAND_RETRY_S); k += 1
            if _touched_down():
                break
            try:
                await asyncio.wait_for(self.drone.action.land(), LAND_RPC_TIMEOUT)
                st.land_state = "sent after retry %d" % k
                fs.log("       Land command sent (retry %d after the fast tries failed)." % k)
                return
            except asyncio.CancelledError:   # Python 3.6: a subclass of Exception - never swallow a cancel (1921p14 i4)
                raise
            except Exception as e:  # noqa: BLE001
                _land_failed(e, "retry %d" % k)
        fs.log("       land retries end: touchdown confirmed (in_air false)")

    async def trigger_emergency_land(self, reason="unknown"):
        st.set_outcome("safety_stop", reason)
        if st.stop_t is None:
            st.stop_t = now()
        _sup_start(self)
        await _request_landing(self, reason, True)

    # ---- STOP supervisor (2026-11-01 codex fallback reply-4 / preflight reply-4 / decisions reply-7 S1): the flight script's
    # main flow is given the stop (its landing step) only while it shows up in time; a main flow stuck in a wait that never
    # ends (connect, health, position, arm / offboard RPC) is cancelled and the supervisor finishes the stop itself. Exactly one
    # of the two makes the result (finalize() runs once). A main flow that ignores its cancel is left behind: the supervisor
    # stops the event loop after the result (the launcher then exits).
    def _sup_start(self):
        if st.sup_task is None:
            st.sup_task = asyncio.ensure_future(_stop_supervisor(self))

    async def _stop_supervisor(self):
        t0 = st.stop_t if st.stop_t is not None else now()
        main = st.flow_task                  # the flight script's main_async task (set by the main_async wrapper)
        why = None
        while why is None:
            if main is None or main.done():
                return                       # nothing to supervise / the main flow ended: its end makes the result
            t = now()
            if st.landing_entered is None and t - t0 >= STOP_HANDOFF_S:
                why = "did not reach its landing step within %.0f s of the STOP" % STOP_HANDOFF_S
            elif t - t0 >= STOP_TOTAL_S:
                why = "did not finish the stop within %.0f s of the STOP" % STOP_TOTAL_S
            else:
                await asyncio.sleep(0.1)
        st.sup_owner = True
        fs.log("!!! STOP supervisor: the main flow %s - cancelling it; the supervisor finishes the stop (landing state, disarm, "
               "result) !!!" % why)
        main.cancel()
        end = t0 + STOP_END_S - END_RESERVE_S
        if st.arm_sent and st.touchdown is True and st.disarm_state is None:
            await _disarm(self, end)         # the main flow saw the touchdown but did not get its disarm done
        elif st.arm_sent and st.touchdown is None:
            landed = await _wait_touchdown(self, until=min(t0 + STOP_TOTAL_S, end))
            if landed and st.disarm_state is None:
                await _disarm(self, end)
            elif not landed:
                fs.log("!!! STOP supervisor: TOUCHDOWN NOT CONFIRMED%s (whole-stop limit %.0f s / touchdown limit %.0f s) - NOT "
                       "disarmed - OPERATOR: take over by RC !!!" % (" (arm RPC without answer)" if st.arm_state == "sent" else "",
                                                                    STOP_TOTAL_S, LAND_CONFIRM_TIMEOUT))
        elif not st.arm_sent:
            fs.log("       STOP supervisor: no arm RPC was sent - nothing to land")
        await finalize(st, fs.log, deadline=t0 + STOP_END_S)
        if not main.done():
            fs.log("!!! STOP supervisor: the main flow did not end after its cancel - event loop stopped, the launcher exits !!!")
            asyncio.get_event_loop().stop()

    async def land_and_disarm(self):
        """replaces the flight script's landing step (same log lines): the shared landing, touchdown only from an in_air=False
        after the land request, disarm only after the touchdown; 'Done!' only when disarmed"""
        fs.log("[7/7] Landing...")
        if st.landing_entered is None:
            st.landing_entered = now()
        await _request_landing(self, "mission end" if st.outcome in (None, "complete") else
                               "after %s (%s)" % (st.outcome, st.outcome_detail), False)
        landed = await _wait_touchdown(self)
        await _collect(st.task)
        for step in ("snapshot", "cancel", "csv", "summary"):
            try:
                if step == "snapshot":
                    self.land_end_snapshot = self.make_position_snapshot("land_end")
                elif step == "cancel":
                    await self._cancel_background_tasks()
                elif step == "csv":
                    self._write_csv_if_needed()
                else:
                    self._print_summary()
            except asyncio.CancelledError:   # Python 3.6: a subclass of Exception - never swallow a cancel (1921p14 i4)
                raise
            except Exception as e:  # noqa: BLE001
                fs.log("       [land] %s: %r" % (step, e))
        fs.log("[disarm]")
        if landed:
            await _disarm(self)
        else:
            fs.log("!!! TOUCHDOWN NOT CONFIRMED within %.0f s of the land request (last in_air %s): NOT disarmed - "
                   "OPERATOR: take over by RC !!!" % (LAND_CONFIRM_TIMEOUT, st.in_air))
        if landed and st.disarm_state == "done":
            fs.log("Done!")

    async def emergency_stop(self):
        st.stopped = True                    # first: no arm / offboard start after this, whatever happens below
        if st.sup_owner:                     # the supervisor finishes this stop (v13's except after the cancel lands here)
            fs.log("       stop: finished by the STOP supervisor")
            return
        in_flow = st.flow_task is not None and asyncio.Task.current_task() is st.flow_task
        if in_flow:                          # v13's main_async calls emergency_stop only from its exception handler
            st.set_outcome("error", st.last_exception or "exception in the flight script's main flow")
            if st.landing_entered is None:
                st.landing_entered = now()   # the main flow itself does the touchdown wait below
        fs.log("!!! STOP (safety layer): land, wait for touchdown, no motor kill !!!")
        await self.trigger_emergency_land("stop requested" if not in_flow else "exception: %s" % (st.last_exception or "?"))
        main = st.flow_task or st.main_task
        if main is not None and asyncio.Task.current_task() is not main and not main.done():
            # a stop from another task (SIGINT) while the main flow runs: the main flow goes on to land_and_disarm
            fs.log("       stop: touchdown wait and disarm left to the mission's landing step")
            await _collect(st.task)
            return
        if st.arm_sent:
            landed = await _wait_touchdown(self)
            if landed:
                await _disarm(self)
            else:
                fs.log("       stop: TOUCHDOWN NOT CONFIRMED within %.0f s of the land request - NOT disarmed, motors NOT "
                       "killed - OPERATOR: take over by RC" % LAND_CONFIRM_TIMEOUT)
        else:
            fs.log("       stop: nothing was armed - no touchdown wait")
        try:
            await self._cancel_background_tasks()
        except asyncio.CancelledError:   # Python 3.6: a subclass of Exception - never swallow a cancel (1921p14 i4)
            raise
        except Exception:  # noqa: BLE001
            pass
        await _collect(st.task)

    async def _collect(t):
        if t is not None and t is not asyncio.Task.current_task():
            if not t.done():
                t.cancel()
            try:
                await t
            except (asyncio.CancelledError, Exception):  # noqa: BLE001
                pass

    async def _monitor(self):
        """real_flight_safe.py: land when position or attitude stops arriving (after arming)"""
        while not self.aborted:
            t = now(); st.poll_voxl(self, t)
            age = st.feed_age(t); old = [k for k in ("pos", "att") if age[k] >= FEED_STALE_S]
            if old:
                await self.trigger_emergency_land("%s feedback: no new message for %s s (stream stopped)" % (
                    "/".join({"pos": "position", "att": "attitude"}[k] for k in old), "/".join("%.1f" % age[k] for k in old)))
                return
            await asyncio.sleep(0.05)

    async def arm_and_start_offboard(self):
        if st.stopped or self.aborted:
            fs.log("!!! arming skipped: a STOP was requested before arming !!!")
            raise RuntimeError("STOP before arming")
        st.source = "mavsdk" if getattr(getattr(self, "args", None), "sim", False) else "voxl"
        fs.log("[safety] feedback freshness source: %s" % st.source)
        st.arm_prep = True                   # the arm step began; arm_sent is set by the Action.arm guard when the RPC goes out
        _inair_start(self)
        r = await O["arm_and_start_offboard"](self)
        st.main_task = asyncio.Task.current_task()
        if st.rx_start is None:
            st.rx_start = now()
        if monitor:
            st.task = asyncio.ensure_future(_monitor(self))
        return r

    C.feedback_state = feedback_state
    C.wait_until_yaw_reached = wait_until_yaw_reached
    C.land_and_disarm = land_and_disarm
    C.trigger_emergency_land = trigger_emergency_land
    C.emergency_stop = emergency_stop
    C.arm_and_start_offboard = arm_and_start_offboard
    if O.get("go_to_relative"):
        C.go_to_relative = _stage("go_to_relative", 3)    # go_to_relative(n_rel, e_rel, alt, name, ...)
    if O.get("go_yaw"):
        C.go_yaw = _stage("go_yaw", 1)                    # go_yaw(target_yaw, name, ...)
    if O.get("execute_mission"):
        C.execute_mission = execute_mission
    return safe_log
