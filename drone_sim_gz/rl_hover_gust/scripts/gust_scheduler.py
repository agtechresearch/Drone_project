#!/usr/bin/env python3
"""Intermittent cross-wind force on the vehicle for the hover-gust episodes (2026-11-03, stage A). Run INSIDE starling-gz with
GZ_PARTITION set:
  python3 gust_scheduler.py <config.json> <batch_dir>            (episodes from <batch_dir>/plan.json)
  python3 gust_scheduler.py <config.json> --clear-only           (one safety clear, then exit)
Per episode it waits for <batch_dir>/ep_XX/ready (written by hover_gust_worker.py when the hover is settled), clears any force,
makes the episode's gust list (rl_demo.contracts.gust_schedule, seed from plan.json; the worker never sees it) and applies each
gust as a PERSISTENT world-frame force on <model>::<link> (ApplyLinkWrench /world/<w>/wrench/persistent) at its sim time, then
removes it (/wrench/clear). At the window end it clears twice and writes ep_XX/cleared; the worker lands only after that.
Every apply / clear is logged with planned and actual sim time (<batch_dir>/gust_events.csv). The force is cleared on every exit
path: normal end, STOP file (<batch_dir>/gust.STOP), SIGINT / SIGTERM, any exception.
Limit: the clear message has no acknowledgement; the evaluator checks the effect (tilt in the quiet periods)."""
import json, os, signal, sys, threading, time
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from rl_demo.contracts import load_config, gust_schedule, link_scoped, AXIS
from gz.transport13 import Node
from gz.msgs10.clock_pb2 import Clock
from gz.msgs10.entity_pb2 import Entity
from gz.msgs10.entity_wrench_pb2 import EntityWrench

cfg = load_config(sys.argv[1]); W = cfg["world"]; LINK = link_scoped(cfg)
node = Node()
pub_p = node.advertise("/world/%s/wrench/persistent" % W, EntityWrench)
pub_c = node.advertise("/world/%s/wrench/clear" % W, Entity)
clock = {"t": None, "wall": None}
node.subscribe(Clock, "/world/%s/clock" % W, lambda m: clock.update(t=m.sim.sec + m.sim.nsec * 1e-9, wall=time.time()))
lock = threading.Lock(); EV = None


def log_ev(ep, kind, planned, force, ok):
    if EV is None:
        return
    with lock:
        EV.write("%s,%s,%s,%.4f,%.3f,%.4f,%d\n" % (ep, kind, "" if planned is None else "%.4f" % planned,
                                                 clock["t"] if clock["t"] is not None else float("nan"), time.time(), force, ok))
        EV.flush()


LAST_CLEAR = [0.0]
CLEAR_TO_APPLY_MIN = 0.2   # wall s. 2026-11-03 logs/2008_wrench_diag.txt: a clear published right before a persistent wrench is
                           # processed AFTER it by ApplyLinkWrench and removes it (dz +0.0004 m vs +0.49 m with 5 N up)


def clear(ep="-", planned=None):
    m = Entity(); m.name = LINK; m.type = Entity.LINK
    ok = pub_c.publish(m); LAST_CLEAR[0] = time.time(); log_ev(ep, "clear", planned, 0.0, ok); return ok


def apply(ep, planned, f):
    """No clear here (see CLEAR_TO_APPLY_MIN): gusts never overlap and every gust ends with a clear >= min_gap_s earlier,
    and the window starts with one, so no force is active before an apply."""
    m = EntityWrench(); m.entity.name = LINK; m.entity.type = Entity.LINK
    d = AXIS["gz_force_dir"]; m.wrench.force.x, m.wrench.force.y, m.wrench.force.z = d[0] * f, d[1] * f, d[2] * f
    wait = CLEAR_TO_APPLY_MIN - (time.time() - LAST_CLEAR[0])
    if wait > 0:
        log_ev(ep, "apply_delayed_after_clear", planned, f, 0); time.sleep(wait)
    ok = pub_p.publish(m); log_ev(ep, "apply", planned, f, ok); return ok


def wait_connected(timeout=10.0):
    t0 = time.time()
    while time.time() - t0 < timeout:
        if pub_p.has_connections() and pub_c.has_connections() and clock["t"] is not None:
            return True
        time.sleep(0.1)
    return False


def safe_clear(ep="-"):
    for _ in range(2):
        clear(ep); time.sleep(0.1)


def stop_handler(sig, frm):
    raise KeyboardInterrupt("signal %d" % sig)


def main():
    global EV
    if not wait_connected():
        print("!! wrench topics or clock not connected (persistent %s clear %s clock %s)" % (
            pub_p.has_connections(), pub_c.has_connections(), clock["t"]), flush=True)
        return 3
    if sys.argv[2] == "--clear-only":
        safe_clear(); print("cleared %s" % LINK, flush=True); return 0
    B = sys.argv[2]; plan = json.load(open(os.path.join(B, "plan.json")))
    EV = open(os.path.join(B, "gust_events.csv"), "a")
    if os.path.getsize(EV.name) == 0:
        EV.write("ep,event,t_sim_planned,t_sim_sent,t_wall,force_n,published\n")
    STOP = os.path.join(B, "gust.STOP")
    print("[gust] %s axis %s, %d episodes" % (LINK, AXIS["name"], len(plan["episodes"])), flush=True)
    safe_clear()
    for ep in plan["episodes"]:
        d = os.path.join(B, "ep_%02d" % ep["id"])
        while not os.path.exists(os.path.join(d, "ready")):
            if os.path.exists(STOP) or os.path.exists(os.path.join(B, "worker_done")):
                print("[gust] stop before episode %d" % ep["id"], flush=True); safe_clear(); return 0
            time.sleep(0.02)
        t0 = clock["t"]; safe_clear(ep["id"])
        sch = gust_schedule(cfg, ep["cond"], ep["seed"])
        json.dump({"episode": ep, "t0_sim": t0, "window_s": cfg["window_s"], "gusts": sch, "axis": AXIS},
                  open(os.path.join(d, "gust_schedule.json"), "w"), indent=1)
        events = []
        for g in sch:
            events += [(t0 + g["t_on"], "on", g["force_n"]), (t0 + g["t_off"], "off", 0.0)]
        print("[gust] ep %d %s seed %d t0 %.3f: %d gusts %s" % (ep["id"], ep["cond"], ep["seed"], t0, len(sch),
              [(g["t_on"], g["force_n"]) for g in sch]), flush=True)
        t_end = t0 + cfg["window_s"]
        while clock["t"] < t_end and not os.path.exists(os.path.join(d, "window_end")):
            if os.path.exists(STOP):
                safe_clear(ep["id"]); return 0
            if events and clock["t"] >= events[0][0]:
                tp, kind, f = events.pop(0)
                (apply(ep["id"], tp, f) if kind == "on" else clear(ep["id"], tp))
            time.sleep(0.002)
        for tp, kind, f in events:          # window ended early (worker): nothing more is applied
            log_ev(ep["id"], "skipped_" + kind, tp, f, 0)
        safe_clear(ep["id"])
        json.dump({"t_sim": clock["t"], "t_wall": time.time()}, open(os.path.join(d, "cleared"), "w"))
    return 0


if __name__ == "__main__":
    signal.signal(signal.SIGTERM, stop_handler)
    rc = 1
    try:
        rc = main()
    except KeyboardInterrupt as e:
        print("[gust] interrupted: %s" % e, flush=True)
    except Exception as e:
        print("[gust] !! %r" % e, flush=True); raise
    finally:
        safe_clear("exit")
        print("[gust] final clear sent, exit %d" % rc, flush=True)
    sys.exit(rc)
