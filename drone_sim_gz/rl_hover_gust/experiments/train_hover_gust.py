#!/usr/bin/env python3
"""PPO trainer for the hover-gust task on the live Gazebo + PX4 SITL (2026-11-03, stage B/C). Host venv (SB3 2.3.2, torch CPU).
  v3_marker/venv/bin/python experiments/train_hover_gust.py --batch <batch_dir> --config <cfg> --updates U --per-update K --seed S
Runs NEXT TO a hover_gust_run.sh batch whose plan.json (written by --make-plan) lists U*K training episodes; episode i flies
policy version i // K. Loop for u = 0..U-1:
  1. export policy_v{u}.json (pure-Python weights, rl_demo/policy_runtime.py) -> the worker may take off with it
  2. wait until the K episodes of version u are finished (episode.json) - the worker waits for policy_v{u+1}.json before the next
     take-off, so this update never overlaps a flight (timeline.csv: update start/end vs the episodes' take-off/disarm)
  3. rewards from the simulator TRUTH (gt.csv; never part of the observation): rl_demo.policy_runtime.reward_step per policy step;
     terminated window (estimate |e| > e_max) = terminal with terminal_penalty; full window = time limit, bootstrapped with
     gamma * V(terminal observation) (SB3's truncation rule; v2: terminal.json = the observation AFTER the last action)
  4. episodes that are not valid (worker status, window RTF < valid.rtf_min, truth gap, fewer policy steps than the window
     without a termination) are NOT used and listed with the reason (excluded.csv)
  5. RolloutBuffer of exactly the valid steps; the logged worker log-prob / value are checked against torch (max |diff| into
     updates.csv; > 1e-3 stops the training); compute_returns_and_advantage; PPO.train(); save model_v{u+1}.zip
Exit: trainer_done (or trainer_failed with the reason)."""
import argparse, csv, json, math, os, random, sys, time, traceback
HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.dirname(HERE); sys.path.insert(0, ROOT)
import numpy as np, torch, gymnasium as gym
from stable_baselines3 import PPO
from stable_baselines3.common.buffers import RolloutBuffer
from stable_baselines3.common.logger import configure
from rl_demo.contracts import load_config, gust_schedule
from rl_demo.policy_runtime import OBS_DIM, POLICY_HZ, reward_step, MLPPolicy
from rl_demo import hover_metrics as HM

STEPS_PER_EP = None


class SpecEnv(gym.Env):
    """spaces only: PPO needs an env at construction; it is never stepped (data comes from the Gazebo worker)"""
    observation_space = gym.spaces.Box(-np.inf, np.inf, (OBS_DIM,), np.float32)
    action_space = gym.spaces.Box(-1.0, 1.0, (1,), np.float32)

    def reset(self, seed=None, options=None):
        raise RuntimeError("SpecEnv is not a simulator")

    def step(self, a):
        raise RuntimeError("SpecEnv is not a simulator")


def export_policy(model, path, version):
    sd = {k: v.detach().cpu().numpy().tolist() for k, v in model.policy.state_dict().items()}
    d = {"version": version, "obs_dim": OBS_DIM, "log_std": sd["log_std"],
         "pi0_w": sd["mlp_extractor.policy_net.0.weight"], "pi0_b": sd["mlp_extractor.policy_net.0.bias"],
         "pi2_w": sd["mlp_extractor.policy_net.2.weight"], "pi2_b": sd["mlp_extractor.policy_net.2.bias"],
         "vf0_w": sd["mlp_extractor.value_net.0.weight"], "vf0_b": sd["mlp_extractor.value_net.0.bias"],
         "vf2_w": sd["mlp_extractor.value_net.2.weight"], "vf2_b": sd["mlp_extractor.value_net.2.bias"],
         "act_w": sd["action_net.weight"], "act_b": sd["action_net.bias"], "val_w": sd["value_net.weight"], "val_b": sd["value_net.bias"]}
    tmp = path + ".tmp"; json.dump(d, open(tmp, "w")); os.replace(tmp, path)


def make_plan(cfg, updates, per_update, seed, p_none=0.2):
    r = random.Random(seed); eps = []
    for i in range(updates * per_update):
        cond = "none" if r.random() < p_none else "gust"
        eps.append({"id": i, "cond": cond, "seed": 100000 + seed * 10000 + i,
                    "ctrl": {"type": "ppo", "policy": "policy/policy_v%03d.json" % (i // per_update), "deterministic": False}})
    for e in eps:
        gust_schedule(cfg, e["cond"], e["seed"])
    return {"episodes": eps, "train": {"seed": seed, "updates": updates, "per_update": per_update}}


class GT(object):
    """incremental reader of the growing gt.csv"""
    def __init__(self, path):
        self.path = path; self.off = 0; self.t = []; self.x = []; self.yaw = []; self.hdr = None

    def update(self):
        if not os.path.exists(self.path):
            return
        with open(self.path) as f:
            f.seek(self.off)
            while True:
                pos = f.tell(); line = f.readline()
                if not line or not line.endswith("\n"):
                    self.off = pos; break
                if self.hdr is None:
                    self.hdr = line.strip().split(","); continue
                v = line.split(","); self.t.append(float(v[0])); self.x.append(float(v[2])); self.yaw.append(float(v[9]))
        self.ta = np.array(self.t); self.xa = np.array(self.x); self.ya = np.array(self.yaw)

    def x_at(self, t):
        return np.interp(t, self.ta, self.xa)

    def max_gap(self, t0, t1):
        seg = self.ta[(self.ta >= t0) & (self.ta <= t1)]          # inside [t0, t1] plus the distance to both edges
        if len(seg) < 2:
            return float("inf")
        return float(max(np.max(np.diff(seg)), seg[0] - t0, t1 - seg[-1]))


YAW_TOL_DEG = 3.0; EST_TOL_M = 0.03


def episode_data(B, ep, cfg, gt):
    """-> (valid, reason, dict with obs, a_raw, rewards, logp, value, terminal obs, info). Same function for training and for the
    evaluation reward (experiments/eval_hover_gust.py), so both use one reference and one transition definition.
    2026-11-03 hg_train_s1: with EKF2_EV_CTRL 7 the PX4 heading drifted up to 54 deg from the truth over ~40 take-offs on one boot,
    so the estimate 'east' no longer was the force axis -> an episode whose PX4 yaw differs from the truth yaw by > YAW_TOL_DEG
    at ready, or whose estimate - truth east offset moves by > EST_TOL_M in the window, is excluded.
    2026-11-03 v2 (codex review 17:52): reference = mean truth over the REF_PRE_S before ready (no action yet, hover_metrics);
    transitions = the worker's actual decision times, the last one ends at t_stop; time-limit bootstrap uses terminal.json."""
    D = os.path.join(B, "ep_%02d" % ep["id"])
    res = json.load(open(os.path.join(D, "episode.json")))
    if res.get("status") != "ok":
        return False, "worker status %s: %s" % (res.get("status"), res.get("reason")), None
    rows = list(csv.DictReader(open(os.path.join(D, "policy.csv"))))
    if not rows or "b" not in rows[0]:
        return False, "policy.csv without decision boundaries (pre-v2 worker)", None
    wend = json.load(open(os.path.join(D, "window_end"))); term = wend.get("terminated")
    tj = os.path.join(D, "terminal.json")
    if not os.path.exists(tj):
        return False, "no terminal.json", None
    tobs = json.load(open(tj))
    ctl = list(csv.DictReader(open(os.path.join(D, "control.csv"))))
    tw = [float(r["t_wall"]) for r in ctl]; ts = [float(r["t_sim"]) for r in ctl]
    rtf = (ts[-1] - ts[0]) / (tw[-1] - tw[0]) if len(ctl) > 10 and tw[-1] > tw[0] else 0.0
    if rtf < cfg.get("valid", {}).get("rtf_min", 0.9):
        return False, "window RTF %.3f < %.2f" % (rtf, cfg["valid"]["rtf_min"]), None
    t = np.array([float(r["t_sim"]) for r in rows]); t_stop = float(tobs["t_sim"])
    prob = HM.transition_problems(list(t), [int(r["b"]) for r in rows], t_stop, [float(r["pv_age"]) for r in rows])
    if prob:
        return False, "; ".join(prob), None
    full = int(round(cfg["window_s"] * POLICY_HZ))
    if not term and len(rows) != full:
        return False, "policy steps %d != %d without termination" % (len(rows), full), None
    t_ready = json.load(open(os.path.join(D, "ready")))["t_sim"]
    gap = gt.max_gap(t_ready - HM.REF_PRE_S, t_stop)
    if gap > 0.1:
        return False, "truth gap %.3f s" % gap, None
    yaw_true_ned = 90.0 - math.degrees(float(np.interp(t_ready, gt.ta, np.unwrap(gt.ya))))
    yerr = (res["yaw_deg"] - yaw_true_ned + 180.0) % 360.0 - 180.0
    if abs(yerr) > YAW_TOL_DEG:
        return False, "PX4 yaw - truth yaw %.2f deg at ready (tol %.1f)" % (yerr, YAW_TOL_DEG), None
    ref_e, nref = HM.truth_ref(gt.t, gt.x, t_ready)
    if ref_e is None:
        return False, "reference: %d truth samples in the %.1f s before ready" % (nref, HM.REF_PRE_S), None
    e_est = np.array([float(r["o0"]) for r in rows]) * 0.1 + res["nominal_ned"][1]      # o0 = (east est - nominal) / E_S
    d_et = e_est - gt.x_at(t); off0 = float(np.median(d_et[:5])); off = float(np.median(d_et))
    if abs(off - off0) > EST_TOL_M or float(np.max(np.abs(d_et - off0))) > 2 * EST_TOL_M:
        return False, "estimate - truth east offset moved in the window (start %.3f, median %.3f, max dev %.3f m)" % (
            off0, off, float(np.max(np.abs(d_et - off0)))), None
    obs = np.array([[float(r["o%d" % i]) for i in range(OBS_DIM)] for r in rows], dtype=np.float32)
    a_raw = np.array([float(r["a_raw"]) for r in rows], dtype=np.float32)
    delta = np.array([float(r["delta"]) for r in rows]); dd = np.diff(np.concatenate([[0.0], delta]))
    ends = np.concatenate([t[1:], [t_stop]]); rew = []
    for j in range(len(rows)):
        dtj = float(ends[j] - t[j]); tt = np.linspace(t[j], ends[j], 6); e = gt.x_at(tt) - ref_e
        e_mean = float(np.mean(e)); ve = float((e[-1] - e[0]) / dtj)
        rew.append(reward_step(e_mean, ve, float(delta[j]), float(dd[j]), cfg, dt=dtj))
    rew = np.array(rew, dtype=np.float32)
    sel = (gt.ta >= t_ready) & (gt.ta <= t_stop); e_true = gt.xa[sel] - ref_e      # same truth samples as hover_gust_eval
    info = {"id": ep["id"], "cond": ep["cond"], "steps": len(rows), "terminated": term, "return": float(rew.sum()),
            "e_rms": float(np.sqrt(np.mean(e_true ** 2))), "e_max": float(np.max(np.abs(e_true))),
            "delta_rms": float(np.sqrt(np.mean(delta ** 2))), "rtf": rtf, "yaw_err_deg": yerr, "est_off_m": off,
            "ref_e": ref_e, "ref_minus_nominal_m": ref_e - res["nominal_ned"][1], "t_stop": t_stop}
    return True, "", {"obs": obs, "a_raw": a_raw, "rew": rew, "logp": np.array([float(r["logp"]) for r in rows]),
                      "value": np.array([float(r["value"]) for r in rows]), "term": bool(term), "info": info,
                      "terminal_obs": np.array(tobs["obs"], dtype=np.float32), "terminal_value": float(tobs["value"]),
                      "version": rows[0]["version"] if rows else ""}


def bootstrap_rewards(rew, term, v_terminal, gamma, terminal_penalty):
    """terminated -> + terminal_penalty, no bootstrap; time limit -> + gamma * V(terminal observation after the last action)"""
    r = np.array(rew, dtype=np.float32).copy()
    r[-1] += terminal_penalty if term else gamma * v_terminal
    return r


def flight_path(p):
    return p.replace("/home/dronesim/starling_sim/", "/home/dronesim/workspace/", 1)


def main(a):
    """single batch (--batch): one hover_gust_run.sh batch flies all versions. Session (--session S --batch-pattern P): every
    update u gets its own freshly booted batch P % u (driver: logs/run_*_session.sh), the trainer writes S/plans/plan_uXXX.json
    (K episodes, local ids 0..K-1, policy = absolute path of S/policy/policy_vXXX.json) after exporting that version."""
    cfg = load_config(a.config)
    if a.session:
        B = a.session; plan = json.load(open(os.path.join(B, "plan.json")))
    else:
        B = a.batch; plan = json.load(open(os.path.join(B, "plan.json")))
    P = os.path.join(B, "policy"); os.makedirs(P, exist_ok=True); os.makedirs(os.path.join(B, "plans"), exist_ok=True)
    K = plan["train"]["per_update"]; U = plan["train"]["updates"]
    torch.manual_seed(a.seed); np.random.seed(a.seed)
    model = PPO("MlpPolicy", SpecEnv(), n_steps=K * int(cfg["window_s"] * POLICY_HZ), batch_size=a.batch_size, n_epochs=a.epochs,
                learning_rate=a.lr, gamma=a.gamma, gae_lambda=0.95, clip_range=0.2, ent_coef=0.0, seed=a.seed, device="cpu",
                policy_kwargs={"net_arch": {"pi": [64, 64], "vf": [64, 64]}, "activation_fn": torch.nn.Tanh, "log_std_init": a.log_std_init})
    if a.init_model:
        model.set_parameters(a.init_model, device="cpu")
    model.set_logger(configure(os.path.join(B, "sb3_log"), ["csv"]))
    export_policy(model, os.path.join(P, "policy_v000.json"), 0); model.save(os.path.join(P, "model_v000.zip"))
    gt = GT(os.path.join(B, "gt.csv")) if not a.session else None
    tl = open(os.path.join(B, "timeline.csv"), "a"); tl.write("t_wall,event,version\n"); tl.flush()
    up = open(os.path.join(B, "updates.csv"), "w")
    up.write("version,valid_eps,excluded_eps,steps,mean_return,mean_e_rms,mean_e_max,mean_delta_rms,terminated,logp_maxdiff,value_maxdiff,"
             "approx_kl,clip_fraction,policy_loss,value_loss,std,update_s\n"); up.flush()
    ex = open(os.path.join(B, "excluded.csv"), "w"); ex.write("version,episode,reason\n"); ex.flush()
    epi = open(os.path.join(B, "episodes_train.csv"), "w"); epi.write("version,id,cond,steps,terminated,return,e_rms,e_max,delta_rms,rtf,yaw_err_deg,est_off_m,ref_minus_nominal_m\n")
    for u in range(U):
        glob = [e for e in plan["episodes"] if e["id"] // K == u]
        if a.session:
            BU = a.batch_pattern % u
            mine = [dict(e, id=i, global_id=e["id"], ctrl=dict(e["ctrl"], policy=flight_path(os.path.join(P, "policy_v%03d.json" % u))))
                    for i, e in enumerate(glob)]
            pp = os.path.join(B, "plans", "plan_u%03d.json" % u)
            json.dump({"episodes": mine, "session": B, "update": u}, open(pp + ".tmp", "w"), indent=1); os.replace(pp + ".tmp", pp)
        else:
            BU, mine = B, glob
        while True:
            done = [os.path.exists(os.path.join(BU, "ep_%02d" % e["id"], "episode.json")) for e in mine]
            if all(done):
                break
            if os.path.exists(os.path.join(BU, "worker_done")) or os.path.exists(os.path.join(B, "driver_failed")):
                time.sleep(2.0)
                if all(os.path.exists(os.path.join(BU, "ep_%02d" % e["id"], "episode.json")) for e in mine):
                    break
                raise RuntimeError("batch of version %d ended before all its episodes were flown (worker_done/driver_failed)" % u)
            time.sleep(1.0)
        tl.write("%.3f,update_start,%d\n" % (time.time(), u)); tl.flush(); t_up = time.time()
        if a.session:
            for i in range(30):                       # gt_logger stops after the worker; take the file once it stopped growing
                if os.path.exists(os.path.join(BU, "gt.csv.STOP")):
                    break
                time.sleep(1.0)
            time.sleep(1.0); gt = GT(os.path.join(BU, "gt.csv"))
        else:
            time.sleep(1.0)
        gt.update()
        data = []
        for e in mine:
            ok, why, d = episode_data(BU, e, cfg, gt)
            if ok:
                data.append(d); i = d["info"]
                epi.write("%d,%d,%s,%d,%s,%.4f,%.5f,%.5f,%.5f,%.3f,%.2f,%.4f,%.4f\n" % (u, e.get("global_id", i["id"]), i["cond"], i["steps"],
                          i["terminated"] or "", i["return"], i["e_rms"], i["e_max"], i["delta_rms"], i["rtf"], i["yaw_err_deg"], i["est_off_m"],
                          i["ref_minus_nominal_m"])); epi.flush()
            else:
                ex.write("%d,%d,%s\n" % (u, e.get("global_id", e["id"]), why)); ex.flush()
        if not data:
            raise RuntimeError("version %d: no valid episode" % u)
        n = sum(len(d["rew"]) for d in data)
        buf = RolloutBuffer(n, model.observation_space, model.action_space, device="cpu", gae_lambda=0.95, gamma=a.gamma, n_envs=1)
        lpd = vd = 0.0
        for d in data:
            obs_t = torch.as_tensor(d["obs"]); act_t = torch.as_tensor(d["a_raw"]).reshape(-1, 1)
            with torch.no_grad():
                v_t, lp_t, _ = model.policy.evaluate_actions(obs_t, act_t)
            lpd = max(lpd, float(np.max(np.abs(lp_t.numpy() - d["logp"])))); vd = max(vd, float(np.max(np.abs(v_t.numpy().ravel() - d["value"]))))
            with torch.no_grad():
                v_term = float(model.policy.predict_values(torch.as_tensor(d["terminal_obs"][None])).item())
            vd = max(vd, abs(v_term - d["terminal_value"]))
            rew = bootstrap_rewards(d["rew"], d["term"], v_term, a.gamma, cfg["reward"]["terminal_penalty"])
            for j in range(len(rew)):
                buf.add(d["obs"][j:j + 1], d["a_raw"][j:j + 1].reshape(1, 1), np.array([rew[j]]), np.array([j == 0]),
                        v_t[j:j + 1].reshape(1, 1), lp_t[j:j + 1])
        if lpd > 1e-3 or vd > 1e-3:          # float32 torch vs float64 runtime: ~1e-5 expected; the buffer holds the torch values
            raise RuntimeError("worker log-prob/value differ from torch (%.2e / %.2e) - policy export or runtime mismatch" % (lpd, vd))
        buf.compute_returns_and_advantage(last_values=torch.zeros(1), dones=np.array([True]))
        model.rollout_buffer = buf; model._current_progress_remaining = 1.0 - u / float(U)
        model.train(); model.num_timesteps += n
        vals = {k: model.logger.name_to_value.get(k, float("nan")) for k in ("train/approx_kl", "train/clip_fraction", "train/policy_gradient_loss",
                                                                              "train/value_loss", "train/std")}
        model.logger.dump(step=model.num_timesteps)
        I = [d["info"] for d in data]
        up.write("%d,%d,%d,%d,%.4f,%.5f,%.5f,%.5f,%d,%.2e,%.2e,%.5f,%.4f,%.5f,%.5f,%.4f,%.1f\n" % (
            u, len(data), len(mine) - len(data), n, np.mean([i["return"] for i in I]), np.mean([i["e_rms"] for i in I]),
            np.mean([i["e_max"] for i in I]), np.mean([i["delta_rms"] for i in I]), sum(1 for i in I if i["terminated"]), lpd, vd,
            vals["train/approx_kl"], vals["train/clip_fraction"], vals["train/policy_gradient_loss"], vals["train/value_loss"],
            vals["train/std"], time.time() - t_up)); up.flush()
        model.save(os.path.join(P, "model_v%03d.zip" % (u + 1)))
        export_policy(model, os.path.join(P, "policy_v%03d.json" % (u + 1)), u + 1)
        tl.write("%.3f,update_end,%d\n" % (time.time(), u)); tl.flush()
        if a.session and u == U - 1:                  # last version: no batch flies it in training (evaluation does)
            pass
        print("[train] v%d: %d eps (%d excluded) %d steps, return %.2f e_rms %.4f, update %.1f s" % (
            u, len(data), len(mine) - len(data), n, np.mean([i["return"] for i in I]), np.mean([i["e_rms"] for i in I]), time.time() - t_up), flush=True)
    return 0


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--batch"); ap.add_argument("--config", required=True)
    ap.add_argument("--session"); ap.add_argument("--batch-pattern", help="host batch dir per update, e.g. .../logs/hg_s1_u%%03d")
    ap.add_argument("--make-plan", help="write the training plan.json to this path and exit")
    ap.add_argument("--updates", type=int, default=25); ap.add_argument("--per-update", type=int, default=5)
    ap.add_argument("--seed", type=int, default=1); ap.add_argument("--lr", type=float, default=3e-4)
    ap.add_argument("--gamma", type=float, default=0.99); ap.add_argument("--epochs", type=int, default=10)
    ap.add_argument("--batch-size", type=int, default=200); ap.add_argument("--log-std-init", type=float, default=-1.0)
    ap.add_argument("--init-model")
    a = ap.parse_args()
    if a.make_plan:
        json.dump(make_plan(load_config(a.config), a.updates, a.per_update, a.seed), open(a.make_plan, "w"), indent=1); sys.exit(0)
    try:
        rc = main(a)          # returns only after every planned update (failures raise -> trainer_failed, rc 1)
        json.dump({"rc": rc, "complete": rc == 0, "t_wall": time.time()}, open(os.path.join(a.session or a.batch, "trainer_done"), "w"))
        sys.exit(rc)
    except Exception as e:
        traceback.print_exc()
        json.dump({"reason": "%s: %s" % (type(e).__name__, e), "t_wall": time.time()}, open(os.path.join(a.session or a.batch, "trainer_failed"), "w"))
        sys.exit(1)
