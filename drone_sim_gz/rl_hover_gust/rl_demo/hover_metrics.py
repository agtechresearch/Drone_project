"""Shared hover-gust definitions for training AND evaluation (2026-11-03 v2, codex hover-reference-contract /
hover-transition-time / hover-terminal-observation). Python 3.6, standard library only.

Task: hold the hover point reached BEFORE any correction acts. The reference is the mean TRUE lateral position (gz x = PX4
east) over the last REF_PRE_S sim seconds before ready. The worker holds the nominal target with delta = 0 for
ready.hold_s >= REF_PRE_S before writing ready, so no action influences it. The nominal-target arrival error
(ref - nominal east) is kept separately as a diagnostic. It is not part of the task error.

Decisions: the worker takes policy decision j at the first control cycle with sim t >= t_ready + j * DT (DT = 1 / POLICY_HZ),
and stores the boundary index b and the actual sim time t_dec. Transition j lasts [t_dec[j], t_dec[j + 1]). The last
transition ends at t_stop, the sim time at which the window ended (time limit) or the termination was detected. The
terminal observation is built at t_stop (terminal.json). Rewards integrate the truth over exactly these intervals."""
import math

REF_PRE_S = 0.5
REF_MIN_SAMPLES = 10
DT_MIN, DT_MAX = 0.04, 0.16          # allowed sim length of one transition (nominal 0.1 s at 20 Hz cycles)
PV_AGE_MAX = 0.2                     # wall s, age of the PX4 estimate used for a decision


def truth_ref(ts, xs, t_ready):
    """(reference, n samples) = mean truth over [t_ready - REF_PRE_S, t_ready]; (None, n) if too few samples"""
    v = [x for t, x in zip(ts, xs) if t_ready - REF_PRE_S <= t <= t_ready]
    if len(v) < REF_MIN_SAMPLES:
        return None, len(v)
    return sum(v) / len(v), len(v)


def transition_problems(t_dec, b, t_stop, pv_age):
    """reasons the decision timing breaks the transition contract ([] = ok)"""
    p = []
    if not t_dec:
        return ["no decision"]
    if any(bb != j for j, bb in enumerate(b)):
        k = next(j for j, bb in enumerate(b) if bb != j)
        p.append("decision boundary skipped at step %d (b %d)" % (k, b[k]))
    ends = list(t_dec[1:]) + [t_stop]
    dts = [e - s for s, e in zip(t_dec, ends)]
    if min(dts[:-1] or [DT_MIN]) < DT_MIN or max(dts[:-1] or [DT_MIN]) > DT_MAX:
        p.append("transition length %.3f..%.3f s outside [%.2f, %.2f]" % (min(dts[:-1]), max(dts[:-1]), DT_MIN, DT_MAX))
    if not (0.0 < dts[-1] <= DT_MAX):
        p.append("last transition %.3f s (t_stop %.3f)" % (dts[-1], t_stop))
    if pv_age and max(pv_age) > PV_AGE_MAX:
        p.append("estimate age %.3f s > %.2f at a decision" % (max(pv_age), PV_AGE_MAX))
    return p


def outcome(status, terminated, problems):
    """completed (time limit reached, valid) | task_failed (terminated: |e| > e_max) | invalid (worker / data problem)"""
    if status != "ok" or problems:
        return "invalid"
    return "task_failed" if terminated else "completed"
