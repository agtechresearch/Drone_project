#!/usr/bin/env python3
"""C3: optimise the C2 rule's parameters on the surrogate (differential evolution), sensing = tof or marker.
  venv/bin/python experiments/opt_rule.py tof   -> results/rule_opt_tof.json"""
import json, os, sys, time
from scipy.optimize import differential_evolution
from common import ROOT, S, C, RULE_KEYS, RULE_BOUNDS, train_set, cost, run_set
sensing = sys.argv[1] if len(sys.argv) > 1 else "tof"
SCS = train_set(48, sensing=sensing)

def params(x):
    return dict(zip(RULE_KEYS, x))

def f(x):
    p = params(x)
    return cost(run_set(lambda: C.make("rule", **p), SCS))

if __name__ == "__main__":
    t0 = time.time()
    base = f([C.Rule.DEFAULT[k] for k in RULE_KEYS])
    print("default rule cost %.3f" % base, flush=True)
    hist = []

    def log_gen(xk, convergence=0):
        hist.append(float(f(xk)))
        print("gen %d cost %.3f" % (len(hist), hist[-1]), flush=True)
        return False          # a truthy return value stops differential_evolution
    r = differential_evolution(f, RULE_BOUNDS, maxiter=30, popsize=8, tol=1e-3, seed=7, workers=16, updating="deferred", polish=False,
                               callback=log_gen)
    best = params(r.x); best["n"] = int(round(best["n"])); best["win"] = int(round(best["win"]))
    out = {"sensing": sensing, "best": best, "cost": float(r.fun), "default_cost": base, "history": hist, "nfev": int(r.nfev), "seconds": time.time() - t0}
    json.dump(out, open(os.path.join(ROOT, "results", "rule_opt_%s.json" % sensing), "w"), indent=1)
    print(json.dumps(out, indent=1))
