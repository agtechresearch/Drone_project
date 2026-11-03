#!/usr/bin/env python3
"""grid_verify.settled(): one age limit on every branch (2026-10-31, codex verifier-truth-coverage reply-2). exit = failures"""
import os, sys
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "scripts"))
import grid_verify as V  # noqa: E402
F = 0
T = [(t, 0.0, 0.0, 0.0) for t in (0.0, 0.5, 13.0)]
for name, te, exp in (("closest within 0.6 s", 0.55, 0.5), ("next sample 3 s after [END] (old 'later' branch)", 10.0, None),
                      ("next sample 1.5 s after [END] (old 'after' branch)", 11.5, None), ("next sample 0.8 s after", 12.2, 13.0),
                      ("previous sample 0.9 s before", 1.4, 0.5), ("previous sample 1.1 s before", 1.6, None),
                      ("no truth at all", 5.0, None)):
    r = V.settled(T if name != "no truth at all" else [], te); got = None if r is None else r[0]
    ok = got == exp; F += not ok
    print("%s  %-50s [END] %.2f -> %s (age limit %.1f s)" % ("PASS" if ok else "FAIL", name, te, got, V.MAX_END_AGE))
print("%d failure(s)" % F); sys.exit(F)
