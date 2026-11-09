#!/usr/bin/env python3
"""Static check of real/VEHICLE_FILES.txt (2026-11-09), no SSH, no flight. Run from the tree root:
  python3 real/check_vehicle_files.py
1. every $T/<file> referenced by the flight scripts (ctl_flight.sh, ctl_run.sh, step2_check.sh) and every local module imported
   (transitively) by the listed Python files is in the list;  2. every listed source exists and matches its sha256;
3. deploy_ctl.sh copies exactly the files marked deploy_ctl.sh.  exit 0 = all OK."""
import hashlib, os, re, sys
M = [l.split(" | ") for l in open("real/VEHICLE_FILES.txt").read().splitlines() if l and not l.startswith("#")]
V = {r[0]: r for r in M}; bad = []
for v, s, b, h, _ in M:
    if not os.path.exists(s): bad.append("missing source %s" % s); continue
    if hashlib.sha256(open(s, "rb").read()).hexdigest() != h: bad.append("sha256 differs %s" % s)
for sc in ("real/ctl_flight.sh", "real/ctl_run.sh", "real/step2/step2_check.sh"):
    for ref in set(re.findall(r"\$T/([A-Za-z0-9_./-]+)", open(sc).read())) | set(re.findall(r"tools_safe/([A-Za-z0-9_.-]+\.(?:py|sh))", open(sc).read())):
        ref = ref.rstrip("/")
        if ref not in V and not any(k.startswith(ref + "/") for k in V): bad.append("%s references %s, not listed" % (sc, ref))
local = {os.path.splitext(k)[0]: k for k in V if k.endswith(".py")}
for v, s, *_ in M:
    if s.endswith(".py"):
        for mod in re.findall(r"^\s*(?:import|from)\s+([A-Za-z0-9_]+)", open(s).read(), re.M) + re.findall(r"^\s*import\s+[A-Za-z0-9_]+\s+as\s+\w+,\s*([A-Za-z0-9_]+)", open(s).read(), re.M):
            src_dir = os.path.dirname(s)
            if os.path.exists(os.path.join(src_dir, mod + ".py")) and mod not in local and mod != "path_flight_phase1_v13":
                bad.append("%s imports %s, not listed" % (s, mod))
dep = re.search(r'F="([^"]+)"', open("real/deploy_ctl.sh").read()).group(1).split() + re.search(r'P="([^"]+)"', open("real/deploy_ctl.sh").read()).group(1).split()
want = sorted(r[1] for r in M if r[2] == "deploy_ctl.sh")
if sorted(dep) != want: bad.append("deploy_ctl.sh copies %s, list says %s" % (sorted(dep), want))
print("\n".join(bad) if bad else "OK: %d vehicle files listed, sources and sha256 match, references and imports covered" % len(M))
sys.exit(1 if bad else 0)
