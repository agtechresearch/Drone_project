#!/usr/bin/env python3
"""real_flight_safe.py + the east correction in v13's take-off hover (2026-11-04). Python 3.6, MAVSDK 0.12. Nothing of v13 or
of the safety layer is edited: this imports v13, installs ctl_patch (DroneController.hover for "post_takeoff" only) and then
runs real_flight_safe.main() with the remaining arguments unchanged.

  python3 real_ctl_safe.py --ctrl zero|ppo [--policy FILE.json] [--log-dir DIR] [--settle-s S] [--end-s S] [--e-stop M]
                           [--stale-s S] [--yaw-max DEG] <real_flight_safe.py arguments: --v13 DIR --own-server ... -- v13 args>

The controller options come first; the first argument that is not one of them starts real_flight_safe's arguments.
Refuses (nothing started) when: --ctrl ppo without a readable policy, the policy's obs_dim / delta limits are wrong."""
import os, sys

HERE = os.path.dirname(os.path.abspath(__file__))
OPTS = {"--ctrl": ("ctrl", str), "--policy": ("policy", str), "--log-dir": ("log_dir", str), "--settle-s": ("settle_s", float),
        "--end-s": ("end_s", float), "--e-stop": ("e_stop_m", float), "--stale-s": ("stale_s", float),
        "--yaw-max": ("yaw_max_deg", float)}


def split(argv):
    o = {}; i = 0
    while i < len(argv) and argv[i] in OPTS:
        k, typ = OPTS[argv[i]]
        if i + 1 >= len(argv):
            sys.exit("real_ctl_safe: %s needs a value (nothing started)" % argv[i])
        o[k] = typ(argv[i + 1]); i += 2
    return o, argv[i:]


def v13_dir(rest):
    if "--v13" in rest:
        return rest[rest.index("--v13") + 1]
    for d in (HERE, os.path.join(HERE, "..", "..")):
        if os.path.exists(os.path.join(d, "path_flight_phase1_v13.py")):
            return d
    sys.exit("real_ctl_safe: path_flight_phase1_v13.py not found (use --v13 <dir>) - nothing started")


def main(argv):
    o, rest = split(argv)
    if o.get("ctrl") not in ("zero", "ppo"):
        sys.exit("real_ctl_safe: --ctrl zero|ppo is required (nothing started)")
    if o["ctrl"] == "ppo" and not (o.get("policy") and os.path.isfile(o["policy"])):
        sys.exit("real_ctl_safe: --ctrl ppo needs --policy <file.json> (got %r) - nothing started" % o.get("policy"))
    if o["ctrl"] == "zero" and o.get("policy"):
        sys.exit("real_ctl_safe: --policy given with --ctrl zero - nothing started")
    sys.path.insert(0, os.path.abspath(v13_dir(rest)))
    sys.path.insert(0, os.path.join(HERE, "..", "scripts")); sys.path.insert(0, HERE)   # host layout; on the VOXL all in one dir
    import path_flight_phase1_v13 as fs
    import ctl_patch
    try:
        ctl_patch.install(fs, o)
    except Exception as e:  # noqa: BLE001
        sys.exit("real_ctl_safe: controller not installed (%s: %s) - nothing started" % (type(e).__name__, e))
    import real_flight_safe
    return real_flight_safe.main(rest)


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
