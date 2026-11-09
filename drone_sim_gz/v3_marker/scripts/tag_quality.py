"""Tag detection quality contract (2026-11-01, codex real-tag-quality-contract). One rule for every consumer of a detection
{"id", "t_cam", "R", "margin", "hamming"} (wk_mission / loc2d online, survey_map offline):
  margin   finite number = the decoder's decision margin (pupil_apriltags in the sim, tag_detect.py)
           null / missing = UNAVAILABLE: the real VOXL tag_detection_t carries no decision margin (real/mpa_parse.py writes
           null, "quality": "unavailable"); never compared as a number, never replaced by a made-up high value
  hamming  int = corrected bits (0 required); null / missing = unavailable as well
Status: "ok" (margin >= min and hamming 0) | "low" (a number below the limit / hamming > 0) | "unavailable" | "invalid"
(margin or hamming present but not a finite number / an int).
Policy for "unavailable": WK_TAG_QUALITY_UNAVAILABLE = reject (default) | accept. "accept" is an explicit operator choice
for the real detector (its own internal checks only); the consumers count those detections separately (COUNT), so a run
log shows how many updates had no quality value. "low" and "invalid" are always rejected."""
import math
import os

POLICY = os.environ.get("WK_TAG_QUALITY_UNAVAILABLE", "reject")
if POLICY not in ("reject", "accept"):
    raise SystemExit("WK_TAG_QUALITY_UNAVAILABLE must be reject or accept, not %r" % POLICY)
COUNT = {"ok": 0, "low": 0, "unavailable": 0, "invalid": 0, "unavailable_accepted": 0}


def _num(v):
    return isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)


def status(det, min_margin):
    m, h = det.get("margin"), det.get("hamming")
    if (m is not None and not _num(m)) or (h is not None and (not isinstance(h, int) or isinstance(h, bool))):
        return "invalid"
    if m is None or h is None:
        return "unavailable"
    return "ok" if m >= min_margin and h == 0 else "low"


def ok(det, min_margin, policy=None):
    """True when the detection may be used; counts every decision in COUNT"""
    s = status(det, min_margin); COUNT[s] += 1
    if s == "unavailable" and (policy or POLICY) == "accept":
        COUNT["unavailable_accepted"] += 1
        return True
    return s == "ok"
