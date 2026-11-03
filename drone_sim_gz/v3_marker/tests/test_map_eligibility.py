import json, os, sys, tempfile
sys.path.insert(0, os.path.expanduser("~/workspace/v3_marker/scripts"))
import wk_mission as W
base = json.load(open(os.path.expanduser("~/workspace/v3_marker/gz/tags_map.json")))
res = []
for mark, env in ((None, "0"), (True, "0"), (False, "0"), (False, "1")):
    j = dict(base)
    if mark is not None:
        j["observability"] = {"eligible": mark}
    p = os.path.join(tempfile.mkdtemp(), "m.json"); json.dump(j, open(p, "w"))
    os.environ["WK_MAP_ALLOW_INELIGIBLE"] = env
    wk = W.WallKeep(True, 0, os.path.join(tempfile.mkdtemp(), "wk.csv"), (2.9, 8.6))
    try:
        wk.enable_tags(0, p); res.append((mark, env, "flown", wk.map_eligible))
    except SystemExit as e:
        res.append((mark, env, "refused", str(e)[:60]))
for r in res: print(r)
ok = [r[2] for r in res] == ["flown", "flown", "refused", "flown"]
print("PASS" if ok else "FAIL", " WK_MAP eligibility: none/true flown, false refused, false + override flown")
print("%d failure(s)" % (0 if ok else 1))
