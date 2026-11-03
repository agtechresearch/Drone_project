#!/usr/bin/env python3
"""Live 2-D minimap of a --wk-loc flight (v3_marker, 2026-10-30). Run on the HOST:
  QVIX_YAW_DEG=7 python3 minimap_server.py [instance=2] [port=8765]      -> http://127.0.0.1:8765/
Serves minimap.html, the rack map (digital twin from qvix_grid + gz/tags_map.json, MAP frame = rack frame) and, for the
newest run of the instance (logs/.last_run_i<N>), the estimator log loc.jsonl (estimated pose, sd, last tag / ToF update)
plus the simulator ground truth (tags.csv true pose, world -> map). On the real vehicle only the estimate exists."""
import json, math, os, sys
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse, parse_qs

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import qvix_grid as G  # noqa: E402

N = int(sys.argv[1]) if len(sys.argv) > 1 else 2
PORT = int(sys.argv[2]) if len(sys.argv) > 2 else 8765
LOGS = os.path.join(HERE, "..", "logs")
TAGS = json.load(open(os.path.join(HERE, "..", "gz", "tags_map.json")))["tags"]


def map_json():
    racks = []
    for room, r in G.ROOMS.items():
        for x0, x1 in G.RACKS_X:
            racks.append({"room": room, "x0": x0, "x1": x1, "y0": r["racks_y"][0], "y1": r["racks_y"][1]})
    return {"rooms": {k: {"y": v["y"], "racks_y": v["racks_y"]} for k, v in G.ROOMS.items()}, "x_max": G.X_MAX,
            "aisles": {k: list(v) for k, v in G.AISLES.items()}, "racks": racks,
            "tags": [{"id": int(k), "x": v["x"], "y": v["y"], "z": v["z"], "face": v.get("face", "")} for k, v in TAGS.items()]}


def run_dir():
    try:
        return open(os.path.join(LOGS, ".last_run_i%d" % N)).read().strip()
    except OSError:
        return None


def last_line(path):
    try:
        with open(path, "rb") as f:
            f.seek(0, 2); size = f.tell(); f.seek(max(0, size - 400))
            lines = f.read().decode(errors="ignore").strip().split("\n")
            return lines[-1] if lines else None
    except OSError:
        return None


def truth(d):
    line = last_line(os.path.join(d, "tags.csv"))
    if not line or line.startswith("t_sim"):
        return None
    p = line.split(",")
    try:
        t, x, y, yaw = float(p[0]), float(p[-3]), float(p[-2]), float(p[-1])
    except (ValueError, IndexError):
        return None
    mx, my, _ = G.world_to_model(x, y)
    return {"t": t, "x": mx, "y": my, "comp": (90.0 - (math.degrees(yaw) - G.MODEL_YAW_DEG)) % 360.0}


class H(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def send(self, body, ctype="application/json"):
        b = body.encode() if isinstance(body, str) else body
        self.send_response(200); self.send_header("Content-Type", ctype); self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(b))); self.end_headers(); self.wfile.write(b)

    def do_GET(self):
        u = urlparse(self.path); q = parse_qs(u.query)
        if u.path == "/":
            return self.send(open(os.path.join(HERE, "minimap.html"), "rb").read(), "text/html; charset=utf-8")
        if u.path == "/map":
            return self.send(json.dumps(map_json()))
        if u.path == "/state":
            d = run_dir(); since = int(q.get("since", ["0"])[0])
            out = {"run": os.path.basename(d) if d else None, "recs": [], "n": since, "truth": None, "cond": ""}
            if d:
                p = os.path.join(d, "loc.jsonl")
                if os.path.exists(p):
                    lines = open(p).read().strip().split("\n") if os.path.getsize(p) else []
                    if since > len(lines):
                        since = 0                                # a new run started
                    out["recs"] = [json.loads(s) for s in lines[since:] if s]; out["n"] = len(lines)
                out["truth"] = truth(d)
                try:
                    out["cond"] = open(os.path.join(d, "condition.txt")).read().split("\n")[0]
                except OSError:
                    pass
            return self.send(json.dumps(out))
        self.send_response(404); self.end_headers()


print("minimap: http://127.0.0.1:%d/  (instance %d, QVIX_YAW_DEG=%g)" % (PORT, N, G.MODEL_YAW_DEG), flush=True)
ThreadingHTTPServer(("127.0.0.1", PORT), H).serve_forever()
