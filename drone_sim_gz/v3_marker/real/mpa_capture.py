#!/usr/bin/env python3
"""Record raw MPA pipe data on the VOXL 2 for the real-sensor check (plans/2026-10-31_real_sensor_check_plan.md, C and E).
Python 3.6, standard library only. NOT YET RUN ON THE VEHICLE: start with --probe.

  python3 mpa_capture.py --probe tof_pc                  # pipe info + the first bytes (check the client protocol and format)
  python3 mpa_capture.py tof_pc 5 C1_rack_0p37m          # 5 s of tof_pc  -> C1_rack_0p37m_tof_pc.bin + .idx + .info.json
  python3 mpa_capture.py tag_detections 10 E2_tag_0p5m   # 10 s of tag detections
  python3 mpa_capture.py tof_conf 5 C1_rack_0p37m        # any pipe (camera pipes are large: keep it short)

How (libmodal_pipe client protocol): /run/mpa/<pipe>/ holds "info" (JSON: type, size, ...) and a "request" FIFO. The client
writes its own name into "request"; the server then creates the FIFO /run/mpa/<pipe>/<name> and streams into it. Bytes are
saved exactly as they arrive (.bin) with one index line per read (.idx: host monotonic time, byte offset, length), so the
framing is done later on the PC (mpa_parse.py) and a wrong guess about the format loses nothing.
If the protocol differs on this voxl-suite, --probe shows what the pipe directory contains; the voxl-inspect-* tools and
voxl-logger are the fallback.

Result status (written to <prefix>_<pipe>.status.json and as the exit code; 2026-11-03, Codex mpa-empty-capture-success):
  0 ok           connected and received > 0 bytes (for record: until the end of the window). Format is NOT verified here.
  2 no_pipe      /run/mpa/<pipe> missing
  3 connect_fail request FIFO could not be opened/written, or the server did not create the client FIFO in time
  4 no_data      connected but 0 bytes arrived within the window (writer never opened, or opened and sent nothing)
  5 eof_early    data arrived, then the server closed the FIFO before the window ended (data kept)
"""
import json, os, select, sys, time

MPA = os.environ.get("MPA_ROOT", "/run/mpa")   # override only for the offline test harness
RC = {"ok": 0, "no_pipe": 2, "connect_fail": 3, "no_data": 4, "eof_early": 5}


class Fail(Exception):
    def __init__(self, status, msg):
        Exception.__init__(self, msg); self.status = status


def info(pipe):
    p = os.path.join(MPA, pipe, "info")
    try:
        return json.load(open(p))
    except (OSError, ValueError) as e:
        return {"error": "%s: %s" % (p, e)}


def connect(pipe, timeout=3.0):
    d = os.path.join(MPA, pipe)
    if not os.path.isdir(d):
        raise Fail("no_pipe", "no pipe %s (ls %s: %s)" % (d, MPA, sorted(os.listdir(MPA)) if os.path.isdir(MPA) else "MISSING"))
    name = "mpacap%d" % os.getpid()
    try:
        req = os.open(os.path.join(d, "request"), os.O_WRONLY | os.O_NONBLOCK)
        os.write(req, (name + "\0").encode()); os.close(req)
    except OSError as e:
        raise Fail("connect_fail", "request FIFO: %s" % e)
    path = os.path.join(d, name); t0 = time.time()
    while not os.path.exists(path):
        if time.time() - t0 > timeout:
            raise Fail("connect_fail", "server did not create %s within %.0f s; pipe dir: %s" % (path, timeout, os.listdir(d)))
        time.sleep(0.02)
    return os.open(path, os.O_RDONLY | os.O_NONBLOCK), path


def read_window(fd, secs, max_bytes=None, sink=None):
    """Read until secs elapse (or max_bytes). A 0-byte read before any data means no writer yet: keep waiting.
    A 0-byte read after data means the writer closed (EOF): stop."""
    t0 = time.monotonic(); off = 0; n = 0; eof = False; first = None
    while time.monotonic() - t0 < secs and (max_bytes is None or off < max_bytes):
        r, _, _ = select.select([fd], [], [], 0.5)
        if not r:
            continue
        b = os.read(fd, 1 << 20)
        if not b:
            if off > 0:
                eof = True; break
            time.sleep(0.05); continue
        if first is None:
            first = time.monotonic() - t0
        if sink:
            sink(b, off)
        off += len(b); n += 1
    return {"bytes": off, "reads": n, "eof": eof, "elapsed": time.monotonic() - t0, "first_data_s": first}


def judge(res, full_window):
    if res["bytes"] == 0:
        return "no_data"
    if res["eof"] and full_window:
        return "eof_early"
    return "ok"


def finish(prefix, pipe, meta):
    meta["rc"] = RC[meta["status"]]
    json.dump(meta, open("%s_%s.status.json" % (prefix, pipe), "w"), indent=1)
    print("STATUS %s rc=%d %s" % (meta["status"], meta["rc"], meta.get("msg", "")))
    return meta["rc"]


def probe(pipe):
    d = os.path.join(MPA, pipe)
    meta = {"mode": "probe", "pipe": pipe, "start_wall": time.time(), "info": info(pipe), "format_verified": False}
    print("pipe dir:", d, sorted(os.listdir(d)) if os.path.isdir(d) else "MISSING")
    print("info:", json.dumps(meta["info"], indent=1))
    try:
        fd, path = connect(pipe)
    except Fail as e:
        meta.update(status=e.status, msg=str(e)); return finish("probe", pipe, meta)
    buf = bytearray()
    res = read_window(fd, 3.0, max_bytes=4096, sink=lambda b, off: buf.extend(b))
    os.close(fd)
    meta.update(res, client_fifo=path, status=judge(res, full_window=False))
    print("read %d bytes in %d reads, %.1f s from %s (eof=%s)" % (res["bytes"], res["reads"], res["elapsed"], path, res["eof"]))
    for i in range(0, min(len(buf), 256), 16):
        print("%04x  %s" % (i, " ".join("%02x" % c for c in buf[i:i + 16])))
    return finish("probe", pipe, meta)


def record(pipe, secs, tag):
    meta = {"mode": "record", "pipe": pipe, "info": info(pipe), "start_wall": time.time(), "start_mono": time.monotonic(),
            "secs": secs, "format_verified": False}
    try:
        fd, path = connect(pipe)
    except Fail as e:
        meta.update(status=e.status, msg=str(e)); return finish(tag, pipe, meta)
    out = open("%s_%s.bin" % (tag, pipe), "wb"); idx = open("%s_%s.idx" % (tag, pipe), "w")

    def sink(b, off):
        out.write(b); idx.write("%.6f %d %d\n" % (time.monotonic(), off, len(b)))
    res = read_window(fd, secs, sink=sink)
    os.close(fd); out.close(); idx.close()
    meta.update(res, client_fifo=path, end_mono=time.monotonic(), status=judge(res, full_window=True))
    json.dump(meta, open("%s_%s.info.json" % (tag, pipe), "w"), indent=1)
    print("%s: %d bytes in %d reads (%.1f s, eof=%s) -> %s_%s.bin" % (pipe, res["bytes"], res["reads"], res["elapsed"], res["eof"], tag, pipe))
    return finish(tag, pipe, meta)


if __name__ == "__main__":
    a = sys.argv[1:]
    if len(a) == 2 and a[0] == "--probe":
        sys.exit(probe(a[1]))
    elif len(a) == 3:
        sys.exit(record(a[0], float(a[1]), a[2]))
    else:
        sys.exit(__doc__)
