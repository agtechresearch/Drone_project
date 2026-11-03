#!/usr/bin/env python3
"""Offline test of mpa_capture.py status/exit codes against a fake MPA server (2026-11-03, Codex mpa-empty-capture-success).
No VOXL, no real MPA. Runs mpa_capture.py as a subprocess with MPA_ROOT pointing to a temp dir.
  python3 test_mpa_capture.py            -> prints one line per case, exit 0 if all expected statuses match."""
import json, os, subprocess, sys, tempfile, threading, time

HERE = os.path.dirname(os.path.abspath(__file__))
CAP = os.path.join(HERE, "mpa_capture.py")


def fake_server(d, mode, stop):
    """mode: stream | silent | eof_once | no_fifo"""
    req = os.path.join(d, "request")
    fd = os.open(req, os.O_RDONLY | os.O_NONBLOCK)
    name = b""
    while not stop.is_set() and b"\0" not in name:
        try:
            name += os.read(fd, 256)
        except BlockingIOError:
            pass
        time.sleep(0.01)
    os.close(fd)
    if stop.is_set() or mode == "no_fifo":
        return
    p = os.path.join(d, name.split(b"\0")[0].decode())
    os.mkfifo(p)
    w = os.open(p, os.O_WRONLY)          # blocks until the client opens for reading
    if mode == "stream":
        while not stop.is_set():
            try:
                os.write(w, b"\xab" * 64)
            except BrokenPipeError:
                break
            time.sleep(0.05)
    elif mode == "eof_once":
        os.write(w, b"\xcd" * 64)
    elif mode == "silent":
        stop.wait()
    os.close(w)


def run_case(mode, args, expect):
    root = tempfile.mkdtemp(prefix="mpa_test_")
    work = tempfile.mkdtemp(prefix="mpa_work_")
    stop = threading.Event(); th = None
    if mode != "no_pipe":
        d = os.path.join(root, "tof_pc"); os.mkdir(d)
        json.dump({"name": "tof_pc", "type": "fake"}, open(os.path.join(d, "info"), "w"))
        os.mkfifo(os.path.join(d, "request"))
        th = threading.Thread(target=fake_server, args=(d, mode, stop), daemon=True); th.start()
        time.sleep(0.1)
    env = dict(os.environ, MPA_ROOT=root)
    r = subprocess.run([sys.executable, CAP] + args, cwd=work, env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                       timeout=30)
    stop.set()
    st = [f for f in os.listdir(work) if f.endswith(".status.json")]
    status = json.load(open(os.path.join(work, st[0])))["status"] if st else None
    ok = (r.returncode == expect[1] and status == expect[0])
    print("%-4s %-9s %-34s rc=%d status=%s (expect %s rc=%d) files=%s" % (
        "PASS" if ok else "FAIL", mode, " ".join(args), r.returncode, status, expect[0], expect[1], sorted(os.listdir(work))))
    if not ok:
        print(r.stdout.decode())
    return ok


if __name__ == "__main__":
    cases = [
        ("stream",   ["--probe", "tof_pc"],        ("ok", 0)),
        ("silent",   ["--probe", "tof_pc"],        ("no_data", 4)),
        ("eof_once", ["--probe", "tof_pc"],        ("ok", 0)),        # probe: some bytes then EOF is still "received"
        ("no_fifo",  ["--probe", "tof_pc"],        ("connect_fail", 3)),
        ("no_pipe",  ["--probe", "tof_pc"],        ("no_pipe", 2)),
        ("stream",   ["tof_pc", "2", "T"],         ("ok", 0)),
        ("silent",   ["tof_pc", "2", "T"],         ("no_data", 4)),
        ("eof_once", ["tof_pc", "2", "T"],         ("eof_early", 5)),
        ("no_fifo",  ["tof_pc", "2", "T"],         ("connect_fail", 3)),
        ("no_pipe",  ["tof_pc", "2", "T"],         ("no_pipe", 2)),
    ]
    res = [run_case(*c) for c in cases]
    print("ALL PASS" if all(res) else "SOME FAIL", "%d/%d" % (sum(res), len(res)))
    sys.exit(0 if all(res) else 1)
