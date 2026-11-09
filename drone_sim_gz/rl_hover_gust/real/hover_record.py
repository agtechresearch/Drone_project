#!/usr/bin/env python3
"""Record several MPA pipes at once during a flight (2026-11-03, real hover stability / fan push). Python 3.6, standard library
only, runs ON THE VOXL. Read-only: it is a normal MPA client (the same protocol as mpa_capture.py, whose connect() it uses).
  python3 hover_record.py <out_dir> [--pipes tag_detections,vvhub_body_wrt_local,px4_vehicle_local_position] [--max-s 300]
Records until <out_dir>/STOP exists, SIGINT / SIGTERM, or --max-s. Per pipe: <pipe>.bin (bytes as received), <pipe>.idx (one line
per read: monotonic s, byte offset, length), <pipe>.status.json (ok | no_pipe | connect_fail | no_data | eof_early, bytes).
record.json: start / end as (wall time, monotonic) pairs - the MPA timestamps are CLOCK_MONOTONIC ns, the launcher log is wall
time, this pair aligns them. A pipe that is missing or fails to connect is recorded as such; the others go on.
Decoding on the PC: tag_detections -> mpa_parse.py / tag_step1.decode, pose_vel_6dof_t -> mpa_parse.py."""
import json, os, select, signal, sys, time
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
import mpa_capture as MC

DEFAULT = "tag_detections,vvhub_body_wrt_local,px4_vehicle_local_position"
STOP = {"why": None}


def main():
    a = sys.argv[1:]
    if not a or a[0].startswith("-"):
        sys.exit(__doc__)
    out = a[0]; pipes = DEFAULT.split(","); max_s = 300.0
    if "--pipes" in a:
        pipes = a[a.index("--pipes") + 1].split(",")
    if "--max-s" in a:
        max_s = float(a[a.index("--max-s") + 1])
    os.makedirs(out, exist_ok=True)
    for sg in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sg, lambda s, f: STOP.__setitem__("why", "signal %d" % s))
    rec = {"pipes": pipes, "max_s": max_s, "start": [time.time(), time.monotonic()], "pid": os.getpid()}
    st = {}; fds = {}
    for p in pipes:
        st[p] = {"status": None, "bytes": 0, "reads": 0, "info": MC.info(p)}
        try:
            fd, path = MC.connect(p)
            fds[fd] = (p, open(os.path.join(out, p + ".bin"), "wb"), open(os.path.join(out, p + ".idx"), "w"))
            st[p]["client_fifo"] = path
        except MC.Fail as e:
            st[p].update(status=e.status, msg=str(e))
    print("recording %s -> %s (STOP file, signal or %.0f s)" % (", ".join("%s:%s" % (p, "connected" if st[p]["status"] is None else st[p]["status"])
                                                                         for p in pipes), out, max_s), flush=True)
    t0 = time.monotonic(); eof = set()
    while fds and STOP["why"] is None:
        if os.path.exists(os.path.join(out, "STOP")):
            STOP["why"] = "STOP file"; break
        if time.monotonic() - t0 >= max_s:
            STOP["why"] = "max %.0f s" % max_s; break
        r, _, _ = select.select(list(fds), [], [], 0.5)
        for fd in r:
            p, fb, fi = fds[fd]
            b = os.read(fd, 1 << 20)
            if not b:
                if st[p]["bytes"] > 0:            # writer closed after data
                    eof.add(p); fb.close(); fi.close(); os.close(fd); del fds[fd]
                continue
            fi.write("%.6f %d %d\n" % (time.monotonic(), st[p]["bytes"], len(b))); fb.write(b)
            st[p]["bytes"] += len(b); st[p]["reads"] += 1
    for fd, (p, fb, fi) in list(fds.items()):
        fb.close(); fi.close(); os.close(fd)
    for p in pipes:
        if st[p]["status"] is None:
            st[p]["status"] = "no_data" if st[p]["bytes"] == 0 else ("eof_early" if p in eof else "ok")
        json.dump(st[p], open(os.path.join(out, p + ".status.json"), "w"), indent=1)
    rec.update(end=[time.time(), time.monotonic()], stopped_by=STOP["why"], status={p: st[p]["status"] for p in pipes},
               bytes={p: st[p]["bytes"] for p in pipes})
    json.dump(rec, open(os.path.join(out, "record.json"), "w"), indent=1)
    print("recorded (%s): %s" % (rec["stopped_by"], ", ".join("%s %s %d B" % (p, st[p]["status"], st[p]["bytes"]) for p in pipes)), flush=True)
    return 0 if all(st[p]["status"] == "ok" for p in pipes) else 1


if __name__ == "__main__":
    sys.exit(main())
