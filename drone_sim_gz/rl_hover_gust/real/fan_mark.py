#!/usr/bin/env python3
"""Fan on/off time marker (2026-11-04). Run ON THE VOXL in a second SSH session while hover_flight.sh flies:
    python3 /data/claude_collect/tools_safe/fan_mark.py
  Enter          -> toggles fan state (first Enter = ON, next = OFF, ...)
  on / off       -> sets the state explicitly
  any other text -> free note (e.g. "fan speed 2")
  q or Ctrl+C    -> quit
Each line is stamped with the VOXL CLOCK_MONOTONIC (same clock as the MPA records -> rec/record.json maps it to wall time)
and the wall clock (UTC). Output: /data/claude_collect/fan_events/fan_<UTC start>.csv (written and flushed per line).
Python 3.6, standard library only. Writes nothing else."""
import datetime, os, sys, time

OUT_DIR = "/data/claude_collect/fan_events"


def now():
    return time.clock_gettime(time.CLOCK_MONOTONIC), time.time()


def main():
    out_dir = sys.argv[1] if len(sys.argv) > 1 else OUT_DIR
    os.makedirs(out_dir, exist_ok=True)
    start = datetime.datetime.utcnow().strftime("%Y%m%d_%H%M%S")
    path = os.path.join(out_dir, "fan_%s.csv" % start)
    f = open(path, "w")
    f.write("mono_s,wall_utc_s,wall_utc_iso,event,note\n"); f.flush()
    state = "off"; m0, w0 = now()
    print("fan marker -> %s" % path)
    print("Enter = toggle (first = ON), 'on' / 'off', other text = note, 'q' = quit")

    def write(ev, note=""):
        m, w = now()
        iso = datetime.datetime.utcfromtimestamp(w).strftime("%H:%M:%S.%f")[:-3]
        f.write("%.6f,%.6f,%s,%s,%s\n" % (m, w, iso, ev, note.replace(",", ";"))); f.flush(); os.fsync(f.fileno())
        print("  %s  %-5s %s  (+%.1f s)" % (iso, ev.upper(), note, m - m0))

    write("start")
    try:
        while True:
            line = sys.stdin.readline()
            if not line:                       # stdin closed
                break
            s = line.strip()
            if s.lower() in ("q", "quit", "exit"):
                break
            if s == "":
                state = "on" if state == "off" else "off"; write(state)
            elif s.lower() in ("on", "off"):
                state = s.lower(); write(state)
            else:
                write("note", s)
    except KeyboardInterrupt:
        pass
    write("stop"); f.close()
    print("saved %s" % path)


if __name__ == "__main__":
    main()
