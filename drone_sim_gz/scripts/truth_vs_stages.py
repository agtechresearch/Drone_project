"""Match gz ground-truth samples (truth_sampler.sh output) to the mission log's stage [END] lines.
Prints, per stage: script's own feedback delta vs gz truth position/yaw at that wall-clock time (gz ENU: x=E, y=N, z=U; yaw_gz=90deg-yaw_ned).
usage: python3 truth_vs_stages.py <mission.log> <truth.txt>"""
import sys, re, math
log, truth = sys.argv[1], sys.argv[2]
def secs(hms):
    h, m, s = hms.split(':'); return int(h)*3600 + int(m)*60 + float(s)
T = []
for line in open(truth, errors='replace'):
    m = re.match(r'(\d+:\d+:\d+\.\d)\s+\[([-\d.e ]+)\]\s*\[([-\d.e ]+)\]', line)
    if m:
        p = [float(v) for v in m.group(2).split()]; r = [float(v) for v in m.group(3).split()]
        T.append((secs(m.group(1)), p, r))
if not T: sys.exit("no truth samples parsed")
z0 = T[0][1][2]
print("truth samples: %d, first %s, z0=%.4f (spawn rest height)" % (len(T), T[0][1], z0))
def nearest(t):
    return min(T, key=lambda s: abs(s[0]-t))
print("\n%-22s %-9s %-44s %-36s %s" % ("stage", "status", "script fb_delta / yaw", "gz truth E,N,U(rel z0),yawNED", "dt(s)"))
for line in open(log, errors='replace'):
    m = re.match(r'\[(\d+:\d+:\d+\.\d+)\] \[(goto|yaw) (\S+)\] \[END\] (\S+)(.*)', line)
    if not m: continue
    t = secs(m.group(1)); name, status, rest = m.group(3), m.group(4), m.group(5)
    fb = re.search(r'fb_delta (dN=\S+ dE=\S+ dU=\S+)', rest); yw = re.search(r'fb_yaw=(\S+)', rest)
    s = nearest(t); yaw_ned = (90.0 - math.degrees(s[2][2]) + 180) % 360 - 180
    print("%-22s %-9s %-44s E%+7.3f N%+7.3f U%+6.3f yaw%+7.1f   %+.1f" % (
        name, status, fb.group(1) if fb else ("yaw " + yw.group(1) if yw else ""), s[1][0], s[1][1], s[1][2]-z0, yaw_ned, s[0]-t))
# landing / extremes
print("\nlast truth sample: E%+.4f N%+.4f U%+.4f (rel z0) at %.1f s after first" % (T[-1][1][0], T[-1][1][1], T[-1][1][2]-z0, T[-1][0]-T[0][0]))
import statistics
print("max truth U: %.3f  max N: %.3f  min N: %.3f  E range: %.3f..%.3f" % (max(s[1][2] for s in T)-z0, max(s[1][1] for s in T), min(s[1][1] for s in T), min(s[1][0] for s in T), max(s[1][0] for s in T)))
rolls = [abs(math.degrees(s[2][0])) for s in T]; pitches = [abs(math.degrees(s[2][1])) for s in T]
print("max |roll| %.1f deg, max |pitch| %.1f deg (sampled every 2.5 s)" % (max(rolls), max(pitches)))
