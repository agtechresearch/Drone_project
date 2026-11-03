#!/usr/bin/env python3
"""IMU lever-arm check, evaluation (2026-11-02). One run dir of logs/run_m3_hover_spin.sh (fs_spin.py fast/slow yaw spin):
  imu_lever_eval.py <run_dir> <rx> <ry> [--ulog <file.ulg>]
<rx> <ry>: IMU position in the model SDF (FLU, m) - 0 0 for the installed model, 0.0043 -0.0073 for m2 / m3.
A rigid body turning at a steady rate omega about a vertical axis through the body origin gives an IMU at r a specific-force
term (toward the axis) of -omega^2 * r_xy in the body frame. Segments are found from the gyro z rate: fast = |w| > 0.8 of the
top rate, slow = 0.15..0.45 of it, each > 3 s, first 1.5 s dropped. Each fast segment is compared with the mean of the slow
segments right before and after it (a linear accelerometer bias drift cancels):
  delta = fast - slow,  expected = -(wf^2 - ws^2) * (rx, ry)   [PX4 FRD: -(wf^2 - ws^2) * (rx, -ry)]
Three views: raw gz IMU (FLU), truth-referenced gz IMU (minus the truth specific force of the body ORIGIN, R^T (a + g e_z) from
gt.csv, which removes the vehicle's own motion and the origin's circle), and raw PX4 sensor_combined (--ulog, after GZBridge)."""
import csv, math, statistics as S, sys

d, rx, ry = sys.argv[1], float(sys.argv[2]), float(sys.argv[3])
ulog = sys.argv[sys.argv.index("--ulog") + 1] if "--ulog" in sys.argv else None


def runs(t, w, lo, hi):
    out, cur = [], None
    for i in range(len(t)):
        if lo < abs(w[i]) <= hi:
            cur = [t[i], t[i]] if cur is None else [cur[0], t[i]]
        elif cur is not None:
            if cur[1] - cur[0] > 3:
                out.append((cur[0] + 1.5, cur[1]))
            cur = None
    return out


def mean_in(t, cols, a, b):
    idx = [i for i in range(len(t)) if a <= t[i] <= b]
    return [S.mean(c[i] for i in idx) if idx else float("nan") for c in cols] + [len(idx)]


def evaluate(label, t, ax, ay, w, sy, ref=None):
    """ref: (t, fx, fy) truth specific force of the origin, subtracted window by window"""
    top = sorted(abs(x) for x in w)[int(0.98 * len(w))]
    fast, slow = runs(t, w, 0.8 * top, 1e9), runs(t, w, 0.15 * top, 0.45 * top)

    def val(a, b):
        m = mean_in(t, (ax, ay, w), a, b)
        if ref:
            f = mean_in(ref[0], ref[1:], a, b)
            m = [m[0] - f[0], m[1] - f[1], m[2], min(m[3], f[2])]
        return m
    print("%s: %d fast / %d slow segments (top rate %.3f rad/s)" % (label, len(fast), len(slow), top))
    rows = []
    for k, (a, b) in enumerate(fast):
        before = [s for s in slow if s[1] <= a][-1:]
        after = [s for s in slow if s[0] >= b][:1]
        if not before or not after or after[0][0] - b > 3 or a - before[0][1] > 3:
            continue
        f, s1, s2 = val(a, b), val(*before[0]), val(*after[0])
        if min(f[3], s1[3], s2[3]) == 0:
            continue
        dw2 = f[2] ** 2 - (s1[2] ** 2 + s2[2] ** 2) / 2
        dx, dy = f[0] - (s1[0] + s2[0]) / 2, f[1] - (s1[1] + s2[1]) / 2
        ex, ey = -dw2 * rx, -dw2 * ry * sy
        rows.append((f[2], dw2, dx, dy, ex, ey, dx - ex, dy - ey))
        print("  fast %2d w %+.4f d(w^2) %.4f: delta ax %+.6f ay %+.6f | expected %+.6f %+.6f | residual %+.6f %+.6f" % ((k,) + rows[-1]))
    if len(rows) > 1:
        for j, nm in ((2, "delta ax"), (3, "delta ay"), (6, "resid ax"), (7, "resid ay")):
            v = [r[j] for r in rows]
            print("  %s mean %+.6f sd %.6f (n %d, sd of mean %.6f)" % (nm, S.mean(v), S.stdev(v), len(v), S.stdev(v) / math.sqrt(len(v))))
        dw = S.mean(r[1] for r in rows)
        # least-squares r from delta = -d(w^2) r (per axis)
        sxx = sum(r[1] ** 2 for r in rows)
        rxf = -sum(r[1] * r[2] for r in rows) / sxx; ryf = -sum(r[1] * r[3] for r in rows) / sxx * sy
        print("  expected mean %+.6f %+.6f (mean d(w^2) %.4f); fitted r (FLU) x %+.5f y %+.5f m vs SDF %+.5f %+.5f" % (
            S.mean(r[4] for r in rows), S.mean(r[5] for r in rows), dw, rxf, ryf, rx, ry))
    return rows


imu = list(csv.DictReader(open(d + "/imu.csv")))
t = [float(r["t_sim"]) for r in imu]
iax, iay, igz = ([float(r[k]) for r in imu] for k in ("ax", "ay", "gz"))
print("SDF IMU r = %.4f %.4f m (FLU)" % (rx, ry))
evaluate("gz IMU raw (FLU)", t, iax, iay, igz, 1)

G, K = 9.8, 5
gt = list(csv.DictReader(open(d + "/gt.csv")))
tg = [float(r["t_sim"]) for r in gt]
P = [(float(r["x"]), float(r["y"]), float(r["z"])) for r in gt]
Q = [(float(r["qw"]), float(r["qx"]), float(r["qy"]), float(r["qz"])) for r in gt]
f0x, f0y, tf = [], [], []
for i in range(K, len(gt) - K):
    h1, h2 = tg[i] - tg[i - K], tg[i + K] - tg[i]
    if h1 <= 0 or h2 <= 0:
        continue
    a = [2 * ((P[i + K][j] - P[i][j]) / h2 - (P[i][j] - P[i - K][j]) / h1) / (h1 + h2) for j in range(3)]
    a[2] += G
    w_, x_, y_, z_ = Q[i]
    R = ((1 - 2 * (y_ * y_ + z_ * z_), 2 * (x_ * y_ - w_ * z_), 2 * (x_ * z_ + w_ * y_)),
         (2 * (x_ * y_ + w_ * z_), 1 - 2 * (x_ * x_ + z_ * z_), 2 * (y_ * z_ - w_ * x_)),
         (2 * (x_ * z_ - w_ * y_), 2 * (y_ * z_ + w_ * x_), 1 - 2 * (x_ * x_ + y_ * y_)))
    f0x.append(sum(R[j][0] * a[j] for j in range(3))); f0y.append(sum(R[j][1] * a[j] for j in range(3))); tf.append(tg[i])
evaluate("gz IMU truth-referenced (FLU, minus truth specific force of the body origin)", t, iax, iay, igz, 1, (tf, f0x, f0y))

if ulog:
    from pyulog import ULog
    sc = ULog(ulog, ["sensor_combined"]).get_dataset("sensor_combined").data
    evaluate("PX4 sensor_combined raw (FRD)", list(sc["timestamp"] / 1e6), list(sc["accelerometer_m_s2[0]"]),
             list(sc["accelerometer_m_s2[1]"]), list(sc["gyro_rad[2]"]), -1)
