"""Make the simulated ToF depth look like a real ToF (v3_marker, 2026-10-31): range-proportional noise, signal-strength
dropout (far and grazing surfaces vanish, like the real confidence cutoff) and flying pixels at depth edges.
The gz depth camera returns every surface inside its clip range with a constant 5 mm noise, so far and oblique surfaces
look much better than on the real sensor (survey replay 2026-10-31: front views reached 2.4 m high from oblique frames).
Parameters: config/tof_sim_model.json (generic until the real sensor is measured). Used by wall_sense.py (WK_TOF_MODEL)."""
import json, math
import numpy as np


class ToFModel(object):
    def __init__(self, path, hfov):
        self.p = json.load(open(path)); self.hfov = hfov
        self.rng = np.random.default_rng(self.p.get("seed", 7)); self.cache = None

    def _rays(self, h, w):
        if self.cache is None or self.cache[0] != (h, w):
            fx = (w / 2.0) / math.tan(self.hfov / 2.0)
            u, v = np.meshgrid(np.arange(w), np.arange(h))
            ry = -(u - (w - 1) / 2.0) / fx; rz = -(v - (h - 1) / 2.0) / fx
            self.cache = ((h, w), ry, rz, np.sqrt(1 + ry * ry + rz * rz))
        return self.cache[1:]

    def apply(self, depth):
        """depth: float32 [h, w], metres along the optical axis, inf/nan = no return -> degraded copy (inf = dropped)"""
        p = self.p; d = depth.astype(np.float32).copy(); h, w = d.shape
        ry, rz, rn = self._rays(h, w)
        ok = np.isfinite(d) & (d > 0)
        # 3-D points and a surface normal from the neighbours (incidence angle)
        X = np.where(ok, d, np.nan); Y = X * ry; Z = X * rz
        with np.errstate(invalid="ignore"):
            gx = [np.gradient(A, axis=1) for A in (X, Y, Z)]; gy = [np.gradient(A, axis=0) for A in (X, Y, Z)]
            n = np.cross(np.stack(gx, -1), np.stack(gy, -1)); nn = np.linalg.norm(n, axis=-1)
            ray = np.stack([np.ones_like(ry), ry, rz], -1) / rn[..., None]
            cosi = np.abs(np.sum(n * ray, -1)) / nn
        cosi = np.where(np.isfinite(cosi), cosi, 1.0)
        rng_m = d * rn                                            # range along the ray
        # signal strength: reflectivity * cos / r^2 against the cutoff
        amp = p["reflectivity"] * cosi / np.maximum(rng_m, 1e-3) ** 2
        cut = 1.0 / p["amp_range_m"] ** 2
        keep = amp >= cut
        soft = keep & (amp < cut * (1 + p["amp_soft"]))
        pdrop = 1.0 - (amp - cut) / (cut * p["amp_soft"])
        keep &= ~(soft & (self.rng.random(d.shape) < pdrop))
        keep &= ok & (rng_m <= p["max_range_m"])
        # flying pixels at depth discontinuities (2026-11-01 codex tof-edge-model): a pixel whose 4-neighbour depth jumps by more
        # than edge_jump_m becomes, with edge_mix_prob, a CONVEX mix of its own depth and that neighbour's (the near and far
        # surface): d + u (d_nb - d), u ~ U(0, 1). The old d + u |jump| * random sign also went past both surfaces (negative
        # depth in front of a 0.3 m face with a 1.0 m background).
        fin = np.where(ok, d, np.nan); best = np.zeros_like(d); nbv = d.copy()
        pad = np.pad(fin, 1, constant_values=np.nan)
        with np.errstate(invalid="ignore"):
            for nb in (pad[:-2, 1:-1], pad[2:, 1:-1], pad[1:-1, :-2], pad[1:-1, 2:]):     # up, down, left, right
                j = np.abs(nb - fin)
                m = j > best                                      # NaN (no return on either side) never wins
                best[m] = j[m]; nbv[m] = nb[m]
        edge = keep & (best > p["edge_jump_m"]) & (self.rng.random(d.shape) < p["edge_mix_prob"])
        d[edge] = d[edge] + self.rng.random(int(edge.sum())).astype(np.float32) * (nbv[edge] - d[edge])
        # range-proportional noise
        d[~keep] = np.inf
        k = np.isfinite(d)
        d[k] = d[k] + self.rng.normal(0.0, 1.0, int(k.sum())).astype(np.float32) * (p["noise_sd0"] + p["noise_sd_rel"] * d[k])
        # range check AFTER the model (codex tof-edge-model acceptance 2): no return (inf) for a depth <= 0 or a range past
        # max_range_m - such a value is never counted as a valid pixel
        with np.errstate(invalid="ignore"):
            d[np.isfinite(d) & ((d <= 0) | (d * rn > p["max_range_m"]))] = np.inf
        return d.astype(np.float32)


def valid_fraction(depth):
    return float(np.mean(np.isfinite(depth) & (depth > 0)))
