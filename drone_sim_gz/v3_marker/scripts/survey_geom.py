#!/usr/bin/env python3
"""Shared survey geometry (v3_marker, 2026-11-01, codex survey-uncertainty / survey-anchor-lines / survey-replay-consistency):
one place for what survey_map.py (solver) and survey_web_export.py (replay) must do the same way.
  tag_cov(C5, y, ymid, sgn, phi)       covariance of a tag position [x, y, z] from the parameter covariance [b, phi, s, y, z]
                                        (x = x_design + b + phi * (y - ymid) + sgn * s): J C J^T, with the cross terms
  turn_cov(C, a)                        a covariance under a FIXED turn by a (rad, CCW in x-y): R C R^T (a fixed shift: unchanged)
  xform_line(off, k, xd, yref, f)       a face line x = xd + off + k * (y - yref) moved by the rigid map f(x, y) -> (x', y'):
                                        two points of the line are moved and the line through them is written again at yref
  line_cov(off, k, C2, xd, yref, f)     its (offset, slope) covariance through the same map (numeric Jacobian)
  sensor_rot(rpy_deg)                   ToF mount rotation (sensor -> body FLU), Rz @ Ry @ Rx
  frame_points(dep, TF, stride, rmax)   one depth frame -> points in body FLU (mount rotation + position applied)
  body_to_map(B, x, y, z, comp)         body FLU points -> map frame at a pose (comp = compass heading, deg)
Slope convention: the solver's face parameter phi is the slope dx/dy of the line; turn_deg = degrees(atan(slope))."""
import math
import numpy as np


def tag_cov(C5, y, ymid, sgn, phi):
    J = np.array([[1.0, y - ymid, sgn, phi, 0.0],
                  [0.0, 0.0, 0.0, 1.0, 0.0],
                  [0.0, 0.0, 0.0, 0.0, 1.0]])
    return J @ np.asarray(C5, float) @ J.T


def rot2(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s], [s, c]])


def turn_cov(C, a):
    C = np.array(C, float); R = np.eye(len(C)); R[:2, :2] = rot2(a)
    return R @ C @ R.T


def turn_about(cx, cy, a):
    """the map turn as a function: points turned by a (rad, CCW) about (cx, cy)"""
    c, s = math.cos(a), math.sin(a)

    def f(px, py):
        return cx + c * (px - cx) - s * (py - cy), cy + s * (px - cx) + c * (py - cy)
    return f


def shift_by(sx, sy):
    def f(px, py):
        return px + sx, py + sy
    return f


def xform_line(off, k, xd, yref, f):
    """line x = xd + off + k (y - yref) through the rigid map f; returns (off', k') of the moved line at the same yref"""
    p = [f(xd + off + k * dy, yref + dy) for dy in (0.0, 1.0)]
    (x0, y0), (x1, y1) = p
    k2 = (x1 - x0) / (y1 - y0)
    return x0 + k2 * (yref - y0) - xd, k2


def line_cov(off, k, C2, xd, yref, f, e=1e-6):
    J = np.zeros((2, 2)); base = np.array(xform_line(off, k, xd, yref, f))
    for j, d in enumerate(((e, 0.0), (0.0, e))):
        J[:, j] = (np.array(xform_line(off + d[0], k + d[1], xd, yref, f)) - base) / e
    return J @ np.asarray(C2, float) @ J.T


def sensor_rot(rpy_deg):
    rr, rp, rw = np.radians(rpy_deg)
    Rx = np.array([[1, 0, 0], [0, math.cos(rr), -math.sin(rr)], [0, math.sin(rr), math.cos(rr)]])
    Ry = np.array([[math.cos(rp), 0, math.sin(rp)], [0, 1, 0], [-math.sin(rp), 0, math.cos(rp)]])
    Rz = np.array([[math.cos(rw), -math.sin(rw), 0], [math.sin(rw), math.cos(rw), 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def frame_points(dep, TF, stride, rmax=None):
    """depth image (metres along the optical axis) -> points (3, n) in body FLU; rmax defaults to TF['range'][1]"""
    H, W = dep.shape
    fpx = (W / 2.0) / math.tan(math.radians(TF["hfov_deg"]) / 2.0)
    us, vs = np.meshgrid(np.arange(0, W, stride), np.arange(0, H, stride))
    ry = -(us - (W - 1) / 2.0) / fpx; rz = -(vs - (H - 1) / 2.0) / fpx      # ray (1, ry, rz): x optical, y left, z up
    dd = dep[::stride, ::stride].astype(np.float32)
    ok = np.isfinite(dd) & (dd > TF["range"][0]) & (dd < (TF["range"][1] if rmax is None else rmax))
    S = np.stack([dd[ok], dd[ok] * ry[ok], dd[ok] * rz[ok]])
    return sensor_rot(TF.get("rpy_deg", [0.0, 0.0, 0.0])) @ S + np.array(TF["pos_body_flu"], float)[:, None]


def body_to_map(B, x, y, z, comp):
    th = math.radians(90.0 - comp); c, s = math.cos(th), math.sin(th)
    return np.stack([x + c * B[0] - s * B[1], y + s * B[0] + c * B[1], z + B[2]], axis=1)
