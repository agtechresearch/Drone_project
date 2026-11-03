"""AprilTag -> vehicle pose on the map (v3_marker, 2026-10-29). Python 3.6 + numpy 1.19 (runs on the real VOXL2 too).
Map frame = QVIX model frame (rack aligned: x across the aisles, y along the aisle, angles CCW from +x).
A detection gives the tag centre in the camera frame t = (x right, y down, z forward) and the rotation R whose third
column is the tag's z axis (pointing INTO the tag, i.e. opposite to its printed face normal) in the camera frame.
Checked in Gazebo (logs/tagtest/static_c9.json): camera heading = horizontal angle of R[:,2] in the camera + map angle of
the tag z axis, position error 3.7-7.6 mm at 0.42-0.46 m, heading within 0.5 deg.
"""
import math
import os

CAM_FWD = 0.075            # camera ahead of the body centre (m), same as the model (front camera pose 0.075 0 0)
CAM_LEFT = 0.0
if os.environ.get("WK_SENSORS", "") == "real":   # tracking_front, extrinsics.conf (cameras_2026-10-30.zip): 6.65 cm ahead, 0.65 cm left
    CAM_FWD, CAM_LEFT = 0.0665, 0.0065


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def body_pose_from_tag(t_cam, R, tag):
    """t_cam: [x, y, z] (m), R: 3x3 nested list, tag: {"x","y","z","normal"} from tags_map.json
    -> (x, y, heading) of the BODY centre in the map, heading = camera/body forward axis angle (rad, CCW from +x)"""
    zx, zz = R[0][2], R[2][2]                         # tag z axis in camera (right, forward)
    ang = math.atan2(zx, zz)                          # + = tag z axis to the right of the camera axis
    phi_z = 0.0 if tag["normal"] < 0 else math.pi     # tag z (into the tag) in the map: -normal
    th = wrap(ang + phi_z)                            # camera heading in the map
    right, fwd = t_cam[0], t_cam[2]
    cx = tag["x"] - (fwd * math.cos(th) + right * math.cos(th - math.pi / 2))
    cy = tag["y"] - (fwd * math.sin(th) + right * math.sin(th - math.pi / 2))
    return (cx - CAM_FWD * math.cos(th) - CAM_LEFT * math.cos(th + math.pi / 2),
            cy - CAM_FWD * math.sin(th) - CAM_LEFT * math.sin(th + math.pi / 2), th)


def fuse(poses, weights=None):
    """weighted mean of (x, y, heading) samples (circular mean for the heading)"""
    w = weights or [1.0] * len(poses)
    sw = sum(w)
    x = sum(p[0] * wi for p, wi in zip(poses, w)) / sw
    y = sum(p[1] * wi for p, wi in zip(poses, w)) / sw
    s = sum(math.sin(p[2]) * wi for p, wi in zip(poses, w)); c = sum(math.cos(p[2]) * wi for p, wi in zip(poses, w))
    return x, y, math.atan2(s, c)


def frame_from_pose(map_pose, vio_pose):
    """map_pose (x, y, heading) and the PX4 estimate at the same time vio_pose (n, e, yaw_ned rad)
    -> transform VIO(NED) <- map: rotation dpsi and translation so that vio = R(dpsi) * map_ned + t
    map_ned: (n = y, e = x) of the map frame; heading map (CCW from +x) -> NED yaw = pi/2 - heading"""
    mx, my, mh = map_pose
    n, e, yaw = vio_pose
    yaw_map_ned = math.pi / 2 - mh
    dpsi = wrap(yaw - yaw_map_ned)                    # rotate map-NED by dpsi (clockwise positive, NED) to get VIO
    c, s = math.cos(dpsi), math.sin(dpsi)
    rn, re = c * my - s * mx, s * my + c * mx
    return dpsi, (n - rn, e - re)
