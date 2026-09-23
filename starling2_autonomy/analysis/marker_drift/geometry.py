"""좌표계 정의와 포즈 수학.

프레임
  M  마커 프레임. 마커 0 중심이 원점. x = 마커 열 방향(기체 진행 방향, 기체가 벽을 보고 있을 때 오른쪽),
     y = 벽 안쪽(마커 법선의 반대), z = 위. 오른손 좌표계. 마커는 y = 0 평면에, 기체는 y < 0 쪽에 있다.
  T  AprilTag 프레임(공식 규약). 태그 중심 원점, x 오른쪽, y 아래, z 태그 안쪽(카메라에서 멀어지는 쪽).
  C  OpenCV 카메라 프레임. x 오른쪽, y 아래, z 전방(광축).
  L  기체 로컬 NED. x 북(이륙 시 헤딩), y 동, z 아래. yaw 는 x->y 방향 양수(위에서 볼 때 시계방향).

표기
  R_A_B, p_A_B : 프레임 B 의 좌표를 프레임 A 로 옮기는 회전과 B 원점의 A 좌표.  v_A = R_A_B v_B + p_A_B

정면 정렬 yaw
  카메라 광축 f = R_M_C[:, 2] 를 수평면에 투영해 +y(벽 정면) 에서 +x 쪽으로 잰 각.
  0 = 벽과 정면으로 마주봄, 양수 = 기체가 오른쪽(+x)으로 돌아간 상태.
"""

import math

import numpy as np

try:
    import cv2
except ImportError:  # 테스트 환경에서 cv2 없이도 순수 수학 부분은 쓸 수 있게
    cv2 = None


# ---------------------------------------------------------------- 기본 회전

def rot_x(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]], float)


def rot_y(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]], float)


def rot_z(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]], float)


def rpy_to_R(roll_deg, pitch_deg, yaw_deg):
    """Z(yaw) * Y(pitch) * X(roll). 축은 호출한 쪽의 프레임 기준."""
    return rot_z(math.radians(yaw_deg)) @ rot_y(math.radians(pitch_deg)) @ rot_x(math.radians(roll_deg))


def wrap_deg(a):
    """각도를 (-180, 180] 로."""
    a = np.asarray(a, float)
    return (a + 180.0) % 360.0 - 180.0


def circular_mean_deg(a):
    a = np.radians(np.asarray(a, float))
    a = a[~np.isnan(a)]
    if a.size == 0:
        return float("nan")
    return math.degrees(math.atan2(np.sin(a).mean(), np.cos(a).mean()))


def invert(R, p):
    """(R, p) 의 역변환."""
    Rt = R.T
    return Rt, -Rt @ p


def compose(R_a_b, p_a_b, R_b_c, p_b_c):
    """T_a_c = T_a_b * T_b_c"""
    return R_a_b @ R_b_c, R_a_b @ p_b_c + p_a_b


# ---------------------------------------------------------------- 태그 <-> 마커 프레임

# 벽에 똑바로 붙은 태그(기체 쪽을 향함)의 T -> M 회전.
#   T.x(오른쪽) -> M.x,  T.y(아래) -> -M.z,  T.z(태그 안쪽) -> +M.y
R_M_T_UPRIGHT = np.array([[1.0, 0.0, 0.0],
                          [0.0, 0.0, 1.0],
                          [0.0, -1.0, 0.0]])


def tag_pose_in_M(marker):
    """마커 사전({id, x, z, [y], [rpy_deg]}) -> (R_M_T, p_M_T).

    rpy_deg 는 M 축 기준으로 똑바로 붙은 상태에서 추가로 돌린 양(설치 기울기 보정용). 기본 0.
    """
    p = np.array([float(marker.get("x", 0.0)), float(marker.get("y", 0.0)), float(marker.get("z", 0.0))])
    rpy = marker.get("rpy_deg", (0.0, 0.0, 0.0)) or (0.0, 0.0, 0.0)
    R = rpy_to_R(*rpy) @ R_M_T_UPRIGHT
    return R, p


def tag_object_points(size_m):
    """태그 코너의 T 좌표. cv2.SOLVEPNP_IPPE_SQUARE 가 요구하는 순서와 AprilTag 코너 순서가 같다.
    [0] (-s/2, +s/2) 좌하  [1] (+s/2, +s/2) 우하  [2] (+s/2, -s/2) 우상  [3] (-s/2, -s/2) 좌상   (y 아래 양수)
    """
    h = float(size_m) / 2.0
    return np.array([[-h, h, 0.0], [h, h, 0.0], [h, -h, 0.0], [-h, -h, 0.0]])


# ---------------------------------------------------------------- 투영 / PnP

def project_points(P_C, K, dist=None):
    """카메라 프레임 점 (N,3) -> 픽셀 (N,2). dist 는 OpenCV 표준 계수(None 이면 무왜곡)."""
    if cv2 is None:
        raise RuntimeError("cv2 가 필요합니다")
    rvec = np.zeros(3)
    tvec = np.zeros(3)
    d = np.zeros(5) if dist is None else np.asarray(dist, float)
    px, _ = cv2.projectPoints(np.asarray(P_C, float).reshape(-1, 1, 3), rvec, tvec, np.asarray(K, float), d)
    return px.reshape(-1, 2)


def project_tag_corners(R_M_C, p_M_C, R_M_T, p_M_T, size_m, K, dist=None):
    """카메라 M 포즈와 태그 M 포즈 -> 태그 코너 픽셀 (4,2) 와 카메라 프레임 깊이 (4,). 합성·커버리지 점검용."""
    obj = tag_object_points(size_m)
    P_M = (np.asarray(R_M_T) @ obj.T).T + np.asarray(p_M_T)
    R_C_M, p_C_M = invert(np.asarray(R_M_C, float), np.asarray(p_M_C, float))
    P_C = (R_C_M @ P_M.T).T + p_C_M
    return project_points(P_C, K, dist), P_C[:, 2]


def solve_tag_pose(corners_px, size_m, K, dist=None, fisheye=False):
    """태그 코너 4점(AprilTag 순서) -> (R_C_T, t_C_T, 재투영 오차 px).

    fisheye=True 면 cv2.fisheye 모델로 코너를 정규화 좌표로 펴고 K=I 로 PnP 한다(트래킹 카메라용).
    """
    if cv2 is None:
        raise RuntimeError("cv2 가 필요합니다")
    obj = tag_object_points(size_m)
    img = np.asarray(corners_px, float).reshape(4, 2)
    K = np.asarray(K, float)
    d = None if dist is None else np.asarray(dist, float)

    if fisheye:
        und = cv2.fisheye.undistortPoints(img.reshape(-1, 1, 2), K, d.reshape(-1, 1) if d is not None else np.zeros((4, 1)))
        img_pnp = und.reshape(4, 2)
        K_pnp = np.eye(3)
        d_pnp = None
    else:
        img_pnp = img
        K_pnp = K
        d_pnp = d

    ok, rvec, tvec = cv2.solvePnP(obj, img_pnp, K_pnp, d_pnp, flags=cv2.SOLVEPNP_IPPE_SQUARE)
    if not ok:
        raise ValueError("solvePnP 실패")
    R, _ = cv2.Rodrigues(rvec)
    t = tvec.reshape(3)

    reproj, _ = cv2.projectPoints(obj, rvec, tvec, K_pnp, d_pnp if d_pnp is not None else np.zeros(5))
    err = float(np.sqrt(((reproj.reshape(4, 2) - img_pnp) ** 2).sum(axis=1).mean()))
    if fisheye:
        # 정규화 좌표 오차를 대략 픽셀로 환산
        err *= float((K[0, 0] + K[1, 1]) / 2.0)
    return R, t, err


def solve_multi_tag_camera_pose(corners_list, tag_poses_M, sizes_m, K, dist=None, fisheye=False):
    """같은 프레임의 태그 여러 개를 한 몸으로 보고 PnP -> (R_M_C, p_M_C, 재투영 RMS px).

    corners_list : [(4,2) 픽셀 코너, ...]  (AprilTag 순서)
    tag_poses_M  : [(R_M_T, p_M_T), ...]   배치 파일의 M 포즈
    sizes_m      : [태그 크기, ...]

    태그 하나짜리 PnP 는 정면에서 볼 때 평면 포즈 모호성(두 해가 비슷한 재투영 오차) 때문에 yaw 가 튀거나
    부호가 뒤집힐 수 있다. 태그 2개 이상이면 기준선(1 m)이 생겨 yaw 가 안정된다. 작은 태그(6~7 cm)에서 특히 중요.
    """
    if cv2 is None:
        raise RuntimeError("cv2 가 필요합니다")
    obj = []
    img = []
    for corners, (R_M_T, p_M_T), s in zip(corners_list, tag_poses_M, sizes_m):
        P_T = tag_object_points(s)
        obj.append((np.asarray(R_M_T) @ P_T.T).T + np.asarray(p_M_T))
        img.append(np.asarray(corners, float).reshape(4, 2))
    obj = np.vstack(obj)
    img = np.vstack(img)
    K = np.asarray(K, float)
    d = None if dist is None else np.asarray(dist, float)
    if fisheye:
        und = cv2.fisheye.undistortPoints(img.reshape(-1, 1, 2), K, d.reshape(-1, 1) if d is not None else np.zeros((4, 1)))
        img_pnp, K_pnp, d_pnp = und.reshape(-1, 2), np.eye(3), None
    else:
        img_pnp, K_pnp, d_pnp = img, K, d
    flags = cv2.SOLVEPNP_SQPNP if len(obj) >= 8 else cv2.SOLVEPNP_ITERATIVE
    ok, rvec, tvec = cv2.solvePnP(obj, img_pnp, K_pnp, d_pnp, flags=flags)
    if not ok:
        raise ValueError("multi-tag solvePnP 실패")
    rvec, tvec = cv2.solvePnPRefineLM(obj, img_pnp, K_pnp, d_pnp, rvec, tvec)
    R_C_M, _ = cv2.Rodrigues(rvec)
    p_C_M = tvec.reshape(3)
    reproj, _ = cv2.projectPoints(obj, rvec, tvec, K_pnp, d_pnp if d_pnp is not None else np.zeros(5))
    err = float(np.sqrt(((reproj.reshape(-1, 2) - img_pnp) ** 2).sum(axis=1).mean()))
    if fisheye:
        err *= float((K[0, 0] + K[1, 1]) / 2.0)
    R_M_C, p_M_C = invert(R_C_M, p_C_M)
    return R_M_C, p_M_C, err


def camera_pose_from_tag(R_C_T, t_C_T, R_M_T, p_M_T):
    """태그의 카메라 프레임 포즈 + 태그의 M 포즈 -> 카메라의 M 포즈 (R_M_C, p_M_C)."""
    R_T_C, p_T_C = invert(np.asarray(R_C_T, float), np.asarray(t_C_T, float))
    return compose(R_M_T, p_M_T, R_T_C, p_T_C)


def camera_yaw_in_M(R_M_C):
    """정면 정렬 yaw(도). 0 = 벽 정면(+y), 양수 = +x(오른쪽)으로 돌아감."""
    f = np.asarray(R_M_C)[:, 2]
    return math.degrees(math.atan2(f[0], f[1]))


def camera_pitch_roll_in_M(R_M_C):
    """진단용 (pitch_deg, roll_deg). pitch 양수 = 광축이 위를 봄. roll 양수 = 카메라 오른쪽이 아래로 내려감."""
    R = np.asarray(R_M_C)
    f = R[:, 2]
    r = R[:, 0]
    pitch = math.degrees(math.asin(max(-1.0, min(1.0, f[2]))))
    roll = math.degrees(math.asin(max(-1.0, min(1.0, -r[2]))))
    return pitch, roll


# ---------------------------------------------------------------- 로컬 NED <-> M

def ned_to_enu(xyz_ned):
    """(N,3) NED -> (N,3) ENU(동, 북, 위). ENU 와 M 은 둘 다 z 위 오른손 프레임이라 수평 회전만으로 이어진다."""
    a = np.asarray(xyz_ned, float).reshape(-1, 3)
    return np.column_stack([a[:, 1], a[:, 0], -a[:, 2]])


def compass_rotate(xy, delta_deg):
    """수평 좌표 (N,2) 를 '컴퍼스 각' 기준으로 delta 만큼 돌린다.

    컴퍼스 각 θ 의 단위벡터를 (sin θ, cos θ) 로 두는 프레임(ENU 의 heading, M 의 정면 yaw 모두 해당)에서
    θ -> θ + delta 가 되도록 하는 선형 변환.
    """
    d = math.radians(delta_deg)
    c, s = math.cos(d), math.sin(d)
    a = np.asarray(xy, float).reshape(-1, 2)
    x, y = a[:, 0], a[:, 1]
    return np.column_stack([x * c + y * s, y * c - x * s])


def heading_from_vector(x, y):
    """컴퍼스 각(도): +y(북/정면) 에서 +x(동/오른쪽) 쪽으로."""
    return math.degrees(math.atan2(x, y))
