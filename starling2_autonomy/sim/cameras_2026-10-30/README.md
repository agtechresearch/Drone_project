# Starling 2 카메라/ToF/AprilTag 캘리브레이션 자료 (2026-10-30 기체 실측)

기체: Starling 2 (M0054, SKU MRB-D0014-4-V1-C26-T8-M36-X0), voxl-suite 1.6.3, voxl-camera-server 2.2.19

## 1. 연결된 카메라 (voxl-camera-server.conf + `voxl-inspect-cam -a` 실측)
`voxl-inspect-cam -l` 옵션은 없음 → `-a`(전체) / `<pipe> -n` 사용.

| camera_id | 이름 | 센서 | 설정 fps | 실측 파이프 (해상도 · 포맷 · fps) | 비고 |
|---|---|---|---|---|---|
| 0 | tracking_front | AR0144 (M0166, 글로벌셔터 모노, 어안) | 30 | `tracking_front` 1280×800 RAW8 30 fps (노출 12 ms, 게인 1596) | VIO·태그 탐지 입력. 파생: `_misp_norm`, `_misp_grey`, `_bayer`, `_misp_encoded`(H265) |
| 1 | hires | IMX412 (컬러, 롤링셔터) | 30 | `hires_large_color` 4056×3040 NV12 30 fps · `hires_small_color` 1024×768 NV12 30 fps · `hires_small_grey` 1024×768 RAW8 · `hires_snapshot` | 내부 파라미터 파일 **없음** |
| 2 | tof | PMD ToF (liow2) | 10 | `tof_depth` / `tof_ir` / `tof_conf` 180×240 RAW8, **실측 2.0 fps** · `tof_pc` 점군 | 설정은 10 fps인데 실측 2 fps (확인 필요) |
| 3 | tracking_down | AR0144 (어안) | 30 | `tracking_down` 1280×800 RAW8 30 fps | VIO·태그 탐지 입력 |

(voxl-inspect-cam 열: name, bytes, w, h, exposure ms, gain, frame_id, latency ms, fps, Mbps, format)

## 2. 내부 파라미터 (`/data/modalai/opencv_<cam>_intrinsics.yml`, `intrinsics/`에 복사)
| 파일 | 해상도 | 모델 | fx, fy | cx, cy | D | reproj err | 캘리브 일자 |
|---|---|---|---|---|---|---|---|
| opencv_tracking_front_intrinsics.yml | 1280×800 | **fisheye** (k1..k4) | 460.99, 460.79 | 621.58, 382.33 | 0.05980, 0.007408, −0.005711, 0.001850 | 0.169 px | 2023-03-02 (공장) |
| opencv_tracking_down_intrinsics.yml | 1280×800 | **fisheye** (k1..k4) | 446.33, 446.13 | 645.43, 377.78 | 0.07439, 0.003785, −0.006188, 0.002536 | 0.167 px | 2023-03-02 (공장) |
| opencv_seek_intrinsics.yml | 320×240 | plumb_bob (k1,k2,p1,p2,k3) | 191.90, 192.07 | 165.50, 123.68 | −0.3881, 0.1519, −0.0000828, 0.000653, −0.0279 | 0.178 px | 2024-10-02 |
| opencv_uvc_intrinsics.yml | 320×240 | plumb_bob | (seek와 동일 값) | | | 0.312 px | 2024-10-02 |

- seek/uvc 는 이 기체에 **장착되지 않은** 열화상/USB 카메라용 잔존 파일(SDK 기본 포함). 실제 쓰이는 건 tracking_front/down 2개.
- hires(IMX412)·ToF 내부 파라미터 파일은 없음. hires 는 필요 시 `voxl-camera-calibration` 으로 생성.
- 태그 탐지기는 undistort_scale 0.6 으로 어안을 펴서 사용 → 보정 후 초점거리 fxrect 276.60/276.48 (front), 267.80/267.68 (down) (voxl-tag-detector 로그).

## 3. 외부 파라미터 (`/etc/modalai/extrinsics.conf`, 이름 D0014_Starling_2)
T_child_wrt_parent [m], RPY_parent_to_child [deg]

| parent → child | T (x, y, z) | RPY |
|---|---|---|
| body → imu_apps | (0.0295, −0.0065, −0.016) | (0, 0, 0) |
| body → imu_px4 | (0.0043, 0.0073, −0.0163) | (0, 0, 0) — PX4 EKF2_IMU_POS_* 와 일치 |
| imu_apps → tracking_front | (0.037, 0.000, 0.0006) | (0, 90, 90) |
| imu_apps → tracking_down | (−0.08825, −0.0045, 0.0269) | (0, 0, 180) |
| imu_apps → hires | (0.0388, 0.0, 0.0186) | (0, 90, 90) |
| body → tof | (0.066, 0.009, −0.012) | (0, 90, 180) |
| body → ground | (0, 0, 0.033) | (0, 0, 0) |
| imu_apps → tracking_rear / lepton0_raw | (미장착, 파일에만 존재) | |

`/data/modalai/ov/ext/` 에 다른 기체 변형용 extrinsics 사본들(ar144, ov72, ov97)이 있으나 이 기체에는 `/etc/modalai/extrinsics.conf` 가 적용됨.

## 4. VIO 카메라 설정 (`vio_cams.conf`)
tracking_front + tracking_down 두 대, IMU `imu_apps`, 추적 파이프는 `*_misp_norm_ion`, 캘리브 파일은 위 fisheye yml.

## 5. AprilTag 탐지기 (`voxl-tag-detector`)
- 서비스 상태: **비활성(inactive)** — 평소 꺼져 있음. 켜려면 `systemctl start voxl-tag-detector` (영구: `enable`).
- 설정(`voxl-tag-detector.conf`): detector_0 = tracking_front, detector_1 = tracking_down, tag36h11, en_undistortion true, undistort_scale 0.6, skip_n_frames 5, fast mode.
- 태그 크기(`tag_locations.conf`): default_size_m **0.40**, id 0 만 등록(loc_type unknown). 실험용 태그 크기가 다르면 여기 size_m 를 맞춰야 거리 추정이 맞음.
- 출력 파이프: `tag_detections` (+ `tracking_front_tag_overlay`, `tracking_down_tag_overlay` 영상 오버레이). 읽기: `voxl-inspect-tags`.
- 2026-10-30 임시 실행 결과: 아래 §6 참고.

## 6. 2026-10-30 `voxl-inspect-tags` 실행 결과
- 평소 서비스가 꺼져 있어 바로 실행하면 `waiting for server at tag_detections` 만 출력됨.
- `voxl-tag-detector` 를 임시(28 s, 서비스 미변경)로 띄우면 `/run/mpa/tag_detections`, `tracking_front_tag_overlay`,
  `tracking_down_tag_overlay` 파이프가 생기고, 탐지기 로그에 두 카메라 연결·렌즈 파라미터 로드가 확인됨.
- 그 상태에서 `voxl-inspect-tags` 10 s 캡처: **탐지 출력 없음** (142 bytes, 대기 메시지만). 카메라 시야에 tag36h11 태그가
  없어 검출이 0건인 상황. 태그를 비춰야 id / 위치(T_tag_wrt_cam) / 회전 / 크기가 줄 단위로 출력됨.
- 출력 구조체(`tag_detection_t`) 파서와 테스트는 이 저장소 `analysis/mpa_tag_detections.py`, `analysis/test_mpa_tag_detections.py` 참고.
- 실행 후 서비스 상태 inactive, 잔여 프로세스 0 — 기체 설정 변경 없음.
