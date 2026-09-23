# 10. 기체 실측 조사 — 마커 드리프트 실험 준비 (2026-09-23)

> docs/09 §10 `[기체 확인]` 8항목을 SSH 접속 1회로 조사한 결과. 기체 상태 변경 없음
> (voxl-logger 8초 시험 기록 `/data/voxl-logger/log0002` 117 MB 생성이 유일한 쓰기).
> 기체: m0054, SDK 1.6.3, system image 1.8.06, 기체 우측 조사 시점 CPU 40 °C, 비행 전 벤치 상태.

---

## 0. 결론 요약

| # | 항목 | 결과 | 설계에 미치는 영향 |
|---|---|---|---|
| 1 | hires 스트림 | `hires_small_color` **1024×768 30 fps NV12** 와 `hires_large_color` **4056×3040 30 fps** 두 파이프가 항상 흐름 | 7 cm 태그 한 변 = small 약 41 px / large 약 160 px. 후처리 정밀도가 필요하면 large 를 기록 |
| 2 | tracking_front | 1280×800 30 fps RAW8, 어안. 캘리브레이션 파일 있음 (fx≈461, fisheye 4계수) | 실시간 정렬 입력 확정 |
| 3 | hires 캘리브레이션 | **없음** (`/data/modalai/` 에 tracking_front/down, seek, uvc 만) | 체커보드 촬영 필요 → `marker_drift calibrate` |
| 4 | 태그 검출기 | `voxl-tag-detector` 0.1.0 **설치됨, 비활성**. 설정은 tracking_front·tracking_down 입력, tag36h11, 5프레임 건너뜀, 어안 보정 포함. `tag_locations.conf` 기본 크기 **0.4 m** | 실시간 정렬은 이 서비스의 `tag_detections` 파이프를 파이썬으로 구독. **크기 0.07 m 로 설정 변경 필요** |
| 5 | 기체 파이썬 | 3.6.9, numpy 1.13.3, mavsdk 0.12.0. **cv2·apriltag 없음** | 기체에서 파이썬 AprilTag 검출 불가 → 4번 경로가 유일 |
| 6 | vision-hub | `vio_pipe` "qvio"(비활성) → 2차 `ov` 사용. `en_tag_fixed_frame` **false**. `en_voa` true | 태그 고정 프레임은 꺼져 있음. 켜면 자체 드리프트 지표가 되지만 PX4 로 가는 odometry 에 영향 → 실험 중에는 끈 채로 두고 후처리로만 비교 |
| 7 | voxl-logger | 0.6.1. 카메라(jpg + data.csv), `ov`, `px4_vehicle_local_position`, `imu_apps` 동시 기록 확인. 8 s 에 117 MB. 여유 61 GB | 후처리 입력 형식 확정 (§3). 파이프라인에 로더 추가 완료 |
| 8 | 시각 동기 | 모든 채널 `timestamp(ns)` = CLOCK_MONOTONIC | 영상·포즈 t_offset = 0 |
| 9 | 네트워크 | **sta+ap 동시**: 랩 와이파이 AgtechLab `192.168.45.156`(mlan0) + 자체 AP `VOXL-2969670613` `192.168.8.1`(uap0). tailscale 설치돼 있으나 로그아웃 | 랩 유선 PC 에서 `starling2_lab` alias 로 바로 접속 가능 |
| 10 | 손 검출 시험 | **미실시** (태그 인쇄 필요, 검출기 가동은 상태 변경이라 별도 승인) | 다음 접속에 |

---

## 1. 카메라

`/etc/modalai/voxl-camera-server.conf` (fsync 켜짐, 4대):

| 카메라 | 타입 | id | 파이프 | 해상도 | fps | 비고 |
|---|---|---|---|---|---|---|
| tracking_front | ar0144 | 0 | `tracking_front` (RAW8), `_misp_grey/_norm`, `_misp_encoded`(h265 1.2 Mbps) | 1280×800 | 30 | en_rotate true, ae lme_msv, 노출 20~12000 µs |
| hires | imx412 | 1 | `hires_small_color` (NV12), `hires_large_color`, `hires_small/large_encoded`(h264), `hires_snapshot` | small 1024×768, large 4056×3040 | 30 | en_preview false, ae isp |
| tof | pmd-tof-liow2 | 2 | `tof`, `tof_depth`, `tof_conf`, `tof_ir`, `tof_pc` | 240×180 | 10 | |
| tracking_down | ar0144 | 3 | `tracking_down` 등 | 1280×800 | 30 | |

`voxl-inspect-cam hires_small_color`: 1024×768, 30.0 fps, 지연 39 ms, 노출 33 ms, gain 1100(실내). `hires_large_color` 도 30 fps.

**태그 픽셀 크기 재계산** (이격 50 cm, 7 cm 태그, 벽이 화상면과 평행):
- hires 화각은 캘리브레이션 전이라 스펙 120.4°로 가정. `hires_small_color` 1024 px → fx ≈ 296 → 한 변 **41 px**. `hires_large_color` 4056 px → fx ≈ 1170 → **164 px**.
- 41 px 는 검출은 되지만(합성 검증 5.5 px/모듈) 포즈 정밀도가 떨어진다. **후처리 기록은 `hires_large_color` 권장**, 단 30 fps × 18.5 MB(NV12) 를 jpg 로 저장하는 부담을 다음 접속에서 `-t 10` 시험으로 확인(CPU·드롭). 대안: `--skip` 으로 10~15 fps.
- tracking_front (fx 461, 1280 px, 어안): 한 변 **약 65 px** (중앙 부근) → 실시간 정렬에 충분.

### hires 위치 (extrinsics.conf, D0014_Starling_2)
hires wrt imu_apps `[0.0388, 0, 0.0186]`, RPY `[0, 90, 90]`; imu_apps wrt body `[0.0295, -0.0065, -0.016]` → hires 는 기체 중심에서 **전방 6.8 cm, 우측 -0.65 cm, 아래 0.26 cm**. tracking_front wrt imu_apps `[0.037, 0, 0.0006]`. yaw 일정한 비행에서는 상수 평행이동으로 정합에 흡수(docs/09 §7).

## 2. 캘리브레이션 파일

```
/data/modalai/opencv_tracking_front_intrinsics.yml   fisheye, 1280x800, fx 460.99 fy 460.79 cx 621.58 cy 382.33
                                                     D [0.0598, 0.00741, -0.00571, 0.00185], reproj 0.169 px, 2023-03-02
/data/modalai/opencv_tracking_down_intrinsics.yml    (동일 형식)
/data/modalai/voxl-imu-server.cal                    2026-05-15
(hires 없음)
```
OpenCV FileStorage 형식 → `marker_drift.intrinsics.Intrinsics.load()` 가 그대로 읽는다(`distortion_model: fisheye` 자동 인식).

## 3. voxl-logger 기록 형식 (시험 기록 log0002)

명령: `voxl-logger -c hires_small_color -v ov -f px4_vehicle_local_position -i imu_apps -t 8 -n marker_drift_test`
결과: cam 258 / vio 232 / pose 810 / imu 8190 샘플, 117 MB.

```
/data/voxl-logger/logNNNN/
├── info.json                       채널 목록, start_time_monotonic_ns, duration
├── etc/modalai/, data/modalai/     설정·캘리브레이션 사본 (재현용)
└── run/mpa/
    ├── hires_small_color/          00000.jpg ... + data.csv
    │     i,timestamp(ns),gain,exposure(ns),format,height,width,frame_id,reserved
    ├── ov/data.csv                 i,timestamp(ns),T_imu_wrt_vio_x/y/z(m),roll/pitch/yaw(rad),vel_*,angular_vel_*,
    │                               gravity_vector_*,T_cam_wrt_imu_*,imu_to_cam_*,features,quality,state,error_code
    ├── px4_vehicle_local_position/data.csv
    │                               i,timestamp(ns),T_ch_wrt_par_x/y/z(m),roll/pitch/yaw(rad),vel_ch_wrt_par_*,angular_vel_*
    └── imu_apps/data.csv           i,timestamp(ns),batch_id,AX,AY,AZ,GX,GY,GZ,T(C)
```
- `timestamp(ns)` 는 모두 CLOCK_MONOTONIC. 카메라 프레임과 포즈를 t_offset 없이 맞출 수 있다.
- `ov` 첫 행은 timestamp `-1000000000`(미초기화) → 로더가 버린다.
- 벤치에서는 `px4_vehicle_local_position` x,y 가 nan (VIO 미초기화) → 로더가 버린다.
- 파이프라인 사용: `detect --frames-dir .../hires_small_color --timestamps .../hires_small_color/data.csv --ts-col "timestamp(ns)" --ts-scale 1e-9`,
  `analyze --log .../px4_vehicle_local_position/data.csv --log-format voxl-logger`.
- 명령 setpoint(P_cmd) 는 voxl-logger 에 없다. 경로 이탈 d(t) 는 v14 CSV(`--csv auto`) 를 함께 써야 한다.

## 4. 태그 검출기

- `voxl-tag-detector` 0.1.0, 서비스 **Disabled / Not Running**.
- `/etc/modalai/voxl-tag-detector.conf`: detector_0 = `tracking_front` (en_undistortion true, undistort_scale 0.6, lens_cal = tracking_front intrinsics, `skip_n_frames 5`, tag36h11), detector_1 = `tracking_down` 동일, detector_2 비활성.
- `/etc/modalai/tag_locations.conf`: id 0 만 등록, `loc_type unknown`, **size 0.4 m**, `default_size_m 0.4`.
- 출력 파이프 `tag_detections`, 구조체 `tag_detection_t` **252 바이트 packed**:
  `uint32 magic, int32 id, float size_m, int64 timestamp_ns, char name[64], int loc_type, float T_tag_wrt_cam[3], float R_tag_to_cam[3][3], float T_tag_wrt_fixed[3], float R_tag_to_fixed[3][3], char cam[64], int reserved`
  → 파이썬 `struct` `"<IifqQ..."` 대신 `"<I i f q 64s i 3f 9f 3f 9f 64s i"` 로 언팩 (총 252 B 확인 필요). `mpa_point_cloud.py` 의 순수 파이썬 구독 패턴 재사용.
- **실험용 변경 필요(다음 접속, 승인 후)**: `tag_locations.conf` 에 id 0~11, 20~31 을 size 0.07 로 등록(`default_size_m 0.07`), `skip_n_frames` 를 5 → 0~1 로 낮춰 30 fps 중 15~30 fps 검출, detector_1(tracking_down) 은 끄기. 서비스는 실험 중에만 `systemctl start` 하고 enable 은 하지 않는다.
- 크기가 0.4 m 로 잡혀 있으면 T_tag_wrt_cam 거리가 5.7배로 나온다. 크기 설정이 정렬 yaw 에는 영향이 없지만(회전은 크기와 무관) 위치에는 직접 영향.

## 5. vision-hub / VIO 상태

- `voxl-vision-hub.conf`: `en_vio true`, `vio_pipe "qvio"`, `secondary_vio_pipe "ov"`, `offboard_mode "off"`, `en_tag_fixed_frame false`, `fixed_frame_filter_len 5`, `en_transform_mavlink_pos_setpoints_from_fixed_frame false`, `en_voa true`.
- 벤치 상태 `voxl-inspect-vio ov`: features 0, quality 15~99 % 요동, INIT/OKAY 반복. `vvhub_aligned_vio`: FAIL −1 %. 정지 벤치라 참고값 아님. **프리플라이트에서 quality·features 를 다시 확인**(v14 절차).
- 진동(`voxl-inspect-vibration`): 모터 정지 상태 전부 GREEN(0.01~0.04 m/s²). 비행 중 값이 기준선.
- CPU: 8코어 39~40 °C, 사용률 24 %. camera-server 58 %, open-vins 47 %, px4 29 %(코어 기준). 메모리 2.6/7.7 GB. hires_large 기록 시 여유 확인 필요.
- **tailscaled 가 실행 중**(로그아웃 상태). 2단계 현장 네트워크 후보.

## 6. 네트워크 / 접속

- WiFi `sta+ap` 동시 모드. Station: `AgtechLab` → `192.168.45.156`. SoftAP: `VOXL-2969670613` → `192.168.8.1`.
- 로컬 `~/.ssh/config`: `starling2`(AP, 192.168.8.1), `starling2_lab`(랩망, 192.168.45.156) 두 alias. 공개키 등록 완료(2026-09-23). `.env.local` 은 `DRONE_HOST=starling2`.
- 랩 유선 PC 에서는 `starling2_lab` 을 쓰면 PC 와이파이를 옮기지 않아도 된다. 랩 DHCP 가 IP 를 바꾸면 alias 갱신.

## 7. 다음 접속에서 할 일

1. **손 검출 시험** — 7 cm 태그 인쇄, `tag_locations.conf` 크기 0.07 로 임시 변경 후 `voxl-tag-detector` 전경 실행,
   `voxl-inspect-tags` 로 50 cm / 1 m 에서 검출·거리 확인. tracking_front 에서 x=0 위치 태그 0·1 동시 가시 확인.
2. **hires 캘리브레이션** — 체커보드를 `voxl-logger -c hires_large_color -t 20` 으로 찍어 `marker_drift calibrate`.
   small 과 large 는 같은 센서의 다른 크롭/스케일이므로 large 로 캘리브레이션하고 small 은 스케일링 검증.
3. **hires_large_color 기록 부하 시험** — `-t 10` 기록 중 `voxl-inspect-cpu`, 드롭 여부(`info.json dropped_data`).
4. **tag_detections 파이썬 구독기** — `tag_detection_t` 252 B 언팩, `mpa_point_cloud.py` 패턴. 정렬 모듈의 입력.
5. 정지 30 s 기록 → `marker_drift noise` 로 허용오차 첫 값.
6. (7월 미결) `voa_pc_out` 좌표 부호 실측.
