# Starling 2 → SITL 재현용 인계 자료 (2026-10-27)

요청받은 3개 묶음(기체 물리값 / PX4 파라미터 / ulog)에 대한 답. 기체: ModalAI **Starling 2** (VOXL2 M0054,
hostname `m0054`), voxl-px4 **1.14.0-2.0.133**, voxl-suite 1.6.3, SKU MRB-D0014-4-V1-C26-T8-M36-X0.

## 상태 요약
| 항목 | 상태 |
|---|---|
| PX4 파라미터 전체 | ✅ `params/starling2_full_export.params` (870개, QGC Save-to-file 형식) |
| SITL용 발췌 | ✅ `params/starling2_sitl.params`, `params/starling2_sitl_param_set.txt` |
| PX4 파라미터 파일(바이너리) | ✅ `params/params_starling2_export.bin` — `px4-param save` 출력(BSON). 이 빌드엔 `export` 명령 없음. SITL `param load` 또는 참고용 |
| 비행 ulog | ✅ `ulog/` 4개 (2026-08-26 sess170). **sess170_log102.ulg 권장** (오프보드 60 s, 1.8 m) |
| 이륙 중량 실측 | ❌ 저울 측정 필요 (아래 참고) |
| 관성 모멘트 | ❌ 측정값 없음. 기하 치수만 제공 → CAD 추정 |

## 1. 기체 물리값
| 항목 | 값 | 근거 / 비고 |
|---|---|---|
| 이륙 중량 (공식) | **285 g** (배터리 제외 182 g) | ModalAI 스펙, Sony VTC6 3000 mAh 2S 팩 기준 |
| 이륙 중량 (우리 기체) | **실측 필요** | PX4 `BAT1_CAPACITY=3950` → 4000 mAh 팩 장착 가능성. 18650 4000급 셀은 3000급보다 무거워 **약 300~330 g** 예상. 배터리 포함 상태로 저울 측정 요망 |
| 호버 추력 비율 | **MPC_THR_HOVER = 0.34** (ulog 비행 중 estimator 평균 **0.356~0.357**) | normalized thrust(추력 모델 적용 전). 즉 **최대추력 합 ≈ 중량 / 0.35 ≈ 2.8 × 중량** |
| 추력 모델 | THR_MDL_FAC = 0.9 | rel_thrust = 0.9·u² + 0.1·u. 호버 시 모터 지령 u ≈ 0.56 |
| ESC | VOXL ESC, 폐루프 RPM 제어 | VOXL_ESC_RPM_MIN 2000 / RPM_MAX 15000 (지령 0~1 → RPM 선형). **호버 RPM 실측 ≈ 9,200~9,700 rpm** (ulog `esc_status` 공중 중앙값, 모터별 9322/9534/9675/9187). 모델 추정 9,300 rpm과 일치 |
| 모터 / 프롭 | **1504 3000 KV / 120 mm 접이식 3엽** | 공식 스펙. 2S 전용 (3S 금지) |
| 모터당 최대 추력 (유도) | **≈ 2.0 N (285 g 기준) ~ 2.4 N (330 g 기준)** | 285×9.81/0.34/4. 추력시험대 실측값 없음. 기존 가정 2.8 N과 같은 자릿수 |
| 배터리 | 2S Li-Ion 18650, 3950 mAh 설정 | BAT1_V_CHARGED 4.1 / V_EMPTY 2.8 / R_INTERNAL 0.022 Ω. 공칭 7.4 V, 만충 8.2 V |
| 로터 위치 (body FRD, m) | R0 (+0.085, +0.0625) R1 (−0.085, −0.0625) R2 (+0.085, −0.0625) R3 (−0.085, +0.0625) | CA_ROTOR*_PX/PY. 대각 모터 간 거리 0.211 m, 전후 0.170 m, 좌우 0.125 m |
| 로터 회전 (KM) | R0,R1 = +0.05, R2,R3 = −0.05 | Quad X 표준 |
| 로터 추력축 | (0,0,−1), CA_ROTOR*_CT 6.5 (기본) | |
| IMU 위치 (m) | (0.0043, 0.0073, −0.016) | EKF2_IMU_POS_* |
| VOXL2 보드 | 70 × 36 mm, 16 g | 관성 추정 시 참고 |
| 관성 모멘트 | 없음 | 위 치수 + 질량 분포(보드 중앙, 배터리 하단)로 CAD/박스 근사. 필요하면 bifilar pendulum 실측 가능 |

## 2. PX4 파라미터 (요청 그룹별 값)
전체는 `params/starling2_full_export.params`. 요청하신 항목만 발췌:

| 그룹 | 값 |
|---|---|
| 각속도 PID | ROLLRATE P 0.072 / I 0.171 / D 0.0009 · PITCHRATE P 0.097 / I 0.228 / D 0.0011 · YAWRATE P 0.15 / I 0.5 / D 0 (K 1.0) |
| 자세 P | MC_ROLL_P 16 · MC_PITCH_P 16 · MC_YAW_P 2.8 |
| 각속도 한계 | ROLL/PITCHRATE_MAX 130 °/s · YAWRATE_MAX 150 °/s · D-term cutoff 30/30/10 Hz |
| 위치 P | MPC_XY_P 3.5 · MPC_Z_P 5.0 |
| 속도 PID | XY_VEL P_ACC 3.0 / I_ACC 0.1 / D_ACC 0 · Z_VEL P_ACC 8.0 / I_ACC 2.0 / D_ACC 0 |
| 추력 | MPC_THR_HOVER 0.34 · THR_MIN 0.07 · THR_MAX 0.6 |
| 속도 한계 | XY_VEL_MAX 5.0 · XY_CRUISE 5.0 · Z_VEL_MAX_UP 3.0 / DN 1.5 · Z_V_AUTO_UP/DN 0.5 |
| 가속/저크 | MPC_ACC_HOR 3.0 · ACC_HOR_MAX 1000 · ACC_UP_MAX 3 · ACC_DOWN_MAX 3 · JERK_MAX 40 · **JERK_AUTO 4.0** · MPC_POS_MODE 4 |
| 틸트 | MPC_TILTMAX_AIR 30° · MAN_TILT_MAX 35° · TILTMAX_LND 12° |
| VIO 융합 | EKF2_EV_CTRL 7 · EV_DELAY 0 ms · EVP/EVV/EVA_NOISE 0.1 · EV_NOISE_MD 0 · EV_QMIN 1 · HGT_REF 3 · GPS_CTRL 0 · MAG_TYPE 5 · RNG_CTRL 1 · BARO_CTRL 1 |
| 제어 할당 | CA_ROTOR_COUNT 4, PX/PY/KM 위 표, CA_AIRFRAME 0, SYS_AUTOSTART 4001 |
| 추력 모델 | THR_MDL_FAC 0.9 |
| 이착륙 | MPC_TKO_SPEED 0.5 · TKO_RAMP_T 1.0 · LAND_SPEED 0.6 · MIS_TAKEOFF_ALT 0.5 · COM_DISARM_LAND 0.1 s · COM_DISARM_PRFLT 20 s · COM_SPOOLUP_TIME 2 s |
| 착륙 감지 | LNDMC_TRIG_TIME 0.5 · ROT_MAX 30 · XY_VEL_MAX 1.5 · Z_VEL_MAX 0.25 |
| 오프보드/실패안전 | COM_RC_IN_MODE 3 · COM_OBL_RC_ACT 0 · COM_OF_LOSS_T 1.0 · NAV_RCL_ACT 2 · COM_OBS_AVOID 0 · GF_* 0 · CP_DIST −1 |

주의: `params/starling2_full_export.params` 는 QGC "Load from file" 로 그대로 올라가지만, SITL에는 CAL_*, TC_*,
IMU_* 필터, RC*, VOXL_ESC_*, SENS_BOARD_* 를 빼고(= `starling2_sitl.params`) 넣는 것을 권장.

## 3. ulog (`ulog/`, 2026-08-26 sess170, SDLOG_PROFILE 129 + esc_status 포함)
| 파일 | 길이 | 공중 | 최고고도 | 모드 | 호버추력(est) | 공중 ESC RPM 중앙값 (M1~M4) | 용도 |
|---|---|---|---|---|---|---|---|
| **sess170_log102.ulg** | 60.8 s | 57.6 s | 1.78 m | Offboard(14) → Auto Land(18) | 0.357 | 9322 / 9534 / 9675 / 9187 | **기본 비교 로그**. 오프보드 setpoint 추종 응답 |
| sess170_log100.ulg | 48.2 s | 43.0 s | 0.62 m | Position(2) | 0.356 | 9418 / 9494 / 9837 / 9150 | 수동 Position 호버. 호버 추력/RPM 정상상태 |
| sess170_log103.ulg | 10.6 s | 8.1 s | 0.90 m | Manual→Position→Offboard | 0.375 | 8913 / 9135 / 9558 / 9163 | 짧음, 참고용 |
| sess170_log104.ulg | 6.5 s | 5.2 s | 0.06 m | Manual/Position | 0.365 | — (지상 스풀업) | 비행 아님 |

- 분석 도구: `pyulog` (`ulog_info`, `ulog2csv`). 위 표는 `vehicle_status.arming_state/nav_state`, `vehicle_land_detected.landed`,
  `vehicle_local_position.z`, `hover_thrust_estimate.hover_thrust`, `esc_status.esc[i].esc_rpm` 로 산출.
- 당시 파라미터는 로그 헤더에 그대로 들어 있음(`ULog.initial_parameters`). 2026-10-27 덤프와 대조 가능.
