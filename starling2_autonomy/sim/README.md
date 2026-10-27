# Starling 2 → SITL 재현용 파라미터 (2026-10-27 실기체 덤프)

- 기체: ModalAI Starling 2 (M0054, hostname `m0054`), voxl-px4 **1.14.0-2.0.133**, voxl-suite 1.6.3
- 덤프 명령: `ssh starling2_lab "px4-param show"` / `px4-param show -c`(비기본값 167개)
- 인계 패키지: `handoff_2026-10-27/` (README_handoff.md, params/, raw/, ulog/, fetch_from_drone.sh)

## 핵심 값 (SITL에서 꼭 맞출 것)
| 영역 | 값 | 의미 |
|---|---|---|
| SYS_AUTOSTART | 4001 | Generic Quad X. SITL은 PX4 1.14 기준 `gz_x500`(4001)과 동일 ID |
| CA_ROTOR*_PX/PY | ±0.085 / ±0.0625 m | 실제 로터 위치. 모터 2,3 KM = −0.05 (회전방향) |
| EKF2_EV_CTRL=7, EKF2_HGT_REF=3, EKF2_GPS_CTRL=0, SYS_HAS_GPS=0, SYS_HAS_MAG=0 | VIO 전용 | GPS·자력계 없음, 외부비전(odometry)으로 위치·속도·yaw·고도 |
| MPC_XY_VEL_MAX=5, MPC_XY_CRUISE=5, MPC_Z_VEL_MAX_UP=3 / DN=1.5 | 속도 한계 | 오프보드 setpoint 추종 속도 상한 |
| MPC_ACC_HOR_MAX=1000, MPC_JERK_MAX=40, MPC_POS_MODE=4 | 가속 기반 모드 | ModalAI 기본 튜닝. CP(충돌방지)는 이 값에서 동작 불가(docs/04) |
| MPC_XY_P=3.5, MPC_Z_P=5, XY_VEL_P_ACC=3, Z_VEL_P_ACC=8 | 위치/속도 게인 | 실기 응답 재현의 핵심 |
| MPC_THR_HOVER=0.34, THR_MAX=0.6 | 호버 추력 | SITL 모델 질량/추력과 다르면 재조정 필요 |
| MPC_TKO_SPEED=0.5, MPC_LAND_SPEED=0.6, MIS_TAKEOFF_ALT=0.5 | 이착륙 | |
| COM_RC_IN_MODE=3, COM_OBL_RC_ACT=0, NAV_RCL_ACT=2 | RC 없음/오프보드 | RC 신호 없이도 arming·오프보드 허용 |
| GF_*=0, CP_DIST=-1, COM_OBS_AVOID=0 | 지오펜스·회피 꺼짐 | 실기와 동일하게 두고 v14 소프트 지오펜스는 스크립트 쪽에서 |
| LNDMC_TRIG_TIME=0.5, LNDMC_ROT_MAX=30 | 착륙감지 | |

## SITL에서 다르게 해야 하는 것
1. **VIO 입력**: 실기는 voxl-vision-hub가 qvio/ov → `VISION_POSITION_ESTIMATE/ODOMETRY`(MAVLink) 로 넣음.
   SITL에는 VIO가 없으므로 둘 중 하나:
   - (간단) SITL은 EKF2_EV_CTRL=0, EKF2_HGT_REF=0(baro) 또는 GPS 그대로 두고 **제어/한계 파라미터만** 맞춘다.
   - (정밀) gz ground-truth pose를 MAVLink ODOMETRY로 브리지해서 EKF2_EV_CTRL=7, HGT_REF=3 그대로 사용.
2. **MC_* 각속도 게인**은 실기(2S, 소형) 튜닝값. gz x500 모델에 그대로 넣으면 진동/불안정 가능 → 먼저 기본값으로 띄우고 필요 시 적용.
3. **배터리**: 실기 2S 3950 mAh. SITL 배터리 시뮬은 `SIM_BAT_*`로 별도 설정. BAT1_* 는 참고용.
4. **제외한 그룹**: CAL_*, TC_*, IMU_* 필터, RC*, VOXL_ESC_*, SENS_BOARD_* — 실하드웨어 전용.
5. 실기 MAVLink 포트: localhost 14551(onboard 앱용), 14550은 외부 GCS. vision-hub `offboard_mode=off`, `en_voa=true`(VOA 데이터 송출만).
