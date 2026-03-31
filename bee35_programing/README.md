# GPS 음영 온실 환경에서 AprilTag 기반 자율 UAV 정렬 시스템

> CIGR 2026 포스터 발표  
> 컴패니언급 온보드 하드웨어를 활용한 비주얼 서보잉 기반 실내 드론 비행 시스템

---

## 개요

본 프로젝트는 GPS가 없는 온실 환경에서 AprilTag 마커를 인식하여 PID 기반 비주얼 서보잉으로 드론을 정렬하는 자율 비행 시스템입니다. 조종 신호를 직접 제어하는 방식으로 마커 중심 오차를 보정하며, 제안 알고리즘의 성능을 순수 LOITER 모드와 비교 실험을 통해 검증합니다.

---

## 하드웨어

| 부품 | 사양 |
|---|---|
| 비행 제어기 | MicoAir H743 V2 + 45A ESC AIO |
| 온보드 컴퓨터 | Raspberry Pi 5 |
| 카메라 | Pi Camera V3 |
| 위치 유지 | 옵티컬 플로우 센서 + LiDAR |
| 펌웨어 | ArduPilot (LOITER / ALT_HOLD 모드) |

**시리얼 포트 배정**
- Serial1 → Raspberry Pi (MAVLink)
- Serial3 → GPS (비행에 미사용, 참고용)
- Serial4 → RC 수신기
- Serial6 → 옵티컬 플로우 센서

---

## 시스템 구조

```
libcamera-vid (TCP 스트림)
        ↓
카메라 스레드 (AprilTag 감지, PID 오차 계산)
        ↓
비행 루프 (RC 오버라이드 → Pixhawk)
        ↓
MAVProxy → 미션플래너 (모니터링)
```

**실행 순서**
```bash
# 터미널 1 — 카메라 스트림
libcamera-vid --width 640 --height 480 --codec mjpeg --framerate 60 -t 0 --listen -o tcp://127.0.0.1:8888

# 터미널 2 — MAVProxy
mavproxy.py --master=/dev/ttyAMA0 --baudrate 115200 \
  --out=udp:192.168.45.242:14550 --out=udp:127.0.0.1:14551

# 터미널 3 — 비행 스크립트
python alighn_l.py   # 정렬 비행
python loit.py       # 순수 LOITER 비교 비행
python alighn_a.py   # ALT_HOLD 전환 변형
```

---

## 비행 스크립트

| 파일 | 설명 |
|---|---|
| `alighn_l.py` | LOITER 모드 + PID 비주얼 서보잉 정렬 |
| `alighn_a.py` | 첫 정렬 후 ALT_HOLD 모드로 전환 |
| `loit.py` | 순수 LOITER 모드 비교 기준선 (보정 없음) |
| `camtest.py` | 카메라 정렬 테스트 (손으로 들고 확인용, 비행 없음) |

### 주요 설정값

```python
TARGET_ALT_M       = 0.85   # 목표 호버링 고도 (m)
ALIGN_THRESHOLD_PX = 60     # 좌우 정렬 허용 범위 (px)
PITCH_THRESHOLD_PX = 30     # 앞뒤 정렬 허용 범위 (px)
TARGET_MARKER_PX   = 90     # 목표 거리에서의 마커 크기 (px)
MAX_CORRECTION     = 100    # RC 오버라이드 최대 보정값 (μs)
UPDATE_HZ          = 60     # 제어 루프 주기
NO_DETECT_HOLD_S   = 0.5    # 마커 미감지 시 마지막 보정값 유지 시간 (s)
```

### PID 게인 (`alighn_l.py`)

```python
KP_ROLL, KI_ROLL, KD_ROLL    = 0.15, 0.005, 0.10
KP_PITCH, KI_PITCH, KD_PITCH = 0.30, 0.02,  0.05
I_LIMIT = 20
```

### PID 게인 (`alighn_a.py` — 튜닝 버전)

```python
KP_ROLL, KI_ROLL, KD_ROLL    = 0.15,  0.005, 0.18
KP_PITCH, KI_PITCH, KD_PITCH = 0.35,  0.02,  0.12
KP_SCALE_OUT_ROLL  = 2.5   # 허용범위 밖 roll KP 배율
KP_SCALE_OUT_PITCH = 4.0   # 허용범위 밖 pitch KP 배율
NO_DETECT_HOLD_S   = 0.5
```

---

## 로그 시스템

### 실시간 CSV 컬럼
```
elapsed_s, err_x_px, err_pitch_px,
target_roll_us, target_pitch_us,
actual_roll_deg, actual_pitch_deg,
vib_x, vib_y, vib_z, aligned
```

### 요약 CSV 컬럼
```
attempt, align_time_s, total_abs_err_px, rms_vib
```

### 파일명 규칙
```
logs_/alighn_l_realtime_YYYYMMDD_HHMMSS.csv
logs_/alighn_l_summary_YYYYMMDD_HHMMSS.csv
logs_/loiter_realtime_YYYYMMDD_HHMMSS.csv
logs_/loiter_summary_YYYYMMDD_HHMMSS.csv
```

---

## 분석 스크립트

| 파일 | 설명 |
|---|---|
| `fix_logs.py` | 미감지 구간 보정 (직전 오차값 유지) |
| `compare.py` | 그룹 평균 비교 (정렬 vs LOITER, 막대/산점도/히스토그램) |
| `alighncompare.py` | 논문용 고품질 그래프 (막대, 박스플롯, CDF) |
| `visualize.py` | 단일 비행 로그 시각화 |
| `visualize2.py` | 다중 비행 비교 시각화 |

```bash
python fix_logs.py          # logs_/ 폴더 미감지 구간 일괄 보정
python compare.py           # 그룹 비교 그래프 생성
python alighncompare.py     # 논문용 그래프 생성 (figures/ 폴더)
```

### 윈도우 배치 파일
```
logdown.bat       — 라즈베리파이에서 로그 SCP 다운로드
runcompare.bat    — compare.py 실행
runcompare2.bat   — alighncompare.py 실행
runvisualize.bat  — visualize.py 실행
runvisualize2.bat — visualize2.py 실행
```

---

## 실험 결과

정렬 비행 5회 vs 순수 LOITER 비행 5회 (2026년 3월 13일)

| 지표 | 정렬 알고리즘 | LOITER 단독 |
|---|---|---|
| 정렬 성공률 | **39.6%** | 26.0% |
| 평균 정렬 소요 시간 | **1.73초** | 2.71초 |
| 평균 좌우 오차 | **76.3px** | 117.6px |
| 정렬 구간 좌우 진동 폭 | 28.9px | 30.4px |
| RMS 진동 | 8.62 | 8.82 |

→ 정렬 알고리즘 적용 시 좌우 오차 **28.3% 감소**

---

## 개발 일지

→ 날짜별 상세 진행 기록은 **[PROGRESS.md](./PROGRESS.md)** 를 참고해주세요.

### 요약

### 2.28 – 3.2
- 드론 조립 완료
- Raspberry Pi 탑재 후 호버링 불안정 확인
  - 피치 방향으로 ±30cm 간헐적 이탈 발생
  - 탑재 중량 증가로 인한 옵티컬 플로우 불안정이 원인으로 확인

### 3.3
- 비행 코드 구조 확립
- CH5 킬스위치: MAVLink 패킷 리스너로 구현 (제어 루프와 독립, 실시간 반응)
- 수신기 위치 변경으로 배터리 케이블 간섭 문제 해결

### 3.4 – 3.6
- `guided_nogps` → LOITER 모드로 전환 (위치 고정 문제 해결)
- RC 오버라이드 우선순위 충돌 문제 해결
- AprilTag 감지기 통합 (ArUco 대체)
- 카메라 TCP 스트림 파이프라인 완성

### 3.9
- `2026_03_09.py`: 비례 제어만 적용 (PID 없음), MAX_CORRECTION=30μs
- 기본 정렬 루프 동작 확인

### 3.11
- `2026_03_11.py`: CSV 로깅 추가 (실시간 + 요약)
- VIBRATION 메시지 리스너 추가
- IMU roll/pitch 로깅 추가

### 3.13
- `alighn_l.py`: PID 제어 완성, MAX_CORRECTION=100μs, TARGET_ALT=0.85m, TARGET_MARKER_PX=90
- `loit.py`: 순수 LOITER 비교용 코드 작성
- 데드밴드 추가 (DEADBAND_ROLL=20px, DEADBAND_PITCH=10px)
- 5+5 비교 비행 실험 진행

### 3.14 – 3.15
- `alighn_a.py`: 첫 정렬 후 ALT_HOLD 전환 변형 개발
- 게인 스케일링 추가 (roll 허용범위 밖 KP×2.5, pitch KP×4.0)
- 미감지 유지 로직 추가 (0.5초간 마지막 보정값 유지)
- KD값 강화 (roll 0.10→0.18, pitch 0.05→0.12)
- 데드밴드 제거 (`alighn_l.py`, `alighn_a.py`)
- `fix_logs.py`: CSV 미감지 구간 일괄 보정 도구
- `compare.py`, `alighncompare.py`: 비교 분석 및 시각화 스크립트
- CIGR 2026 포스터 영문 초록 완성

---

## 초록 (영문)

> Small UAVs relying solely on optical-flow sensors struggle to achieve the precision hovering required in confined indoor agricultural environments without GPS. This study presents an autonomous flight system integrating AprilTag-based visual servoing with companion-grade onboard hardware and evaluates its stability in a GPS-denied greenhouse setting. The proposed system corrects the error between the marker center and the image frame through a PID-based outer control loop, directly commanding flight control signals to align the UAV within predefined tolerance ranges before proceeding to subsequent operations. The platform combines a Pixhawk flight controller running ArduPilot firmware with a Raspberry Pi 5, stabilized by low-cost optical-flow and LiDAR sensors. To validate precision improvement over conventional hovering, the system is compared against pure LOITER mode, which relies exclusively on optical-flow without external correction. Experimental results show an alignment success rate of 39.6% and a mean alignment time of 1.73 s, with RMS vibration of 8.62 and lateral oscillation of 28.9 px, representing significant improvement over LOITER-only operation (26.0%, 2.71 s) and a 28.3% reduction in lateral error. Further validation under varied greenhouse lighting is expected to broaden applicability in controlled-environment agriculture.

**Keywords:** UAV, visual servoing, AprilTag, GPS-denied, greenhouse automation

---

## 폴더 구조

```
Drone_project/
├── advancedroneprograming/         # 기존 드론 프로그래밍 폴더
└── bee35_programing/               # BEE35 드론 비행 시스템 (본 폴더)
    ├── README.md                   # 프로젝트 설명 (현재 파일)
    │
    ├── 비행 스크립트 (라즈베리파이에서 실행)
    ├── alighn_l.py                 # LOITER + PID 정렬 비행 (메인)
    ├── alighn_a.py                 # 첫 정렬 후 ALT_HOLD 전환 변형
    ├── loit.py                     # 순수 LOITER 비교 기준선
    ├── camtest.py                  # 카메라 정렬 테스트 (비행 없음)
    │
    ├── 분석 스크립트 (로컬 윈도우에서 실행)
    ├── fix_logs.py                 # 미감지 구간 CSV 일괄 보정
    ├── compare.py                  # 그룹 평균 비교 시각화
    ├── alighncompare.py            # 논문용 고품질 그래프
    ├── visualize.py                # 단일 비행 로그 시각화
    ├── visualize2.py               # 다중 비행 비교 시각화
    │
    ├── 배치 파일 (윈도우)
    ├── logdown.bat                 # 라즈베리파이 → 로컬 로그 동기화
    ├── runcompare.bat              # compare.py 실행
    ├── runcompare2.bat             # alighncompare.py 실행
    ├── runvisualize.bat            # visualize.py 실행
    ├── runvisualize2.bat           # visualize2.py 실행
    │
    └── logs_/                      # 비행 로그 CSV (라즈베리파이 생성)
        ├── original/               # fix_logs.py 실행 전 원본 백업
        ├── alighn_l_realtime_*.csv # 정렬 비행 실시간 로그
        ├── alighn_l_summary_*.csv  # 정렬 비행 요약 로그
        ├── loiter_realtime_*.csv   # LOITER 비행 실시간 로그
        └── loiter_summary_*.csv    # LOITER 비행 요약 로그
```

> **라즈베리파이 작업 경로**
> - 비행 스크립트: `~/bee35_programing/` (또는 `~/`)
> - 가상환경: `~/drone/`
> - 로그 저장: `~/bee35_programing/logs_/`

---

## 의존성 설치

```bash
pip install dronekit pupil-apriltags opencv-python numpy pandas matplotlib
```
