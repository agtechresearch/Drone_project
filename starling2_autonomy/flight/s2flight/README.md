# s2flight — Starling 2 실험 비행 공용 패키지

VOXL 2 / PX4 / MAVSDK 기체에서 실험 비행 코드를 짤 때 쓰는 패키지. 기체 파이썬 **3.6**, mavsdk **0.12**,
numpy·cv2 없음을 전제로 표준 라이브러리만 쓴다. 첫 사용처는 마커 드리프트 실험
([`flight/marker_drift_flight.py`](../marker_drift_flight.py), [docs/09](../../docs/09_marker_drift_experiment_plan.md)).

`path_flight_phase1_v14.py` 는 건드리지 않았다. `vehicle/geofence/pose/recorder` 는 v14 `DroneController` 를
동작 변경 없이 분해한 것이고, `tags/align/mpa` 는 `mpa_tag_detections.py`·`tag_align.py`·`mpa_point_cloud.py` 의
사본이다. **앞으로의 실험 코드는 이 패키지를 쓰고, 원본 파일들은 참조용으로 남긴다.**

## 모듈

| 모듈 | 내용 | 출처 |
|---|---|---|
| `util` | `mono_time()`(CLOCK_MONOTONIC), 각도 정규화, body→NED, 추종 평가, `log()` | v14 모듈 함수 |
| `config` | `FlightConfig` 비행 공통 파라미터 (기본값 = v14; 단 `initial_yaw="current"` 는 신규), `add_arguments`/`from_args` | v14 argparse |
| `geofence` | `Geofence.check(now, target, actual, home)` 추종 오차 + 홈 절대 펜스 | v14 `_geofence_check` |
| `pose` | `voxl-inspect-pose -n <pipe>` 파서·리더 태스크 | v14 |
| `recorder` | `FlightRecorder`: 동기 CSV(+`mono_ns`), 이벤트, 단계 샘플, 추가 표, summary.json | v14 CSV 확장 |
| `vehicle` | `Vehicle`: 연결·프리플라이트·**정면 기준 `capture_initial_yaw`**·arm/offboard·`takeoff/hover/go_to_relative/move_body/go_yaw/turn/preland/land`·**`move_until`(도달 판정 없는 조건 정지)**·비상 | v14 `DroneController` (+정면 기준) |
| `voxl_logger` | `VoxlLogger.start()/stop()` — voxl-logger 자식 프로세스 (SIGINT 종료, `-t` 상한) | 신규 |
| `mpa` | `/run/mpa` 파이프 구독 공용 클라이언트 | `mpa_point_cloud.py` |
| `tags` | `tag_detections` 레코드 파서, `FrameAssembler`, `TagFeed`/`MpaTagFeed`(asyncio add_reader) | `mpa_tag_detections.py` |
| `align` | 태그 연결선 yaw 오차 `measure_alignment`, 폐루프 `AlignController` | `tag_align.py` |
| `align_stage` | `run_alignment(vehicle, feed, params, recorder)` — 정렬 단계 실행기, t0 확정 | 신규 |
| `drift` | `BannerLayout`, `DriftMonitor`(태그 프레임마다 실제 위치 vs 자기 추정 기록, `centered_stop` 정지 조건), `center_on_tag` 시작점 센터링 | 신규 (2026-11-09 설계) |
| `sim` | SITL 입력 어댑터: `sim_pose_task`(MAVSDK 텔레메트리 → 포즈 dict), `UdpTagFeed`(시뮬 검출기 UDP JSON → TagFrame). `FlightConfig.sim` / `--sim` | 신규 (v13 `--sim` 과 같은 접근) |

## 새 실험 스크립트 쓰는 법

```python
import asyncio, argparse
from s2flight.config import FlightConfig
from s2flight.recorder import FlightRecorder
from s2flight.vehicle import Vehicle

async def fly(cfg, rec):
    v = Vehicle(cfg, recorder=rec)
    await v.connect(); await v.wait_position_stable(); await v.check_vio_quality_preflight()
    await v.capture_initial_mavsdk_position()
    await v.start_monitors_and_offboard()          # 스트림·감시 → arm → offboard → 지오펜스 ARM
    try:
        await v.takeoff(0.6)
        await v.turn(-90, "aim", "LEFT90")         # 상대 yaw (음수 = 좌회전)
        await v.move_body(0.0, 2.0, "right2m", "RIGHT", speed_mps=0.5)   # 계획 yaw 기준 우측 2 m
        await v.hover(5.0, "hold")
        await v.preland_and_land()
    except Exception:
        await v.emergency_stop(); raise
    finally:
        v.print_summary(); rec.summary["vehicle"] = v.summary_dict(); rec.write_all()

p = argparse.ArgumentParser(); FlightConfig.add_arguments(p); args = p.parse_args()
cfg = FlightConfig.from_args(args)
rec = FlightRecorder(cfg, out_dir="/home/root/logs/my_exp", prefix="run1")
asyncio.get_event_loop().run_until_complete(fly(cfg, rec))    # py3.6: asyncio.run 없음
```

규칙
- **계획 위치**(`plan_n/e/alt/yaw`)는 "명령대로 갔다면 있어야 할 곳". `move_body`/`turn` 은 여기서 상대 이동한다.
  단계가 실패하면 계획 위치를 갱신하지 않고 현재 피드백 위치를 setpoint 로 잡는다(v14 와 동일).
- 도달 판정은 `--reach-pipe`(기본 `px4_vehicle_local_position`) 포즈로만. MAVSDK 텔레메트리는 지오펜스·비교용.
- `v.aborted` 가 True 면 모든 원시명령은 즉시 False 를 돌려준다. 시퀀스는 단계마다 `if v.aborted` 를 보고 착륙으로 간다.
- 시각은 `util.mono_time()`(CLOCK_MONOTONIC). voxl-logger `data.csv` 의 `timestamp(ns)`, `tag_detections` 의
  `timestamp_ns` 와 같은 시계라 `t_offset = 0` 으로 맞출 수 있다.
- 실험 고유 열은 `v.set_sync_field("key", value)` 로 동기 CSV 에 붙인다. 순간 기록은 `v.event("name", detail, **fields)`.
- 기체 파이썬 3.6: `asyncio.run`, `create_task`, dataclass, `monotonic_ns`, `text=` 금지. `asyncio.ensure_future` 사용.

## 테스트 / 배포

```bash
python analysis/test_s2flight.py            # 79항목: 단위 + 가짜 MAVSDK·가짜 벽 카메라로 전체 시퀀스
./tools/deploy_pkg.sh                       # flight/s2flight + flight/marker_drift_flight.py → 기체 $DRONE_DIR
./tools/deploy_pkg.sh flight/<다른 실험>.py
```

기체에서 패키지는 `$DRONE_DIR/s2flight/` 에, 진입 스크립트는 `$DRONE_DIR/` 에 놓인다. 진입 스크립트가
`sys.path` 에 자기 폴더를 넣으므로 같은 폴더에 있기만 하면 된다.

## 산출물 (FlightRecorder)

| 파일 | 내용 |
|---|---|
| `<prefix>_sync.csv` | 주기 샘플: `mono_time/mono_ns/unix_time/wallclock`, 단계·phase, 목표 rel, 피드백 cmd/rel/yaw, MAVSDK abs/rel, 파이프별 `voxl_<pipe>_*`, 실험 열 |
| `<prefix>_events.csv` | `armed, offboard_started, stage_start/end, hover_start/end, align_*, aligned(t0), land_cmd, landed, disarmed, emergency_land` |
| `<prefix>_stages.csv` | 단계 진행 중 MAVSDK 갱신마다의 샘플 (도달 지연 분석용) |
| `<prefix>_<table>.csv` | `recorder.add_row(table, row)` 로 넣은 추가 표 (예: `align_log`) |
| `<prefix>_summary.json` | 파라미터·설정·결과 표·스냅샷·voxl-logger 폴더 |

## 비행 전 확인 (패키지는 실비행 미검증)

v14 코드를 그대로 옮겼지만 파일이 달라졌으므로 "실비행 실적" 은 새로 쌓아야 한다. 프롭 탈착이 필요한 지상 arm 시험은
하지 않는다 (프롭을 자주 빼면 안 됨, 2026-11-09). 대신 **시뮬레이션에서 먼저** 돌린다.
1. SITL (연구실 서버, `sim/marker_drift_gz/README.md`): `sim/run_sim_drift.sh --no-align --distance 0.5 --condition T` 로 패키지 점검,
   `sim/run_sim_drift.sh --condition A --run 1` 로 정렬 포함 본 시퀀스. 입력 어댑터(`s2flight.sim`, `--sim`)만 다르고 비행 로직은 같다.
2. 실기 짧은 호버·회전 시험: `--no-align --distance 0.5 --hover-ref 3 --hover-far 2 --hover-return 2 --condition T` 로
   이륙→좌회전→0.5 m 왕복→착륙. 요약 표가 전부 `OK`, `fb_end_rel` 수 cm 이면 통과.
3. 그 다음에 태그 정렬 포함 파일럿 비행.
