# v3_marker 실행 진입점 (2026-10-31, codex 검토 반영)

현재 지원하는 진입점과, 보관만 하는 옛 진입점을 구분한다. 옛 진입점은 지우지 않고 남겨 두되, `LEGACY_OK=1` 없이는 실행되지 않는다.

## 지원 (현재 파이프라인)
| 진입점 | 실행 위치 | 역할 | 결과·종료 코드 |
|---|---|---|---|
| `wk_fly.sh` | 호스트 | 비행 1회(인스턴스 2~5만) | `logs/<run>/`(→ /990data), `status.json`. 종료 코드 0 정상 / 10 미션 실패 / 11 판정 실패(원래 qvix: grid_verify, 회전·측량 월드: loc_compare 진실값 clearance FAIL) / 12 보관 실패(mkdir·복사·링크 교체) / 13 안전 착륙 / 14 판정 불가(clearance INCOMPLETE·오류·없음) / 15 status.json 못 씀 / 1·2 시작 안 됨(인스턴스 정리 실패 포함) (`run_status.sh`) |
| `wk_tofh_batch.sh` | 호스트 | 인스턴스 2~5 병렬 4레인 | `logs/tofh_<B>_runs.txt`, `logs/tofh_<B>_status.txt`. 종료 코드 = 정상이 아닌 실행 수 |
| `run_wk_instance.sh` | starling-gz | PX4 + gz 인스턴스 기동(`wk_fly.sh`가 호출) | 설치 실패 시 PX4를 시작하지 않음 |
| `stop_sitl_instance.sh N` | 호스트 | 인스턴스 N의 gz·PX4·센서, 비행 쪽 `wk_mission`·`mavsdk_server`(포트 정확 일치) 정리 | 종료 코드 0 비었음 / 1 남음 / 2 조회 실패 / 4 인스턴스 2~5 밖. 대상 PID·시작 시각·명령줄은 `logs/stop_sitl_N.log` |
| `install_tof.sh` | starling-gz | starling2_tof 모델·에어프레임 4202 설치 | 잠금. 대상 폴더 먼저 검사. 모델은 `models/.versions/<이름>.<해시>` + 심볼릭 링크 한 번 교체(`repo_export/scripts/px4_model_swap.sh`). 같으면 변경 없음 |
| `make_tof_model.py`, `make_survey_world.py` | 호스트(cad venv: rack shift) | 모델·측량 월드 생성 | `gz/models`, `gz/worlds`, `gz/survey/*_truth.json` |
| `survey_map.py`, `survey_eval.py`, `survey_web_export.py` | 호스트(cad venv) | 측량 지도, 진실값 비교, 재생 데이터 | `<run>/survey/` |
| `loc_compare.py` | 호스트 | 진실값 비교, 회전 날개–랙 최소 거리 | 표준 출력 |
| `minimap_server.py` | 호스트 | 실시간 미니맵 | http://127.0.0.1:8765 |
| `real_flight_safe.py` (+ `safety_patch.py`) | **실기** (VOXL2, Python 3.6) | 원본 `path_flight_phase1_v13.py`를 수정 없이 import해서 안전 계층만 적용해 실행. v13 인자는 `--` 뒤에 그대로. 정지(Ctrl+C·예외·타임아웃) = 착륙 + 착지 대기, 모터 kill 없음(`--allow-kill` 때만). 착륙 명령 3회 재시도, 착지 미확인 = 무장 해제 안 함, 위치·자세 1 s 무수신 = 착륙. 배포: 두 파일을 v13 옆에 | v13 자신의 종료 코드 |
| `safety_patch.py` | (모듈) | 위 안전 계층. `wk_mission.py`(SITL)와 `real_flight_safe.py`(실기)가 같은 코드를 쓴다 | - |
| `tests/test_safety_offline.py`, `tests/test_run_status.sh`, `tests/test_stop_instance.sh`, `tests/test_clearance_offline.py` | starling-flight / 호스트 | 비행 없는 안전·상태 검사 | 종료 코드 = 실패 수 |

스위치 환경변수(`WK_TAGS`, `WK_LOC`, `WK_RECHECK`, `WK_TOF_HEADING`, `WK_TOF_HOLD`, `WK_ALIGN`, `WK_SPAWN_ROT`, `WK_SPAWN_ALIGN`, `NO_VIDEO`, `WK_TOF_RAW`)는 미지정·빈값·`0`·`false`·`no`·`off`이면 끔, `1`·`true`·`yes`·`on`이면 켬이다(소문자만). 그 밖의 값은 실행 전에 거부된다(exit 2). 기본으로 켜진 `CHASE`, `WK_LOC_DRIFTV`, `WK_LOC_EARLY`는 미지정·빈값이면 켬이고, `0`·`false`·`no`·`off`를 줘야 꺼진다.

## 옛 진입점 (보관, `LEGACY_OK=1` 필요)
| 진입점 | 문제 | 관련 의견 |
|---|---|---|
| `fly_gui.sh` (v3, v2_wallkeep, repo_export 세 사본) | 컨테이너 전체의 px4/gz/mavsdk_server를 종료한다(모든 인스턴스, 작업대) | legacy-demo-isolation |
| `wk_batch.sh` | v3 안에 있지만 v2_wallkeep 미션을 실행하고 v2 로그를 남긴다 | legacy-workspace-entrypoints |
| `wk_page.py` | 템플릿 `wk_page_template.html`이 v2_wallkeep/scripts에만 있다 | legacy-workspace-entrypoints |
| `bench.sh` (+ `bench_ctl.py`) | 리셋 대기 중 stop/reset이 대기 중인 미션 시작을 취소하지 못한다 | bench-cancel-during-reset |
| v2_wallkeep `install_tof.sh` | 잠금 없이 공유 starling2_tof를 지우고 다시 복사한다 | model-install-atomicity |
| `install_into_px4_gz.sh` (v2, v3 사본) | 원본 경로가 없는 사본이었다. 지금은 `repo_export/scripts/install_into_px4_gz.sh`로 넘긴다 | model-install-atomicity |

`scripts/`의 그 밖의 파일(grid_fly.sh, wk_report.py, wk_faces*.sh 등 v2·repo_export에서 복사한 도구)은 이번에 하나씩 점검하지 않았다. 현재 파이프라인에서는 호출하지 않는다.
