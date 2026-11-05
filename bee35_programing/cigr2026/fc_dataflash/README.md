# CIGR 2026 포스터 실험 — FC dataflash 로그 (.BIN)

ArduPilot(ArduCopter 4.6.2, MicoAir H743 V2) 비행 제어기에 남아 있던 dataflash 로그 중
포스터 본실험 4회·대조군 5회가 들어 있는 7개 파일. 2026-11-05에 기체 FC에서 내려받음.

- 받은 경로: 라즈베리파이 UART(`/dev/ttyAMA0`, 115200) → MAVLink `LOG_REQUEST_DATA` → PC. 파이와 PC의 sha256 일치 확인.
- 파일 이름은 FC의 원래 로그 번호 그대로(`00000022.BIN` = FC 로그 22번). 파일 안의 내용은 수정하지 않음.
- 미션플래너 로그 목록에는 날짜가 `1980-01-01`로 보이지만(실내라 GPS 미고정, 지상국 시각 전송 없음),
  로그 안 `GPS` 메시지의 GPS 주/밀리초(`GWk`, `GMS`)는 유효해서 부팅 시각을 복원했다(아래 표 "FC 부팅").
  미션플래너/MAVExplorer에서 열면 시간축이 1980년으로 보일 수 있으나 상대 시간(`TimeUS`)은 정상이다.
- 로그 하나 = FC 전원 한 번(첫 arm 시도부터 전원 끌 때까지, `LOG_DISARMED=0`, `LOG_FILE_DSRMROT=0`).
  그래서 한 파일에 비행이 여러 번 들어 있고, 첫 비행의 arm 이벤트는 기록 시작 직전이라 없다.
  비행 구간은 모터 출력(`RCOU` > 1150 µs)으로 잡았다.
- 스크립트 CSV(`../test16/poster/...`)의 파일명 시각은 스크립트 시작 시각이고, 모터 ON과 1~3 s 차이.
  CSV의 `elapsed_s = 0`은 모터 ON 약 8~9 s 뒤(이륙 후 정렬 시작)에 해당한다. .BIN과 겹쳐 볼 때 `TimeUS = 모터ON + 8.5 s + elapsed_s` 정도로 맞추면 된다.
- 검증(2026-11-05): 본실험 4회는 CSV의 RC 오버라이드 명령 쌍(target_roll_us, target_pitch_us)이 .BIN `RCIN` C1/C2에 ±0.5 s 안에서 100% 그대로 나타남(다른 비행과 비교 시 최대 40%). 대조군 5회는 PID 구간이 짧아 명령 대조 대신 GPS 복원 시각(1~3 s 일치)·비행 길이·기수 방위(yaw 0.2° 이내 일치)로 확인.

## 로그 ↔ 비행 매칭 (`flight_table.csv`와 동일)

| 로그 | 크기 | FC 부팅(KST) | 비행 | 모터 ON(KST) | 길이 | 구분 | 스크립트 CSV |
|---|---|---|---|---|---|---|---|
| 00000022.BIN | 11.9 MB | 06-17 20:13:34 | 1 | 06-17 20:22:04 | 66 s | 기타(CSV 없음) | - |
| 00000022.BIN | 11.9 MB | 06-17 20:13:34 | 2 | 06-17 20:26:25 | 51 s | 기타(CSV 없음) | - |
| 00000022.BIN | 11.9 MB | 06-17 20:13:34 | 3 | 06-17 20:31:17 | 77 s | 본실험(PID 정렬) | `test16/poster/poster/loiter_poster_realtime_20260617_203116.csv` |
| 00000022.BIN | 11.9 MB | 06-17 20:13:34 | 4 | 06-17 20:34:17 | 32 s | 기타(CSV 없음) | - |
| 00000024.BIN | 7.4 MB | 06-17 20:45:09 | 1 | 06-17 20:46:36 | 76 s | 대조군(LOITER) | `test16/poster/baseline_poster/loiter_baseline_poster_realtime_20260617_204634.csv` |
| 00000024.BIN | 7.4 MB | 06-17 20:45:09 | 2 | 06-17 20:51:23 | 77 s | 본실험(PID 정렬) | `test16/poster/poster/loiter_poster_realtime_20260617_205121.csv` |
| 00000025.BIN | 3.7 MB | 06-17 20:53:24 | 1 | 06-17 20:55:40 | 77 s | 대조군(LOITER) | `test16/poster/baseline_poster/loiter_baseline_poster_realtime_20260617_205537.csv` |
| 00000026.BIN | 11.7 MB | 06-17 20:58:14 | 1 | 06-17 21:13:10 | 73 s | 기타(CSV 없음) | - |
| 00000026.BIN | 11.7 MB | 06-17 20:58:14 | 2 | 06-17 21:16:01 | 83 s | 대조군(LOITER) | `test16/poster/baseline_poster/loiter_baseline_poster_realtime_20260617_211600.csv` |
| 00000026.BIN | 11.7 MB | 06-17 20:58:14 | 3 | 06-17 21:19:40 | 88 s | 본실험(PID 정렬) | `test16/poster/poster/loiter_poster_realtime_20260617_211937.csv` |
| 00000027.BIN | 12.2 MB | 06-17 21:23:40 | 1 | 06-17 21:30:20 | 36 s | 기타(CSV 없음) | - |
| 00000027.BIN | 12.2 MB | 06-17 21:23:40 | 2 | 06-17 21:31:53 | 89 s | 본실험(PID 정렬) | `test16/poster/poster/loiter_poster_realtime_20260617_213151.csv` |
| 00000027.BIN | 12.2 MB | 06-17 21:23:40 | 3 | 06-17 21:35:46 | 85 s | 기타(CSV 없음) | - |
| 00000027.BIN | 12.2 MB | 06-17 21:23:40 | 4 | 06-17 21:40:52 | 16 s | 기타(CSV 없음) | - |
| 00000031.BIN | 4.3 MB | 06-18 19:58:27 | 1 | 06-18 19:59:42 | 93 s | 대조군(LOITER) | `test16/poster/baseline_poster/loiter_baseline_poster_realtime_20260618_195940.csv` |
| 00000032.BIN | 4.2 MB | 06-22 11:16:53 | 1 | 06-22 11:19:24 | 90 s | 대조군(LOITER) | `test16/poster/baseline_poster/loiter_baseline_poster_realtime_20260622_111923.csv` |

"기타(CSV 없음)"는 같은 전원 세션 안의 튜닝·확인 비행으로, 포스터 실험 데이터가 아니다.

## 같이 보면 좋은 자료 (리포 밖)

- 미션플래너 텔레메트리 로그(tlog, 6/13~6/22, 333 MB): 용량 때문에 리포에 넣지 않음. 실험 PC `C:\Users\ericj\drone_data\cigr2026_logs\tlog\`.
- 6월 전체 비행 70회(arm~disarm) 목록과 CSV 매칭표: 같은 폴더 `arm_csv_matching.csv`.
- FC에는 6/13~7/02 로그 37개(282 MB)가 더 남아 있음(2026-11-05 기준). 필요하면 같은 방법으로 받을 수 있다.

## 읽는 법

```python
from pymavlink import DFReader
r = DFReader.DFReader_binary("00000022.BIN")
while (m := r.recv_msg()) is not None:
    if m.get_type() == "ATT": print(m.TimeUS, m.Roll, m.Pitch, m.Yaw)
```
