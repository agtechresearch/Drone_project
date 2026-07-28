# voa_pc_out 파이썬 파서 (단계1-B 첫 조각)

> 작성일: 2026-07-28
> 파일: `flight/mpa_point_cloud.py` (562행) / 테스트 `analysis/test_mpa_point_cloud.py`
> 상태: **로컬 검증 완료 (60/60). 기체 실측 미검증** — SSH 미접속 상태에서 작성했다.

## 1. 왜 필요한가

`docs/04`·`06`·`07`에서 **PX4 Collision Prevention 이 Offboard 모드에 개입하지 못한다**는
것을 3중 확증했다. 따라서 회피는 파이썬 레벨에서 직접 해야 하고, 그러려면 먼저
장애물 데이터를 읽어야 한다. 그 입력이 `voa_pc_out` 이다.

`voa_pc_out` 을 고른 이유는 `docs/04` 에 있다. vision-hub 가 이미 필터링·좌표변환·
시간누적을 끝낸 결과물이고, **level 프레임**(원점 기체본체, 롤·피치만 제거, X전방·
Y우측·Z하방)이라 우리가 좌표 변환을 할 필요가 없다.

## 2. 왜 직접 짜는가

| 후보 | 왜 못 쓰나 |
|---|---|
| `libmodal-pipe` 의 `python/pympa.py` | 66행짜리이고 IMU·카메라 전용. 포인트클라우드 지원 없음. 자동생성 `pympa_types` 모듈과 `libmodal_pipe.so` 를 요구 |
| `voxl-inspect-points` 출력 파싱 | 텍스트 요약만 나온다. 700점 × 20Hz 의 좌표 전체를 텍스트로 받아 파싱하는 것은 비행 루프에 넣을 물건이 아니다 |
| `.so` 직접 바인딩 (ctypes) | 콜백 기반 C API 라 파이썬에서 감싸기 번거롭고, 얻는 것도 없다 |

→ **바이너리 FIFO 를 직접 읽는다.** 표준 라이브러리만 쓰므로 numpy 도 필요 없다.

## 3. 구현

### 3.1 구조

I/O 와 파싱을 분리했다. 기체 없이 테스트하기 위해서다.

```
PointCloudStream(read_fn)     ← read 콜러블만 있으면 동작. 파싱 전담
    ↑
MpaPointCloudClient(pipe)     ← /run/mpa FIFO 구독. 리눅스 전용
    
PointCloudFrame               ← 한 프레임 + 장애물 요약 메서드
```

### 3.2 와이어 포맷 (docs/07 §1-A)

```python
HEADER_FMT  = "<IqIII32sI"          # magic / ts_ns / n_points / format / id / server[32] / reserved
HEADER_SIZE = 60                     # packed. struct.calcsize 로 검산
POINT_CLOUD_MAGIC = 0x564F584C       # "VOXL" — 카메라·ToF·IMU와 공용이라 이것만으론 구분 불가
```

포맷 6종(`FLOAT_XYZ` 12B ~ `FLOAT_XYZCRGB` 19B)을 모두 처리한다. `voa_pc_out` 은
`FLOAT_XYZ`(0) 이고 이 경로는 한 번의 `struct.unpack` 으로 처리해 가장 빠르다.

### 3.3 구독 프로토콜

```
/run/mpa/<파이프>/request 존재 확인          ← 서버 생존 확인
클라이언트 이름 = "pypc" + 8자리 난수         ← client.c 와 동일 규칙
request FIFO 에 이름 write (NULL 종료 포함)
서버가 /run/mpa/<파이프>/<그이름> mkfifo
그 FIFO 를 열어 read
```

죽은 서버에 매달리지 않도록 `O_WRONLY | O_NONBLOCK` 으로 열어 `ENXIO` 를 감지하고,
읽기는 `select` 로 타임아웃을 건다.

### 3.4 스트림 방어

FIFO 는 부분 read 가 흔하고 서버 재시작·오버런으로 경계가 깨질 수 있다.

- 헤더 magic 을 매 프레임 검증, 어긋나면 다음 magic 위치까지 버리고 재동기화
- magic 이 맞아도 `format` 이 정의된 값인지, `n_points` 가 상한(200만) 이내인지 확인
  → 깨진 헤더를 믿고 수 GB 를 할당하는 사고를 막는다
- 통계(`stats`)로 재동기화 횟수·폐기 바이트·가짜 헤더 수를 노출

### 3.5 장애물 요약 (진단용)

level 프레임이라 회전 변환 없이 바로 판정할 수 있다.

- `nearest_forward(half_width, half_height, min_x)` — 전방 직육면체 통로 안 최근접 X
- `sector_min_x(n_sectors, fov_deg)` — 전방 부채꼴을 좌→우 n등분한 각 구역 최근접 거리

두 번째가 중요하다. "앞이 막혔다"만으로는 회피 방향을 못 고르기 때문이다.
**다만 이건 진단·프로토타이핑용 요약이다.** 실제 회피 정책은 운용 환경을 정한 뒤
따로 설계한다.

## 4. 검증

### 4.1 단위 테스트 60/60 (기체 불필요)

```bash
python analysis/test_mpa_point_cloud.py
```

| # | 항목 | 결과 |
|---|---|---|
| 1 | 와이어 포맷 상수 (헤더 60B, magic, stride) | 4/4 |
| 2 | 정상 프레임 파싱 (좌표·타임스탬프·server_name NULL 종료·0점 프레임) | 7/7 |
| 3 | 연속 프레임 5개 + 통계 | 5/5 |
| 4 | **부분 read** — 1 / 7 / 59 바이트씩 공급해도 복원 | 4/4 |
| 5 | 재동기화 — 스트림 앞 쓰레기 40B | 3/3 |
| 6 | 재동기화 — 프레임 사이 오염, 페이로드 잘림 | 5/5 |
| 7 | **가짜 헤더 방어** — 잘못된 format, n_points 10억, max_points 초과 | 4/4 |
| 8 | 다른 포맷 (XYZC 16B / XYZRGB 15B / XY 8B) | 4/4 |
| 9 | 스트림 종료 (빈 스트림 / 헤더 중 EOF / 페이로드 중 EOF / 잔여 바이트) | 4/4 |
| 10 | 장애물 요약 — 통로 밖·후방·근접노이즈·높이 초과 제외 | 7/7 |
| 11 | 구역 분할 — 방향별 최근접, FOV 밖 제외 | 8/8 |
| 12 | **실전 규모** — 700점 × 20프레임(1초분), float32 왕복 무손상 | 5/5 |

4번과 7번이 실질적인 항목이다. 4번은 FIFO 의 실제 동작 조건이고, 7번은
깨진 헤더로 메모리를 터뜨리지 않는지를 본다.

### 4.2 성능 (개발 PC, x86)

| 항목 | 시간 |
|---|---|
| 700점 프레임 파싱 | **0.122 ms** |
| `nearest_forward()` | 0.040 ms |
| `sector_min_x(5)` | 0.172 ms |

20Hz 면 프레임당 예산이 50ms 다. VOXL2(ARM)가 6배 느리다고 봐도 파싱+요약이
2ms 미만이므로 **예산의 4% 이하**다. 순수 파이썬으로 충분하다.

## 5. 알려진 한계

**페이로드만 잘린 프레임은 탐지할 수 없다.** 프로토콜에 체크섬도 프레임 종료
구분자도 없다. 헤더가 온전하면 파서는 `n_points` 를 믿고 그만큼 읽으므로, 그
프레임 하나는 다음 프레임의 앞부분을 좌표로 오인한 채 나온다. 다음 호출에서
재동기화로 복구된다.

막으려면 프레임 끝 직후에 다음 magic 이 있는지 미리 확인하면 되지만, 그러면 다음
프레임이 도착할 때까지(20Hz 기준 최대 50ms) 현재 프레임을 못 내보낸다. **회피
판단에서 50ms 지연이 오염 프레임 하나보다 나쁘다**고 보고 하지 않았다.

→ **회피 로직은 단일 프레임을 신뢰하지 말고 연속 몇 프레임으로 판단할 것.**
   오염 발생 여부는 `stats["resyncs"]` 로 확인한다.

부수적으로: `FLOAT_XY` 계열(포맷 4·5)은 높이 정보가 없어 z=0 으로 채운다.
`voa_pc_out` 은 해당 없지만 다른 파이프에 쓸 때 주의.

## 6. 기체에서 해야 할 검증 (미완)

작성 시점에 SSH 3개 alias 가 전부 접속 불가라 **기체 실측을 못 했다.**
연결되면 아래 순서로 확인한다.

```bash
# 0. 배포
./tools/deploy.sh flight/mpa_point_cloud.py

# 1. 파이프가 살아있는지
ssh $DRONE_HOST "ls /run/mpa/ | head -30"
ssh $DRONE_HOST "cat /run/mpa/voa_pc_out/info"

# 2. ModalAI 도구로 기준값 확보 (tail 말고 head — tail 은 SIGTERM 에 잘린다)
ssh $DRONE_HOST "timeout 6 voxl-inspect-points voa_pc_out 2>&1 | head -10"

# 3. 우리 파서로 같은 파이프 구독
ssh $DRONE_HOST "python3 /home/root/mpa_point_cloud.py voa_pc_out --seconds 5"

# 4. 대조 항목
#    - 점 개수가 비슷한가 (docs/04 실측 ~700점)
#    - 프레임률이 20Hz 근처인가 (헤더 타임스탬프 기준 값도 함께 출력된다)
#    - 재동기화가 0인가 (0이 아니면 스트림 해석에 문제가 있는 것)
#    - format 이 FLOAT_XYZ 인가
#    - 좌표 부호가 level 프레임과 맞는가
#      → 기체 앞 1~2m 에 사람이 서서 x 가 그 값 근처로 잡히는지,
#        오른쪽으로 옮기면 y 가 양수로 가는지 확인 (docs/04 좌표계 검증)
```

**4번의 마지막 항목이 특히 중요하다.** 좌표계는 소스를 읽어 확정했지만 실물로
확인한 적이 없다. 부호가 반대면 회피가 장애물 쪽으로 가는 방향으로 동작한다.

## 7. 다음

- [ ] 위 6장 기체 실측 (SSH 복구 후 즉시)
- [ ] **회피 정책 설계** — 선행조건인 **운용 환경(온실/실내/실외) 확인이 여전히 미해결**.
      출발점은 ModalAI 공식값 `CP_DIST` 실내 1.0m / 실외 3.0m (`docs/07` §1-C).
      단 ModalAI 는 `CP_GO_NO_DATA=1` 로 사각지대를 "보호하지 않음" 처리하므로,
      측·후방이 완전 사각지대인 우리 기체는 더 보수적이어야 한다
- [ ] `voxl-mapper/obs_pc_filter.cc` 직독 — ModalAI 의 ToF 장애물 필터 참조 구현
- [ ] v14 통합 — 현재 v14 는 단일 파일이다. 모듈로 import 할지 인라인으로 넣을지는
      회피 로직 규모가 정해진 뒤 결정
