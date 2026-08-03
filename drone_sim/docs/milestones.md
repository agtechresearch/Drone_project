# Milestones

주간 목표와 진행 상황. 각 날짜별로 완료 항목에 체크하고 저널을 `docs/dayN_journal.md`에 남긴다.

## Week 1 (7/28 ~ 8/1) — Baseline & Sim-to-Real Gap

**일정 재조정**: 7/28 Day 1 이후, QVIX 정밀 재현 트랙에서 시간 소진(실패)으로 인해 후반 일정이 뒤로 밀렸다. 8/3에 Day 2와 Day 3를 연속 진행. Day 4, 5는 별도 세션.

### Day 1 (7/28): 시뮬레이션 베이스라인 완성 ✓

- [x] 서버 환경 조사 및 스택 확정
- [x] Container A (SITL) Dockerfile 작성 및 빌드
- [x] Container A: PX4 v1.14.0 + Gazebo Classic 11 정상 실행 확인
- [x] Gazebo GUI 원격 접근 (VNC + noVNC)
- [x] Container B (flight_code) Dockerfile 작성 및 빌드
- [x] Container B: Python 3.6.9 + MAVSDK 0.12.0 환경 확인
- [x] 두 컨테이너 UDP 연결 검증 (mavsdk_server 경유)
- [x] 실기 flight_code 실행 → 예상 실패 지점 확인 (voxl-inspect-pose 없음)
- [x] 이미지 스냅샷 백업 (`starling-sitl:day1-complete`, `starling-flight:day1-complete`)

**성공 기준**: 실기 flight_code 코드를 수정 없이 SITL로 실행 → 달성  
**저널**: [day1_journal.md](day1_journal.md)

### Day 2 (8/3): Starling 2 airframe 이식 ✓

- [x] ModalAI 포크에서 Starling 2 실기 파라미터 추출 (Mac 경유)
- [x] HITL config에서 로터 배치, IMU 오프셋 등 실측치 확보
- [x] iris 모델을 iris_starling으로 복사 후 물리 파라미터 수정
- [x] airframe 스크립트 작성 (`4200_gazebo-classic_iris_starling`)
- [x] CMake 등록 (models 목록 + CMakeLists.txt)
- [x] 빌드 성공 및 SITL 실행 검증
- [x] Gazebo GUI에서 이륙/착륙 시각 확인
- [x] 파라미터 로드 확인 (`CA_ROTOR0_PY=0.25`, `EKF2_IMU_POS_X=0.027`, `IMU_GYRO_RATEMAX=800`)
- [x] 이미지 스냅샷 (`starling-sitl:day2-airframe`)
- [x] PX4-Autopilot v1.18-beta 오염 복구 및 v1.14.0 재고정
- [x] `git remote remove origin`으로 자동 업데이트 재발 방지

**성공 기준**: iris보다 실기에 가까운 물리로 SITL이 정상 동작 → 달성  
**저널**: [day2_journal.md](day2_journal.md)

### Day 3 (8/3): flight_code의 SITL 대응 ✓

- [x] `sim_pose_reader` 함수 작성 (MAVSDK telemetry 기반)
- [x] CLI 플래그 `--sim` 추가
- [x] `arm_and_start_offboard`에 리더 선택 분기 추가
- [x] CSV 경로 하드코딩 우회 (SITL/실기 자동 폴백)
- [x] 실기 코드 경로 무손상 (기존 로직 한 줄도 안 바꿈)
- [x] SITL에서 미션 20단계 완주 확인
- [x] CSV 로깅 정상 작동 (실이동 10m 확인)
- [x] 이미지 스냅샷 (`starling-sitl:day3-complete`, `starling-flight:day3-sim`)

**성공 기준**: 실기 flight_code가 수정 없는 로직 흐름 그대로 SITL에서 미션 완주 → 달성  
**저널**: [day3_journal.md](day3_journal.md)

### Day 4 (다음 세션): 실기 대조 실험

- [ ] 실기 Starling 2에서 SITL과 동일한 미션 실행
- [ ] 지표 수집 (SITL과 동일 포맷)
- [ ] 데이터 정리 및 pair 매칭
- [ ] 실기 CSV와 SITL CSV의 컬럼 정합성 확인

**성공 기준**: SITL 결과와 실기 결과 데이터 세트 준비 완료

### Day 5: sim-to-real gap 분석

- [ ] 양쪽 지표 비교 (위치 오차, 응답 지연, 실패 모드)
- [ ] 어느 축에서 gap이 큰지 정량화
- [ ] 결과 정리 (표, 그래프, 짧은 리포트)
- [ ] Day 3의 `mav_abs_n` stale 문제 정리 (SITL에서 `mavsdk_position_monitor` 스킵 옵션)

**성공 기준**: "SITL이 실기의 어떤 부분을 얼마나 재현하는가"에 대한 답

## Week 2+ (향후)

### 별도 트랙 A: HITL 확장

이번 Week 1 밖으로 분리. Day 5 결과 나온 후 정말 필요한지 판단.

- [ ] voxl-px4의 HITL 지원 여부 확인
- [ ] 서버 ↔ 실기 시리얼/이더넷 연결
- [ ] Gazebo 센서 데이터를 실기 PX4로 주입
- [ ] SITL vs HITL vs 실기 3자 비교

### 별도 트랙 B: QVIX 정밀 재현

Week 1 밖으로 분리. 실측 방문이 별도로 필요.

- [ ] NextOn 방문 계측 계획 (도면, 선반 치수, 조명, 텍스처)
- [ ] 재배 선반 CAD 또는 실측 데이터 기반 SDF
- [ ] 딸기 잎/열매 텍스처 매핑
- [ ] 조명 조건 정밀 재현
- [ ] VIO feature density 검증

### 별도 트랙 C: 확장 응용

- [ ] 수확 드론 개발
- [ ] 모바일 충전 스테이션
- [ ] Multi-drone 시나리오

## 진행 방식

- 각 항목 완료 시 체크박스 체크 + 커밋 메시지에 언급
- 하루가 끝날 때 `docs/dayN_journal.md` 작성
- 계획이 바뀌면 이 파일 자체를 수정 (버전 관리로 이력 남음)
