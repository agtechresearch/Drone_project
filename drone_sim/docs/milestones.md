# Milestones

주간 목표와 진행 상황. 각 날짜별로 완료 항목에 체크하고 저널을 `docs/dayN_journal.md`에 남긴다.

## Week 1 (7/28 ~ 8/1) — Baseline & Sim-to-Real Gap

**일정 재조정**: 7/28 Day 1은 예정대로 완료했으나, 8/3에 Day 2를 진행하게 되어 이후 일정이 뒤로 밀렸다. 새 일정은 아래 각 Day 참조.

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

### Day 3 (다음 세션): flight_code의 SITL 대응

- [ ] `voxl_pose_reader` 함수의 SITL 버전 작성 (MAVSDK telemetry 기반)
- [ ] CLI 플래그 `--sim` 또는 환경변수로 두 리더 중 선택
- [ ] 실기 코드 경로는 100% 그대로 유지
- [ ] SITL에서 미션 20단계 전체 실행 완료
- [ ] 트래킹 지표 로깅 (위치 오차, 자세, 속도)
- [ ] CSV 출력 경로를 SITL 환경에 맞게 조정 (`/home/root/` 하드코딩 우회)

**성공 기준**: 정량적 비교 가능한 SITL 로그 확보

### Day 4: 실기 대조 실험

- [ ] 실기 Starling 2에서 같은 미션 실행 (기존 데이터 활용 가능)
- [ ] 지표 수집 (SITL과 동일 포맷)
- [ ] 데이터 정리 및 pair 매칭

**성공 기준**: SITL 결과와 실기 결과 데이터 세트 준비 완료

### Day 5: sim-to-real gap 분석

- [ ] 양쪽 지표 비교 (위치 오차, 응답 지연, 실패 모드)
- [ ] 어느 축에서 gap이 큰지 정량화
- [ ] 결과 정리 (표, 그래프, 짧은 리포트)

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
