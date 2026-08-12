# Milestones

주간 목표와 진행 상황. 각 날짜별로 완료 항목에 체크하고 저널을 `docs/dayN_journal.md`에 남긴다.

## Week 1 (7/28 ~ 8/12) — Baseline & Sim-to-Real Pipeline

**일정 재조정 반복**: 초기 5일 계획에서 여러 사유로 다일수 소요.

### Day 1 (7/28): 시뮬레이션 베이스라인 완성 ✓

- [x] 서버 환경 조사 및 스택 확정
- [x] Container A (SITL) Dockerfile 작성 및 빌드
- [x] Container A: PX4 v1.14.0 + Gazebo Classic 11 정상 실행 확인
- [x] Gazebo GUI 원격 접근 (VNC + noVNC)
- [x] Container B (flight_code) Dockerfile 작성 및 빌드
- [x] Container B: Python 3.6.9 + MAVSDK 0.12.0 환경 확인
- [x] 두 컨테이너 UDP 연결 검증 (mavsdk_server 경유)
- [x] 실기 flight_code 실행 → 예상 실패 지점 확인
- [x] 이미지 스냅샷 백업 (`starling-sitl:day1-complete`, `starling-flight:day1-complete`)

**저널**: [day1_journal.md](day1_journal.md)

### Day 2 (8/3): Starling 2 airframe 이식 ✓

- [x] ModalAI 포크에서 Starling 2 실기 파라미터 추출
- [x] iris 모델을 iris_starling으로 복사 후 물리 파라미터 수정
- [x] airframe 스크립트 작성 (`4200_gazebo-classic_iris_starling`)
- [x] CMake 등록 및 빌드 성공
- [x] Gazebo GUI에서 이륙/착륙 시각 확인
- [x] 이미지 스냅샷 (`starling-sitl:day2-airframe`)
- [x] PX4-Autopilot v1.18-beta 오염 복구 및 v1.14.0 재고정

**저널**: [day2_journal.md](day2_journal.md)

### Day 3 (8/3): flight_code의 SITL 대응 ✓

- [x] `sim_pose_reader` 함수 작성 (MAVSDK telemetry 기반)
- [x] CLI 플래그 `--sim` 추가
- [x] `arm_and_start_offboard`에 리더 선택 분기
- [x] CSV 경로 하드코딩 우회
- [x] 실기 코드 경로 무손상
- [x] SITL에서 미션 20단계 완주 확인
- [x] 이미지 스냅샷 (`starling-sitl:day3-complete`, `starling-flight:day3-sim`)

**저널**: [day3_journal.md](day3_journal.md)

### Day 4 (8/5, 8/12): QVIX 3D 맵 제작 및 SITL 통합 ✓ (부분)

**8/5**: Fusion 360으로 QVIX R Module 3D 맵 제작
- [x] 도면 분석 (전체 22400 x 5800, 재배대 16개, 딸기방)
- [x] 재배대 컴포넌트 제작 (기둥 4개 + 선반 7개)
- [x] 재배대 16개 배치 (하부 8개 + 딸기방 + 상부 8개)
- [x] 외벽, 딸기방 벽, 문 자리 추가
- [x] STL export

**8/12**: Gazebo 통합
- [x] STL을 Gazebo 모델로 이식 (model.config, SDF, meshes)
- [x] world 파일 작성 (`qvix.world`)
- [x] CMake `set(worlds ...)`에 qvix 등록
- [x] 새 make 타겟 `gazebo-classic_iris_starling__qvix` 빌드 성공
- [x] STL 좌표계 실측 분석 (Y축 그룹 배치, X축 재배대 열 위치 확정)
- [x] QVIX pose 최적화 (드론이 딸기방 위 여유공간에서 이륙하도록)
- [x] Container B flight_code(--sim) 실행하여 QVIX 안에서 미션 시작 확인
- [x] 이미지 스냅샷 (`starling-sitl:qvix-integrated`)
- [ ] iris_starling mesh scale 조정 (드론 시각 크기가 실기보다 커서 재배대 통로에서 충돌 - Day 5로 이월)

**저널**: [day4_journal.md](day4_journal.md)

### Day 5 (다음 세션): sim-to-real gap 분석 및 개선

- [ ] iris_starling mesh scale 조정 (0.7 정도로 축소)
- [ ] QVIX 안에서 미션 20단계 완주 검증
- [ ] 실기 데이터와 SITL 데이터 pair 매칭
- [ ] 지표 비교: 위치 오차, 응답 지연, 실패 모드
- [ ] 결과 리포트 (표, 그래프, 짧은 요약)

**성공 기준**: "SITL이 실기의 어떤 부분을 얼마나 재현하는가"에 대한 답

## Week 2+ (향후)

### 별도 트랙 A: HITL 확장

Week 1 밖으로 분리. Day 5 결과 나온 후 정말 필요한지 판단.

- [ ] voxl-px4의 HITL 지원 여부 확인
- [ ] 서버 ↔ 실기 시리얼/이더넷 연결
- [ ] Gazebo 센서 데이터를 실기 PX4로 주입
- [ ] SITL vs HITL vs 실기 3자 비교

### 별도 트랙 B: QVIX 정밀 재현

Week 1 밖으로 분리. 실측 방문이 별도로 필요.

- [ ] NextOn 방문 계측 계획
- [ ] 재배 선반 CAD 정밀도 향상
- [ ] 딸기 잎/열매 텍스처 매핑
- [ ] 조명 조건 정밀 재현
- [ ] VIO feature density 검증
- [ ] AprilTag 위치 정확한 배치

### 별도 트랙 C: 확장 응용

- [ ] 수확 드론 개발
- [ ] 모바일 충전 스테이션
- [ ] Multi-drone 시나리오

## 진행 방식

- 각 항목 완료 시 체크박스 체크 + 커밋 메시지에 언급
- 하루가 끝날 때 `docs/dayN_journal.md` 작성
- 계획이 바뀌면 이 파일 자체를 수정 (버전 관리로 이력 남음)
