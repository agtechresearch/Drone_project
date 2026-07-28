# 원격 접속·제어 가이드

Claude가 이 PC(Windows)에서 SSH로 기체(VOXL2)를 원격 제어하는 방식과, 사람이 직접 쓰는 명령을 정리.

## 1. 동작 원리
- 기체(VOXL2)에는 Claude가 설치돼 있지 않음.
- 개발 PC의 SSH 클라이언트 → 기체 접속 → 단발성 명령 실행/파일 수정.
- Claude는 `ssh $DRONE_HOST "명령"` 형태로 원격 명령을 대신 실행.
- 대화형 세션 유지는 불가 → 매 명령을 독립적으로 실행하는 구조.

## 2. 접속

접속 대상은 `~/.ssh/config` 의 alias 로 관리하고, 실제 IP 는 저장소에 커밋하지 않는다
(`.env.local`, `.gitignore` 처리).

```bash
# ~/.ssh/config
Host <DRONE_HOST>            # 예: Starling2_AgtechLab
    HostName <기체 IP>       # 랩 네트워크 / 기체 자체 AP / 핫스팟 중 현재 연결된 것
    User root
    IdentityFile ~/.ssh/id_ed25519
```

```bash
cp .env.example .env.local   # DRONE_HOST 를 본인 alias 로 채운다
ssh $DRONE_HOST
```

- 기체는 접속 경로가 셋이다: **랩 네트워크 / 기체 자체 AP / 모바일 핫스팟.**
  네트워크마다 IP가 달라지므로 alias 를 셋 만들어두고 상황에 맞는 것을 쓴다.
- 키 인증을 설정해두면 비밀번호 없이 자동 실행 가능.
- SSH 접속 시 post-quantum 경고 메시지는 무해(필터링해서 봄).

## 3. 자주 쓰는 조회 명령 (기체에서)
```bash
# 실행 중인 voxl/px4 프로세스
ps aux | grep -iE "px4|voxl|python" | grep -v grep

# 사용 가능한 MPA 파이프 목록
voxl-list-pipes            # 또는  ls /run/mpa/

# 센서 실시간 확인
voxl-inspect-tof                 # 전방 ToF 깊이 (center point m)
voxl-inspect-rangefinders        # 하방 거리센서
voxl-inspect-pose -n px4_vehicle_local_position   # PX4 로컬 위치
voxl-inspect-vio                 # VIO 상태

# vision-hub / VOA 설정
cat /etc/modalai/voxl-vision-hub.conf
```

## 4. 파일 가져오기/올리기

**이 저장소가 비행 코드의 원본이다.** 기체에서 직접 편집하지 말고 로컬에서 고쳐 커밋한 뒤 배포한다.

```bash
# 로컬 → 기체 (배포)
./tools/deploy.sh                       # flight/path_flight_phase1_v14.py
./tools/deploy.sh flight/<파일>.py      # 다른 파일

# 기체 ↔ 저장소 대조 (drift 감지)
./tools/pull.sh                         # 차이만 보여줌
./tools/pull.sh --apply                 # 기체 내용을 로컬에 회수 (직후 반드시 커밋)
```

`deploy.sh` 는 **로컬 문법검증 → 기체 타임스탬프 백업 → scp → md5 대조 → 기체 python3 문법검증
→ 배포기록(`/home/root/.deploy_log`)** 을 순서대로 수행하고, 한 단계라도 실패하면 중단한다.
롤백 명령은 배포 종료 시 화면에 출력된다.

원시 명령이 필요할 때:
```bash
scp $DRONE_HOST:/home/root/<파일> ./flight/          # 내려받기
ssh $DRONE_HOST "cp /home/root/<파일> /home/root/<파일>.bak"   # 백업 후
scp ./flight/<수정본> $DRONE_HOST:/home/root/<대상>  # 올리기
```

## 5. 비행 코드 실행 (주의: 실제 비행)
```bash
# 기체에서 (실제 모터 구동 — 반드시 사전 확인)
python3 /home/root/path_flight_phase1_v14.py --csv auto --csv-sample-sec 1.0
```
- ⚠️ 실제 비행/파라미터 변경은 Claude가 **먼저 사용자 확인** 후에만 실행.
- RC가 OFFBOARD 모드여야 offboard 시작됨.

## 6. 안전 수칙
- 조회·로그·빌드성 명령: 자유 실행.
- 되돌리기 어려운 동작(arm, offboard start, land, kill, px4 param set): 사전 확인 필수.
- 원격 파일 수정 전 항상 백업 사본 생성.
