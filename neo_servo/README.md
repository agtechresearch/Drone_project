# DJI Neo Visual Servo

BEE35의 연속 스트리밍 제어 방식을 DJI Neo(폐쇄 스택)에 이식.
scrcpy 미러링 + AprilTag + pyautogui 스틱 스트리밍으로 자율 정렬.

## 파일 구조

```
neo_servo/
├── config.yaml           # 모든 세팅 (폰/UI 좌표/마커/PID/비상정지 핫키)
├── stream.py             # 프레임 소스 추상화 (scrcpy ↔ RTMP)
├── vision.py             # AprilTag 탐지 + PID → 스틱 오프셋
├── control.py            # pyautogui 연속 스틱 스트리밍 + 이산 버튼
├── emergency.py          # 전역 핫키 비상 정지 리스너 ★
├── main.py               # 진입점, 스레드 오케스트레이션, 임무 루프
├── calibrate.py          # UI 좌표 캘리브레이션 도구
├── requirements.txt      # Python 의존성
├── start_scrcpy.bat      # scrcpy 실행 배치 (Windows)
├── .vscode/launch.json   # VSCode F5 실행 설정
└── README.md
```

## 최초 셋업 (Windows + 갤럭시 노트9 기준)

### 1. 시스템 도구 설치

```powershell
# PowerShell (관리자)
Set-ExecutionPolicy Bypass -Scope Process -Force
iwr -useb https://chocolatey.org/install.ps1 | iex
choco install scrcpy adb
```

수동 설치는 https://github.com/Genymobile/scrcpy/releases 에서 zip 다운 후 PATH 등록.

**OBS Studio**: https://obsproject.com/download 에서 인스톨러 실행.

### 2. Python 의존성

```powershell
pip install -r requirements.txt
```

### 3. 노트9 준비

1. 설정 → 휴대전화 정보 → 소프트웨어 정보 → **빌드번호 7번 탭** (개발자 옵션 활성화)
2. 설정 → 개발자 옵션 → **USB 디버깅 ON**
3. USB-C 데이터 케이블로 PC 연결 → 폰에서 "이 컴퓨터에서 항상 허용" 체크
4. `adb devices`로 폰이 잡히는지 확인

### 4. DJI Neo 연결

1. 노트9에 **DJI Fly** 앱 설치 및 계정 로그인
2. Neo와 페어링 (앱 안내대로)
3. **화면 컨트롤 모드**(가상 조이스틱)로 진입

## 실행 흐름

```
1. start_scrcpy.bat 더블클릭  → NoteNine 창 등장
2. Neo 켜기 → DJI Fly 앱에서 조이스틱/이륙버튼 표시 확인
3. python calibrate.py         → 5초 카운트다운으로 UI 좌표 캡처
     결과 YAML 스니펫을 config.yaml에 붙여넣기
4. OBS → 창 캡처(NoteNine) → 가상 카메라 시작
5. python main.py --dry-run    → AprilTag 인식 및 dx/dy 부호 확인
     부호 안 맞으면 config.yaml의 control.sign을 -1로
6. python main.py              → 실비행
```

## VSCode에서 F5로 실행

프로젝트 폴더를 VSCode로 열면 좌측 실행/디버그 탭(Ctrl+Shift+D)에서 아래 선택 가능:

- **Calibrate: capture** — UI 좌표 순차 캡처
- **Calibrate: track** — 마우스 상대좌표 실시간 표시
- **Main: dry-run** — 비전만 확인 (스틱 조작 없음)
- **Main: 실비행** — 전체 임무 실행

## 캘리브레이션 스크립트 사용법

```powershell
python calibrate.py                    # capture 모드 (기본)
python calibrate.py --mode track       # 실시간 마우스 좌표 표시
python calibrate.py --title "OtherName"  # 다른 창 제목
```

**capture 모드 순서:**
1. 좌 스틱 원의 중심에 마우스 → 5초 대기 → 자동 캡처
2. 우 스틱 원의 중심 → 캡처
3. 우 스틱 원의 **가장자리** → 캡처 (중심과의 거리로 반경 자동 계산)
4. 이륙 버튼, 이륙 확인 슬라이더 끝, 착륙 버튼, 촬영 버튼 순차 캡처
5. 종료 시 `config.yaml`에 붙여넣을 YAML 스니펫 출력

## PID 튜닝 순서 (BEE35 원칙 = 한 번에 하나씩)

1. `--dry-run`으로 dx/dy/size_err 부호 검증 → 안 맞으면 `control.sign`을 -1로
2. `kp_x` 낮은 값(0.05)부터 → 발산 안 하는 상한까지 상향
3. `kp_dist` → `kp_y` 순으로 튜닝
4. `tolerance.pos_px/size_px`로 정렬 완료 민감도 조정

## 안전

- 첫 실행은 **프로펠러 뗀 상태**로 이륙 명령까지 시퀀스 검증
- **비상 정지: F12** (기본값, `config.yaml`의 `emergency.hotkey`로 변경 가능)
  - 어느 창이 포커스여도 즉시 감지 (pynput 전역 훅)
  - 스틱 즉시 해제 → Neo가 자체 호버로 그 자리에서 멈춤
  - 그 후 수동 조작(폰의 실제 조이스틱)으로 착륙시키면 됨
- `Ctrl+C`도 여전히 유효 (콘솔에 포커스가 있을 때)
- 노트북 키보드에서 F12가 Fn 조합인 경우 `emergency.hotkey`를 `<esc>`나 `<ctrl>+q` 같은 조합으로 변경 권장
