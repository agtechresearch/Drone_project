#!/usr/bin/env bash
#
# 로컬 저장소의 비행 코드를 기체(VOXL2)에 배포한다.
#
#   ./tools/deploy.sh                              # flight/path_flight_phase1_v14.py 배포
#   ./tools/deploy.sh flight/<파일>.py             # 다른 파일 배포
#   ./tools/deploy.sh --as path_flight_phase1.py   # 기체에 다른 이름으로 배포
#   ./tools/deploy.sh -y                           # 확인 프롬프트 생략
#
# 배포 절차: 로컬 문법검증 → 기체 백업 → scp → md5 대조 → 기체 문법검증 → 배포기록
# 기체 원본은 항상 타임스탬프 백업 후 덮어쓴다.

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"

die() { printf '\033[31m[에러]\033[0m %s\n' "$*" >&2; exit 1; }
info() { printf '\033[36m[%s]\033[0m %s\n' "$1" "${*:2}"; }
ok()   { printf '\033[32m  OK\033[0m %s\n' "$*"; }
warn() { printf '\033[33m[경고]\033[0m %s\n' "$*"; }

# ---- 설정 로드 ----
[[ -f .env.local ]] || die ".env.local 이 없습니다. 'cp .env.example .env.local' 후 값을 채우세요."
# shellcheck disable=SC1091
source .env.local
: "${DRONE_HOST:?.env.local 에 DRONE_HOST 가 필요합니다}"
DRONE_DIR="${DRONE_DIR:-/home/root}"

# ---- 인자 파싱 ----
LOCAL_FILE=""
REMOTE_NAME=""
ASSUME_YES=0
while [[ $# -gt 0 ]]; do
  case "$1" in
    --as)  REMOTE_NAME="$2"; shift 2 ;;
    -y|--yes) ASSUME_YES=1; shift ;;
    -h|--help) sed -n '2,15p' "$0" | sed 's/^# \?//'; exit 0 ;;
    -*) die "알 수 없는 옵션: $1" ;;
    *)  LOCAL_FILE="$1"; shift ;;
  esac
done
LOCAL_FILE="${LOCAL_FILE:-flight/path_flight_phase1_v14.py}"
REMOTE_NAME="${REMOTE_NAME:-$(basename "$LOCAL_FILE")}"
REMOTE_PATH="$DRONE_DIR/$REMOTE_NAME"

[[ -f "$LOCAL_FILE" ]] || die "로컬 파일이 없습니다: $LOCAL_FILE"

# ---- 1. 로컬 문법 검증 ----
info 1/6 "로컬 문법 검증"
if command -v python >/dev/null 2>&1; then
  python -m py_compile "$LOCAL_FILE" || die "로컬 문법 오류. 배포 중단."
  ok "python -m py_compile 통과"
else
  warn "로컬 python 없음 — 기체 검증에만 의존합니다"
fi

# ---- 2. git 상태 ----
info 2/6 "git 상태"
GIT_DESC="(git 아님)"
if git rev-parse --git-dir >/dev/null 2>&1; then
  GIT_SHA="$(git rev-parse --short HEAD 2>/dev/null || echo 'no-commit')"
  if [[ -n "$(git status --porcelain -- "$LOCAL_FILE")" ]]; then
    GIT_DESC="$GIT_SHA-dirty"
    warn "$LOCAL_FILE 에 커밋되지 않은 변경이 있습니다 → 배포본을 나중에 재현할 수 없습니다"
    warn "기록을 남기려면 먼저 커밋하세요: git add -A && git commit"
  else
    GIT_DESC="$GIT_SHA"
    ok "커밋됨: $GIT_SHA"
  fi
fi

LOCAL_MD5="$(md5sum "$LOCAL_FILE" | awk '{print $1}')"

# ---- 3. 확인 ----
printf '\n  로컬 : %s (md5 %s, %s)\n  기체 : %s:%s\n\n' \
  "$LOCAL_FILE" "${LOCAL_MD5:0:8}" "$GIT_DESC" "$DRONE_HOST" "$REMOTE_PATH"
if [[ $ASSUME_YES -eq 0 ]]; then
  read -rp "배포할까요? [y/N] " ans
  [[ "$ans" =~ ^[Yy]$ ]] || die "취소됨"
fi

# ---- 4. 기체 백업 ----
STAMP="$(date +%Y%m%d-%H%M%S)"
info 3/6 "기체 접속 및 기존 파일 백업"
ssh -o ConnectTimeout=10 "$DRONE_HOST" "
  set -e
  if [ -f '$REMOTE_PATH' ]; then
    cp -p '$REMOTE_PATH' '$REMOTE_PATH.bak.$STAMP'
    echo \"  백업: $REMOTE_NAME.bak.$STAMP (\$(md5sum '$REMOTE_PATH' | cut -c1-8))\"
  else
    echo '  기존 파일 없음 — 신규 배포'
  fi
" || die "기체 접속 실패. 네트워크와 DRONE_HOST($DRONE_HOST) 를 확인하세요."

# ---- 5. 업로드 + 검증 ----
info 4/6 "업로드"
scp -q "$LOCAL_FILE" "$DRONE_HOST:$REMOTE_PATH" || die "scp 실패"
ok "전송 완료"

info 5/6 "md5 대조 및 기체 문법 검증"
REMOTE_MD5="$(ssh "$DRONE_HOST" "md5sum '$REMOTE_PATH' | cut -d' ' -f1")"
[[ "$LOCAL_MD5" == "$REMOTE_MD5" ]] \
  || die "md5 불일치! 로컬 $LOCAL_MD5 / 기체 $REMOTE_MD5 — 전송이 손상되었습니다."
ok "md5 일치: $LOCAL_MD5"

ssh "$DRONE_HOST" "cd '$DRONE_DIR' && python3 -m py_compile '$REMOTE_NAME'" \
  || die "기체 python3 문법 검증 실패. 백업본($REMOTE_NAME.bak.$STAMP)으로 되돌리세요."
ok "기체 python3 문법 통과"

# ---- 6. 배포 기록 ----
info 6/6 "배포 기록"
ssh "$DRONE_HOST" "echo '$STAMP  $REMOTE_NAME  md5=$LOCAL_MD5  git=$GIT_DESC' >> '$DRONE_DIR/.deploy_log'"
ok "기체 $DRONE_DIR/.deploy_log 에 기록"

printf '\n\033[32m배포 완료\033[0m  %s → %s:%s\n' "$LOCAL_FILE" "$DRONE_HOST" "$REMOTE_PATH"
printf '롤백: ssh %s "cp %s.bak.%s %s"\n' "$DRONE_HOST" "$REMOTE_PATH" "$STAMP" "$REMOTE_PATH"
