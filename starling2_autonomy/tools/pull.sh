#!/usr/bin/env bash
#
# 기체의 비행 코드를 로컬과 대조한다 (drift 감지).
#
#   ./tools/pull.sh                      # flight/path_flight_phase1_v14.py 대조
#   ./tools/pull.sh flight/<파일>.py     # 다른 파일 대조
#   ./tools/pull.sh --apply              # 차이가 있으면 기체 내용을 로컬에 덮어쓴다
#
# 이 저장소는 "로컬 git 이 원본"이다. 기체에서 급히 고친 내용이 있을 때만
# --apply 로 회수하고, 반드시 그 직후 커밋해 이력을 남긴다.

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"

die() { printf '\033[31m[에러]\033[0m %s\n' "$*" >&2; exit 1; }
ok()   { printf '\033[32m  OK\033[0m %s\n' "$*"; }

[[ -f .env.local ]] || die ".env.local 이 없습니다. 'cp .env.example .env.local' 후 값을 채우세요."
# shellcheck disable=SC1091
source .env.local
: "${DRONE_HOST:?.env.local 에 DRONE_HOST 가 필요합니다}"
DRONE_DIR="${DRONE_DIR:-/home/root}"

LOCAL_FILE=""
APPLY=0
while [[ $# -gt 0 ]]; do
  case "$1" in
    --apply) APPLY=1; shift ;;
    -h|--help) sed -n '2,12p' "$0" | sed 's/^# \?//'; exit 0 ;;
    -*) die "알 수 없는 옵션: $1" ;;
    *)  LOCAL_FILE="$1"; shift ;;
  esac
done
LOCAL_FILE="${LOCAL_FILE:-flight/path_flight_phase1_v14.py}"
REMOTE_NAME="$(basename "$LOCAL_FILE")"

mkdir -p .pull_tmp
TMP=".pull_tmp/$REMOTE_NAME"

printf '기체에서 내려받는 중: %s:%s/%s\n' "$DRONE_HOST" "$DRONE_DIR" "$REMOTE_NAME"
scp -q -o ConnectTimeout=10 "$DRONE_HOST:$DRONE_DIR/$REMOTE_NAME" "$TMP" \
  || die "내려받기 실패. 네트워크와 파일 경로를 확인하세요."

if [[ ! -f "$LOCAL_FILE" ]]; then
  printf '로컬에 %s 가 없습니다 — 기체 파일을 새로 만듭니다.\n' "$LOCAL_FILE"
  cp "$TMP" "$LOCAL_FILE"; rm -rf .pull_tmp
  ok "생성: $LOCAL_FILE  → 커밋하세요"
  exit 0
fi

if diff -q "$LOCAL_FILE" "$TMP" >/dev/null; then
  ok "일치 — 기체와 저장소가 같습니다 (md5 $(md5sum "$LOCAL_FILE" | cut -c1-8))"
  rm -rf .pull_tmp
  exit 0
fi

printf '\n\033[33m[차이 발견]\033[0m 기체 코드가 저장소와 다릅니다.\n'
printf '  로컬 %s  md5 %s\n' "$LOCAL_FILE" "$(md5sum "$LOCAL_FILE" | cut -c1-8)"
printf '  기체 %s  md5 %s\n\n' "$REMOTE_NAME" "$(md5sum "$TMP" | cut -c1-8)"
diff -u "$LOCAL_FILE" "$TMP" | head -60 || true
printf '\n(위는 최대 60줄. 전체: diff -u %s %s)\n' "$LOCAL_FILE" "$TMP"

if [[ $APPLY -eq 1 ]]; then
  cp "$TMP" "$LOCAL_FILE"
  rm -rf .pull_tmp
  printf '\n\033[32m로컬에 반영했습니다.\033[0m 반드시 커밋해 이력을 남기세요:\n'
  printf '  git add %s && git commit -m "기체에서 회수: <수정 이유>"\n' "$LOCAL_FILE"
else
  printf '\n반영하려면: ./tools/pull.sh %s --apply\n' "$LOCAL_FILE"
  printf '비교본은 %s 에 남겨둡니다.\n' "$TMP"
fi
