#!/usr/bin/env bash
#
# s2flight 패키지 + 실험 진입 스크립트를 기체(VOXL2)에 배포한다. (deploy.sh 는 단일 파일용)
#
#   ./tools/deploy_pkg.sh                                   # flight/s2flight + flight/marker_drift_flight.py
#   ./tools/deploy_pkg.sh flight/other_experiment.py        # 패키지 + 다른 진입 스크립트
#   ./tools/deploy_pkg.sh -y                                # 확인 생략
#
# 절차: 로컬 문법검증(compileall) → 기체 기존 패키지 백업(tar) → tar 전송 → 파일별 md5 대조
#       → 기체 python3 compileall → 배포기록.  기체 경로: $DRONE_DIR/s2flight/, $DRONE_DIR/<진입스크립트>
# 설정은 deploy.sh 와 같은 .env.local (DRONE_HOST, DRONE_DIR).

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"

die()  { printf '\033[31m[에러]\033[0m %s\n' "$*" >&2; exit 1; }
info() { printf '\033[36m[%s]\033[0m %s\n' "$1" "${*:2}"; }
ok()   { printf '\033[32m  OK\033[0m %s\n' "$*"; }
warn() { printf '\033[33m[경고]\033[0m %s\n' "$*"; }

[[ -f .env.local ]] || die ".env.local 이 없습니다. 'cp .env.example .env.local' 후 값을 채우세요."
# shellcheck disable=SC1091
source .env.local
: "${DRONE_HOST:?.env.local 에 DRONE_HOST 가 필요합니다}"
DRONE_DIR="${DRONE_DIR:-/home/root}"

PKG_DIR="flight/s2flight"
ENTRY=""
ASSUME_YES=0
while [[ $# -gt 0 ]]; do
  case "$1" in
    -y|--yes) ASSUME_YES=1; shift ;;
    -h|--help) sed -n '2,12p' "$0" | sed 's/^# \?//'; exit 0 ;;
    -*) die "알 수 없는 옵션: $1" ;;
    *)  ENTRY="$1"; shift ;;
  esac
done
ENTRY="${ENTRY:-flight/marker_drift_flight.py}"
[[ -d "$PKG_DIR" ]] || die "패키지 폴더가 없습니다: $PKG_DIR"
[[ -f "$ENTRY" ]]   || die "진입 스크립트가 없습니다: $ENTRY"
ENTRY_NAME="$(basename "$ENTRY")"

# 전송 목록 (pyc·캐시 제외)
FILES=()
while IFS= read -r f; do FILES+=("$f"); done < <(find "$PKG_DIR" -name '*.py' -not -path '*/__pycache__/*' | sort)
FILES+=("$ENTRY")

info 1/6 "로컬 문법 검증"
if command -v python >/dev/null 2>&1; then
  python -m compileall -q "$PKG_DIR" >/dev/null || die "패키지 문법 오류. 배포 중단."
  python -m py_compile "$ENTRY" || die "진입 스크립트 문법 오류. 배포 중단."
  ok "compileall 통과 (${#FILES[@]} 파일)"
else
  warn "로컬 python 없음 — 기체 검증에만 의존합니다"
fi

info 2/6 "git 상태"
GIT_DESC="(git 아님)"
if git rev-parse --git-dir >/dev/null 2>&1; then
  GIT_SHA="$(git rev-parse --short HEAD 2>/dev/null || echo 'no-commit')"
  if [[ -n "$(git status --porcelain -- "$PKG_DIR" "$ENTRY")" ]]; then
    GIT_DESC="$GIT_SHA-dirty"; warn "커밋되지 않은 변경이 있습니다 → 배포본을 나중에 재현할 수 없습니다"
  else
    GIT_DESC="$GIT_SHA"; ok "커밋됨: $GIT_SHA"
  fi
fi

printf '\n  로컬 : %s (%d 파일) + %s   [%s]\n  기체 : %s:%s/{s2flight/, %s}\n\n' \
  "$PKG_DIR" "$(( ${#FILES[@]} - 1 ))" "$ENTRY" "$GIT_DESC" "$DRONE_HOST" "$DRONE_DIR" "$ENTRY_NAME"
if [[ $ASSUME_YES -eq 0 ]]; then
  read -rp "배포할까요? [y/N] " ans
  [[ "$ans" =~ ^[Yy]$ ]] || die "취소됨"
fi

STAMP="$(date +%Y%m%d-%H%M%S)"
info 3/6 "기체 접속 및 기존 파일 백업"
ssh -o ConnectTimeout=10 "$DRONE_HOST" "
  set -e
  mkdir -p '$DRONE_DIR'
  cd '$DRONE_DIR'
  if [ -d s2flight ] || [ -f '$ENTRY_NAME' ]; then
    tar czf 's2flight.bak.$STAMP.tgz' s2flight '$ENTRY_NAME' 2>/dev/null || tar czf 's2flight.bak.$STAMP.tgz' s2flight
    echo '  백업: s2flight.bak.$STAMP.tgz'
  else
    echo '  기존 파일 없음 — 신규 배포'
  fi
" || die "기체 접속 실패. 네트워크와 DRONE_HOST($DRONE_HOST) 를 확인하세요."

info 4/6 "업로드 (tar over ssh)"
# flight/ 기준 상대경로로 묶어 $DRONE_DIR 에 그대로 푼다: s2flight/*.py, <entry>.py
tar czf - -C flight "s2flight" --exclude='__pycache__' --exclude='*.pyc' -C "$REPO_ROOT/$(dirname "$ENTRY")" "$ENTRY_NAME" \
  | ssh "$DRONE_HOST" "cd '$DRONE_DIR' && rm -rf s2flight && tar xzf -" || die "전송 실패"
ok "전송 완료"

info 5/6 "md5 대조 및 기체 문법 검증"
LOCAL_SUMS="$(for f in "${FILES[@]}"; do
  rel="${f#flight/}"; printf '%s  %s\n' "$(md5sum "$f" | cut -d' ' -f1)" "$rel"; done | sort -k2)"
REMOTE_SUMS="$(ssh "$DRONE_HOST" "cd '$DRONE_DIR' && (find s2flight -name '*.py' | sort; echo '$ENTRY_NAME') | xargs md5sum" | awk '{print $1"  "$2}' | sort -k2)"
if [[ "$LOCAL_SUMS" != "$REMOTE_SUMS" ]]; then
  echo "--- 로컬"; echo "$LOCAL_SUMS"; echo "--- 기체"; echo "$REMOTE_SUMS"
  die "md5 불일치 — 전송이 손상되었거나 기체에 다른 파일이 남아 있습니다."
fi
ok "md5 일치 (${#FILES[@]} 파일)"
ssh "$DRONE_HOST" "cd '$DRONE_DIR' && python3 -m compileall -q s2flight >/dev/null && python3 -m py_compile '$ENTRY_NAME'" \
  || die "기체 python3 문법 검증 실패. 백업본(s2flight.bak.$STAMP.tgz)으로 되돌리세요."
ok "기체 python3 문법 통과"

info 6/6 "배포 기록"
ssh "$DRONE_HOST" "echo '$STAMP  s2flight+$ENTRY_NAME  git=$GIT_DESC' >> '$DRONE_DIR/.deploy_log'"
ok "기체 $DRONE_DIR/.deploy_log 에 기록"

printf '\n\033[32m배포 완료\033[0m  → %s:%s/{s2flight/, %s}\n' "$DRONE_HOST" "$DRONE_DIR" "$ENTRY_NAME"
printf '실행  : ssh %s "cd %s && python3 %s --help"\n' "$DRONE_HOST" "$DRONE_DIR" "$ENTRY_NAME"
printf '롤백  : ssh %s "cd %s && rm -rf s2flight && tar xzf s2flight.bak.%s.tgz"\n' "$DRONE_HOST" "$DRONE_DIR" "$STAMP"
