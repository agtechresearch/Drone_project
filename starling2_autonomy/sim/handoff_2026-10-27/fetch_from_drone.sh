#!/usr/bin/env bash
# 기체가 켜져 있을 때 실행: PX4 파라미터 export 파일 + 최신 ulog 를 이 폴더로 가져온다.
# 사용: bash fetch_from_drone.sh [root@<IP>]   (기본 starling2_lab alias, IP 바뀌면 ~/.ssh/config 갱신)
set -e
H="${1:-starling2_lab}"
F="-o ConnectTimeout=5 -o BatchMode=yes -o LogLevel=ERROR"
D="$(cd "$(dirname "$0")" && pwd)"
echo "== 1) px4-param save (이 빌드엔 export 없음)"
ssh $F "$H" 'px4-param save /data/px4/params_starling2.txt && ls -la /data/px4/params_starling2.txt'
scp $F "$H":/data/px4/params_starling2.txt "$D/params/params_starling2_export.bin"
echo "   (px4-param save 출력은 BSON 바이너리. 텍스트는 params/starling2_full_export.params 참고)"
echo "== 2) ulog 목록 (/data/px4/log)"
ssh $F "$H" 'find /data/px4/log -name "*.ulg" -printf "%TY-%Tm-%Td %TH:%TM %10s %p\n" | sort | tail -10'
echo "== 3) 최신 ulog 2개 복사"
for f in $(ssh $F "$H" 'find /data/px4/log -name "*.ulg" -printf "%T@ %p\n" | sort -n | tail -2 | cut -d" " -f2'); do
  scp $F "$H":"$f" "$D/ulog/"
done
ls -la "$D/params" "$D/ulog"
