# VOXL 읽기 전용 수집 지시서 (2026-11-02, 11-03 수정)

> 원본: 시뮬 서버 `plans/2026-11-02_voxl_readonly_collection.md`. 본문의 `plans/…`, `v3_marker/…` 경로는 시뮬 서버 기준이다.

**받는 사람:** 기체(VOXL 2) 안에서 실행되는 Claude 에이전트
**보낸 곳:** 시뮬 서버 쪽 Claude(claude-main). 결과는 사람이 zip으로 시뮬 서버에 옮긴다.
**목적:** 실기 시험(RC·Offboard 벤치, 저고도 비행)과 실기 입력 어댑터 작성에 앞서, 기체의 현재 설정과 데이터 형식을 **바꾸지 않고** 기록한다.

이 문서만으로 작업할 수 있게 썼다. 모르는 것은 추측하지 말고 "확인 못 함"으로 남긴다.

---

## 0. 절대 하지 말 것

아래 중 하나라도 필요해 보이면 멈추고 사람에게 묻는다. 결과 파일에는 "하지 않음: 이유"로 적는다.

1. **arm, 이륙, 모드 변경을 하지 않는다.** 모터·ESC 명령, MAVLink 명령(COMMAND_LONG 등), offboard setpoint, 액추에이터 시험도 모두 금지다.
2. **PX4 파라미터를 바꾸지 않는다.** `px4-param set`, `param save`, `param reset`, `param import`, QGC 쓰기가 모두 해당한다. **읽기(`show`)만 한다.**
3. **서비스 상태를 바꾸지 않는다.** `systemctl start/stop/restart/enable/disable`과 `voxl-configure-*`를 쓰지 않는다.
   - 예: `voxl-tag-detector`가 꺼져 있으면 켜지 않는다.
   - `systemctl status`, `list-units`, `cat` 같은 읽기는 해도 된다.
4. **설정 파일을 고치지 않는다.** `/etc/modalai/`, `/data/`, `/etc/systemd/` 아래가 모두 해당한다. 복사(`cp`로 결과 폴더에)만 한다.
5. **비행 코드를 고치거나 실행하지 않는다.** `path_flight_phase1_v13.py`(실기 비행 코드), `real_flight_safe.py`와 같은 비행 스크립트가 모두 해당한다. 위치와 sha256만 기록한다.
6. **패키지를 설치·삭제·업데이트하지 않는다.** apt, pip, opkg가 모두 해당한다. 재부팅하지 않는다.
7. **삭제하지 않는다.** 결과 폴더 밖에는 아무것도 쓰지 않는다.
8. **외부로 보내지 않는다.** 업로드, 메일, 네트워크 전송을 하지 않는다. 결과는 기체 안의 zip 하나로 끝낸다.
9. **실행 중인 프로세스를 죽이지 않는다.** `mavsdk_server`가 이미 떠 있어도 그대로 둔다.
10. **`mavsdk_server` 바이너리를 어떤 옵션으로도 실행하지 않는다.** `--version`, `--help`도 금지다(옵션을 모르면 서버가 떠서 MAVLink에 붙을 수 있다). 구성은 §4처럼 파일·프로세스 정보로만 확인한다. 다른 데몬 바이너리(`voxl-*-server`, `px4`)도 직접 실행하지 않는다.

**읽기 전용의 예외(허용하는 쓰기는 이것뿐이다):**
- 결과 폴더 `$OUT`(=`/data/claude_collect/<시각>/`) 아래 파일과, 그 상위의 최종 zip 하나.
- MPA 구독(§7): `mpa_capture.py`가 `/run/mpa/<pipe>/request` FIFO에 클라이언트 이름(`mpacap<pid>`)을 한 번 쓰고, MPA 서버가 `/run/mpa/<pipe>/mpacap<pid>` 임시 FIFO를 만든다. 구독자를 하나 더 붙이는 것이며 서비스 제어가 아니다. 이 임시 FIFO는 지우지 않는다(서버가 정리한다). 남아 있으면 `NOTES.md`에 이름만 적는다.
- `voxl-inspect-*` 읽기 도구도 같은 방식의 구독이다.

## 1. 준비

```bash
OUT=/data/claude_collect/$(date +%Y%m%d_%H%M%S)
mkdir -p $OUT/{sys,px4,mavsdk,mpa,conf,flightcode,capture}
# mpa_capture.py 위치 (저장소를 받은 곳에 맞춘다)
TOOLS=/data/claude_collect/Drone_project/starling2_autonomy/flight/tools_onboard
cd $OUT
# 명령 하나 = 파일 하나. 명령·종료코드·시각을 함께 남긴다.
run(){ local f=$1; shift; { echo "# \$ $*"; echo "# start $(date -Is)"; "$@"; echo "# rc=$? end $(date -Is)"; } > "$OUT/$f" 2>&1; }
```

- 디스크 여유를 먼저 확인한다(`df -h /data`). 여유가 500 MB 미만이면 §4의 캡처를 줄인다.
- 모든 명령은 `run` 함수로 실행한다. 명령이 없거나 실패해도 그 파일을 남기고 다음으로 넘어간다. 실패 자체가 정보다.
- 명령 이름은 voxl-suite 1.x 기준으로 적었다. 이름이 다르면 `ls /usr/bin | grep -i <키워드>`로 찾아 쓰고, 바꾼 이름을 `NOTES.md`에 적는다.

## 2. 시스템과 서비스 (우선순위 1)

```bash
run sys/voxl_version.txt      voxl-version
run sys/uname.txt             uname -a
run sys/os_release.txt        cat /etc/os-release
run sys/uptime.txt            uptime
run sys/df.txt                df -h
run sys/date_sync.txt         timedatectl
run sys/services_voxl.txt     voxl-inspect-services
run sys/units_all.txt         systemctl list-units --all --no-pager
run sys/unit_files.txt        systemctl list-unit-files --no-pager
run sys/ps.txt                ps -eo pid,ppid,user,etimes,args --sort=pid
run sys/listen_ports.txt      ss -lntup
run sys/dpkg_modal.txt        sh -c "dpkg -l | grep -iE 'voxl|modal|mpa|px4|mavsdk|qvio|open-vins|libmodal'"
run sys/ip.txt                ip -br addr
```

`voxl-px4`, `voxl-vision-hub`, `voxl-camera-server`, `voxl-open-vins-server`(또는 `voxl-qvio-server`), `voxl-tag-detector`, `voxl-mavlink-server`, mavsdk 관련 서비스 각각에 대해 아래를 기록한다.

```bash
for s in <위 서비스 이름들>; do
  run sys/status_$s.txt systemctl status $s --no-pager -l
  run sys/unit_$s.txt   systemctl cat $s --no-pager
done
```

## 3. PX4 파라미터와 버전 (우선순위 1)

```bash
run px4/param_show_all.txt      px4-param show
run px4/param_show_changed.txt  px4-param show -c      # 옵션이 없으면 실패 파일만 남는다
run px4/ver.txt                 px4-ver all             # 없으면 voxl-version 결과로 대신
run px4/status_commander.txt    px4-commander status    # 읽기 전용 상태 출력. 없으면 넘어간다
```

아래 값은 `param_show_all.txt`에서 뽑아 `px4/key_params.txt`에 따로 모은다. 기존 덤프(2026-10-27 handoff)와 대조하는 데 쓴다.

- 실패 동작: `COM_OBL_RC_ACT COM_FAIL_ACT_T NAV_RCL_ACT COM_RC_OVERRIDE COM_RC_STICK_OV COM_RCL_EXCEPT COM_OF_LOSS_T COM_RC_LOSS_T COM_RC_IN_MODE NAV_DLL_ACT`
- RC 채널: `RC_MAP_*`(특히 `RC_MAP_KILL_SW`, `RC_MAP_FLTMODE`), `COM_FLTMODE*`
- 추정기: `EKF2_EV_CTRL EKF2_EV_DELAY EKF2_EVP_NOISE EKF2_EVA_NOISE EKF2_HGT_REF EKF2_IMU_POS_X/Y/Z EKF2_GPS_CTRL EKF2_MAG_TYPE`
- 비행: `MPC_THR_HOVER MPC_XY_VEL_MAX MPC_Z_VEL_MAX_UP MPC_Z_VEL_MAX_DN THR_MDL_FAC`
- 귀환: `RTL_RETURN_ALT RTL_CONE_ANG RTL_MIN_DIST NAV_ACC_RAD`

로그 목록(파일은 복사하지 않음):

```bash
run px4/ulog_list.txt  sh -c "ls -la --time-style=full-iso /data/px4/log/ 2>&1; find /data/px4/log -name '*.ulg' -printf '%TY-%Tm-%Td %TH:%TM %s %p\n' | sort | tail -30"
```

## 4. mavsdk_server 구성 (우선순위 1)

실기에서 `real_flight_safe.py --own-server`를 쓸 수 있는지 판단하려는 항목이다(plans/2026-11-01_rc_failsafe_params.md §4).

**`mavsdk_server`는 실행하지 않는다(§0-10).** 아래는 모두 파일·프로세스 읽기다.

```bash
run mavsdk/mounts.txt         sh -c "findmnt -no TARGET,SOURCE,FSTYPE / /data 2>&1; df -h / /data"
run mavsdk/which.txt          sh -c "which -a mavsdk_server; ls -la /usr/bin/mavsdk_server* /usr/local/bin/mavsdk_server* 2>&1"
# /data가 별도 마운트면 -xdev 검색에서 빠지므로 /data를 따로 찾는다
run mavsdk/find_bin.txt       sh -c "find / -xdev -name 'mavsdk_server*' -type f -perm -u+x ! -name '*.log' 2>/dev/null; echo '# /data'; find /data -xdev -name 'mavsdk_server*' -type f -perm -u+x ! -name '*.log' 2>/dev/null; echo '# home'; find /home /root -xdev -name 'mavsdk_server*' -type f -perm -u+x ! -name '*.log' 2>/dev/null"
run mavsdk/running.txt        sh -c "ps -eo pid,ppid,user,etimes,args | grep -i [m]avsdk"
# 떠 있는 서버마다: 실제 실행 파일, 명령줄, 부모, 소켓
run mavsdk/proc.txt           sh -c 'for p in $(pgrep -x mavsdk_server); do echo "## pid $p"; ls -la /proc/$p/exe; tr "\0" " " < /proc/$p/cmdline; echo; grep -E "^(Name|PPid)" /proc/$p/status; pp=$(grep PPid /proc/$p/status | tr -dc 0-9); ps -o pid,args -p $pp; cat /proc/$p/cgroup; done 2>&1'
run mavsdk/sockets.txt        sh -c "ss -lntup 2>&1 | grep -iE 'mavsdk|5005[0-9]|1455[0-9]|1454[0-9]'; ss -ntup 2>&1 | grep -i mavsdk"
# 찾은 바이너리마다 해시·패키지 소유. 실행하지 않는다
run mavsdk/bin_sha256.txt     sh -c "for f in \$( { which -a mavsdk_server; find /data /usr /opt /home -xdev -name 'mavsdk_server*' -type f -perm -u+x ! -name '*.log'; for p in \$(pgrep -x mavsdk_server); do readlink -f /proc/\$p/exe; done; } 2>/dev/null | xargs -r readlink -f | sort -u ); do echo; echo \"## \$f\"; ls -la \$f; sha256sum \$f; dpkg -S \$f 2>&1; file \$f 2>&1; done"
run mavsdk/dpkg.txt           sh -c "dpkg -l | grep -iE 'mavsdk|grpc'; for p in \$(dpkg -l | awk '/mavsdk/{print \$2}'); do dpkg -s \$p; dpkg -L \$p | head -40; done 2>&1"
run mavsdk/pip.txt            sh -c "python3 -m pip show mavsdk 2>&1; python3 -c 'import mavsdk, os; print(mavsdk.__file__); print(os.listdir(os.path.dirname(mavsdk.__file__)+\"/bin\"))' 2>&1"
run mavsdk/python.txt         sh -c "which -a python3; python3 --version"
run mavsdk/autostart.txt      sh -c "grep -rIl -i mavsdk /etc/systemd /lib/systemd /etc/init.d /etc/rc.local /home /data 2>/dev/null | head -50"
run mavsdk/mavlink_server.txt sh -c "cat /etc/modalai/voxl-mavlink-server.conf 2>&1"
```

- `autostart.txt`에서 찾은 유닛·스크립트는 `systemctl cat <유닛>` 또는 `cat`으로 내용을 `mavsdk/` 아래에 남긴다.
- 버전은 패키지 정보(`dpkg -s`, pip 버전, 파일 이름)에서만 읽는다. 거기서 안 나오면 **"버전 미확인(바이너리 미실행)"**으로 적는다.
- 결과 요약 `mavsdk/SUMMARY.md`에 다음을 적는다.
  - 상주 서비스인지(유닛 이름, 자동 시작 여부)
  - 바이너리 경로·sha256·소유 패키지·버전(또는 미확인). 여러 개면 모두
  - 떠 있는 서버의 `/proc/<pid>/exe` 경로가 위 바이너리 중 어느 것인지
  - 지금 떠 있는 서버의 연결 문자열(예: `udp://:14551`)과 gRPC 포트(50051 등)
  - 비행 코드가 어느 서버에 붙는지 알 수 있으면 그 근거

## 5. 설정 파일 복사 (우선순위 2)

```bash
cp -a /etc/modalai $OUT/conf/etc_modalai 2>> $OUT/conf/cp_errors.txt
ls -laR /etc/modalai > $OUT/conf/ls_etc_modalai.txt 2>&1
for f in /data/modalai/*.conf /data/modalai/*.yml /data/modalai/*.json; do [ -f "$f" ] && cp -a "$f" $OUT/conf/; done
```

특히 다음 파일이 있는지 확인하고 `conf/CHECK.md`에 있음/없음을 적는다.
- `extrinsics.conf`, `voxl-vision-hub.conf`, `voxl-camera-server.conf`, `voxl-tag-detector.conf`, `tag_locations.conf`, `vio_cams.conf`, `voxl-open-vins-server.conf`(또는 qvio 설정)
- `opencv_*_intrinsics.yml`

2026-10-30 사본과 달라졌는지는 시뮬 서버 쪽에서 비교한다. 여기서는 복사만 한다.

## 6. MPA 파이프 형식 (우선순위 2)

```bash
run mpa/ls_run_mpa.txt  ls -la /run/mpa/
for d in /run/mpa/*/; do p=$(basename $d); cp $d/info $OUT/mpa/info_$p.json 2>/dev/null; done
run mpa/headers.txt     sh -c "ls -la /usr/include/modal_pipe* /usr/include/modal_*.h 2>&1; find / -xdev -name 'modal_pipe*.h' 2>/dev/null"
mkdir -p $OUT/mpa/headers; find / -xdev -name 'modal_pipe*.h' -exec cp {} $OUT/mpa/headers/ \; 2>/dev/null
run mpa/inspect_cam_all.txt  timeout 10 voxl-inspect-cam -a
```

## 7. 짧은 파이프 기록 (우선순위 3, 기체는 바닥에 정지)

도구: `mpa_capture.py`(이 저장소의 `starling2_autonomy/flight/tools_onboard/mpa_capture.py`. §1의 `$TOOLS`. 파이썬 3.6 표준 라이브러리만 쓴다). 실기에서 한 번도 돌려 보지 않았으므로 `--probe`부터 한다.

`mpa_capture.py`는 결과를 종료 코드와 `<접두어>_<pipe>.status.json`으로 남긴다(2026-11-03 수정). **"성공"은 rc 0뿐이다.**

| rc | status | 뜻 |
|---|---|---|
| 0 | ok | 연결됐고 데이터가 1바이트 이상 왔다(기록은 정해진 시간 끝까지). **형식 검증은 아니다** |
| 2 | no_pipe | `/run/mpa/<pipe>` 없음 |
| 3 | connect_fail | request FIFO에 못 쓰거나, 서버가 3 s 안에 클라이언트 FIFO를 만들지 않음 |
| 4 | no_data | 연결됐지만 0바이트 |
| 5 | eof_early | 데이터가 오다가 시간 전에 끊김(받은 데이터는 보존) |

대상 파이프 목록을 먼저 정한다. **지금 `/run/mpa/`에 이미 있는 파이프뿐이다.** 서비스를 켜서 파이프를 만들지 않는다.
- `tof_pc`
- `tag_detections` (있을 때만)
- pose 계열: `ls /run/mpa`에서 이름에 `qvio`, `ov`, `vvhub`, `body_wrt`가 들어간 것 **전부**
- 카메라 영상 파이프는 크므로 제외

```bash
cd $OUT/capture
ls /run/mpa > pipes_all.txt
PIPES="tof_pc tag_detections $(ls /run/mpa | grep -E 'qvio|ov|vvhub|body_wrt' | tr '\n' ' ')"
echo "$PIPES" > pipes_selected.txt
# 1) 선택한 모든 파이프를 먼저 probe한다 (pose 포함)
for p in $PIPES; do
  [ -d /run/mpa/$p ] || { echo "$p MISSING" >> probe_rc.txt; continue; }
  { echo "# start $(date -Is)"; python3 $TOOLS/mpa_capture.py --probe $p; echo "# rc=$? end $(date -Is)"; } > probe_$p.txt 2>&1
  echo "$p $(tail -1 probe_$p.txt)" >> probe_rc.txt
done
```

2) **probe가 rc 0인 파이프만** 기록한다. 시간: `tof_pc` 5 s, pose 각 10 s, `tag_detections` 10 s.

```bash
python3 $TOOLS/mpa_capture.py <pipe> <초> S0_static > rec_<pipe>.txt 2>&1; echo "<pipe> rc=$?" >> rec_rc.txt
```

- rc 4·5이거나 probe가 실패한 파이프는 실패 파일(`*.status.json`, `*.txt`, 빈 `.bin`)을 **지우지 않고** 그대로 둔다. 그리고 읽기 도구 출력으로 대신한다: `timeout 5 voxl-inspect-points tof_pc`, `timeout 10 voxl-inspect-pose <pipe>`, `timeout 10 voxl-inspect-tags`.
- `tag_detections`는 카메라 앞에 태그가 없으면 데이터가 안 올 수 있다. 이 경우 rc 4는 정상적인 결과다. `SCENE.md`의 태그 유무와 함께 적는다.
- 데이터가 왔다는 것(rc 0)은 형식이 맞다는 뜻이 아니다. 프레임 경계·timestamp 검증은 시뮬 서버에서 `mpa_parse.py`로 한다. 요약에 "형식 검증 완료"라고 쓰지 않는다.
- 기체 주변에 사람이 무엇을 두었는지(벽 거리, 태그 유무)를 사람에게 물어 `capture/SCENE.md`에 적는다. 모르면 "모름"으로 둔다.

## 8. 비행 코드 위치 (우선순위 3)

```bash
run flightcode/find.txt sh -c "find / -xdev \( -name 'path_flight_phase1_v13.py' -o -name 'real_flight_safe.py' -o -name 'wk_mission.py' \) 2>/dev/null"
# 찾은 파일마다 sha256만 기록한다. 실행하지 않는다.
run flightcode/sha256.txt sh -c "find / -xdev \( -name 'path_flight_phase1_v13.py' -o -name 'real_flight_safe.py' \) -exec sha256sum {} \; 2>/dev/null"
```

## 9. 마무리

1. `$OUT/NOTES.md`를 쓴다.
   - 실행 시각, 실행자(에이전트), 기체 상태(전원·배터리·프로펠러 장착 여부는 사람에게 묻는다)
   - 명령 이름을 바꾼 것, 건너뛴 것과 그 이유, 실패한 명령 목록
   - §0의 금지 사항을 어긴 것이 없다는 확인. 혹시 어겼으면 무엇을 했는지 그대로 적는다
2. 수집 전후로 서비스 상태가 같은지 확인한다.
   ```bash
   systemctl list-units --all --no-pager > $OUT/sys/units_all_after.txt 2>&1
   diff <(awk '{print $1,$3,$4}' $OUT/sys/units_all.txt) <(awk '{print $1,$3,$4}' $OUT/sys/units_all_after.txt) > $OUT/sys/units_diff.txt
   ```
   ```bash
   ps -eo pid,ppid,user,etimes,args | grep -i [m]avsdk > $OUT/mavsdk/running_after.txt 2>&1
   ```
   차이가 있으면 `NOTES.md`에 적는다(서비스 상태, mavsdk_server PID). 되돌리려고 서비스를 건드리지는 않는다.
3. 목록과 해시를 남기고 묶는다.
   ```bash
   cd $OUT && find . -type f ! -name MANIFEST.sha256 -exec sha256sum {} \; | sort -k2 > MANIFEST.sha256
   cd $(dirname $OUT) && zip -qr voxl_collect_$(basename $OUT).zip $(basename $OUT) && ls -la voxl_collect_$(basename $OUT).zip
   ```
4. 사람에게 zip 경로와 크기, 그리고 다음 세 줄 요약을 보고한다.
   - mavsdk_server: 상주 여부 / 바이너리 경로·sha256·버전(또는 미확인) / 연결 문자열
   - 실패 동작 파라미터 4개의 현재 값(COM_OBL_RC_ACT, COM_FAIL_ACT_T, NAV_RCL_ACT, COM_RC_IN_MODE)
   - 파이프별 probe·기록 rc와 status(`probe_rc.txt`, `rec_rc.txt`). rc 0만 "수신됨"으로 쓴다

## 완료 기준
- §2~§4가 모두 파일로 있다. 실패한 명령도 파일로 남아 있다.
- `units_diff.txt`가 비었거나, 차이에 대한 설명이 `NOTES.md`에 있다.
- `MANIFEST.sha256`와 zip이 있다.
- 파라미터·서비스·설정·비행 코드를 바꾼 것이 없다(`NOTES.md`에 확인).
- `mavsdk_server`를 실행하지 않았고, 수집 전후 `mavsdk/running.txt`와 같은 서버 PID가 그대로 있다(`ps` 한 번 더 찍어 `mavsdk/running_after.txt`).
- §7의 모든 대상 파이프에 probe 결과(rc, status)가 있다.

## 시뮬 서버 쪽에서 이어서 할 일 (참고, 에이전트가 할 일 아님)
- 2026-10-27 handoff 파라미터와 대조한다. 특히 EKF2_IMU_POS, 실패 동작 값, RC 매핑.
- 2026-10-30 `cameras_2026-10-30.zip` 설정과 대조한다.
- mavsdk_server 구성을 보고 `--own-server` 적용 방법을 정한다.
- 파이프 원시 자료로 `mpa_parse.py`를 검증하고, 실기 입력 어댑터를 작성한다.
- 그 다음 사람 입회 벤치 시험 절차서(plans/2026-11-01_rc_failsafe_params.md §3)를 기체 구성에 맞춰 고친다.
