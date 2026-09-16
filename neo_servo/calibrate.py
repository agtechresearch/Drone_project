"""DJI Fly 앱 UI 좌표 캘리브레이션.

두 가지 모드:
  - capture (기본): 각 UI 요소를 5초 카운트다운으로 순차 캡처
                    → config.yaml에 붙여넣을 YAML 스니펫 출력
  - track: 마우스 상대좌표 실시간 표시 (창 크기/원점 확인용)

사전 준비:
  1. scrcpy로 폰 미러링 (예: scrcpy --window-title "NoteNine")
  2. 폰의 DJI Fly 앱을 이륙 대기 화면으로 (조이스틱/버튼이 다 보이도록)
  3. python calibrate.py [--title NoteNine] [--mode capture|track]
"""
from __future__ import annotations
import dpi_setup  # noqa: F401 - Windows DPI awareness (반드시 최상단)
import argparse
import math
import sys
import time

try:
    import pyautogui
    import pywinctl as pw
except ImportError as e:
    print(f"필수 패키지 미설치: {e.name}")
    print("pip install pyautogui pywinctl")
    sys.exit(1)


# ------------------------------------------------------------
# 캡처 대상 (순서 = 사용자 클릭 순서)
# 튜플: (yaml_key, 화면 안내 문구, 결과 그룹)
#   그룹 'stick_edge'는 반경 계산 후 폐기
# ------------------------------------------------------------
ITEMS = [
    ("left_stick_center",  "좌 조이스틱 원의 중심",                    'coord'),
    ("right_stick_center", "우 조이스틱 원의 중심",                    'coord'),
    ("_stick_edge",        "우 조이스틱 원의 가장자리 (한 지점, 반경 측정용)", 'edge'),
    ("takeoff_button",     "이륙 버튼 (앱 좌측의 이/착륙 아이콘)",       'coord'),
    ("takeoff_confirm",    "이륙 확인 홀드 슬라이더의 도착점(오른쪽 끝)", 'coord'),
    ("landing_button",     "착륙 버튼 (없으면 takeoff와 같은 지점)",     'coord'),
    ("capture_button",     "촬영/사진 버튼",                            'coord'),
]

COUNTDOWN_SEC = 5


def find_window(title: str):
    wins = pw.getWindowsWithTitle(title)
    if not wins:
        return None
    return wins[0]


# ============================================================
# 모드 1: 실시간 추적
# ============================================================
def track_mode(win):
    ox, oy = win.left, win.top
    print(f"창 제목: {win.title!r}")
    print(f"창 원점: ({ox}, {oy})")
    print(f"창 크기: {win.width} x {win.height}")
    print("→ 마우스를 창 위로 움직여보세요. Ctrl+C로 종료.\n")
    try:
        while True:
            x, y = pyautogui.position()
            rx, ry = x - ox, y - oy
            inside = 0 <= rx < win.width and 0 <= ry < win.height
            mark = "IN " if inside else "OUT"
            print(f"\r[{mark}] 절대=({x:5d},{y:5d})  상대=({rx:5d},{ry:5d})   ",
                  end='', flush=True)
            time.sleep(0.05)
    except KeyboardInterrupt:
        print()


# ============================================================
# 모드 2: 순차 캡처
# ============================================================
def capture_mode(win):
    ox, oy = win.left, win.top
    w, h = win.width, win.height
    print(f"\n창 제목: {win.title!r}")
    print(f"창 원점: ({ox}, {oy}), 크기: {w} x {h}")
    print(f"각 항목마다 {COUNTDOWN_SEC}초 카운트다운 후 마우스 좌표를 자동 캡처합니다.")
    print("Ctrl+C로 언제든 중단 가능.\n")

    results: dict[str, tuple[int, int]] = {}

    try:
        for key, desc, _kind in ITEMS:
            print(f"── {desc} ──")
            for i in range(COUNTDOWN_SEC, 0, -1):
                x, y = pyautogui.position()
                rx, ry = x - ox, y - oy
                print(f"\r   {i}초 후 캡처... 현재 상대좌표=({rx:4d},{ry:4d})    ",
                      end='', flush=True)
                time.sleep(1)
            x, y = pyautogui.position()
            rx, ry = x - ox, y - oy
            results[key] = (rx, ry)
            print(f"\r   ★ 캡처됨: [{rx}, {ry}]                                \n")
    except KeyboardInterrupt:
        print("\n중단됨. 지금까지 캡처된 값만 출력:")

    # 반경 계산
    stick_radius = None
    if "_stick_edge" in results and "right_stick_center" in results:
        ex, ey = results["_stick_edge"]
        cx, cy = results["right_stick_center"]
        stick_radius = round(math.hypot(ex - cx, ey - cy))
    results.pop("_stick_edge", None)

    # ---- YAML 스니펫 출력 ----
    print("\n" + "=" * 56)
    print("config.yaml의 phone / ui_layout 섹션에 붙여넣기:")
    print("=" * 56)
    print(f"""
phone:
  window_title: "{win.title}"
  window_size: [{w}, {h}]

ui_layout:""")
    keys_ordered = [
        "left_stick_center", "right_stick_center",
        "takeoff_button", "takeoff_confirm",
        "landing_button", "capture_button",
    ]
    for k in keys_ordered:
        if k in results:
            rx, ry = results[k]
            print(f"  {k+':':<22} [{rx}, {ry}]")
    if stick_radius is not None:
        print(f"  stick_radius:          {stick_radius}")
    else:
        print(f"  stick_radius:          # 미측정")
    print("  mask_regions: []\n")


# ============================================================
def main():
    p = argparse.ArgumentParser(description="DJI Fly UI 좌표 캘리브레이션")
    p.add_argument("--title", default="NoteNine",
                   help="scrcpy 창 제목 (기본: NoteNine)")
    p.add_argument("--mode", choices=["capture", "track"], default="capture",
                   help="capture=순차 캡처(기본), track=실시간 좌표 표시")
    args = p.parse_args()

    win = find_window(args.title)
    if win is None:
        print(f"창을 찾을 수 없음: {args.title!r}")
        print("scrcpy가 실행 중인지, --title이 실제 창 제목과 일치하는지 확인.")
        print("팁: scrcpy 실행 시 --window-title \"NoteNine\" 옵션으로 제목 고정.")
        sys.exit(1)

    if args.mode == "track":
        track_mode(win)
    else:
        capture_mode(win)


if __name__ == "__main__":
    main()