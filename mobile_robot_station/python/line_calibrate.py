#!/usr/bin/env python3
"""
바닥 테이프 색상 캘리브레이션 GUI + 샘플 라이브러리 관리.

카메라 화면을 띄우고 마우스로 영역을 드래그하면 그 영역의 색을
HSV/Lab로 변환해 라인(테이프)과 바닥의 색을 샘플링합니다.

두 개의 파일로 관리됩니다:
  config/line_samples.json  = 원본 샘플 라이브러리 (이 도구로 편집)
  config/line_calib.json    = 위 샘플들을 컴파일한 결과 (line_follow.py가 읽음)

시작하면 기존 line_samples.json 을 자동으로 불러오므로, 실증 중 조명이
바뀌는 지점에서 그 지점 색을 "추가"하면 기존 샘플에 누적되어 검출 범위가
union 으로 넓어집니다. 필요 없는 샘플은 선택해서 삭제하거나 클래스/메모를
수정할 수 있어, JSON 을 손으로 편집할 필요가 없습니다.

조작법 (키 입력은 이 스크립트를 실행한 터미널에서도 받습니다 — 라즈베리파이에
키보드가 없어도 SSH 터미널에서 조작 가능. 마우스 드래그는 카메라 창에서):
  l : 이후 드래그 = 라인(테이프) 샘플로 추가
  f : 이후 드래그 = 바닥 샘플로 추가
  (마우스 왼쪽 드래그로 영역 선택 → 샘플 추가)
  k / j : 샘플 목록에서 선택 위/아래 이동
  d : 선택한 샘플 삭제
  c : 선택한 샘플의 클래스 전환 (line ↔ floor)
  n : 선택한 샘플에 메모 입력 (예: "입구-형광등")
  u : 직전에 추가한 샘플 하나 취소
  r : 전체 초기화
  p : 계산된 마스크 미리보기 on/off
  m : 미리보기/저장 모드 전환 (dark → hsv → lab)
  s : line_samples.json + line_calib.json 저장
  q / ESC : 종료

창 크기:
  --scale 3.0  으로 시작 시 배율 지정 (기본 2.0)
  실행 중에도 창 테두리를 마우스로 드래그하면 자유롭게 리사이즈 가능
"""
import os
import sys
import json
import select
import termios
import tty
import cv2
import numpy as np

CAMERA_INDEX = 0
FRAME_WIDTH  = 320
FRAME_HEIGHT = 240

ROI_TOP_RATIO = 0.55  # line_follow.py 와 동일하게 맞춰야 미리보기가 의미 있음

CONFIG_DIR   = os.path.join(os.path.dirname(__file__), "..", "config")
SAMPLES_PATH = os.path.join(CONFIG_DIR, "line_samples.json")
CALIB_PATH   = os.path.join(CONFIG_DIR, "line_calib.json")

WINDOW = "line_calibrate"

# 창 초기 배율 (--scale 로 변경 가능). 창 테두리를 드래그해서도 자유롭게 조절 가능.
DEFAULT_SCALE = 2.0
LIST_ROWS  = 6   # 샘플 목록에서 한 번에 보이는 줄 수
PANEL_LINES = 12  # 하단 정보 패널 고정 줄 수 (창 크기 안정)


def summarize(patch, sample_class, note=""):
    """드래그한 픽셀 패치를 저장용 요약(퍼센타일 색 통계)으로 변환."""
    px = patch.astype(np.uint8).reshape(-1, 1, 3)
    gray = cv2.cvtColor(px, cv2.COLOR_BGR2GRAY).reshape(-1)
    hsv = cv2.cvtColor(px, cv2.COLOR_BGR2HSV).reshape(-1, 3)
    lab = cv2.cvtColor(px, cv2.COLOR_BGR2LAB).reshape(-1, 3)

    def pct(a):
        return [[int(v) for v in np.percentile(a, p, axis=0)] for p in (5, 50, 95)]

    return {
        "class": sample_class,
        "note": note,
        "n_px": int(px.shape[0]),
        "gray": float(np.median(gray)),
        "hsv": pct(hsv),   # [[p5],[p50],[p95]] per channel
        "lab": pct(lab),
    }


class Calibrator:
    def __init__(self, scale=DEFAULT_SCALE):
        self.cap = cv2.VideoCapture(CAMERA_INDEX)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,  FRAME_WIDTH)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
        if not self.cap.isOpened():
            raise RuntimeError(f"카메라 {CAMERA_INDEX} 열기 실패")

        self.sample_mode = "line"     # 다음 드래그를 어느 클래스로 추가할지
        self.preview_mode = "dark"    # 'dark' | 'hsv' | 'lab'
        self.preview_on = False

        self.samples = []  # 라이브러리: summarize() 결과 dict들의 리스트
        self.sel = 0       # 선택 커서 (목록 인덱스)

        self.dragging  = False
        self.start_pt  = (0, 0)
        self.cur_pt    = (0, 0)
        self.last_frame = None
        self._orig_term = None

        self.calib = None  # 컴파일 결과 dict

        self._load()

        # QT5 backend라 WINDOW_NORMAL로 만들면 창 테두리 드래그로 자유 리사이즈 가능
        cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(WINDOW, int(FRAME_WIDTH * scale),
                          int((FRAME_HEIGHT + 20 * (PANEL_LINES + 1)) * scale))
        cv2.setMouseCallback(WINDOW, self._on_mouse)

    # ── 라이브러리 로드/저장 ────────────────────────────────────────
    def _load(self):
        if os.path.exists(SAMPLES_PATH):
            with open(SAMPLES_PATH) as f:
                data = json.load(f)
            self.samples = data.get("samples", [])
            print(f"기존 샘플 {len(self.samples)}개 로드: {SAMPLES_PATH}")
        # 이전에 저장한 모드가 있으면 미리보기 모드로 복원
        if os.path.exists(CALIB_PATH):
            try:
                with open(CALIB_PATH) as f:
                    self.preview_mode = json.load(f).get("mode", self.preview_mode)
            except Exception:
                pass
        self._recompute()

    def save(self):
        if not self.samples:
            print("저장할 샘플이 없습니다 (먼저 라인 영역을 드래그하세요)")
            return
        os.makedirs(CONFIG_DIR, exist_ok=True)
        with open(SAMPLES_PATH, "w") as f:
            json.dump({"samples": self.samples}, f, indent=2, ensure_ascii=False)

        if self.calib:
            out = dict(self.calib)
            out["mode"] = self.preview_mode
            with open(CALIB_PATH, "w") as f:
                json.dump(out, f, indent=2, ensure_ascii=False)
            print(f"저장 완료: {SAMPLES_PATH} (샘플 {len(self.samples)}개)"
                  f" + {CALIB_PATH} (mode={self.preview_mode})")
        else:
            print(f"저장 완료: {SAMPLES_PATH} (샘플만 저장 — line 샘플이 없어 calib 미생성)")

    # ── 마우스 → 샘플 추가 ──────────────────────────────────────────
    def _on_mouse(self, event, x, y, flags, _param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.dragging = True
            self.start_pt = (x, y)
            self.cur_pt = (x, y)
        elif event == cv2.EVENT_MOUSEMOVE and self.dragging:
            self.cur_pt = (x, y)
        elif event == cv2.EVENT_LBUTTONUP and self.dragging:
            self.dragging = False
            self.cur_pt = (x, y)
            self._commit_sample()

    def _commit_sample(self):
        if self.last_frame is None:
            return
        x0, y0 = self.start_pt
        x1, y1 = self.cur_pt
        x0, x1 = sorted((max(0, x0), max(0, x1)))
        y0, y1 = sorted((max(0, y0), max(0, y1)))
        if x1 - x0 < 3 or y1 - y0 < 3:
            return

        patch = self.last_frame[y0:y1, x0:x1].reshape(-1, 3)
        self.samples.append(summarize(patch, self.sample_mode))
        self.sel = len(self.samples) - 1
        print(f"[샘플] {self.sample_mode} 추가 ({patch.shape[0]}px)  총 {len(self.samples)}개")
        self._recompute()

    # ── 편집 ────────────────────────────────────────────────────────
    def _clamp_sel(self):
        self.sel = 0 if not self.samples else max(0, min(self.sel, len(self.samples) - 1))

    def _undo_sample(self):
        if not self.samples:
            print("[undo] 지울 샘플이 없습니다")
            return
        self.samples.pop()
        self._clamp_sel()
        print(f"[undo] 직전 샘플 삭제  남은 {len(self.samples)}개")
        self._recompute()

    def _delete_selected(self):
        if not self.samples:
            print("[삭제] 샘플이 없습니다")
            return
        s = self.samples.pop(self.sel)
        self._clamp_sel()
        print(f"[삭제] [{self.sel}] {s['class']} 삭제  남은 {len(self.samples)}개")
        self._recompute()

    def _toggle_class_selected(self):
        if not self.samples:
            return
        s = self.samples[self.sel]
        s["class"] = "floor" if s["class"] == "line" else "line"
        print(f"[클래스] [{self.sel}] → {s['class']}")
        self._recompute()

    def _edit_note_selected(self):
        if not self.samples:
            return
        if not sys.stdin.isatty():
            print("[메모] 터미널(TTY)에서만 입력 가능")
            return
        fd = sys.stdin.fileno()
        termios.tcsetattr(fd, termios.TCSADRAIN, self._orig_term)  # 잠시 표준 입력 모드
        try:
            note = input(f"\r[{self.sel}] 메모 입력 후 Enter: ").strip()
        except EOFError:
            note = ""
        finally:
            tty.setcbreak(fd)
        self.samples[self.sel]["note"] = note
        print(f"[메모] [{self.sel}] = '{note}'")

    # ── 컴파일: 샘플 라이브러리 → 검출 범위 ─────────────────────────
    def _recompute(self):
        line = [s for s in self.samples if s["class"] == "line"]
        if not line:
            self.calib = None
            return

        def lower(key):
            return [min(s[key][0][i] for s in line) for i in range(3)]

        def upper(key):
            return [max(s[key][2][i] for s in line) for i in range(3)]

        line_gray = float(np.median([s["gray"] for s in line]))
        result = {
            "line_sample_px": sum(s["n_px"] for s in line),
            "line_gray_median": line_gray,
            "hsv_lower": lower("hsv"), "hsv_upper": upper("hsv"),
            "lab_lower": lower("lab"), "lab_upper": upper("lab"),
        }

        floor = [s for s in self.samples if s["class"] == "floor"]
        if floor:
            floor_gray = float(np.median([s["gray"] for s in floor]))
            gap = floor_gray - line_gray
            result["floor_sample_px"] = sum(s["n_px"] for s in floor)
            result["floor_gray_median"] = floor_gray
            result["gray_gap"] = gap
            # 라인-바닥 명도 차의 절반만 오프셋으로 (5~80 clamp)
            result["dark_offset"] = max(5, min(80, gap * 0.5))
        else:
            result["dark_offset"] = None

        self.calib = result

    # ── 미리보기 마스크 ─────────────────────────────────────────────
    def _build_preview_mask(self, frame):
        h, w = frame.shape[:2]
        roi_top = int(h * ROI_TOP_RATIO)
        roi = frame[roi_top:h, :]

        if self.preview_mode == "hsv" and self.calib and self.calib.get("hsv_lower"):
            hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, np.array(self.calib["hsv_lower"]), np.array(self.calib["hsv_upper"]))
        elif self.preview_mode == "lab" and self.calib and self.calib.get("lab_lower"):
            lab = cv2.cvtColor(roi, cv2.COLOR_BGR2LAB)
            mask = cv2.inRange(lab, np.array(self.calib["lab_lower"]), np.array(self.calib["lab_upper"]))
        else:
            gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
            blurred = cv2.GaussianBlur(gray, (5, 5), 0)
            otsu_val, _ = cv2.threshold(blurred, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
            offset = (self.calib or {}).get("dark_offset") or 25
            thresh = max(0, otsu_val - offset)
            _, mask = cv2.threshold(blurred, thresh, 255, cv2.THRESH_BINARY_INV)

        full_mask = np.zeros((h, w), dtype=np.uint8)
        full_mask[roi_top:h, :] = mask
        return full_mask

    def _sample_list_lines(self):
        """선택 커서 주변 LIST_ROWS개 샘플을 표시용 문자열로."""
        n = len(self.samples)
        header = f"-- samples ({n})  k/j=sel d=del c=class n=note --"
        if n == 0:
            return [header, "  (없음 — 드래그로 추가)"]
        start = max(0, min(self.sel - LIST_ROWS // 2, n - LIST_ROWS))
        rows = []
        for i in range(start, min(start + LIST_ROWS, n)):
            s = self.samples[i]
            cur = ">" if i == self.sel else " "
            note = f" {s['note']}" if s.get("note") else ""
            rows.append(f"{cur}[{i}] {s['class']:5} g{s['gray']:.0f}{note}")
        return [header] + rows

    def _draw_overlay(self, frame):
        viz = frame.copy()
        h, w = viz.shape[:2]
        roi_top = int(h * ROI_TOP_RATIO)
        cv2.line(viz, (0, roi_top), (w, roi_top), (0, 255, 255), 1)

        if self.preview_on:
            mask = self._build_preview_mask(frame)
            red = np.zeros_like(viz)
            red[:, :, 2] = mask
            viz = cv2.addWeighted(viz, 1.0, red, 0.5, 0)

        if self.dragging:
            cv2.rectangle(viz, self.start_pt, self.cur_pt, (0, 255, 0), 1)

        lines = [
            f"add-class: {self.sample_mode.upper()} (l=line f=floor)   "
            f"preview: {'ON' if self.preview_on else 'off'} ({self.preview_mode}, m)   "
            f"u=undo r=reset s=save q=quit",
        ]
        if self.calib:
            if self.calib.get("dark_offset") is not None:
                lines.append(
                    f"gray  line={self.calib['line_gray_median']:.0f}"
                    f"  floor={self.calib.get('floor_gray_median', float('nan')):.0f}"
                    f"  gap={self.calib.get('gray_gap', float('nan')):.0f}"
                    f"  -> dark_offset={self.calib['dark_offset']:.0f}"
                )
            else:
                lines.append(f"gray  line={self.calib['line_gray_median']:.0f}  (floor 샘플 없음)")
            lines.append(f"hsv  {self.calib['hsv_lower']} ~ {self.calib['hsv_upper']}")
            lines.append(f"lab  {self.calib['lab_lower']} ~ {self.calib['lab_upper']}")
        lines += self._sample_list_lines()

        lines = lines[:PANEL_LINES]
        lines += [""] * (PANEL_LINES - len(lines))

        canvas = np.zeros((h + 20 * (PANEL_LINES + 1), w, 3), dtype=np.uint8)
        canvas[:h, :, :] = viz
        for i, text in enumerate(lines):
            cv2.putText(canvas, text, (5, h + 20 * (i + 1)), cv2.FONT_HERSHEY_SIMPLEX,
                        0.42, (255, 255, 255), 1, cv2.LINE_AA)
        return canvas

    # ── 키 입력 ─────────────────────────────────────────────────────
    def _read_stdin_key(self):
        """스크립트를 실행한 터미널(SSH 포함)에서 눌린 키 1개 반환. 없으면 255."""
        if not sys.stdin.isatty():
            return 255
        if select.select([sys.stdin], [], [], 0)[0]:
            ch = sys.stdin.read(1)
            if ch:
                return ord(ch)
        return 255

    def _handle_key(self, key):
        """키 처리. 종료 요청이면 True 반환."""
        if key in (ord('q'), 27):
            return True
        elif key == ord('l'):
            self.sample_mode = "line"
        elif key == ord('f'):
            self.sample_mode = "floor"
        elif key == ord('p'):
            self.preview_on = not self.preview_on
        elif key == ord('m'):
            self.preview_mode = {"dark": "hsv", "hsv": "lab", "lab": "dark"}[self.preview_mode]
        elif key == ord('k'):
            self.sel -= 1; self._clamp_sel()
        elif key == ord('j'):
            self.sel += 1; self._clamp_sel()
        elif key == ord('d'):
            self._delete_selected()
        elif key == ord('c'):
            self._toggle_class_selected()
        elif key == ord('n'):
            self._edit_note_selected()
        elif key == ord('u'):
            self._undo_sample()
        elif key == ord('r'):
            self.samples.clear()
            self.sel = 0
            self.calib = None
            print("전체 초기화")
        elif key == ord('s'):
            self.save()
        return False

    def run(self):
        print(__doc__)
        # 터미널을 cbreak 모드로 → Enter 없이 키 1개씩 즉시 수신 (Ctrl+C는 그대로)
        if sys.stdin.isatty():
            self._orig_term = termios.tcgetattr(sys.stdin.fileno())
            tty.setcbreak(sys.stdin.fileno())
        try:
            while True:
                ret, frame = self.cap.read()
                if not ret:
                    continue
                self.last_frame = frame

                canvas = self._draw_overlay(frame)
                cv2.imshow(WINDOW, canvas)

                # waitKey는 GUI 이벤트 펌프(마우스 콜백/렌더링)에 필수.
                # 카메라 창 포커스 시 여기서, SSH 터미널에서는 stdin으로 키 수신.
                key = cv2.waitKey(30) & 0xFF
                if key == 255:
                    key = self._read_stdin_key()

                if self._handle_key(key):
                    break
        finally:
            if self._orig_term is not None:
                termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, self._orig_term)
            self.cap.release()
            cv2.destroyAllWindows()


def main():
    args = sys.argv[1:]
    scale = DEFAULT_SCALE
    if "--scale" in args:
        scale = float(args[args.index("--scale") + 1])
    Calibrator(scale=scale).run()


if __name__ == "__main__":
    main()
