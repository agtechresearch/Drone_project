#!/usr/bin/env python3
"""Collect every video / figure of a wall-keep comparison into one results folder with a README index (2026-10-29).
  python3 wk_export.py <out_dir> <faces_report_dir> <faces_list> [<set_report_dir> <set_name> <set_runs_list>]
Layout:
  <out>/README.md                         index (what each file is, numbers from summary.json)
  <out>/summary/                          summary.md, summary.json, faces_grid.png
  <out>/faces/<F>/                        face_<F>.png (static, all 4 runs), topdown_<F>.mp4, along_anim_<F>.mp4 (synced)
  <out>/faces/<F>/<cond>_<off|on>/        run.mp4 (composite), chase.mp4, cam.mp4, tof.mp4, strip.png, strip_anim.mp4,
                                          metrics.json, condition.txt, source.txt (original run dir)
  <out>/<set_name>/                       extra run set (e.g. C aisle with no error): topdown.mp4, along_anim.mp4,
                                          compare_distance.png, summary.md + per run folders
Files are hard-linked when possible (same disk), else copied.
"""
import json, os, shutil, sys

ERR = {"yaw15": "VIO 헤딩 1.5°", "drift04": "드리프트 0.4 cm/s", "none": "오차 없음"}
FACE_NAME = lambda f: "%s통로 %s쪽 면" % (f[0], "서" if f[1] == "W" else "동")   # noqa: E731
RUN_FILES = ["run.mp4", "chase.mp4", "cam.mp4", "tof.mp4", "strip.png", "strip_anim.mp4", "metrics.json", "condition.txt"]


def put(src, dst):
    if not os.path.exists(src):
        return False
    os.makedirs(os.path.dirname(dst), exist_ok=True)
    if os.path.exists(dst):
        os.remove(dst)
    try:
        os.link(src, dst)
    except OSError:
        shutil.copy2(src, dst)
    return True


def run_folder(src, dst, sfx=""):
    """sfx: a second report's own files (run_cw6.mp4, strip_cw6.png, ...) are exported under the plain names"""
    got = []
    for f in RUN_FILES:
        base, ext = os.path.splitext(f)
        alt = os.path.join(src, base + sfx + ext)
        if put(alt if sfx and os.path.exists(alt) else os.path.join(src, f), os.path.join(dst, f)):
            got.append(f)
    open(os.path.join(dst, "source.txt"), "w").write(src + "\n")
    return got


def main():
    out, frep, flist = sys.argv[1:4]
    os.makedirs(out, exist_ok=True)
    summ = {m["dir"].rstrip("/"): m for m in json.load(open(os.path.join(frep, "summary.json")))}
    for f in ("summary.md", "summary.json", "faces_grid.png"):
        put(os.path.join(frep, f), os.path.join(out, "summary", f))
    faces = {}
    for line in open(flist):
        p = line.split()
        if len(p) < 4 or p[3] == "MISSING":
            continue
        f, c, w, d = p
        faces.setdefault(f, []).append((c, w, d))
        run_folder(d, os.path.join(out, "faces", f, "%s_%s" % (c, w)))
    for f in faces:
        for name in ("face_%s.png" % f, "topdown_%s.mp4" % f, "along_anim_%s.mp4" % f):
            put(os.path.join(frep, name), os.path.join(out, "faces", f, name))
    lines = ["# Wall-keep 비교 결과 — %s" % os.path.basename(out.rstrip("/")), "",
             "재배단 거리 유지(wall-keep) 시험 비행의 영상·그래프 모음. 모든 영상은 15 fps, 이륙 시각 기준으로 같은 길이라 나란히 재생하면 동기화된다.",
             "", "## 폴더", "",
             "| 위치 | 내용 |", "|---|---|",
             "| `summary/` | 수치표(summary.md/json), 8면 요약 그림 faces_grid.png |",
             "| `faces/<면>/face_<면>.png` | 통로를 따라 본 거리(진실값), 그 면의 4개 비행, ● 출발점, 화살표 = 진행 방향 |",
             "| `faces/<면>/along_anim_<면>.mp4` | 위 그래프가 시간에 따라 그려지는 영상 |",
             "| `faces/<면>/topdown_<면>.mp4` | 위에서 본 궤적, 4개 비행 동기화(통로 폭 방향 확대) |",
             "| `faces/<면>/<오차>_<off/on>/run.mp4` | 합성: 전지적 시점 + 정면 카메라 + ToF + 시간 그래프 |",
             "| `…/chase.mp4`, `cam.mp4`, `tof.mp4` | 전지적 시점, 정면 카메라, ToF 깊이(맞춘 평면) 원본 영상 |",
             "| `…/strip.png`, `strip_anim.mp4` | 시간 그래프(레그 구간·출발점), 커서가 움직이는 영상 |",
             "| `…/metrics.json`, `source.txt` | 수치, 원본 로그 폴더 |", "",
             "## 면별 결과 (통로 중앙 대비 최대 오차 cm: 레그 중 / 이륙 직후 포함, 판정)", "",
             "| 면 | 조건 | 보정 끔 | 보정 켬 |", "|---|---|---|---|"]
    order = sorted(faces, key=lambda f: ("ABCDE".index(f[0]), f[1]))
    for f in order:
        for c in sorted({c for c, _, _ in faces[f]}):
            cells = []
            for w in ("off", "on"):
                d = next((d for cc, ww, d in faces[f] if cc == c and ww == w), None)
                m = summ.get(d.rstrip("/")) if d else None
                if not m or "err_max_abs" not in m:
                    cells.append("-"); continue
                legs = m.get("err_max_abs_legs")
                cells.append("%.1f / %.1f %s" % ((legs if legs is not None else m["err_max_abs"]) * 100, m["err_max_abs"] * 100, m["grid_verify"]))
            lines.append("| %s %s | %s | %s | %s |" % (f, FACE_NAME(f), ERR.get(c, c), cells[0], cells[1]))
    if len(sys.argv) > 6:
        srep, sname, slist = sys.argv[4:7]
        sdir = os.path.join(out, sname)
        for name in ("topdown.mp4", "along_anim.mp4", "compare_distance.png", "summary.md", "summary.json", "compare.mp4"):
            put(os.path.join(srep, name), os.path.join(sdir, name))
        for k, d in enumerate(l.strip() for l in open(slist) if l.strip()):
            run_folder(d, os.path.join(sdir, "%02d_%s" % (k + 1, os.path.basename(d).split("_", 3)[-1])), os.environ.get("WK_SET_SUFFIX", ""))
        lines += ["", "## `%s/`" % sname, "", "추가 묶음: 같은 통로의 조건별 비행, topdown.mp4 / along_anim.mp4 / compare_distance.png 와 비행별 폴더."]
    lines += ["", "비교 페이지(같은 영상, 끔/켬 동기 재생): https://claude.ai/artifact/6UJKuqb1aimaHc139DukYs", ""]
    open(os.path.join(out, "README.md"), "w").write("\n".join(lines))
    n = sum(len(fs) for _, _, fs in os.walk(out))
    print("exported %s (%d files)" % (out, n))


if __name__ == "__main__":
    main()
