#!/usr/bin/env python3
"""Build the wall-keep comparison page (HTML + media folder) from wk_report.py outputs. 2026-10-29.
  python3 wk_page.py <page_dir> <faces_report_dir> <faces_final_list> [<maps_dir>]   (maps from wk_map.py)
Copies/re-encodes media into <page_dir>/media (run videos scaled to 948 px, crf 30, to keep the page small) and writes
<page_dir>/index.html with the numbers taken from summary.json (nothing typed by hand).
"""
# 2026-10-31 (codex review): LEGACY entry point, not part of the current pipeline (see scripts/ENTRYPOINTS.md). Known problem: its template wk_page_template.html exists only in v2_wallkeep/scripts, codex legacy-workspace-entrypoints
import os as _os, sys as _sys
if _os.environ.get("LEGACY_OK", "0") != "1":
    _sys.exit("!! legacy entry point %s: template missing in v3. Set LEGACY_OK=1 to run anyway (see scripts/ENTRYPOINTS.md)." % _os.path.basename(__file__))
import json, os, shutil, subprocess, sys, html

page, frep, flist = sys.argv[1:4]
maps = sys.argv[4] if len(sys.argv) > 4 else ""
arep = sys.argv[5] if len(sys.argv) > 5 else ""          # alignment-hold comparison report (wk_report run-list mode)
srep = sys.argv[6] if len(sys.argv) > 6 else ""          # sweep dir: sweep.json, sweep_summary.png, <kind>/ reports, runs.txt
media = os.path.join(page, "media"); os.makedirs(media, exist_ok=True)
ERR = {"yaw15": "VIO 헤딩 1.5°", "drift04": "드리프트 0.4 cm/s", "none": "오차 없음"}
FACE_NAME = lambda f: "%s통로 %s쪽 면" % (f[0], "서" if f[1] == "W" else "동")   # noqa: E731


def small(src, dst):
    if not os.path.exists(dst):
        subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-i", src, "-vf", "scale=948:-2", "-c:v", "libx264", "-crf", "30",
                        "-preset", "veryfast", "-pix_fmt", "yuv420p", "-movflags", "+faststart", dst], check=True)


def copy(src, dst):
    shutil.copy(src, dst)


faces = {}
summ = {m["dir"].rstrip("/"): m for m in json.load(open(os.path.join(frep, "summary.json")))}
for line in open(flist):
    f, c, w, d = line.split()
    if d == "MISSING":
        faces.setdefault(f, {})[(c, w)] = None; continue
    key = "%s_%s_%s" % (f, c, w)
    small(os.path.join(d, "run.mp4"), os.path.join(media, key + ".mp4"))
    m = summ.get(d.rstrip("/"), {})
    faces.setdefault(f, {})[(c, w)] = {"video": "media/%s.mp4" % key, "m": m}
FACES = sorted(faces, key=lambda f: ("ABCDE".index(f[0]), f[1]))
for f in FACES:
    copy(os.path.join(frep, "topdown_%s.mp4" % f), os.path.join(media, "topdown_%s.mp4" % f))
    copy(os.path.join(frep, "face_%s.png" % f), os.path.join(media, "face_%s.png" % f))
    copy(os.path.join(frep, "along_anim_%s.mp4" % f), os.path.join(media, "along_anim_%s.mp4" % f))
copy(os.path.join(frep, "faces_grid.png"), os.path.join(media, "faces_grid.png"))
for m in ("map_top.png", "map_side.png"):
    if maps:
        copy(os.path.join(maps, m), os.path.join(media, m))


ALIGN_ROWS = ALIGN_NOTE = ""
if arep:
    small(os.path.join(arep, "compare.mp4"), os.path.join(media, "align_compare.mp4"))
    copy(os.path.join(arep, "topdown.mp4"), os.path.join(media, "align_topdown.mp4"))
    copy(os.path.join(arep, "along_anim.mp4"), os.path.join(media, "align_along.mp4"))
    copy(os.path.join(arep, "compare_distance.png"), os.path.join(media, "align_distance.png"))


def cm(v):
    return "%.1f" % (v * 100)


def cell(e):
    if e is None:
        return '<td class="na">없음</td>'
    m = e["m"]
    if "err_max_abs" not in m:
        return '<td class="na">-</td>'
    front, back = m["rotor_clear_min_front"], m["rotor_clear_min_back"]
    worst = min(front, back)
    cls = "bad" if worst < 0 else ("warn" if worst < 0.05 else "ok")
    side = "앞" if front <= back else "뒤"
    legs = m.get("err_max_abs_legs")
    return '<td class="%s"><b>%s</b><span>이륙 직후 포함 %s</span><span>%s 여유 %s</span></td>' % (
        cls, cm(legs if legs is not None else m["err_max_abs"]), cm(m["err_max_abs"]), side, cm(worst))


rows = []
for f in FACES:
    r = faces[f]
    rows.append("<tr><th scope=\"row\"><span class=\"fc\">%s</span>%s</th>%s</tr>" % (
        f, FACE_NAME(f), "".join(cell(r.get(k)) for k in (("yaw15", "off"), ("yaw15", "on"), ("drift04", "off"), ("drift04", "on")))))
ons = [e["m"]["err_max_abs"] for f in FACES for k, e in faces[f].items() if e and k[0] != "none" and k[1] == "on" and "err_max_abs" in e["m"]]
ons_legs = [e["m"]["err_max_abs_legs"] for f in FACES for k, e in faces[f].items() if e and k[0] != "none" and k[1] == "on" and e["m"].get("err_max_abs_legs") is not None]
offs = [e["m"]["err_max_abs"] for f in FACES for k, e in faces[f].items() if e and k[0] != "none" and k[1] == "off" and "err_max_abs" in e["m"]]
on_pass = sum(1 for f in FACES for k, e in faces[f].items() if e and k[0] != "none" and k[1] == "on" and e["m"].get("grid_verify") == "PASS")
off_fail = sum(1 for f in FACES for k, e in faces[f].items() if e and k[0] != "none" and k[1] == "off" and e["m"].get("grid_verify") == "FAIL")
n_on = sum(1 for f in FACES for k, e in faces[f].items() if e and k[0] != "none" and k[1] == "on")
n_off = sum(1 for f in FACES for k, e in faces[f].items() if e and k[0] != "none" and k[1] == "off")
data = {f: {"%s_%s" % k: (e["video"] if e else None) for k, e in faces[f].items()} for f in FACES}
info = {f: {"%s_%s" % k: ({"err": cm(e["m"]["err_max_abs_legs"] if e["m"].get("err_max_abs_legs") is not None else e["m"]["err_max_abs"]), "errall": cm(e["m"]["err_max_abs"]), "front": cm(e["m"]["rotor_clear_min_front"]),
                          "back": cm(e["m"]["rotor_clear_min_back"]), "v": e["m"].get("grid_verify", "?"),
                          "cy": e["m"].get("final_c", {}).get("cY"), "ce": e["m"].get("final_c", {}).get("cE")}
                         if e and "err_max_abs" in e["m"] else None) for k, e in faces[f].items()} for f in FACES}
none = faces.get("CW", {})
def _nn(w):
    e = none.get(("none", w))
    return "%s %s cm (%s)" % ("보정 끔" if w == "off" else "보정 켬", cm(e["m"]["err_max_abs"]), e["m"].get("grid_verify")) if e and "err_max_abs" in e["m"] else ""
NONE_NOTE = ("C통로 서쪽 면(CW)은 오차 없음 조건도 비행했습니다: 최대 오차 " + _nn("off") + ", " + _nn("on") + ". 아래 재생에서 CW를 고르고 '오차 없음'을 누르면 볼 수 있습니다.") if none.get(("none", "on")) else ""

if arep:
    am = json.load(open(os.path.join(arep, "summary.json")))
    ALIGN_ROWS = "".join("<tr><th scope=\"row\">%s</th><td>%s</td><td>%s</td><td>%s</td><td>%s</td><td>%s</td><td>%s</td></tr>" % (
        html.escape(m["label"]), cm(m["err_max_abs_legs"]), cm(m["err_max_abs"]), cm(m["err_leg1_start"]), cm(m["err_leg1_first5s_max"]),
        "%.2f" % m["align"]["seconds"] if m.get("align") else "-", "%.1f" % m["flight_s"]) for m in am)
    secs = [m["align"]["seconds"] for m in am if m.get("align")]
    pre = [(a_, b_) for a_, b_ in zip(am[0::2], am[1::2])]
    dr = [p_ for p_ in pre if "드리프트" in p_[0]["label"]]
    ALIGN_NOTE = ("정렬 대기는 3회 모두 %.2f~%.2f s 만에 수렴(최소 유지 시간 1 s)했습니다. 회전 단계 끝에서 기수가 맞춰지는 동안 이미 보정이 작동해, 레그 ① 시작 무렵에는 오차가 거의 남아 있지 않았기 때문입니다. " % (min(secs), max(secs)) if secs else "") + \
        ("드리프트 조건에서 레그 ① 첫 5 s 최대 오차는 %s → %s cm로 줄었지만, 이륙·회전 중에 생기는 최대 오차(대기 없음 %s, 대기 있음 %s cm)는 대기 단계 이전에 생기므로 대기로는 없앨 수 없습니다(두 값의 차이는 비행마다의 편차). 대가는 비행 시간 약 1~2 s 증가입니다." % (
            cm(dr[0][0]["err_leg1_first5s_max"]), cm(dr[0][1]["err_leg1_first5s_max"]), cm(dr[0][0]["err_max_abs"]), cm(dr[0][1]["err_max_abs"])) if dr else "")

SWEEP = {"kinds": []}; SWEEP_ROWS = SWEEP_NOTE = ROT_ROWS = ""
if srep:
    copy(os.path.join(srep, "sweep_summary.png"), os.path.join(media, "sweep_summary.png"))
    sw = json.load(open(os.path.join(srep, "sweep.json")))
    KN = {"yaw": ("VIO 헤딩", "°"), "drift": ("드리프트", "cm/s"), "step": ("계단 오차", "cm")}
    for kind in ("yaw", "drift", "step"):
        kdir = os.path.join(srep, kind)
        if not os.path.isdir(kdir):
            continue
        copy(os.path.join(kdir, "topdown.mp4"), os.path.join(media, "sweep_%s_topdown.mp4" % kind))
        copy(os.path.join(kdir, "along_anim.mp4"), os.path.join(media, "sweep_%s_along.mp4" % kind))
        mags = {}
        for r in sw:
            if r["kind"] != kind or r["mag"] == 0:
                continue
            key = "sweep_%s_%g_%s" % (kind, r["mag"], r["wk"])
            src = os.path.join(r["dir"], "run_sweep.mp4")
            if os.path.exists(src):
                small(src, os.path.join(media, key + ".mp4"))
                mags.setdefault(r["mag"], {})[r["wk"]] = "media/%s.mp4" % key
        SWEEP["kinds"].append({"key": kind, "name": KN[kind][0], "unit": KN[kind][1], "topdown": "media/sweep_%s_topdown.mp4" % kind,
                               "along": "media/sweep_%s_along.mp4" % kind,
                               "mags": [{"mag": m, "off": v.get("off"), "on": v.get("on")} for m, v in sorted(mags.items())]})
    def cellw(r):
        if r is None:
            return "-"
        return "%s%s / %.1f / %s" % ("" if r["wk"] == "off" else ("%.1f / " % ((r["err_legs"] or r["err_all"]) * 100)), "%.1f" % (r["err_all"] * 100), r["clear"] * 100, r["verdict"])
    for kind in ("yaw", "drift", "step"):
        for mag in sorted({r["mag"] for r in sw if r["kind"] == kind}):
            off = next((r for r in sw if r["kind"] == kind and r["mag"] == mag and r["wk"] == "off"), None)
            on = next((r for r in sw if r["kind"] == kind and r["mag"] == mag and r["wk"] == "on"), None)
            SWEEP_ROWS += "<tr><th scope=\"row\">%s</th><td>%g %s</td><td>%s</td><td>%s</td></tr>" % (KN[kind][0], mag, KN[kind][1], cellw(off), cellw(on))
    ROT_ROWS = ""
    rdir = os.path.join(srep, "rot")
    if os.path.isdir(rdir):
        small(os.path.join(rdir, "compare.mp4"), os.path.join(media, "rot_compare.mp4"))
        copy(os.path.join(rdir, "topdown.mp4"), os.path.join(media, "rot_topdown.mp4"))
        for m in json.load(open(os.path.join(rdir, "summary.json"))):
            ROT_ROWS += "<tr><th scope=\"row\">%s</th><td>%s</td><td>%s</td><td>%s</td></tr>" % (html.escape(m["label"]), cm(m["err_max_abs_legs"]),
                        cm(min(m["rotor_clear_min_front"], m["rotor_clear_min_back"])), m["grid_verify"])
    note = os.path.join(srep, "note.txt")
    SWEEP_NOTE = open(note).read().strip() if os.path.exists(note) else ""

tpl = open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "wk_page_template.html")).read()
out = (tpl.replace("{{ROWS}}", "\n".join(rows))
          .replace("{{N_ON}}", str(n_on)).replace("{{ON_PASS}}", str(on_pass))
          .replace("{{N_OFF}}", str(n_off)).replace("{{OFF_FAIL}}", str(off_fail))
          .replace("{{ON_MAX}}", cm(max(ons)) if ons else "-").replace("{{ON_MAX_LEGS}}", cm(max(ons_legs)) if ons_legs else "-").replace("{{OFF_MIN}}", cm(min(offs)) if offs else "-")
          .replace("{{OFF_MAX}}", cm(max(offs)) if offs else "-")
          .replace("{{DATA}}", json.dumps(data)).replace("{{INFO}}", json.dumps(info, ensure_ascii=False))
          .replace("{{FACES}}", json.dumps(FACES)).replace("{{FACENAMES}}", json.dumps({f: FACE_NAME(f) for f in FACES}, ensure_ascii=False))
          .replace("{{NONE_NOTE}}", NONE_NOTE).replace("{{ALIGN_ROWS}}", ALIGN_ROWS).replace("{{ALIGN_NOTE}}", ALIGN_NOTE)
          .replace("{{SWEEP}}", json.dumps(SWEEP, ensure_ascii=False)).replace("{{SWEEP_ROWS}}", SWEEP_ROWS).replace("{{SWEEP_NOTE}}", SWEEP_NOTE).replace("{{ROT_ROWS}}", ROT_ROWS))
open(os.path.join(page, "index.html"), "w").write(out)
print("page", page, "faces", len(FACES), "on", n_on, on_pass, "off", n_off, off_fail)
