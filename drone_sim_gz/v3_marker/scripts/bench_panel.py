#!/usr/bin/env python3
"""Workbench control panel: a small always-on-top window with real buttons, placed over the right side of the workbench's
Gazebo window on the VNC desktop (run on the HOST; bench.sh starts it):  DISPLAY=:1 python3 bench_panel.py
Buttons send the same commands as the hotkeys / bench.sh cmd (topic /bench/cmd); the status line shows the controller's
last status (logs/bench_ctl.log). The Gazebo play/pause button and hotkeys keep working alongside.
Why a separate window: the stock Gazebo GUI panels (Publisher, TopicEcho) only offer a generic "message type / topic /
message" form and ignore their configured topic, which is no usable control panel (user feedback 2026-10-28).
"""
import os, re, subprocess, sys, threading, time
import tkinter as tk
from tkinter import ttk

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
N = int(os.environ.get("BENCH_INSTANCE", "1"))
PART = "starling_%d" % N
LOGF = os.path.expanduser("~/starling_sim/logs/bench_ctl.log")
GEOM = os.environ.get("BENCH_PANEL_GEOMETRY", "300x930+1606+100")
try:
    import bench_ctl                                       # the preset list lives in the controller
    PRESETS = bench_ctl.PRESETS
except Exception:
    PRESETS = [("A통로 동쪽 면", "survey R1A E")]

FONT = ("Noto Sans CJK KR", 11); FONT_B = ("Noto Sans CJK KR", 12, "bold"); FONT_S = ("Noto Sans CJK KR", 10)


def send(cmd):
    def go():
        subprocess.run(["docker", "exec", "-e", "GZ_PARTITION=" + PART, "starling-gz", "gz", "topic", "-t", "/bench/cmd",
                        "-m", "gz.msgs.StringMsg", "-p", 'data: "%s"' % cmd], capture_output=True, timeout=15)
    threading.Thread(target=go, daemon=True).start()
    status_var.set("보냄: %s" % cmd)


def last_status():
    try:
        with open(LOGF, "rb") as f:
            f.seek(0, 2); size = f.tell(); f.seek(max(0, size - 6000))
            lines = f.read().decode(errors="ignore").splitlines()
        for line in reversed(lines):
            m = re.match(r"\[(\d\d:\d\d:\d\d)\] STATUS (.*)", line)
            if m:
                return "%s  %s" % (m.group(1), m.group(2))
    except OSError:
        pass
    return "상태 없음 (제어기가 실행 중인지 확인: bench.sh status)"


root = tk.Tk()
root.title("작업대 조종")
root.geometry(GEOM)
root.attributes("-topmost", True)
root.configure(bg="#f4f6f3")
style = ttk.Style(); style.theme_use("clam")
style.configure("TButton", font=FONT, padding=6)
style.configure("Big.TButton", font=FONT_B, padding=8)

frm = tk.Frame(root, bg="#f4f6f3", padx=12, pady=10); frm.pack(fill="both", expand=True)
tk.Label(frm, text="작업대 조종", font=("Noto Sans CJK KR", 14, "bold"), bg="#f4f6f3").pack(anchor="w")
tk.Label(frm, text="인스턴스 %d · 단축키: 3D 화면 클릭 후 1-9 R X L P" % N, font=FONT_S, fg="#5e6a62", bg="#f4f6f3").pack(anchor="w", pady=(0, 8))

tk.Label(frm, text="경로", font=FONT_B, bg="#f4f6f3").pack(anchor="w")
route_var = tk.StringVar(value="1. " + PRESETS[0][0])
combo = ttk.Combobox(frm, textvariable=route_var, state="readonly", font=FONT,
                     values=["%d. %s" % (i + 1, p[0]) for i, p in enumerate(PRESETS)])
combo.pack(fill="x", pady=(2, 8))


def sel():
    return route_var.get().split(".")[0]


ttk.Button(frm, text="▶  비행 시작", style="Big.TButton", command=lambda: send("run " + sel())).pack(fill="x", pady=3)
row = tk.Frame(frm, bg="#f4f6f3"); row.pack(fill="x", pady=3)
ttk.Button(row, text="❚❚ 일시정지", command=lambda: send("pause")).pack(side="left", expand=True, fill="x", padx=(0, 3))
ttk.Button(row, text="▶ 재개", command=lambda: send("resume")).pack(side="left", expand=True, fill="x", padx=(3, 0))
row = tk.Frame(frm, bg="#f4f6f3"); row.pack(fill="x", pady=3)
ttk.Button(row, text="↺ 리셋", command=lambda: send("reset")).pack(side="left", expand=True, fill="x", padx=(0, 3))
ttk.Button(row, text="■ 중지", command=lambda: send("stop")).pack(side="left", expand=True, fill="x", padx=(3, 0))

tk.Label(frm, text="반복", font=FONT_B, bg="#f4f6f3").pack(anchor="w", pady=(12, 2))
row = tk.Frame(frm, bg="#f4f6f3"); row.pack(fill="x", pady=3)
rep_var = tk.IntVar(value=3)
tk.Spinbox(row, from_=2, to=100, textvariable=rep_var, width=5, font=FONT).pack(side="left")
ttk.Button(row, text="회 반복 비행", command=lambda: (send("select " + sel()), root.after(1500, lambda: send("repeat %d" % rep_var.get())))).pack(side="left", expand=True, fill="x", padx=(6, 0))
row = tk.Frame(frm, bg="#f4f6f3"); row.pack(fill="x", pady=3)
ttk.Button(row, text="계속 반복 켬", command=lambda: (send("select " + sel()), root.after(1500, lambda: send("loop on")))).pack(side="left", expand=True, fill="x", padx=(0, 3))
ttk.Button(row, text="반복 끔", command=lambda: send("loop off")).pack(side="left", expand=True, fill="x", padx=(3, 0))

tk.Label(frm, text="상태", font=FONT_B, bg="#f4f6f3").pack(anchor="w", pady=(14, 2))
status_var = tk.StringVar(value="")
tk.Label(frm, textvariable=status_var, font=FONT_S, fg="#8a4b12", bg="#f4f6f3", wraplength=270, justify="left").pack(anchor="w")
ctl_var = tk.StringVar(value=last_status())
tk.Label(frm, textvariable=ctl_var, font=FONT, bg="#ffffff", relief="groove", wraplength=270, justify="left",
         padx=8, pady=8, anchor="nw").pack(fill="x", pady=(4, 0))
tk.Label(frm, text="판정 리포트: logs/bench_<시각>_report.txt", font=FONT_S, fg="#5e6a62", bg="#f4f6f3").pack(anchor="w", pady=(8, 0))


def tick():
    ctl_var.set(last_status())
    root.after(1000, tick)


tick()
root.mainloop()
