"""비행 기록 — 동기 CSV(v14 형식 + 단조시계 열), 이벤트 표, 단계별 샘플, 요약 JSON.

시각 열
    mono_time  : CLOCK_MONOTONIC 초 (= VOXL MPA timestamp(ns) / 1e9, voxl-logger 의 data.csv 와 같은 시계)
    mono_ns    : 같은 값을 ns 정수로
    unix_time  : time.time()
    wallclock  : HH:MM:SS.mmm

이벤트는 단계 시작/끝, 정렬 완료(t0), 비상 등 "순간" 을 적는다. 후처리에서
`--hover/--move/--end` 창을 이 표에서 읽는다.
"""

import csv
import json
import os
import time

from .util import horizontal_norm, log, mono_ns, mono_time, wallclock


class FlightRecorder(object):
    def __init__(self, cfg, out_dir=".", prefix="flight"):
        self.cfg = cfg
        self.out_dir = out_dir
        self.prefix = prefix
        self.sync_samples = []
        self.stage_samples = {}
        self.events = []
        self.extra_tables = {}          # name -> list of dict (예: align_log)
        self.summary = {}
        self.t_created = mono_time()

    # -- 이벤트 -------------------------------------------------------------

    def event(self, name, detail="", **fields):
        row = {
            "mono_time": mono_time(),
            "mono_ns": mono_ns(),
            "unix_time": time.time(),
            "wallclock": wallclock(),
            "event": name,
            "detail": detail,
        }
        row.update(fields)
        self.events.append(row)
        log("[event] {} {}".format(name, detail))
        return row

    def add_row(self, table, row):
        self.extra_tables.setdefault(table, []).append(row)

    # -- 샘플 (Vehicle 이 부른다) --------------------------------------------

    def stage_sample(self, v):
        """진행 중인 단계의 샘플 (MAVSDK 위치 갱신마다)."""
        if v.active_stage is None:
            return
        fb = v.feedback_state()
        now = mono_time()
        t0 = v.active_stage["t_start"]
        mav_rel_n, mav_rel_e, mav_rel_u = v.mav_rel_position()
        target_n, target_e, target_u = v.rel_target()
        if fb is not None:
            h_dist = horizontal_norm(fb["cmd_n"] - v.target.north_m, fb["cmd_e"] - v.target.east_m)
            v_dist = abs(fb["cmd_d"] - v.target.down_m)
            fb_rel_n, fb_rel_e, fb_rel_u = fb["rel_n"], fb["rel_e"], fb["rel_u"]
            fb_vn, fb_ve, fb_age = fb["vn"], fb["ve"], fb["age_sec"]
        else:
            h_dist = v_dist = float("nan")
            fb_rel_n = fb_rel_e = fb_rel_u = float("nan")
            fb_vn = fb_ve = fb_age = float("nan")
        sample = {
            "stage": v.active_stage["name"], "phase": v.active_phase,
            "t": now - t0, "mono_time": now, "wallclock": wallclock(),
            "fb_rel_n": fb_rel_n, "fb_rel_e": fb_rel_e, "fb_rel_u": fb_rel_u,
            "fb_vn": fb_vn, "fb_ve": fb_ve, "fb_age": fb_age,
            "mav_rel_n": mav_rel_n, "mav_rel_e": mav_rel_e, "mav_rel_u": mav_rel_u,
            "mav_vn": v.mav_vn, "mav_ve": v.mav_ve,
            "target_n": target_n, "target_e": target_e, "target_u": target_u,
            "h_dist": h_dist, "v_dist": v_dist,
        }
        self.stage_samples.setdefault(v.active_stage["name"], []).append(sample)

    def sync_sample(self, v):
        """주기 동기 샘플 (v14 sync CSV 한 행 + mono_ns)."""
        now = mono_time()
        mav_rel_n, mav_rel_e, mav_rel_u = v.mav_rel_position()
        target_n, target_e, target_u = v.rel_target()
        fb = v.feedback_state()
        if fb is not None:
            fb_h = horizontal_norm(fb["cmd_n"] - v.target.north_m, fb["cmd_e"] - v.target.east_m)
            fb_v = abs(fb["cmd_d"] - v.target.down_m)
        else:
            fb_h = fb_v = ""

        row = {
            "unix_time": time.time(),
            "wallclock": wallclock(),
            "mono_time": now,
            "mono_ns": int(round(now * 1e9)),
            "stage": v.active_stage["name"] if v.active_stage else "NONE",
            "phase": v.active_phase,
            "target_rel_n": target_n, "target_rel_e": target_e, "target_rel_u": target_u,
            "target_yaw_deg": v.current_target_yaw,
            "reach_pipe": v.reach_pipe,
            "fb_cmd_n": fb["cmd_n"] if fb else "", "fb_cmd_e": fb["cmd_e"] if fb else "",
            "fb_cmd_d": fb["cmd_d"] if fb else "",
            "fb_yaw_deg": fb.get("yaw_deg", "") if fb else "",
            "fb_rel_n": fb["rel_n"] if fb else "", "fb_rel_e": fb["rel_e"] if fb else "",
            "fb_rel_u": fb["rel_u"] if fb else "",
            "fb_vn": fb["vn"] if fb else "", "fb_ve": fb["ve"] if fb else "", "fb_vd": fb["vd"] if fb else "",
            "fb_age_sec": fb["age_sec"] if fb else "",
            "fb_h_dist": fb_h, "fb_v_dist": fb_v,
            "mav_abs_n": v.mav_x, "mav_abs_e": v.mav_y, "mav_abs_d": v.mav_z,
            "mav_rel_n": mav_rel_n, "mav_rel_e": mav_rel_e, "mav_rel_u": mav_rel_u,
            "mav_vn": v.mav_vn, "mav_ve": v.mav_ve, "mav_vd": v.mav_vd,
            "mav_updates": v.mav_update_count,
        }
        row.update(v.extra_sync_fields())

        for pipe in v.cfg.voxl_pipes:
            prefix = "voxl_{}_".format(pipe)
            latest = v.voxl_latest.get(pipe)
            initial = v.voxl_initial.get(pipe)
            if latest is None:
                for key in ["age_sec", "ts_ms", "x", "y", "z", "rel_x", "rel_y", "rel_z",
                            "vx", "vy", "vz", "roll_deg", "pitch_deg", "yaw_deg", "raw"]:
                    row[prefix + key] = ""
                continue
            row[prefix + "age_sec"] = now - latest["mono_time"]
            row[prefix + "ts_ms"] = latest["voxl_ts_ms"]
            row[prefix + "x"] = latest["pos_x"]
            row[prefix + "y"] = latest["pos_y"]
            row[prefix + "z"] = latest["pos_z"]
            if initial:
                row[prefix + "rel_x"] = latest["pos_x"] - initial["pos_x"]
                row[prefix + "rel_y"] = latest["pos_y"] - initial["pos_y"]
                row[prefix + "rel_z"] = latest["pos_z"] - initial["pos_z"]
            else:
                row[prefix + "rel_x"] = row[prefix + "rel_y"] = row[prefix + "rel_z"] = ""
            row[prefix + "vx"] = latest["vel_x"]
            row[prefix + "vy"] = latest["vel_y"]
            row[prefix + "vz"] = latest["vel_z"]
            row[prefix + "roll_deg"] = latest["roll_deg"]
            row[prefix + "pitch_deg"] = latest["pitch_deg"]
            row[prefix + "yaw_deg"] = latest["yaw_deg"]
            row[prefix + "raw"] = latest["raw"]

        self.sync_samples.append(row)
        return row

    # -- 저장 ---------------------------------------------------------------

    def path(self, suffix):
        return os.path.join(self.out_dir, "{}_{}".format(self.prefix, suffix))

    @staticmethod
    def _write_rows(path, rows):
        if not rows:
            return False
        fieldnames = []
        seen = set()
        for r in rows:
            for k in r.keys():
                if k not in seen:
                    seen.add(k)
                    fieldnames.append(k)
        with open(path, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=fieldnames)
            w.writeheader()
            for r in rows:
                w.writerow(r)
        return True

    def write_all(self):
        """sync.csv, events.csv, stages.csv, <extra>.csv, summary.json 을 쓴다. 쓴 경로 목록 반환."""
        written = []
        try:
            os.makedirs(self.out_dir, exist_ok=True)
        except OSError as e:
            log("[rec] out_dir 생성 실패 {}: {}".format(self.out_dir, e))
        for suffix, rows in [("sync.csv", self.sync_samples), ("events.csv", self.events)]:
            p = self.path(suffix)
            try:
                if self._write_rows(p, rows):
                    written.append(p)
            except Exception as e:
                log("[rec] failed to write {}: {}".format(p, e))
        stage_rows = []
        for name, rows in self.stage_samples.items():
            stage_rows.extend(rows)
        p = self.path("stages.csv")
        try:
            if self._write_rows(p, stage_rows):
                written.append(p)
        except Exception as e:
            log("[rec] failed to write {}: {}".format(p, e))
        for name, rows in self.extra_tables.items():
            p = self.path(name + ".csv")
            try:
                if self._write_rows(p, rows):
                    written.append(p)
            except Exception as e:
                log("[rec] failed to write {}: {}".format(p, e))
        p = self.path("summary.json")
        try:
            with open(p, "w", encoding="utf-8") as f:
                json.dump(self.summary, f, ensure_ascii=False, indent=2, default=str)
            written.append(p)
        except Exception as e:
            log("[rec] failed to write {}: {}".format(p, e))
        for w in written:
            log("[rec] wrote {}".format(w))
        return written
