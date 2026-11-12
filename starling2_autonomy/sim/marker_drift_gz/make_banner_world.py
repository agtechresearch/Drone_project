#!/usr/bin/env python3
"""드리프트 실험 현수막 월드 생성기 (gz Harmonic, drone_sim_gz 용). 호스트(cad venv, PIL)에서 실행.

    python3 make_banner_world.py [--out ~/starling_sim/marker_drift/gz] [--size 0.20] [--standoff 0.70]
                                 [--ids 0-9] [--spacing 0.5] [--z 0.60] [--upper] [--base-world ...] [--tags-dir ...]

만드는 것 (모두 --out 아래)
    models/drift_banner/model.sdf, model.config, materials/textures/tag_NN.png
        흰 현수막 판(길이 = 태그 구간 + 0.5 m, 높이 --banner-h) 과 그 앞 1 mm 에 tag36h11 판(검은 사각형 한 변 --size,
        인쇄 크기 = 10/8 배) 을 --spacing 간격으로 둔다. 현수막 평면 x = -standoff, 법선 +x (기체 쪽).
    worlds/drift_banner.sdf
        drone_sim_gz/gz/worlds/qvix.sdf 의 QVIX 모듈 include 를 drift_banner include 로 바꾼 것 (물리·플러그인·조명 동일).
    banner_layout.json
        태그 id → 월드 좌표(x, y, z), 현수막 평면, 권장 스폰 포즈. 진실값 비교용.

좌표 약속 (docs/09 §5.2 와 같은 배치를 gz 월드에 옮김)
    기체 스폰 (0, 0, 0.035). 현수막은 -x 에 있다. v0.2.4 부터 run_sim_drift.sh 기본 스폰은 서향(yaw 3.1416) = 현수막 정면·회전 없음
    (종전: 북향 1.5708 + 좌회전 90°).
    비행: 이륙 → 좌회전 90° (서향, 현수막 정면) → 정렬 → "우측" 이동 = +y 로 --distance m. 태그 k 는 y = k * spacing 에 있어
    태그 0 이 시작점 정면, 태그 9 가 4.5 m 지점 정면이다 (2026-11-10 실험장: 현수막이 짧아 0~9).

설치: 생성 뒤 starling-gz 안에서 install_banner_world.sh (PX4 Tools/simulation/gz/{models,worlds} 로 복사).
"""

import argparse
import io
import json
import os
import re
import shutil
import sys

HOME = os.path.expanduser("~")
DEF_BASE_WORLD = os.path.join(HOME, "starling_sim", "repo_export", "gz", "worlds", "qvix.sdf")
DEF_TAGS_DIR = os.path.join(HOME, "starling_sim", "v3_marker", "gz", "tags")                       # tag36_11_00000.png (10x10)
DEF_TEX_DIR = os.path.join(HOME, "starling_sim", "v3_marker", "gz", "models", "qvix_tags", "materials", "textures")  # tag_00.png (400x400)
DEF_OUT = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "gz"))


def parse_ids(text):
    ids = []
    for part in text.split(","):
        part = part.strip()
        if not part:
            continue
        if "-" in part:
            a, b = part.split("-", 1)
            ids.extend(range(int(a), int(b) + 1))
        else:
            ids.append(int(part))
    return ids


def make_texture(tag_id, tags_dir, tex_dir, dst):
    """10x10 원본을 NEAREST 로 400x400 확대(PIL). PIL 이 없으면 미리 확대된 qvix_tags 텍스처를 복사."""
    src_raw = os.path.join(tags_dir, "tag36_11_{:05d}.png".format(tag_id))
    src_tex = os.path.join(tex_dir, "tag_{:02d}.png".format(tag_id))
    try:
        from PIL import Image
    except ImportError:
        Image = None
    if Image is not None and os.path.isfile(src_raw):
        im = Image.open(src_raw).convert("RGB").resize((400, 400), Image.NEAREST)
        im.save(dst)
        return "pil:" + src_raw
    if os.path.isfile(src_tex):
        shutil.copyfile(src_tex, dst)
        return "copy:" + src_tex
    raise SystemExit("태그 {} 이미지가 없다: {} 도 {} 도 없음 (--tags-dir / --textures-dir)".format(tag_id, src_raw, src_tex))


def build(args):
    ids = parse_ids(args.ids)
    if len(ids) < 2:
        raise SystemExit("태그가 둘 이상이어야 한다: --ids {}".format(args.ids))
    rows = [(ids, args.z)]
    if args.upper:
        rows.append((parse_ids(args.upper_ids), args.upper_z))

    name = args.name
    mdir = os.path.join(args.out, "models", name)
    tdir = os.path.join(mdir, "materials", "textures")
    if os.path.isdir(mdir):
        shutil.rmtree(mdir)
    os.makedirs(tdir)
    os.makedirs(os.path.join(args.out, "worlds"), exist_ok=True)

    print_size = args.size * 10.0 / 8.0          # tag36h11: 8x8 코드 + 1셀 흰 테두리
    x_plane = -args.standoff                      # 태그 면
    tag_cx = x_plane - 0.001                      # 두께 2 mm 상자 중심 → 앞면이 x_plane
    y_min = min(k * args.spacing for k in range(len(ids)))
    y_max = max(k * args.spacing for k in range(len(ids)))
    banner_len = (y_max - y_min) + args.banner_margin * 2.0
    banner_cy = (y_min + y_max) / 2.0
    banner_cx = x_plane - 0.001 - 0.002 - 0.001   # 태그 상자 뒤에 4 mm 판

    layout = {"family": "tag36h11", "size_m": args.size, "print_size_m": round(print_size, 4),
              "standoff_m": args.standoff, "banner_plane_x": x_plane, "normal": [1.0, 0.0, 0.0],
              "spacing_m": args.spacing, "frame": "gz world (x east, y north, z up); 기체 스폰 (0,0) 북향",
              "spawn_pose": "0,0,0.035,0,0,1.5708", "flight": "takeoff -> turn -90 (face -x) -> right = +y",
              "tags": {}}
    links = ""
    for ri, (row_ids, z) in enumerate(rows):
        for k, tid in enumerate(row_ids):
            y = k * args.spacing
            fn = "tag_{:02d}.png".format(tid)
            src = make_texture(tid, args.tags_dir, args.textures_dir, os.path.join(tdir, fn))
            key = str(tid) if ri == 0 else "{}_r{}".format(tid, ri)      # 상단열이 같은 ID 를 쓰면 키 충돌 방지
            layout["tags"][key] = {"id": tid, "row": ri, "x": round(x_plane, 4), "y": round(y, 4), "z": round(z, 4),
                                   "size": args.size, "texture": src}
            links += """    <link name="tag_{tid:02d}_r{ri}">
      <pose>{cx:.4f} {y:.4f} {z:.4f} 0 0 0</pose>
      <visual name="v">
        <geometry><box><size>0.002 {p:.4f} {p:.4f}</size></box></geometry>
        <material><diffuse>1 1 1 1</diffuse><ambient>1 1 1 1</ambient><emissive>0 0 0 1</emissive>
          <pbr><metal><albedo_map>model://{name}/materials/textures/{fn}</albedo_map><metalness>0</metalness><roughness>1</roughness></metal></pbr>
        </material>
        <cast_shadows>false</cast_shadows>
      </visual>
    </link>
""".format(tid=tid, ri=ri, cx=tag_cx, y=y, z=z, p=print_size, name=name, fn=fn)

    banner_rows = ""
    for row_ids, z in rows:
        banner_rows += """    <link name="banner_z{zt}">
      <pose>{cx:.4f} {cy:.4f} {z:.4f} 0 0 0</pose>
      <visual name="v">
        <geometry><box><size>0.004 {L:.4f} {H:.4f}</size></box></geometry>
        <material><diffuse>0.97 0.97 0.95 1</diffuse><ambient>0.9 0.9 0.9 1</ambient><specular>0.05 0.05 0.05 1</specular></material>
        <cast_shadows>false</cast_shadows>
      </visual>
      <collision name="c"><geometry><box><size>0.004 {L:.4f} {H:.4f}</size></box></geometry></collision>
    </link>
""".format(zt=int(round(z * 100)), cx=banner_cx, cy=banner_cy, z=z, L=banner_len, H=args.banner_h)

    with io.open(os.path.join(mdir, "model.sdf"), "w", encoding="utf-8") as f:
        f.write("""<?xml version="1.0"?>
<!-- Generated by starling2_autonomy/sim/marker_drift_gz/make_banner_world.py: 드리프트 실험 현수막 (tag36h11 {n} tags, {s:.2f} m, standoff {d:.2f} m) -->
<sdf version="1.9"><model name="{name}"><static>true</static>
{banner}{links}</model></sdf>
""".format(n=sum(len(r[0]) for r in rows), s=args.size, d=args.standoff, name=name, banner=banner_rows, links=links))
    with io.open(os.path.join(mdir, "model.config"), "w", encoding="utf-8") as f:
        f.write("""<?xml version="1.0"?><model><name>{name}</name><version>1.0</version>
<sdf version="1.9">model.sdf</sdf><description>Marker drift experiment banner (starling2_autonomy docs/09)</description></model>
""".format(name=name))

    # world: 기본 월드의 QVIX include → 현수막 include
    if not os.path.isfile(args.base_world):
        raise SystemExit("기본 월드가 없다: {} (--base-world)".format(args.base_world))
    src = io.open(args.base_world, encoding="utf-8").read()
    src, n_name = re.subn(r'<world name="[^"]+">', '<world name="{}">'.format(name), src, count=1)
    inc = re.compile(r"([ \t]*<!--[^\n]*-->\n)?[ \t]*<include>\s*<uri>model://qvix_r_module</uri>.*?</include>\n", re.S)
    new_inc = """    <!-- 드리프트 실험 현수막: 태그 면 x = {x:.3f} (기체 스폰 (0,0) 에서 standoff {d:.2f} m 왼쪽), 태그 y = 0 .. {ymax:.1f} -->
    <include>
      <uri>model://{name}</uri>
      <name>{name}</name>
      <pose>0 0 0 0 0 0</pose>
    </include>
""".format(x=x_plane, d=args.standoff, ymax=y_max, name=name)
    src, n_inc = inc.subn(new_inc, src, count=1)
    if n_inc != 1:
        raise SystemExit("기본 월드에서 qvix_r_module include 를 찾지 못했다: {}".format(args.base_world))
    world_path = os.path.join(args.out, "worlds", name + ".sdf")
    io.open(world_path, "w", encoding="utf-8").write(src)

    layout["world"] = world_path
    layout["model_dir"] = mdir
    with io.open(os.path.join(args.out, "banner_layout.json"), "w", encoding="utf-8") as f:
        json.dump(layout, f, indent=1, ensure_ascii=False)

    print("model : {}".format(mdir))
    print("world : {} (world name {}, renamed={})".format(world_path, name, n_name))
    print("layout: {}".format(os.path.join(args.out, "banner_layout.json")))
    print("tags  : {} rows, ids {}{}; size {:.2f} m (print {:.3f}), spacing {:.2f} m, y 0..{:.1f}".format(
        len(rows), ids[0], "-{}".format(ids[-1]) if len(ids) > 1 else "", args.size, print_size, args.spacing, y_max))
    print("banner: plane x = {:+.3f} m (standoff {:.2f}), {:.2f} m long, {:.2f} m high, centre z {}".format(
        x_plane, args.standoff, banner_len, args.banner_h, ", ".join("{:.2f}".format(r[1]) for r in rows)))
    print("spawn : {}  (layout spawn pose; run_sim_drift.sh default now faces W = the banner, no turn)".format(layout["spawn_pose"]))


def main():
    p = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    p.add_argument("--out", default=DEF_OUT, help="출력 폴더 (기본 {})".format(DEF_OUT))
    p.add_argument("--name", default="drift_banner", help="모델·월드 이름")
    p.add_argument("--base-world", default=DEF_BASE_WORLD, help="물리·플러그인을 가져올 기본 월드 (qvix.sdf)")
    p.add_argument("--tags-dir", default=DEF_TAGS_DIR, help="tag36_11_NNNNN.png 원본 폴더 (PIL 로 400x400 NEAREST 확대)")
    p.add_argument("--textures-dir", default=DEF_TEX_DIR, help="PIL 이 없을 때 복사할 확대본 tag_NN.png 폴더")
    p.add_argument("--size", type=float, default=0.20, help="태그 검은 사각형 한 변 m (2026-11-09 확정 0.20)")
    p.add_argument("--standoff", type=float, default=0.70, help="현수막 평면과 스폰점 거리 m (확정 0.70)")
    p.add_argument("--ids", default="0-9", help="하단열 태그 id (예: 0-9 또는 0,1,2). 2026-11-10 실험장 0~9")
    p.add_argument("--spacing", type=float, default=0.5, help="태그 간격 m")
    p.add_argument("--z", type=float, default=0.65, help="하단열 태그 중심 높이 m. 2026-11-10 실측 0.65 (상단 0.75 / 하단 0.55); 비행 고도는 0.6")
    p.add_argument("--upper", action="store_true", help="상단열(조건 C)도 만든다")
    p.add_argument("--upper-ids", default="0-9", help="상단열 ID. 2026-11-10 실험장: 하단열과 같은 0~9 (높이로만 구분)")
    p.add_argument("--upper-z", type=float, default=2.00)
    p.add_argument("--banner-h", type=float, default=0.50, help="현수막 높이 m (실물 50 cm)")
    p.add_argument("--banner-margin", type=float, default=0.25, help="양 끝 태그 바깥 여유 m")
    args = p.parse_args()
    args.out = os.path.abspath(os.path.expanduser(args.out))
    args.base_world = os.path.expanduser(args.base_world)
    args.tags_dir = os.path.expanduser(args.tags_dir)
    args.textures_dir = os.path.expanduser(args.textures_dir)
    build(args)


if __name__ == "__main__":
    main()
