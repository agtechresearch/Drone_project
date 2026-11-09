#!/usr/bin/env python3
"""Split the single QVIX module mesh into walls (shell + door thresholds) and racks so the SDF can colour them
differently (2026-10-27, user: grey-on-grey was hard to read in the GUI). Run with the cad venv (trimesh):
    source cad/venv/bin/activate && python3 repo_export/scripts/split_qvix_mesh.py
Writes qvix_r_walls.stl and qvix_r_racks.stl next to qvix_r_full.stl in repo_export/qvix_integration/qvix_r_module/meshes/
(the source make_gz_model.py copies into repo_export/gz/models/) and rewrites that model's SDF: collision keeps the full
mesh, visuals are split. Rack components are the connected parts that lie inside the rack x bands (0.75-1.28, 2.0-2.55,
3.27-3.8, 4.53-5.06 m, see qvix_grid.RACKS_X); everything else is wall/floor/ceiling.
"""
import os
import trimesh

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
MDIR = os.path.join(ROOT, "repo_export", "qvix_integration", "qvix_r_module")
MESHES = os.path.join(MDIR, "meshes")
RACKS_X_MM = [(700, 1300), (2000, 2600), (3250, 3850), (4500, 5100)]   # generous bands around the rack lines

m = trimesh.load(os.path.join(MESHES, "qvix_r_full.stl"))
parts = m.split(only_watertight=False)
racks, walls = [], []
for p in parts:
    (x0, _, _), (x1, _, _) = p.bounds
    inside = any(a <= x0 and x1 <= b for a, b in RACKS_X_MM)
    (racks if inside and (x1 - x0) < 700 else walls).append(p)
print("components %d -> racks %d, walls %d" % (len(parts), len(racks), len(walls)))
trimesh.util.concatenate(racks).export(os.path.join(MESHES, "qvix_r_racks.stl"))
trimesh.util.concatenate(walls).export(os.path.join(MESHES, "qvix_r_walls.stl"))

SDF = """<?xml version="1.0"?>
<sdf version="1.6">
  <model name="qvix_r_module">
    <static>true</static>
    <link name="link">
      <collision name="collision">
        <geometry>
          <mesh>
            <uri>model://qvix_r_module/meshes/qvix_r_full.stl</uri>
            <scale>0.001 0.001 0.001</scale>
          </mesh>
        </geometry>
      </collision>
      <!-- visuals split by split_qvix_mesh.py: walls/floor/ceiling light warm grey, racks green -->
      <visual name="walls">
        <geometry>
          <mesh>
            <uri>model://qvix_r_module/meshes/qvix_r_walls.stl</uri>
            <scale>0.001 0.001 0.001</scale>
          </mesh>
        </geometry>
        <material>
          <ambient>0.62 0.60 0.56 1</ambient>
          <diffuse>0.88 0.86 0.80 1</diffuse>
          <specular>0.05 0.05 0.05 1</specular>
        </material>
      </visual>
      <visual name="racks">
        <geometry>
          <mesh>
            <uri>model://qvix_r_module/meshes/qvix_r_racks.stl</uri>
            <scale>0.001 0.001 0.001</scale>
          </mesh>
        </geometry>
        <material>
          <ambient>0.18 0.42 0.20 1</ambient>
          <diffuse>0.30 0.68 0.32 1</diffuse>
          <specular>0.10 0.10 0.10 1</specular>
        </material>
      </visual>
    </link>
  </model>
</sdf>
"""
open(os.path.join(MDIR, "qvix_r_module.sdf"), "w").write(SDF)
print("wrote", os.path.join(MDIR, "qvix_r_module.sdf"))
