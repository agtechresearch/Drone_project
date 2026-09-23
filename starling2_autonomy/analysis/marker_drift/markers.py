"""마커 배치(layout) 파일.

YAML 형식:
  name: lab_default
  tag_family: tag36h11
  default_size_m: 0.15
  markers:
    - {id: 0, x: 0.0, z: 0.60}          # y 생략 = 0 (벽 평면), size_m 생략 = default_size_m
    - {id: 3, x: 3.0, z: 0.60, size_m: 0.15, rpy_deg: [0, 0, 0]}

좌표는 M 프레임(geometry.py). 설치 후 실측값으로 x, z 를 갱신해 쓴다.
"""

import os

import yaml

from marker_drift import geometry

LAYOUT_DIR = os.path.join(os.path.dirname(__file__), "layouts")


def default_lab_layout(spacing_m=1.0, length_m=5.5, low_z=0.60, high_z=2.00, size_m=0.07):
    """docs/09 §4.1 기본 배치: 하단열 ID 0~6 (60 cm), 상단열 ID 10~16 (200 cm), 1 m 간격 + 마지막 0.5 m."""
    xs = []
    x = 0.0
    while x < length_m - 1e-9:
        xs.append(round(x, 3))
        x += spacing_m
    xs.append(round(length_m, 3))
    markers = []
    for i, x in enumerate(xs):
        markers.append({"id": i, "x": x, "y": 0.0, "z": low_z})
    for i, x in enumerate(xs):
        markers.append({"id": 10 + i, "x": x, "y": 0.0, "z": high_z})
    return {
        "name": "lab_default",
        "tag_family": "tag36h11",
        "default_size_m": size_m,
        "markers": markers,
    }


class Layout:
    def __init__(self, data):
        self.name = data.get("name", "layout")
        self.tag_family = data.get("tag_family", "tag36h11")
        self.default_size_m = float(data.get("default_size_m", 0.07))
        self.markers = {}
        for m in data.get("markers", []):
            m = dict(m)
            m.setdefault("y", 0.0)
            m["size_m"] = float(m.get("size_m", self.default_size_m))
            self.markers[int(m["id"])] = m

    @classmethod
    def load(cls, path):
        with open(path, "r", encoding="utf-8") as f:
            return cls(yaml.safe_load(f))

    @classmethod
    def default(cls):
        return cls(default_lab_layout())

    def save(self, path):
        data = {
            "name": self.name,
            "tag_family": self.tag_family,
            "default_size_m": self.default_size_m,
            "markers": [self.markers[k] for k in sorted(self.markers)],
        }
        with open(path, "w", encoding="utf-8") as f:
            yaml.safe_dump(data, f, allow_unicode=True, sort_keys=False)

    def ids(self):
        return sorted(self.markers)

    def size(self, tag_id):
        return self.markers[int(tag_id)]["size_m"]

    def pose(self, tag_id):
        return geometry.tag_pose_in_M(self.markers[int(tag_id)])

    def row_of(self, tag_id):
        """상/하단열 구분용: 같은 z 를 가진 마커 묶음의 z 값을 돌려준다."""
        return float(self.markers[int(tag_id)]["z"])

    def __contains__(self, tag_id):
        return int(tag_id) in self.markers

    def __len__(self):
        return len(self.markers)
