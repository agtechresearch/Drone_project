"""카메라 내부 파라미터 로드/저장.

지원 형식
  1) 자체 YAML:
       model: pinhole | fisheye
       width: 1024
       height: 768
       K: [[fx, 0, cx], [0, fy, cy], [0, 0, 1]]
       dist: [k1, k2, p1, p2, k3]        # fisheye 면 [k1, k2, k3, k4]
  2) OpenCV FileStorage (%YAML 헤더). VOXL 의 /data/modalai/opencv_<cam>_intrinsics.yml 이 이 형식이다.
     노드 이름은 M/D 또는 camera_matrix/distortion_coefficients 를 시도한다. 왜곡 계수가 4개고
     노드에 fisheye 표시가 있거나 --fisheye 를 주면 fisheye 모델로 본다.
"""

import numpy as np
import yaml


class Intrinsics:
    def __init__(self, K, dist, width=None, height=None, model="pinhole"):
        self.K = np.asarray(K, float).reshape(3, 3)
        self.dist = None if dist is None else np.asarray(dist, float).reshape(-1)
        self.width = width
        self.height = height
        self.model = model

    @property
    def fisheye(self):
        return self.model == "fisheye"

    def to_dict(self):
        return {
            "model": self.model,
            "width": self.width,
            "height": self.height,
            "K": self.K.tolist(),
            "dist": None if self.dist is None else self.dist.tolist(),
        }

    def save(self, path):
        with open(path, "w", encoding="utf-8") as f:
            yaml.safe_dump(self.to_dict(), f, sort_keys=False)

    @classmethod
    def load(cls, path, fisheye=None):
        with open(path, "r", encoding="utf-8", errors="replace") as f:
            head = f.read(64)
        if head.lstrip().startswith("%YAML"):
            return cls._load_opencv(path, fisheye)
        with open(path, "r", encoding="utf-8") as f:
            d = yaml.safe_load(f)
        model = d.get("model", "pinhole")
        if fisheye is not None:
            model = "fisheye" if fisheye else "pinhole"
        return cls(d["K"], d.get("dist"), d.get("width"), d.get("height"), model)

    @classmethod
    def _load_opencv(cls, path, fisheye=None):
        import cv2
        fs = cv2.FileStorage(path, cv2.FILE_STORAGE_READ)
        K = None
        D = None
        for name in ("M", "camera_matrix", "K"):
            node = fs.getNode(name)
            if not node.empty():
                K = node.mat()
                break
        for name in ("D", "distortion_coefficients", "dist"):
            node = fs.getNode(name)
            if not node.empty():
                D = node.mat().reshape(-1)
                break
        width = height = None
        for name in ("width", "image_width"):
            node = fs.getNode(name)
            if not node.empty():
                width = int(node.real())
        for name in ("height", "image_height"):
            node = fs.getNode(name)
            if not node.empty():
                height = int(node.real())
        model_node = fs.getNode("distortion_model")
        model = "pinhole"
        if not model_node.empty() and "fisheye" in str(model_node.string()).lower():
            model = "fisheye"
        if fisheye is not None:
            model = "fisheye" if fisheye else "pinhole"
        fs.release()
        if K is None:
            raise ValueError("OpenCV 파일에서 카메라 행렬(M/camera_matrix)을 찾지 못함: {}".format(path))
        return cls(K, D, width, height, model)

    @classmethod
    def simple(cls, fx, fy, cx, cy, width=None, height=None):
        return cls([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], np.zeros(5), width, height)
