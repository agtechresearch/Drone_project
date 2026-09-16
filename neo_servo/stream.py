"""프레임 소스: scrcpy(OBS 가상카메라) 또는 RTMP 스트림.

두 소스 모두 동일한 read()/release() 인터페이스로 노출하므로,
main.py 쪽 코드는 소스 종류를 신경 쓸 필요 없다.
"""
from __future__ import annotations
from abc import ABC, abstractmethod
import cv2


class FrameSource(ABC):
    @abstractmethod
    def read(self):
        """(bool, ndarray | None) 반환."""
        ...

    @abstractmethod
    def release(self):
        ...


class ScrcpyOBSSource(FrameSource):
    """OBS 가상카메라를 통해 scrcpy 미러링 화면을 캡처.

    폰 화면 그대로라 DJI Fly UI 오버레이가 포함됨. 마커 탐지 시 UI 마스크 필요.

    주의: OpenCV VideoCapture는 명시적으로 해상도를 요청하지 않으면
    카메라 드라이버 기본값(대개 640x480)으로 열림. OBS 가상카메라가 4K로
    송출해도 클라이언트가 저해상도 요청하면 downscale되어 넘어옴.
    → width/height를 반드시 명시.
    """
    def __init__(self, camera_index: int = 0, fps: int = 30,
                 width: int = 2960, height: int = 1440):
        # Windows에서는 DSHOW 백엔드가 OBS 가상카메라와 호환성 좋음
        self.cap = cv2.VideoCapture(camera_index, cv2.CAP_DSHOW)
        if not self.cap.isOpened():
            # 기본 백엔드로 재시도
            self.cap = cv2.VideoCapture(camera_index)
        if not self.cap.isOpened():
            raise RuntimeError(f"OBS 가상카메라 열기 실패 (index={camera_index})")

        # 해상도 명시 요청 (반영은 카메라가 지원하는 경우에만 됨)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,  width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_FPS, fps)
        # 오래된 프레임 즉시 버림
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        # 실제 반영된 값 확인용
        actual_w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        print(f"[stream] 요청 {width}x{height} → 실제 {actual_w}x{actual_h}")

    def read(self):
        return self.cap.read()

    def release(self):
        self.cap.release()


class RTMPSource(FrameSource):
    """DJI Fly 앱의 RTMP 스트림 직접 수신.

    UI 오버레이 없는 순수 카메라 프리뷰. 대신 지연이 500ms~2s 발생 가능.
    로컬 nginx-rtmp 또는 MediaMTX 서버가 떠 있어야 함.
    """
    def __init__(self, url: str, buffer_size: int = 1):
        # ffmpeg 백엔드로 저지연 옵션 강제
        self.cap = cv2.VideoCapture(url, cv2.CAP_FFMPEG)
        if not self.cap.isOpened():
            raise RuntimeError(f"RTMP 스트림 열기 실패: {url}")
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, buffer_size)

    def read(self):
        return self.cap.read()

    def release(self):
        self.cap.release()


def create_source(config: dict) -> FrameSource:
    """설정에 따라 적절한 소스 인스턴스 생성."""
    stype = config['source']['type']
    if stype == 'scrcpy':
        s = config['source']['scrcpy']
        return ScrcpyOBSSource(
            camera_index=s['camera_index'],
            fps=s['fps'],
            width=s.get('width', 2960),
            height=s.get('height', 1440),
        )
    elif stype == 'rtmp':
        s = config['source']['rtmp']
        return RTMPSource(url=s['url'], buffer_size=s['buffer_size'])
    else:
        raise ValueError(f"알 수 없는 source.type: {stype!r} (scrcpy/rtmp 중 하나)")