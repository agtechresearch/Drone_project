"""Windows DPI awareness 강제.

4K 모니터에서 Windows 디스플레이 스케일링이 100%가 아니면 (125%, 150% 등),
DPI-aware가 아닌 파이썬 프로세스는 창 크기/위치를 잘못 인식하고
pyautogui가 마우스 좌표를 이상하게 다룬다.

각 진입점(main.py, calibrate.py 등) 최상단에서 그냥 import만 하면 자동 적용:
    import dpi_setup  # noqa: F401
"""
import sys

if sys.platform == 'win32':
    import ctypes

    # 우선순위: Per-monitor DPI aware v2 > Per-monitor v1 > System aware
    try:
        # Windows 10 1703+ (PROCESS_PER_MONITOR_DPI_AWARE_V2 = 2)
        ctypes.windll.shcore.SetProcessDpiAwareness(2)
    except (AttributeError, OSError):
        try:
            # 오래된 Windows 폴백
            ctypes.windll.user32.SetProcessDPIAware()
        except (AttributeError, OSError):
            pass
