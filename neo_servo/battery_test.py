"""배터리 지속시간 테스트: 제자리에서 사각형 반복 비행.

한 사이클 = 좌 이동 → 상승 → 우 이동 → 하강
배터리 소진, 최대 시간/사이클, Ctrl+C, F12 중 하나에 도달할 때까지 반복.

방향별 offset을 개별 지정할 수 있음:
  - Neo는 하강 명령을 자체 안전 로직으로 감쇠시키므로
    하강 offset을 상승보다 크게 잡아야 실제로 내려감.

주의:
  - Neo는 이 사각형을 정확히 그리지 않음. VIO 드리프트/관성 때문.
    배터리 시간 측정이 목적이므로 문제 없음.
  - 각 사이클의 실제 시간이 로그에 찍히므로 그걸로 배터리별 사이클 수 산출.

사용:
  python battery_test.py
  python battery_test.py --lateral 60 --ascend 60 --descend 120
  python battery_test.py --dry-run                    # 스틱 조작 없이 흐름만
  python battery_test.py --max-cycles 5               # 5 사이클만
"""
from __future__ import annotations
import dpi_setup  # noqa: F401 - Windows DPI awareness (반드시 최상단)
import argparse
import os
import sys
import time
import yaml

from control import NeoController
from emergency import EmergencyStop


# ============================================================
# 로거
# ============================================================
class Logger:
    def __init__(self, log_dir: str, prefix: str):
        os.makedirs(log_dir, exist_ok=True)
        ts = time.strftime('%Y%m%d_%H%M%S')
        self.path = os.path.join(log_dir, f"{prefix}_{ts}.log")
        self.file = open(self.path, 'a', encoding='utf-8')
        self.start = time.time()

    def log(self, msg: str):
        line = f"[+{time.time()-self.start:7.2f}s] {msg}"
        print(line)
        self.file.write(line + "\n")
        self.file.flush()

    def close(self):
        self.file.close()


class RunState:
    """비상 정지 감지용 공유 플래그."""
    def __init__(self):
        self.running = True


def load_config(path: str) -> dict:
    with open(path, 'r', encoding='utf-8') as f:
        return yaml.safe_load(f)


# ============================================================
# 사각형 사이클
# ============================================================
def square_cycle(controller: NeoController,
                 offset_lateral: int, offset_ascend: int, offset_descend: int,
                 duration: float, pause: float,
                 state: RunState, logger: Logger, cycle_num: int) -> bool:
    """한 사이클: 좌→상→우→하. 정상 완료면 True, 중단이면 False.

    방향별 offset:
      - lateral: 좌/우 이동 강도
      - ascend : 상승 강도
      - descend: 하강 강도 (Neo 자체 안전 감쇠 보상 위해 크게 잡음)
    """
    logger.log(f"───── Cycle {cycle_num} 시작 ─────")
    t_cycle = time.time()

    # (안내 이름, stick, offset_x, offset_y)
    moves = [
        ("← 좌 이동", 'right', -offset_lateral, 0),
        ("↑ 상승   ", 'left',   0, -offset_ascend),
        ("→ 우 이동", 'right', +offset_lateral, 0),
        ("↓ 하강   ", 'left',   0, +offset_descend),
    ]

    for name, stick, ox, oy in moves:
        if not state.running:
            return False
        logger.log(f"  {name}  stick={stick} offset=({ox},{oy}) dur={duration}s")
        controller.execute_stick_action(stick, ox, oy, duration)
        if not state.running:
            return False
        if pause > 0:
            time.sleep(pause)

    cycle_time = time.time() - t_cycle
    logger.log(f"───── Cycle {cycle_num} 완료 ({cycle_time:.2f}s) ─────")
    return True


# ============================================================
# 메인
# ============================================================
def main():
    p = argparse.ArgumentParser(
        description="배터리 지속시간 테스트: 사각형 반복 비행",
    )
    p.add_argument('--config',    default='config.yaml')
    p.add_argument('--duration',  type=float, default=1.5,
                   help='각 변 스틱 유지 시간 (초, 기본 1.5)')

    # 방향별 offset (Neo는 하강이 약해서 별도 지정 가능)
    p.add_argument('--lateral', type=int, default=60,
                   help='좌/우 이동 스틱 오프셋 (px, 기본 60)')
    p.add_argument('--ascend', type=int, default=60,
                   help='상승 스틱 오프셋 (px, 기본 60)')
    p.add_argument('--descend', type=int, default=80,
                   help='하강 스틱 오프셋 (px, 기본 80). '
                        '너무 강하면 낮추고, 너무 약하면 100~120까지 올릴 것')

    p.add_argument('--pause',     type=float, default=0.5,
                   help='각 변 사이 대기 시간 (초, 기본 0.5)')
    p.add_argument('--max-cycles', type=int, default=0,
                   help='최대 사이클 수 (0=무한, 기본 0)')
    p.add_argument('--max-minutes', type=float, default=25.0,
                   help='최대 실행 시간 (분, 기본 25)')
    p.add_argument('--dry-run', action='store_true',
                   help='실제 이륙/스틱 조작 없이 흐름 로그만')
    args = p.parse_args()

    cfg = load_config(args.config)
    logger = Logger(cfg['logging']['log_dir'], 'battery_test')

    # 사이클 예상 시간 계산 (참고용)
    est_cycle_s = 4 * (args.duration + args.pause + 0.3)  # 0.3s는 스틱 정리 시간
    logger.log(f"config: {args.config}")
    logger.log(f"duration={args.duration}s pause={args.pause}s")
    logger.log(f"offset  lateral={args.lateral}px "
               f"ascend={args.ascend}px descend={args.descend}px")
    logger.log(f"max_cycles={args.max_cycles or '무한'}  max_minutes={args.max_minutes}")
    logger.log(f"예상 사이클 시간: 약 {est_cycle_s:.1f}s")

    state = RunState()
    controller = None if args.dry_run else NeoController(cfg, logger=logger)

    # 비상 정지 리스너
    hotkey = cfg.get('emergency', {}).get('hotkey', '<f12>')
    def emergency_stop():
        logger.log(f"!!!!! 비상 정지 (hotkey={hotkey}) !!!!!")
        if controller is not None:
            controller.emergency_release()
        state.running = False
    es = EmergencyStop(on_stop=emergency_stop, hotkey=hotkey)
    es.start()
    logger.log(f"비상 정지 핫키 활성: {hotkey}")

    completed_cycles = 0
    flight_start = None

    try:
        # ── 이륙 ──
        if not args.dry_run:
            logger.log("이륙 시작")
            controller.takeoff()
            time.sleep(3)   # 안정화 대기
            logger.log("이륙 완료 → 사각형 사이클 시작")
        else:
            logger.log("DRY-RUN: 이륙 스킵")

        flight_start = time.time()
        deadline = flight_start + args.max_minutes * 60

        # ── 사각형 반복 ──
        cycle = 0
        while state.running and time.time() < deadline:
            cycle += 1
            if args.max_cycles > 0 and cycle > args.max_cycles:
                logger.log(f"max_cycles={args.max_cycles} 도달")
                cycle -= 1
                break

            if args.dry_run:
                # dry-run: 실제 조작 없이 예상 시간만 sleep
                logger.log(f"───── Cycle {cycle} (dry-run) ─────")
                total = 4 * (args.duration + args.pause)
                slept = 0.0
                while slept < total and state.running:
                    time.sleep(0.1)
                    slept += 0.1
                if state.running:
                    logger.log(f"───── Cycle {cycle} 완료 (dry-run, {total:.1f}s) ─────")
                    completed_cycles = cycle
            else:
                if square_cycle(controller,
                                args.lateral, args.ascend, args.descend,
                                args.duration, args.pause,
                                state, logger, cycle):
                    completed_cycles = cycle
                else:
                    logger.log("사이클 중단됨")
                    break

        # ── 통계 ──
        if flight_start is not None:
            total_flight = time.time() - flight_start
            avg_cycle = total_flight / completed_cycles if completed_cycles else 0
            logger.log("═" * 50)
            logger.log(f"완료 사이클: {completed_cycles}")
            logger.log(f"총 비행 시간: {total_flight:.1f}s ({total_flight/60:.2f}분)")
            logger.log(f"평균 사이클 시간: {avg_cycle:.2f}s")
            logger.log("═" * 50)

    except KeyboardInterrupt:
        logger.log("사용자 중단 (Ctrl+C)")

    finally:
        # ── 착륙 ──
        if controller is not None:
            try:
                controller.emergency_release()   # 혹시 잡고 있던 스틱 해제
                if state.running:
                    logger.log("착륙 시작")
                    controller.landing()
                else:
                    logger.log("비상 정지 상태 - 자동 착륙 건너뜀 (수동 착륙 필요)")
            except Exception as e:
                logger.log(f"착륙 중 예외: {e!r}")

        state.running = False
        es.stop()
        logger.log("종료")
        logger.close()


if __name__ == "__main__":
    main()