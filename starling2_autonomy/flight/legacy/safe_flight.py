#!/usr/bin/env python3
"""
Safe takeoff/hover/land for Starling 2 via MAVSDK.
DRY_RUN_LEVEL:
  0 = real flight (takeoff/hover/land)
  1 = arm + disarm only
  2 = arm + offboard enter + setpoint streaming + offboard exit
  3 = arm + offboard + hold + land() sequence verification
"""
import asyncio
import signal
import sys
from mavsdk import System
from mavsdk.offboard import OffboardError, PositionNedYaw

# ==== CONFIG ====
DRY_RUN_LEVEL = 3
TAKEOFF_ALT = 0.8
HOVER_TIME = 5.0
TAKEOFF_DURATION = 4.0
SETPOINT_HZ = 20.0
POS_OK_HOLD_SEC = 5.0
DRY_RUN_L2_DURATION = 8.0
DRY_RUN_L3_HOLD = 5.0       # Level 3에서 offboard hold 시간
LAND_WAIT_TIMEOUT = 20.0    # land() 후 landed=True 대기 최대 시간
# ================

class DroneController:
    def __init__(self):
        self.drone = System(mavsdk_server_address="localhost", port=50051)
        self.target = PositionNedYaw(0.0, 0.0, 0.0, 0.0)
        self.streaming = False
        self.aborted = False
        self.stream_task = None
        self.initial_x = 0.0
        self.initial_y = 0.0
        self.initial_z = 0.0

    async def connect(self):
        print("[1/8] Connecting...")
        await self.drone.connect()
        async for state in self.drone.core.connection_state():
            if state.is_connected:
                print("      Connected!")
                return
            await asyncio.sleep(0.1)

    async def wait_position_stable(self, hold_sec=POS_OK_HOLD_SEC, timeout=60.0):
        print("[2/8] Waiting for stable position estimate ({}s hold)...".format(hold_sec))
        ok_start = None
        deadline = asyncio.get_event_loop().time() + timeout
        async for health in self.drone.telemetry.health():
            now = asyncio.get_event_loop().time()
            if now > deadline:
                raise RuntimeError("Position estimate did not stabilize within timeout")
            if health.is_local_position_ok:
                if ok_start is None:
                    ok_start = now
                    print("      Position OK detected, holding...")
                elif now - ok_start >= hold_sec:
                    print("      Position stable for {}s. OK.".format(hold_sec))
                    return
            else:
                if ok_start is not None:
                    print("      Position lost, resetting hold timer")
                ok_start = None
            await asyncio.sleep(0.2)

    async def capture_initial_position(self):
        print("[3/8] Capturing initial position as hold setpoint...")
        async for pos in self.drone.telemetry.position_velocity_ned():
            self.initial_x = pos.position.north_m
            self.initial_y = pos.position.east_m
            self.initial_z = pos.position.down_m
            print("      Initial NED: x={:.3f} y={:.3f} z={:.3f}".format(
                self.initial_x, self.initial_y, self.initial_z))
            self.target = PositionNedYaw(self.initial_x, self.initial_y, self.initial_z, 0.0)
            return

    async def setpoint_streamer(self):
        period = 1.0 / SETPOINT_HZ
        send_count = 0
        while self.streaming:
            try:
                await self.drone.offboard.set_position_ned(self.target)
                send_count += 1
                if send_count % int(SETPOINT_HZ) == 0:
                    print("      [streamer] sent {} setpoints, z={:.3f}".format(
                        send_count, self.target.down_m))
            except Exception as e:
                print("      [streamer] send failed: {}".format(e))
            await asyncio.sleep(period)

    async def arm_and_start_offboard(self):
        print("[4/8] Priming setpoint stream (2s)...")
        self.streaming = True
        self.stream_task = asyncio.ensure_future(self.setpoint_streamer())
        await asyncio.sleep(2.0)

        print("[5/8] Arming...")
        await self.drone.action.arm()
        await asyncio.sleep(1.0)

        if DRY_RUN_LEVEL == 1:
            print("      [DRY_RUN_LEVEL=1] Skipping offboard.")
            return

        print("      Starting offboard mode...")
        try:
            await self.drone.offboard.start()
            print("      Offboard mode active!")
        except OffboardError as e:
            print("      Offboard start FAILED: {}".format(e._result.result))
            await self.emergency_stop()
            raise

    async def takeoff_smooth(self):
        if DRY_RUN_LEVEL >= 1:
            return
        print("[6/8] Smooth takeoff to {}m over {}s...".format(TAKEOFF_ALT, TAKEOFF_DURATION))
        steps = int(TAKEOFF_DURATION * SETPOINT_HZ)
        for i in range(steps):
            if self.aborted:
                return
            progress = (i + 1) / steps
            if progress < 0.5:
                eased = 2.0 * progress * progress
            else:
                eased = 1.0 - 2.0 * (1.0 - progress) * (1.0 - progress)
            alt = TAKEOFF_ALT * eased
            self.target = PositionNedYaw(self.initial_x, self.initial_y,
                                          self.initial_z - alt, 0.0)
            await asyncio.sleep(1.0 / SETPOINT_HZ)
        self.target = PositionNedYaw(self.initial_x, self.initial_y,
                                      self.initial_z - TAKEOFF_ALT, 0.0)
        print("      Reached {}m".format(TAKEOFF_ALT))

    async def hover(self):
        if DRY_RUN_LEVEL == 0:
            print("[6/8] Hovering for {}s...".format(HOVER_TIME))
            await asyncio.sleep(HOVER_TIME)
        elif DRY_RUN_LEVEL == 2:
            print("[6/8] Holding in offboard for {}s (L2)...".format(DRY_RUN_L2_DURATION))
            await asyncio.sleep(DRY_RUN_L2_DURATION)
        elif DRY_RUN_LEVEL == 3:
            print("[6/8] Holding in offboard for {}s (L3)...".format(DRY_RUN_L3_HOLD))
            await asyncio.sleep(DRY_RUN_L3_HOLD)

    async def monitor_land_state(self, duration):
        """vehicle_land_detected를 일정 시간 동안 polling하면서 상태 출력."""
        async def poll():
            count = 0
            async for landed in self.drone.telemetry.landed_state():
                count += 1
                print("      [land_monitor] state={}".format(landed))
                await asyncio.sleep(0.5)
                if count > duration * 2:
                    break
        try:
            await asyncio.wait_for(poll(), timeout=duration)
        except asyncio.TimeoutError:
            pass

    async def land_and_verify(self):
        """L3: land() 호출 후 landed 상태가 될 때까지 polling."""
        print("[7/8] Calling action.land()...")
        await self.drone.action.land()
        print("      land() command sent. Watching landed_state...")

        # in_air → landed로 전환되는지 LAND_WAIT_TIMEOUT 초까지 대기
        start = asyncio.get_event_loop().time()
        landed_confirmed = False
        async for in_air in self.drone.telemetry.in_air():
            elapsed = asyncio.get_event_loop().time() - start
            print("      [land_check] t={:.1f}s in_air={}".format(elapsed, in_air))
            if not in_air:
                print("      *** LANDED detected by telemetry.in_air ***")
                landed_confirmed = True
                break
            if elapsed > LAND_WAIT_TIMEOUT:
                print("      *** LAND TIMEOUT after {}s ***".format(LAND_WAIT_TIMEOUT))
                break
            await asyncio.sleep(0.5)

        return landed_confirmed

    async def stop_offboard_and_land(self):
        if DRY_RUN_LEVEL == 0:
            # 실비행: land 호출 후 자연스럽게 disarm 될 때까지 대기
            print("[7/8] Calling action.land()...")
            await self.drone.action.land()
            async for in_air in self.drone.telemetry.in_air():
                if not in_air:
                    print("      Landed.")
                    break
                await asyncio.sleep(0.5)
        elif DRY_RUN_LEVEL == 2:
            print("[7/8] Stopping offboard...")
            try:
                await self.drone.offboard.stop()
                print("      Offboard stopped cleanly.")
            except Exception as e:
                print("      Offboard stop warning: {}".format(e))
        elif DRY_RUN_LEVEL == 3:
            # L3: 진짜 land() 시퀀스 검증
            await self.land_and_verify()

        # Streamer 정지
        self.streaming = False
        if self.stream_task:
            try:
                await asyncio.wait_for(self.stream_task, timeout=1.0)
            except Exception:
                pass

        print("[8/8] Final cleanup...")
        # disarm 시도 (이미 auto-disarm 됐을 수 있음)
        try:
            await self.drone.action.disarm()
            print("      Disarmed.")
        except Exception as e:
            print("      Disarm note: {}".format(e))

        # L3에서 disarm 실패하면 fallback
        if DRY_RUN_LEVEL == 3:
            await asyncio.sleep(1.0)
            # 여전히 armed면 강제 정리
            async for armed in self.drone.telemetry.armed():
                if armed:
                    print("      Still armed, attempting kill...")
                    try:
                        await self.drone.action.kill()
                        print("      Kill command sent.")
                    except Exception as e:
                        print("      Kill failed: {}".format(e))
                else:
                    print("      Confirmed disarmed.")
                break
        print("Done!")

    async def emergency_stop(self):
        print("\n!!! EMERGENCY STOP !!!")
        self.aborted = True
        self.streaming = False
        if DRY_RUN_LEVEL == 0:
            try:
                await self.drone.action.land()
            except Exception:
                pass
            await asyncio.sleep(0.5)
            try:
                await self.drone.action.kill()
            except Exception:
                pass
        else:
            try:
                await self.drone.offboard.stop()
            except Exception:
                pass
            try:
                await self.drone.action.disarm()
            except Exception:
                pass
            await asyncio.sleep(0.5)
            try:
                await self.drone.action.kill()
            except Exception:
                pass

async def main():
    ctrl = DroneController()

    def sigint_handler():
        print("\nSIGINT received")
        asyncio.ensure_future(ctrl.emergency_stop())

    loop = asyncio.get_event_loop()
    try:
        loop.add_signal_handler(signal.SIGINT, sigint_handler)
    except NotImplementedError:
        pass

    try:
        await ctrl.connect()
        await ctrl.wait_position_stable()
        await ctrl.capture_initial_position()
        await ctrl.arm_and_start_offboard()
        await ctrl.takeoff_smooth()
        await ctrl.hover()
        await ctrl.stop_offboard_and_land()
    except Exception as e:
        print("\nException: {}".format(e))
        await ctrl.emergency_stop()
        sys.exit(1)

if __name__ == "__main__":
    print("=" * 50)
    print("Starling 2 safe flight - DRY_RUN_LEVEL={}".format(DRY_RUN_LEVEL))
    if DRY_RUN_LEVEL == 0:
        print("  *** REAL FLIGHT - alt: {}m, hover: {}s ***".format(TAKEOFF_ALT, HOVER_TIME))
    elif DRY_RUN_LEVEL == 1:
        print("  DRY RUN L1: arm + disarm only")
    elif DRY_RUN_LEVEL == 2:
        print("  DRY RUN L2: arm + offboard + {}s hold".format(DRY_RUN_L2_DURATION))
    elif DRY_RUN_LEVEL == 3:
        print("  DRY RUN L3: arm + offboard + {}s hold + land() verification".format(DRY_RUN_L3_HOLD))
    print("  Setpoint rate: {}Hz".format(SETPOINT_HZ))
    print("=" * 50)
    print("CHECKLIST:")
    print("  [ ] RC controller ON, kill switch ready")
    if DRY_RUN_LEVEL >= 1:
        print("  [ ] PROPELLERS REMOVED")
    else:
        print("  [ ] Propellers attached, area clear")
    print("  [ ] vision-hub running, VIO converged")
    print("  [ ] Battery >50%")
    input("Press Enter when ready...")

    asyncio.get_event_loop().run_until_complete(main())