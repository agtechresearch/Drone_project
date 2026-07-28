#!/usr/bin/env python3
"""
python3 ~/path_flight_phase1_patched.py

Starling 2 - Path flight Phase 1 (PATCHED VERSION v2)
- Each step waits until target is reached before proceeding.
- Tolerance-based with stability check and timeout.

[PATCHES APPLIED]
  [FIX 1] H/V tolerance separated for accurate takeoff/altitude check
  [FIX 2] Single emergency lock to prevent double-trigger
  [FIX 3] trigger_emergency_land stops setpoint streaming first (CRITICAL)
  [FIX 4] Geofence margin increased + dynamic relative to FORWARD_DISTANCE
  [FIX 5] wait_until_reached timeout adjusted realistically
  [FIX 6] Monitor task graceful cancellation
  [FIX 7] XYZ stability + STALE detection in capture_initial_position
  [FIX 8] setpoint_streamer auto-trigger emergency on consecutive failures
  [FIX 9] Step numbering fixed
  [FIX 12] check_vio_quality_preflight loop refactored
  [Python 3.6 compat] get_event_loop().run_until_complete used
"""
import asyncio
import signal
import sys
import math
from mavsdk import System
from mavsdk.offboard import OffboardError, PositionNedYaw

# ==== CONFIG ====
TAKEOFF_ALT = 1.0
FORWARD_DISTANCE = 1.0

# 도달 판정 - [FIX 1] H/V 분리
HORIZONTAL_TOLERANCE = 0.4
VERTICAL_TOLERANCE = 0.25
STABLE_TIME = 1.0
MOVE_TIMEOUT = 20.0

# 호버
HOVER_TIME = 3.0

SETPOINT_HZ = 20.0
POS_OK_HOLD_SEC = 5.0
LAND_WAIT_TIMEOUT = 15.0

# Geofence - [FIX 4]
MAX_ALT_DEVIATION = 1.5
MAX_HORIZONTAL_DEV = FORWARD_DISTANCE + 1.5

# [FIX 8] streamer 연속 실패 한계
MAX_STREAM_FAIL = 5
# ================


class DroneController:
    def __init__(self):
        self.drone = System(mavsdk_server_address="localhost", port=50051)
        self.target = PositionNedYaw(0.0, 0.0, 0.0, 0.0)
        self.streaming = False
        self.aborted = False
        self.stream_task = None
        self.monitor_task = None
        self.vio_monitor_task = None
        self.initial_x = 0.0
        self.initial_y = 0.0
        self.initial_z = 0.0
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0
        self.takeoff_started = False

        # [FIX 2] 단일 emergency lock
        self._emergency_lock = asyncio.Lock()
        self._emergency_triggered = False

        # [FIX 8] streamer 실패 카운터
        self._stream_fail_count = 0

    async def connect(self):
        print("[1/7] Connecting...")
        await self.drone.connect()
        async for state in self.drone.core.connection_state():
            if state.is_connected:
                print("       Connected!")
                return
            await asyncio.sleep(0.1)

    async def wait_position_stable(self, hold_sec=POS_OK_HOLD_SEC, timeout=60.0):
        print("[2/7] Waiting for stable position estimate ({}s)...".format(hold_sec))
        ok_start = None
        deadline = asyncio.get_event_loop().time() + timeout
        async for health in self.drone.telemetry.health():
            now = asyncio.get_event_loop().time()
            if now > deadline:
                raise RuntimeError("Position estimate did not stabilize")
            if health.is_local_position_ok:
                if ok_start is None:
                    ok_start = now
                elif now - ok_start >= hold_sec:
                    print("       Position stable. OK.")
                    return
            else:
                ok_start = None
            await asyncio.sleep(0.2)

    async def check_vio_quality_preflight(self):
        print("[3/7] Checking VIO stability (5 samples)...")
        count = 0
        async for health in self.drone.telemetry.health():
            if not health.is_local_position_ok:
                raise RuntimeError("VIO degraded during pre-check")
            count += 1
            if count >= 5:
                break
            await asyncio.sleep(0.5)
        print("       VIO stable.")

    async def capture_initial_position(self):
        print("[4/7] Capturing initial position (10 samples over ~3s)...")
        xs, ys, zs = [], [], []
        count = 0
        async for pos in self.drone.telemetry.position_velocity_ned():
            xs.append(pos.position.north_m)
            ys.append(pos.position.east_m)
            zs.append(pos.position.down_m)
            count += 1
            if count >= 10:
                break
            await asyncio.sleep(0.3)

        self.initial_x = sum(xs) / len(xs)
        self.initial_y = sum(ys) / len(ys)
        self.initial_z = sum(zs) / len(zs)
        self.current_x = self.initial_x
        self.current_y = self.initial_y
        self.current_z = self.initial_z

        x_range = max(xs) - min(xs)
        y_range = max(ys) - min(ys)
        z_range = max(zs) - min(zs)
        print("       Initial NED: x={:.3f} y={:.3f} z={:.3f}".format(
            self.initial_x, self.initial_y, self.initial_z))
        print("       Ranges: x={:.3f} y={:.3f} z={:.3f}".format(
            x_range, y_range, z_range))

        # [FIX 7] VIO stale 감지 (모든 range가 정확히 0이면 stale)
        if x_range == 0.0 and y_range == 0.0 and z_range == 0.0:
            raise RuntimeError(
                "VIO data appears STALE (zero variance over 3s). "
                "VIO not updating - try:\n"
                "  1. systemctl restart voxl-open-vins-server\n"
                "  2. Shake drone for 10s to reinit VIO\n"
                "  3. Verify with: voxl-inspect-pose vvhub_body_wrt_local"
            )

        # 너무 큰 범위면 VIO 발산
        if x_range > 0.05 or y_range > 0.05 or z_range > 0.05:
            raise RuntimeError(
                "Initial position UNSTABLE: x_r={:.3f} y_r={:.3f} z_r={:.3f}. "
                "Wait longer or check VIO".format(x_range, y_range, z_range))

        self.target = PositionNedYaw(self.initial_x, self.initial_y, self.initial_z, 0.0)

    async def position_monitor(self):
        async for pos in self.drone.telemetry.position_velocity_ned():
            if not self.streaming:
                break
            self.current_x = pos.position.north_m
            self.current_y = pos.position.east_m
            self.current_z = pos.position.down_m

            if self.takeoff_started:
                dx = self.current_x - self.initial_x
                dy = self.current_y - self.initial_y
                horizontal_dev = math.sqrt(dx*dx + dy*dy)
                target_z = self.target.down_m
                alt_dev = abs(self.current_z - target_z)

                if horizontal_dev > MAX_HORIZONTAL_DEV:
                    asyncio.ensure_future(self.trigger_emergency_land(
                        "GEOFENCE: horizontal {:.2f}m > {}m".format(
                            horizontal_dev, MAX_HORIZONTAL_DEV)))

                if alt_dev > MAX_ALT_DEVIATION:
                    asyncio.ensure_future(self.trigger_emergency_land(
                        "GEOFENCE: altitude {:.2f}m > {}m".format(
                            alt_dev, MAX_ALT_DEVIATION)))
            await asyncio.sleep(0.05)

    async def vio_quality_monitor(self):
        consecutive_bad = 0
        async for health in self.drone.telemetry.health():
            if not self.streaming:
                break
            if not health.is_local_position_ok:
                consecutive_bad += 1
                if consecutive_bad >= 3:
                    asyncio.ensure_future(self.trigger_emergency_land(
                        "VIO LOST (3 consecutive bad samples)"))
            else:
                consecutive_bad = 0
            await asyncio.sleep(0.3)

    async def trigger_emergency_land(self, reason="unknown"):
        """[FIX 2,3] 중복 호출 방지 + setpoint 우선 중단"""
        async with self._emergency_lock:
            if self._emergency_triggered:
                return
            self._emergency_triggered = True

        print("\n!!! EMERGENCY LAND TRIGGERED: {} !!!".format(reason))

        # [FIX 3] CRITICAL: setpoint 먼저 중단
        self.streaming = False
        self.aborted = True
        await asyncio.sleep(0.2)

        try:
            await self.drone.action.land()
            print("       Land command sent.")
        except Exception as e:
            print("       Emergency land failed: {}, trying kill".format(e))
            try:
                await self.drone.action.kill()
                print("       Kill command sent.")
            except Exception as ke:
                print("       Kill also failed: {}".format(ke))

    async def setpoint_streamer(self):
        period = 1.0 / SETPOINT_HZ
        while self.streaming:
            try:
                await self.drone.offboard.set_position_ned(self.target)
                self._stream_fail_count = 0
            except Exception as e:
                self._stream_fail_count += 1
                print("       [streamer] send failed ({}/{}) : {}".format(
                    self._stream_fail_count, MAX_STREAM_FAIL, e))
                if self._stream_fail_count >= MAX_STREAM_FAIL:
                    asyncio.ensure_future(self.trigger_emergency_land(
                        "setpoint stream failed {} times".format(MAX_STREAM_FAIL)))
                    break
            await asyncio.sleep(period)

    async def arm_and_start_offboard(self):
        print("[5/7] Priming setpoint stream (3s)...")
        self.streaming = True
        self.stream_task = asyncio.ensure_future(self.setpoint_streamer())
        self.monitor_task = asyncio.ensure_future(self.position_monitor())
        self.vio_monitor_task = asyncio.ensure_future(self.vio_quality_monitor())
        await asyncio.sleep(3.0)

        print("[6/7] Arming...")
        await self.drone.action.arm()
        await asyncio.sleep(1.0)

        print("       *** RC must be in OFFBOARD mode ***")
        print("       Starting offboard...")
        try:
            await self.drone.offboard.start()
            print("       Offboard active!")
        except OffboardError as e:
            print("       Offboard FAILED: {}".format(e._result.result))
            await self.emergency_stop()
            raise

    async def wait_until_reached(self, name,
                                 h_tol=HORIZONTAL_TOLERANCE,
                                 v_tol=VERTICAL_TOLERANCE,
                                 stable_time=STABLE_TIME,
                                 timeout=MOVE_TIMEOUT):
        """[FIX 1] H/V 분리 도달 판정"""
        print("       [wait] reaching {} (h_tol={}m, v_tol={}m, stable={}s, timeout={}s)".format(
            name, h_tol, v_tol, stable_time, timeout))
        start_time = asyncio.get_event_loop().time()
        in_tolerance_since = None
        last_print = -1.0

        while True:
            if self.aborted:
                return False
            now = asyncio.get_event_loop().time()
            elapsed = now - start_time

            dx = self.current_x - self.target.north_m
            dy = self.current_y - self.target.east_m
            dz = self.current_z - self.target.down_m
            h_dist = math.sqrt(dx*dx + dy*dy)
            v_dist = abs(dz)

            if elapsed - last_print >= 1.0:
                last_print = elapsed
                rel_x = self.current_x - self.initial_x
                rel_y = self.current_y - self.initial_y
                rel_alt = self.initial_z - self.current_z
                print("       [{}] t={:.1f}s h={:.3f}m v={:.3f}m pos=({:.2f},{:.2f},{:.2f})".format(
                    name, elapsed, h_dist, v_dist, rel_x, rel_y, rel_alt))

            if h_dist <= h_tol and v_dist <= v_tol:
                if in_tolerance_since is None:
                    in_tolerance_since = now
                    print("       [{}] in tolerance at t={:.1f}s".format(name, elapsed))
                elif now - in_tolerance_since >= stable_time:
                    print("       [{}] REACHED (stable {}s)".format(name, stable_time))
                    return True
            else:
                if in_tolerance_since is not None:
                    print("       [{}] left tolerance (h={:.3f}, v={:.3f})".format(
                        name, h_dist, v_dist))
                    in_tolerance_since = None

            if elapsed > timeout:
                print("\n!!! TIMEOUT {} after {}s, h={:.3f}m v={:.3f}m !!!".format(
                    name, timeout, h_dist, v_dist))
                return False

            await asyncio.sleep(0.1)

    async def go_to(self, x_rel, y_rel, alt, name, speed=0.3):
        """일정 속도로 setpoint 점진 갱신 + 도달 확인"""
        if self.aborted:
            return False

        end_x = self.initial_x + x_rel
        end_y = self.initial_y + y_rel
        end_z = self.initial_z - alt

        start_x = self.target.north_m
        start_y = self.target.east_m
        start_z = self.target.down_m

        dx = end_x - start_x
        dy = end_y - start_y
        dz = end_z - start_z
        distance = math.sqrt(dx*dx + dy*dy + dz*dz)

        if distance < 0.01:
            self.target = PositionNedYaw(end_x, end_y, end_z, 0.0)
            return True

        duration = distance / speed
        steps = max(int(duration * SETPOINT_HZ), 10)

        print("[goto {}] dist={:.2f}m speed={}m/s duration={:.1f}s".format(
            name, distance, speed, duration))

        for i in range(steps):
            if self.aborted:
                return False
            progress = (i + 1) / steps
            x = start_x + dx * progress
            y = start_y + dy * progress
            z = start_z + dz * progress
            self.target = PositionNedYaw(x, y, z, 0.0)
            await asyncio.sleep(1.0 / SETPOINT_HZ)

        self.target = PositionNedYaw(end_x, end_y, end_z, 0.0)

        # [FIX 5] 동적 timeout
        arrival_timeout = max(duration * 0.5 + 5.0, 8.0)
        return await self.wait_until_reached(name,
                                             h_tol=HORIZONTAL_TOLERANCE,
                                             v_tol=VERTICAL_TOLERANCE,
                                             stable_time=STABLE_TIME,
                                             timeout=arrival_timeout)

    async def hover(self, duration, name):
        if self.aborted:
            return
        print("[hover @ {}] for {}s".format(name, duration))
        elapsed = 0.0
        while elapsed < duration:
            if self.aborted:
                return
            await asyncio.sleep(0.5)
            elapsed += 0.5
            rel_alt = self.initial_z - self.current_z
            print("       [hover {}] t={:.1f}s alt={:.2f}".format(name, elapsed, rel_alt))

    async def execute_mission(self):
        self.takeoff_started = True

        ok = await self.go_to(0.0, 0.0, TAKEOFF_ALT, "takeoff")
        if not ok:
            return
        await self.hover(HOVER_TIME, "post_takeoff")
        if self.aborted:
            return

        ok = await self.go_to(FORWARD_DISTANCE, 0.0, TAKEOFF_ALT, "forward_1m")
        if not ok:
            return
        await self.hover(HOVER_TIME, "forward_position")
        if self.aborted:
            return

        ok = await self.go_to(0.0, 0.0, TAKEOFF_ALT, "return_home")
        if not ok:
            return
        await self.hover(HOVER_TIME, "home_hover")

    async def land_and_disarm(self):
        print("[7/7] Landing...")
        if not self.aborted:
            self.streaming = False
            await asyncio.sleep(0.2)
            try:
                await self.drone.action.land()
            except Exception as e:
                print("       Land cmd warning: {}".format(e))

        start = asyncio.get_event_loop().time()
        async for in_air in self.drone.telemetry.in_air():
            elapsed = asyncio.get_event_loop().time() - start
            rel_alt = self.initial_z - self.current_z
            print("       [land] t={:.1f}s in_air={} alt={:.2f}".format(
                elapsed, in_air, rel_alt))
            if not in_air:
                print("       *** LANDED ***")
                break
            if elapsed > LAND_WAIT_TIMEOUT:
                print("       [land] timeout, breaking")
                break
            await asyncio.sleep(0.5)

        await self._cancel_background_tasks()

        print("[disarm]")
        try:
            await self.drone.action.disarm()
            print("       Disarmed.")
        except Exception as e:
            print("       Disarm note: {}".format(e))
        print("Done!")

    async def _cancel_background_tasks(self):
        """[FIX 6] 백그라운드 태스크 종료"""
        self.streaming = False
        for task in [self.stream_task, self.monitor_task, self.vio_monitor_task]:
            if task and not task.done():
                task.cancel()
                try:
                    await asyncio.wait_for(task, timeout=2.0)
                except (asyncio.CancelledError, asyncio.TimeoutError):
                    pass
                except Exception:
                    pass

    async def emergency_stop(self):
        print("\n!!! EMERGENCY STOP !!!")
        await self.trigger_emergency_land("emergency_stop called")
        await asyncio.sleep(0.5)
        try:
            await self.drone.action.kill()
        except Exception:
            pass
        await self._cancel_background_tasks()


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
        await ctrl.check_vio_quality_preflight()
        await ctrl.capture_initial_position()
        await ctrl.arm_and_start_offboard()
        await ctrl.execute_mission()
        await ctrl.land_and_disarm()
    except Exception as e:
        print("\nException: {}".format(e))
        await ctrl.emergency_stop()
        sys.exit(1)


if __name__ == "__main__":
    print("=" * 60)
    print("Starling 2 - PATH FLIGHT (PATCHED v2)")
    print("  Sequence: takeoff -> hover -> forward 1m -> hover -> return -> hover -> land")
    print("  Altitude: {}m, Forward: {}m".format(TAKEOFF_ALT, FORWARD_DISTANCE))
    print("  Tolerance: H={}m V={}m (stable for {}s)".format(
        HORIZONTAL_TOLERANCE, VERTICAL_TOLERANCE, STABLE_TIME))
    print("  Geofence: alt +-{}m, horizontal {}m".format(
        MAX_ALT_DEVIATION, MAX_HORIZONTAL_DEV))
    print("=" * 60)
    print("CHECKLIST:")
    print("  [ ] Propellers attached and tight")
    print("  [ ] CLEAR SPACE: 2m forward, 1m sides")
    print("  [ ] RC ON, OFFBOARD mode, in HAND")
    print("  [ ] Kill switch confirmed")
    print("  [ ] Battery >70%")
    print("  [ ] Drone on FLAT GROUND")
    print("  [ ] *** SHAKE drone 10s before flight to init VIO ***")
    print("  [ ] Verify: voxl-inspect-pose vvhub_body_wrt_local shows updates")
    print()
    input("Type Enter when confirmed...")

    # Python 3.6 호환
    loop = asyncio.get_event_loop()
    loop.run_until_complete(main())