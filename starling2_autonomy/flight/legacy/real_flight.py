#!/usr/bin/env python3
"""
Starling 2 - Conservative flight (takeoff progress check removed).
Other safety: geofence, VIO health, attitude, initial position sanity remain.
"""
import asyncio
import signal
import sys
import math
from mavsdk import System
from mavsdk.offboard import OffboardError, PositionNedYaw

# ==== CONFIG ====
DRY_RUN_LEVEL = 0
TAKEOFF_ALT = 0.5
HOVER_TIME = 3.0
TAKEOFF_DURATION = 6.0
SETPOINT_HZ = 20.0
POS_OK_HOLD_SEC = 10.0
LAND_WAIT_TIMEOUT = 15.0

MAX_ALT_DEVIATION = 0.5
MAX_HORIZONTAL_DEV = 0.5
MAX_INITIAL_DISTANCE = 1.0
MAX_INITIAL_TILT_DEG = 5.0
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
        self.emergency_land_triggered = False
        self.takeoff_started = False

    async def connect(self):
        print("[1/11] Connecting...")
        await self.drone.connect()
        async for state in self.drone.core.connection_state():
            if state.is_connected:
                print("       Connected!")
                return
            await asyncio.sleep(0.1)

    async def wait_position_stable(self, hold_sec=POS_OK_HOLD_SEC, timeout=60.0):
        print("[2/11] Waiting for stable position estimate ({}s hold)...".format(hold_sec))
        ok_start = None
        deadline = asyncio.get_event_loop().time() + timeout
        async for health in self.drone.telemetry.health():
            now = asyncio.get_event_loop().time()
            if now > deadline:
                raise RuntimeError("Position estimate did not stabilize within timeout")
            if health.is_local_position_ok:
                if ok_start is None:
                    ok_start = now
                    print("       Position OK detected, holding for {}s...".format(hold_sec))
                elif now - ok_start >= hold_sec:
                    print("       Position stable. OK.")
                    return
            else:
                if ok_start is not None:
                    print("       Position lost, resetting hold timer")
                ok_start = None
            await asyncio.sleep(0.2)

    async def check_attitude_level(self):
        print("[3/11] Checking attitude is level (max {}deg tilt)...".format(MAX_INITIAL_TILT_DEG))
        sample_count = 0
        max_roll = 0.0
        max_pitch = 0.0
        async for att in self.drone.telemetry.attitude_euler():
            roll = att.roll_deg
            pitch = att.pitch_deg
            max_roll = max(max_roll, abs(roll))
            max_pitch = max(max_pitch, abs(pitch))
            sample_count += 1
            if sample_count >= 5:
                if max_roll > MAX_INITIAL_TILT_DEG or max_pitch > MAX_INITIAL_TILT_DEG:
                    raise RuntimeError(
                        "Drone not level: max roll={:.1f}deg pitch={:.1f}deg (limit {}deg)".format(
                            max_roll, max_pitch, MAX_INITIAL_TILT_DEG))
                print("       Attitude OK: roll<={:.1f}deg pitch<={:.1f}deg".format(max_roll, max_pitch))
                return
            await asyncio.sleep(0.1)

    async def check_vio_quality_preflight(self):
        print("[4/11] Checking VIO stability...")
        for i in range(5):
            async for health in self.drone.telemetry.health():
                if not health.is_local_position_ok:
                    raise RuntimeError("VIO degraded during pre-check")
                print("       Check {}/5: position OK".format(i+1))
                break
            await asyncio.sleep(0.5)
        print("       VIO appears stable.")

    async def capture_initial_position(self):
        print("[5/11] Capturing initial position...")
        async for pos in self.drone.telemetry.position_velocity_ned():
            self.initial_x = pos.position.north_m
            self.initial_y = pos.position.east_m
            self.initial_z = pos.position.down_m
            self.current_x = self.initial_x
            self.current_y = self.initial_y
            self.current_z = self.initial_z
            print("       Initial NED: x={:.3f} y={:.3f} z={:.3f}".format(
                self.initial_x, self.initial_y, self.initial_z))

            distance = math.sqrt(
                self.initial_x**2 + self.initial_y**2 + self.initial_z**2)
            if distance > MAX_INITIAL_DISTANCE:
                raise RuntimeError(
                    "Initial position too far from origin ({:.2f}m > {}m). "
                    "VIO may have drifted. Restart PX4 and try again.".format(
                        distance, MAX_INITIAL_DISTANCE))
            print("       Distance from origin: {:.3f}m. OK.".format(distance))

            self.target = PositionNedYaw(self.initial_x, self.initial_y, self.initial_z, 0.0)
            return

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

                if horizontal_dev > MAX_HORIZONTAL_DEV and not self.emergency_land_triggered:
                    print("\n!!! GEOFENCE: horizontal dev {:.2f}m > {}m !!!".format(
                        horizontal_dev, MAX_HORIZONTAL_DEV))
                    self.emergency_land_triggered = True
                    asyncio.ensure_future(self.trigger_emergency_land())

                if alt_dev > MAX_ALT_DEVIATION and not self.emergency_land_triggered:
                    print("\n!!! GEOFENCE: altitude dev {:.2f}m > {}m (z={:.2f}, tgt={:.2f}) !!!".format(
                        alt_dev, MAX_ALT_DEVIATION, self.current_z, target_z))
                    self.emergency_land_triggered = True
                    asyncio.ensure_future(self.trigger_emergency_land())

            await asyncio.sleep(0.05)

    async def vio_quality_monitor(self):
        consecutive_bad = 0
        async for health in self.drone.telemetry.health():
            if not self.streaming:
                break
            if not health.is_local_position_ok:
                consecutive_bad += 1
                print("\n!!! VIO health degraded ({}/3) !!!".format(consecutive_bad))
                if consecutive_bad >= 3 and not self.emergency_land_triggered:
                    print("!!! VIO LOST - EMERGENCY LAND !!!")
                    self.emergency_land_triggered = True
                    asyncio.ensure_future(self.trigger_emergency_land())
            else:
                consecutive_bad = 0
            await asyncio.sleep(0.3)

    async def trigger_emergency_land(self):
        print("!!! EMERGENCY LAND TRIGGERED !!!")
        self.aborted = True
        # Stop offboard setpoint stream so PX4 LAND can take over cleanly
        self.streaming = False
        await asyncio.sleep(0.2)
        try:
            await self.drone.action.land()
        except Exception as e:
            print("Emergency land failed: {}, trying kill".format(e))
            try:
                await self.drone.action.kill()
            except Exception:
                pass

    async def setpoint_streamer(self):
        period = 1.0 / SETPOINT_HZ
        send_count = 0
        while self.streaming:
            try:
                await self.drone.offboard.set_position_ned(self.target)
                send_count += 1
                if send_count % int(SETPOINT_HZ * 2) == 0:
                    actual_alt = self.initial_z - self.current_z
                    target_alt = self.initial_z - self.target.down_m
                    print("       [streamer] n={} target_alt={:.2f} actual_alt={:.2f}".format(
                        send_count, target_alt, actual_alt))
            except Exception as e:
                print("       [streamer] send failed: {}".format(e))
            await asyncio.sleep(period)

    async def arm_and_start_offboard(self):
        print("[6/11] Priming setpoint stream (3s)...")
        self.streaming = True
        self.stream_task = asyncio.ensure_future(self.setpoint_streamer())
        self.monitor_task = asyncio.ensure_future(self.position_monitor())
        self.vio_monitor_task = asyncio.ensure_future(self.vio_quality_monitor())
        await asyncio.sleep(3.0)

        print("[7/11] Arming...")
        await self.drone.action.arm()
        await asyncio.sleep(1.0)

        print("       *** RC must be in OFFBOARD mode ***")
        await asyncio.sleep(0.5)

        print("       Starting offboard mode...")
        try:
            await self.drone.offboard.start()
            print("       Offboard active!")
        except OffboardError as e:
            print("       Offboard start FAILED: {}".format(e._result.result))
            await self.emergency_stop()
            raise

    async def takeoff_smooth(self):
        print("[8/11] SLOW takeoff to {}m over {}s...".format(TAKEOFF_ALT, TAKEOFF_DURATION))
        self.takeoff_started = True
        steps = int(TAKEOFF_DURATION * SETPOINT_HZ)

        for i in range(steps):
            if self.aborted:
                print("       Takeoff aborted.")
                return
            progress = (i + 1) / steps
            if progress < 0.5:
                eased = 2.0 * progress * progress
            else:
                eased = 1.0 - 2.0 * (1.0 - progress) * (1.0 - progress)
            alt = TAKEOFF_ALT * eased
            self.target = PositionNedYaw(self.initial_x, self.initial_y, self.initial_z - alt, 0.0)
            await asyncio.sleep(1.0 / SETPOINT_HZ)

        self.target = PositionNedYaw(self.initial_x, self.initial_y, self.initial_z - TAKEOFF_ALT, 0.0)
        print("       Reached target {}m".format(TAKEOFF_ALT))

    async def hover(self):
        if self.aborted:
            return
        print("[9/11] Hovering for {}s...".format(HOVER_TIME))
        elapsed = 0.0
        step = 0.5
        while elapsed < HOVER_TIME:
            if self.aborted:
                print("       Hover aborted.")
                return
            await asyncio.sleep(step)
            elapsed += step
            actual_alt = self.initial_z - self.current_z
            print("       [hover] t={:.1f}s alt={:.2f} (target {:.2f})".format(
                elapsed, actual_alt, TAKEOFF_ALT))

    async def land_and_disarm(self):
        print("[10/11] Landing...")
        if not self.aborted:
            try:
                await self.drone.action.land()
            except Exception as e:
                print("       Land cmd warning: {}".format(e))

        start = asyncio.get_event_loop().time()
        async for in_air in self.drone.telemetry.in_air():
            elapsed = asyncio.get_event_loop().time() - start
            actual_alt = self.initial_z - self.current_z
            print("       [land] t={:.1f}s in_air={} alt={:.2f}".format(
                elapsed, in_air, actual_alt))
            if not in_air:
                print("       *** LANDED ***")
                break
            if elapsed > LAND_WAIT_TIMEOUT:
                print("       Land timeout, forcing disarm")
                break
            await asyncio.sleep(0.5)

        self.streaming = False
        for task in [self.stream_task, self.monitor_task, self.vio_monitor_task]:
            if task:
                try:
                    await asyncio.wait_for(task, timeout=1.0)
                except Exception:
                    pass

        print("[11/11] Disarming...")
        try:
            await self.drone.action.disarm()
            print("       Disarmed.")
        except Exception as e:
            print("       Disarm note: {}".format(e))
        print("Done!")

    async def emergency_stop(self):
        print("\n!!! EMERGENCY STOP !!!")
        self.aborted = True
        self.streaming = False
        try:
            await self.drone.action.land()
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
        await ctrl.check_attitude_level()
        await ctrl.check_vio_quality_preflight()
        await ctrl.capture_initial_position()
        await ctrl.arm_and_start_offboard()
        await ctrl.takeoff_smooth()
        await ctrl.hover()
        await ctrl.land_and_disarm()
    except Exception as e:
        print("\nException: {}".format(e))
        await ctrl.emergency_stop()
        sys.exit(1)

if __name__ == "__main__":
    print("=" * 60)
    print("Starling 2 - SAFE FLIGHT v3 (no takeoff progress check)")
    print("  Altitude: {}m".format(TAKEOFF_ALT))
    print("  Hover: {}s".format(HOVER_TIME))
    print("  Takeoff duration: {}s".format(TAKEOFF_DURATION))
    print("  Geofence: alt +-{}m, horizontal {}m".format(MAX_ALT_DEVIATION, MAX_HORIZONTAL_DEV))
    print("=" * 60)
    print("PRE-FLIGHT CHECKLIST:")
    print("  [ ] Propellers attached and tight")
    print("  [ ] Area clear within 1.5m radius")
    print("  [ ] Ceiling height >= 2m")
    print("  [ ] RC ON, IN HAND, OFFBOARD mode")
    print("  [ ] Kill switch position confirmed")
    print("  [ ] Battery >70%")
    print("  [ ] You >=2m from drone")
    print("  [ ] Drone on FLAT surface, near (0,0,0)")
    print("  [ ] PX4 restarted since last flight")
    print()
    print(">>> REAL FLIGHT: 50cm up, 3s hover, land <<<")
    input("Type Enter when ALL above confirmed...")

    asyncio.get_event_loop().run_until_complete(main())