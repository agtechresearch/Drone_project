#!/usr/bin/env python3
# MAVSDK 0.12 (python3.6) takeoff/land check - the same path the real flight code uses
import asyncio, time
from mavsdk import System
async def main():
    d = System(); await d.connect(system_address="udp://:14540")
    async for s in d.core.connection_state():
        if s.is_connected: print("connected"); break
    async for h in d.telemetry.health():
        print("health: gyro=%s accel=%s mag=%s local=%s global=%s home=%s" % (h.is_gyrometer_calibration_ok, h.is_accelerometer_calibration_ok, h.is_magnetometer_calibration_ok, h.is_local_position_ok, h.is_global_position_ok, h.is_home_position_ok))
        if h.is_local_position_ok and h.is_home_position_ok: break
    await d.action.set_takeoff_altitude(1.5)
    print("arm"); await d.action.arm()
    print("takeoff"); await d.action.takeoff()
    t0 = time.time(); peak = -9
    async for p in d.telemetry.position():
        peak = max(peak, p.relative_altitude_m)
        if int((time.time()-t0)*10) % 20 == 0: print("  t=%4.1f rel_alt=%.2f" % (time.time()-t0, p.relative_altitude_m))
        if time.time()-t0 > 15: break
    print("peak rel_alt %.2f" % peak)
    print("land"); await d.action.land()
    async for a in d.telemetry.armed():
        if not a: print("disarmed"); break
    print("RESULT:", "PASS" if peak > 1.0 else "FAIL")
asyncio.get_event_loop().run_until_complete(main())
