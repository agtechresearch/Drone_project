#!/usr/bin/env python3
import asyncio
from mavsdk import System

async def main():
    drone = System(mavsdk_server_address="localhost", port=50051)
    await drone.connect()
    async for state in drone.core.connection_state():
        if state.is_connected:
            break
    print("Checking health flags (5 samples)...")
    count = 0
    async for health in drone.telemetry.health():
        print(f"  gyrometer_calibration_ok:    {health.is_gyrometer_calibration_ok}")
        print(f"  accelerometer_calibration_ok:{health.is_accelerometer_calibration_ok}")
        print(f"  magnetometer_calibration_ok: {health.is_magnetometer_calibration_ok}")
        print(f"  local_position_ok:           {health.is_local_position_ok}")
        print(f"  global_position_ok:          {health.is_global_position_ok}")
        print(f"  home_position_ok:            {health.is_home_position_ok}")
        print(f"  armable:                     {health.is_armable}")
        print("---")
        count += 1
        if count >= 3:
            break
        await asyncio.sleep(1)

asyncio.get_event_loop().run_until_complete(main())
