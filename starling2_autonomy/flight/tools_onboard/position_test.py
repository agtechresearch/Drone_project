#!/usr/bin/env python3

import asyncio
from mavsdk import System


async def main():
    drone = System(mavsdk_server_address="localhost", port=50051)

    print("Connecting...")
    await drone.connect()

    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Connected!")
            break

    print("Reading position_velocity_ned...")
    print("Move the drone by hand: forward/back/left/right/up/down.")
    print("Check whether N/E/D values change.\n")

    async for pos in drone.telemetry.position_velocity_ned():
        n = pos.position.north_m
        e = pos.position.east_m
        d = pos.position.down_m

        vn = pos.velocity.north_m_s
        ve = pos.velocity.east_m_s
        vd = pos.velocity.down_m_s

        print(
            "NED pos: N={:.3f}, E={:.3f}, D={:.3f} | "
            "vel: VN={:.3f}, VE={:.3f}, VD={:.3f}".format(
                n, e, d, vn, ve, vd
            )
        )

        await asyncio.sleep(0.2)


if __name__ == "__main__":
    loop = asyncio.get_event_loop()
    loop.run_until_complete(main())