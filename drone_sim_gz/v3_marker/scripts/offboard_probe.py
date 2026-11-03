#!/usr/bin/env python3
# Diagnose estimation vs control: hold offboard setpoints and log EKF NED (MAVSDK) at 2 Hz; ground truth is sampled externally.
import asyncio, time, sys
from mavsdk import System
from mavsdk.offboard import PositionNedYaw, OffboardError
async def main():
    d = System(mavsdk_server_address="localhost", port=50051); await d.connect()
    async for s in d.core.connection_state():
        if s.is_connected: break
    async for h in d.telemetry.health():
        print("health local=%s global=%s home=%s" % (h.is_local_position_ok, h.is_global_position_ok, h.is_home_position_ok)); sys.stdout.flush()
        if h.is_local_position_ok: break
    await asyncio.sleep(3)
    pv0 = None
    async for pv in d.telemetry.position_velocity_ned(): pv0 = pv; break
    n0,e0,d0 = pv0.position.north_m, pv0.position.east_m, pv0.position.down_m
    print("EKF start N=%.2f E=%.2f D=%.2f" % (n0,e0,d0)); sys.stdout.flush()
    await d.action.arm()
    sp = PositionNedYaw(n0, e0, d0, 0.0); await d.offboard.set_position_ned(sp)
    try: await d.offboard.start()
    except OffboardError as ex: print("offboard start failed", ex); return
    plan = [("hold 0.5m", n0, e0, d0-0.5, 20), ("north 2m", n0+2.0, e0, d0-0.5, 20), ("back", n0, e0, d0-0.5, 20), ("hold 1.0m", n0, e0, d0-1.0, 15)]
    t_start=time.time()
    for name,n,e,dd,dur in plan:
        await d.offboard.set_position_ned(PositionNedYaw(n,e,dd,0.0)); t0=time.time()
        print("== %s  (t=%.0f) target rel N=%.2f E=%.2f U=%.2f" % (name, t0-t_start, n-n0, e-e0, -(dd-d0))); sys.stdout.flush()
        while time.time()-t0 < dur:
            async for pv in d.telemetry.position_velocity_ned():
                print("  t=%5.1f %s EKF rel N=%+.2f E=%+.2f U=%+.2f" % (time.time()-t_start, time.strftime("%M:%S"), pv.position.north_m-n0, pv.position.east_m-e0, -(pv.position.down_m-d0))); sys.stdout.flush(); break
            await asyncio.sleep(2.0)
    await d.offboard.set_position_ned(PositionNedYaw(n0, e0, d0-0.3, 0.0)); await asyncio.sleep(6)
    await d.offboard.stop(); await d.action.land()
    async for a in d.telemetry.armed():
        if not a: print("disarmed"); break
asyncio.get_event_loop().run_until_complete(main())
