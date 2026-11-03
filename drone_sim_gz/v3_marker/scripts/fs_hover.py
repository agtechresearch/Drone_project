#!/usr/bin/env python3
"""Offboard-loss / RC failsafe SITL test: the flying side (2026-11-01, user: test COM_OBL_RC_ACT 4, COM_FAIL_ACT_T 3,
NAV_RCL_ACT 3 at low altitude, RC first). Runs in starling-flight (python3.6, MAVSDK 0.12) against a running mavsdk_server.
  fs_hover.py --port 50053 [--alt 1.0] [--hold 90] [--north 0]
arm -> Offboard position hold ALT m above the start point (then NORTH m north of it, away from home) -> prints "HOVER_READY" -> keeps the setpoint (mavsdk_server
re-sends it on its own) until it is killed or --hold s pass -> action.land (normal end, "LANDED" / "LAND NOT CONFIRMED").
The harness (logs/run_fs_rc_sitl.sh) kills this process and/or mavsdk_server to make PX4 see an Offboard loss."""
import argparse, asyncio, sys, time
from mavsdk import System
from mavsdk.offboard import PositionNedYaw


def log(msg):
    print("%.3f %s" % (time.time(), msg), flush=True)


async def first(gen):
    async for v in gen:
        return v


async def main(a):
    d = System(mavsdk_server_address="localhost", port=a.port)
    await d.connect()
    async for s in d.core.connection_state():
        if s.is_connected:
            break
    log("connected")
    async for h in d.telemetry.health():
        if h.is_local_position_ok and h.is_home_position_ok:
            break
    pv = await first(d.telemetry.position_velocity_ned())
    att = await first(d.telemetry.attitude_euler())
    n0, e0, yaw = pv.position.north_m, pv.position.east_m, att.yaw_deg
    log("start n %.3f e %.3f d %.3f yaw %.1f" % (n0, e0, pv.position.down_m, yaw))
    await d.offboard.set_position_ned(PositionNedYaw(n0, e0, pv.position.down_m - a.alt, yaw))
    await d.action.arm()
    log("armed")
    await d.offboard.start()
    log("offboard started, target %.2f m up" % a.alt)
    t0 = time.time()
    up = False
    async for p in d.telemetry.position_velocity_ned():
        if not up and pv.position.down_m - p.position.down_m >= a.alt - 0.1:
            up = True
            if a.north:
                await d.offboard.set_position_ned(PositionNedYaw(n0 + a.north, e0, pv.position.down_m - a.alt, yaw))
                log("at height, moving %.1f m north" % a.north)
        if up and abs(p.position.north_m - (n0 + a.north)) < 0.15 and abs(p.velocity.north_m_s) < 0.1:
            log("HOVER_READY n %.3f e %.3f d %.3f" % (p.position.north_m, p.position.east_m, p.position.down_m))
            break
        if time.time() - t0 > 40:
            log("!! no hover after 40 s")
            return 1
    await asyncio.sleep(a.hold)
    log("hold over - normal land")
    await d.action.land()
    t1 = time.time()
    async for air in d.telemetry.in_air():
        if not air:
            log("LANDED")
            return 0
        if time.time() - t1 > 30:
            log("LAND NOT CONFIRMED")
            return 1


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=50053)
    ap.add_argument("--alt", type=float, default=1.0)
    ap.add_argument("--hold", type=float, default=90.0)
    ap.add_argument("--north", type=float, default=0.0)
    sys.exit(asyncio.get_event_loop().run_until_complete(main(ap.parse_args())))
