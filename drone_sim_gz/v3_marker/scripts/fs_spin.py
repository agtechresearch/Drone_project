#!/usr/bin/env python3
"""IMU lever-arm check flight (2026-11-02, codex sim-imu-offset-consistency: "compare sensor values with the expected lever-arm
effect in a rotation test"). Runs in starling-flight (python3.6, MAVSDK 0.12) against a running mavsdk_server.
  fs_spin.py --port 50053 [--alt 1.0] [--rate 140] [--slow 40] [--seg 6] [--cycles 12] [--base 6]
arm -> Offboard position hold ALT m up -> phases, each printed as "PHASE <name> <t_start> <t_end>" (host clock):
  base_start -> CYCLES x [fast (yaw setpoint advanced at RATE deg/s) -> slow (SLOW deg/s)] (first half +, second half -) ->
  base_end -> action.land ("LANDED" / "LAND NOT CONFIRMED"). The lever-arm term is -omega^2 r, so fast minus the slow segments
  on both sides of it leaves -(wf^2 - ws^2) r while a linear accelerometer bias drift cancels (2026-11-02 first try with 15-20 s
  hold / spin phases: the gz accelerometer bias moved 0.03-0.04 m/s^2 in 10 s, as large as the expected shift).
The yaw setpoint is streamed at 20 Hz so PX4 tracks a steady yaw rate about the held point; the harness measures the real
rate from the gyro and compares the mean accelerometer shift between spin and base phases with -omega^2 * r_imu."""
import argparse, asyncio, math, sys, time
from mavsdk import System
from mavsdk.offboard import PositionNedYaw


def log(msg):
    print("%.3f %s" % (time.time(), msg), flush=True)


async def first(gen):
    async for v in gen:
        return v


async def phase(d, name, n, e, down, yaw0, rate, dur):
    t0 = time.time()
    while True:
        t = time.time() - t0
        if t >= dur:
            break
        yaw = (yaw0 + rate * t + 180.0) % 360.0 - 180.0
        await d.offboard.set_position_ned(PositionNedYaw(n, e, down, yaw))
        await asyncio.sleep(0.05)
    log("PHASE %s %.3f %.3f" % (name, t0, time.time()))
    return (yaw0 + rate * dur + 180.0) % 360.0 - 180.0


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
    down = pv.position.down_m - a.alt
    log("start n %.3f e %.3f d %.3f yaw %.1f" % (n0, e0, pv.position.down_m, yaw))
    await d.offboard.set_position_ned(PositionNedYaw(n0, e0, down, yaw))
    await d.action.arm()
    log("armed")
    await d.offboard.start()
    t0 = time.time()
    async for p in d.telemetry.position_velocity_ned():
        if p.position.down_m - down < 0.1 and abs(p.velocity.down_m_s) < 0.1:
            log("HOVER_READY n %.3f e %.3f d %.3f" % (p.position.north_m, p.position.east_m, p.position.down_m))
            break
        if time.time() - t0 > 40:
            log("!! no hover after 40 s")
            return 1
    await asyncio.sleep(3)
    yaw = await phase(d, "base_start", n0, e0, down, yaw, 0.0, a.base)
    for k in range(a.cycles):
        sgn = 1 if k < a.cycles // 2 else -1
        yaw = await phase(d, "fast%d" % k, n0, e0, down, yaw, sgn * a.rate, a.seg)
        yaw = await phase(d, "slow%d" % k, n0, e0, down, yaw, sgn * a.slow, a.seg)
    yaw = await phase(d, "base_end", n0, e0, down, yaw, 0.0, a.base)
    log("phases over - normal land")
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
    ap.add_argument("--rate", type=float, default=140.0)
    ap.add_argument("--slow", type=float, default=40.0)
    ap.add_argument("--seg", type=float, default=6.0)
    ap.add_argument("--cycles", type=int, default=12)
    ap.add_argument("--base", type=float, default=6.0)
    sys.exit(asyncio.get_event_loop().run_until_complete(main(ap.parse_args())))
