#!/usr/bin/env python3
"""v14 _geofence_check 단위 테스트.

mavsdk가 로컬에 없으므로 sys.modules에 스텁을 심고 모듈을 import한다.
그 다음 DroneController의 _geofence_check만 떼어내 가짜 상태에 바인딩해 검증한다.
"""
import sys, types, math, importlib.util, argparse, pathlib

# ---- mavsdk 스텁 ----
class PositionNedYaw(object):
    def __init__(self, n, e, d, yaw=0.0):
        self.north_m, self.east_m, self.down_m, self.yaw_deg = n, e, d, yaw

mavsdk = types.ModuleType("mavsdk")
mavsdk.System = object
offboard = types.ModuleType("mavsdk.offboard")
offboard.OffboardError = type("OffboardError", (Exception,), {})
offboard.PositionNedYaw = PositionNedYaw
mavsdk.offboard = offboard
sys.modules["mavsdk"] = mavsdk
sys.modules["mavsdk.offboard"] = offboard

REPO = pathlib.Path(__file__).resolve().parent.parent
SRC = REPO / "flight" / "path_flight_phase1_v14.py"
spec = importlib.util.spec_from_file_location("v14", SRC)
v14 = importlib.util.module_from_spec(spec)
spec.loader.exec_module(v14)

# ---- 가짜 시계 ----
class FakeLoop(object):
    t = 0.0
    def time(self): return FakeLoop.t
v14.asyncio.get_event_loop = lambda: FakeLoop()

DEFAULTS = dict(
    no_geofence=False, max_horizontal_dev=2.5, max_alt_deviation=1.5,
    geofence_violation_sec=1.0, geofence_jump_thresh=0.5,
    geofence_jump_timeout=30.0, max_home_dist=0.0, max_alt_agl=0.0,
)

class Ctl(object):
    """_geofence_check가 실제로 건드리는 필드만 갖춘 최소 스텁."""
    def __init__(self, **over):
        a = dict(DEFAULTS); a.update(over)
        self.args = argparse.Namespace(**a)
        self.geofence_armed = True
        self._gf_violation_since = None
        self._gf_last_target = None
        self._gf_jump_pending = False
        self._gf_jump_since = 0.0
        self.target = PositionNedYaw(0.0, 0.0, -1.0)
        self.mav_x = self.mav_y = 0.0
        self.mav_z = -1.0
        self.initial_x = self.initial_y = self.initial_z = 0.0
    check = v14.DroneController._geofence_check

def at(t): FakeLoop.t = t

results = []
def ok(name, cond):
    results.append((name, cond))
    print(("  PASS  " if cond else "  FAIL  ") + name)

print("\n=== 1. 기본 동작 ===")
at(0.0); c = Ctl(); ok("펜스 안 -> None", c.check() is None)

c = Ctl(); c.geofence_armed = False; c.mav_x = 99.0
ok("armed=False면 위반이어도 None", c.check() is None)

c = Ctl(no_geofence=True); c.mav_x = 99.0
ok("--no-geofence면 None", c.check() is None)

print("\n=== 2. 추종오차 수평 위반 + 지속시간 ===")
at(10.0); c = Ctl(); c.mav_x = 3.0           # target 0 -> 오차 3.0m > 2.5m
ok("첫 위반 샘플은 경고만, 발동 안함", c.check() is None)
at(10.5); ok("0.5s 경과 -> 아직 발동 안함", c.check() is None)
at(11.1); r = c.check()
ok("1.0s 초과 -> 발동", r is not None and "horizontal" in r)
print("        reason: %s" % r)

at(20.0); c = Ctl(); c.mav_x = 3.0
c.check()
at(20.4); c.mav_x = 0.0                       # 펜스 안으로 복귀
ok("복귀하면 None", c.check() is None)
ok("복귀 시 위반타이머 리셋", c._gf_violation_since is None)
at(20.5); c.mav_x = 3.0; c.check()
at(21.0); ok("재위반은 타이머 새로 시작", c.check() is None)

print("\n=== 3. 고도 위반 ===")
at(30.0); c = Ctl(); c.mav_z = -3.0            # target -1.0 -> 오차 2.0m > 1.5m
c.check(); at(31.1); r = c.check()
ok("고도 오차 발동", r is not None and "altitude" in r)
print("        reason: %s" % r)

print("\n=== 4. ramp 이동은 점프로 오인하지 않아야 함 ===")
at(40.0); c = Ctl(); c.check()
jumped = False
for i in range(1, 41):                          # 0.5m/s * 0.05s = 0.025m/샘플
    at(40.0 + i * 0.05)
    c.target = PositionNedYaw(i * 0.025, 0.0, -1.0)
    c.mav_x = i * 0.025                          # 잘 따라가는 중
    c.check()
    if c._gf_jump_pending: jumped = True
ok("ramp 중 jump_pending 발생 안함", not jumped)
ok("ramp 중 미발동", c.check() is None)

print("\n=== 5. setpoint 즉시 점프 -> 유예 ===")
at(50.0); c = Ctl(); c.check()
at(50.05); c.target = PositionNedYaw(10.0, 0.0, -1.0)   # 10m 즉시 점프
r = c.check()
ok("점프 직후 미발동", r is None)
ok("jump_pending 설정됨", c._gf_jump_pending)
at(55.0); c.mav_x = 4.0                          # 아직 6m 뒤처짐 = 오차 6m > 2.5m
ok("유예 중이면 큰 추종오차도 미발동", c.check() is None)
at(60.0); c.mav_x = 9.5                          # 오차 0.5m -> 수렴
ok("수렴하면 None", c.check() is None)
ok("수렴 후 jump_pending 해제", not c._gf_jump_pending)
at(60.1); c.mav_x = 4.0                          # 수렴 후 다시 폭주
c.check(); at(61.2); r = c.check()
ok("수렴 후 재폭주는 정상 발동", r is not None)
print("        reason: %s" % r)

print("\n=== 6. 점프 후 수렴 실패 -> 타임아웃 발동 ===")
at(70.0); c = Ctl(); c.check()
at(70.05); c.target = PositionNedYaw(10.0, 0.0, -1.0)
c.check()
at(95.0); ok("타임아웃 전 미발동", c.check() is None)
at(101.0); r = c.check()
ok("30s 초과 -> 발동", r is not None and "converge" in r)
print("        reason: %s" % r)

print("\n=== 7. 홈 기준 절대 펜스 ===")
at(110.0); c = Ctl(max_home_dist=15.0)
c.target = PositionNedYaw(10.0, 0.0, -1.0); c.mav_x = 10.0   # 추종은 완벽
ok("홈 10m, 한계 15m -> 통과", c.check() is None)
c.target = PositionNedYaw(20.0, 0.0, -1.0); c.mav_x = 20.0
c.check(); at(111.5); r = c.check()
ok("홈 20m > 15m -> 발동", r is not None and "home distance" in r)
print("        reason: %s" % r)

at(120.0); c = Ctl()                              # max_home_dist=0 (기본)
c.target = PositionNedYaw(50.0, 0.0, -1.0); c.mav_x = 50.0
ok("max_home_dist=0이면 아무리 멀어도 통과", c.check() is None)

print("\n=== 8. AGL 절대 펜스 ===")
at(130.0); c = Ctl(max_alt_agl=3.0)
c.target = PositionNedYaw(0.0, 0.0, -5.0); c.mav_z = -5.0     # 추종 완벽, 5m 상승
c.check(); at(131.5); r = c.check()
ok("AGL 5m > 3m -> 발동", r is not None and "AGL" in r)
print("        reason: %s" % r)

print("\n=== 9. 실제 MISSION_PLAN 시나리오 회귀 검증 ===")
# CR식 홈 반경 2.5m 펜스였다면 여기서 오발동했어야 한다.
at(200.0); c = Ctl(); c.check()
t = 200.0
tripped = None
for leg in [1.35, 10.0, -9.5]:                  # right 1.35 -> right 10.0 -> left 9.5
    steps = int(abs(leg) / 0.025)
    base = c.target.north_m
    for i in range(1, steps + 1):
        t += 0.05; at(t)
        pos = base + (leg / abs(leg)) * i * 0.025
        c.target = PositionNedYaw(pos, 0.0, -1.0)
        c.mav_x = pos - (leg / abs(leg)) * 0.15   # 15cm 정도 뒤처져 따라감
        r = c.check()
        if r and tripped is None: tripped = r
ok("홈에서 11m+ 이동해도 오발동 없음", tripped is None)
if tripped: print("        (오발동) %s" % tripped)

print("\n" + "=" * 55)
passed = sum(1 for _, v in results if v)
print("결과: %d / %d 통과" % (passed, len(results)))
if passed != len(results):
    print("실패 항목:")
    for n, v in results:
        if not v: print("  - " + n)
sys.exit(0 if passed == len(results) else 1)
