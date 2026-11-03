"""Timeline of a SITL mission ulog: PX4 messages, nav_state/failsafe transitions, failsafe_flags that became true,
failure detector, attitude extremes, position vs setpoint. Usage: python3 ulog_mission_check.py <file.ulg> [t_from t_to]"""
import sys, math, numpy as np
from pyulog import ULog
u = ULog(sys.argv[1])
t_from = float(sys.argv[2]) if len(sys.argv) > 2 else -1e9
t_to = float(sys.argv[3]) if len(sys.argv) > 3 else 1e9
def topic(name, inst=0):
    for d in u.data_list:
        if d.name == name and d.multi_id == inst: return d
    return None
def ts(d): return d.data['timestamp'] / 1e6
P = u.initial_parameters
print("== params ==")
for k in ['COM_RC_IN_MODE','COM_RCL_EXCEPT','NAV_RCL_ACT','COM_OF_LOSS_T','COM_OBL_RC_ACT','COM_OBL_ACT','FD_FAIL_R','FD_FAIL_P','FD_FAIL_R_TTRI','CBRK_FLIGHTTERM',
          'EKF2_EV_CTRL','EKF2_MAG_TYPE','EKF2_HGT_REF','EKF2_EVP_NOISE','EKF2_EVA_NOISE','EKF2_EV_DELAY','EKF2_EV_NOISE_MD',
          'MC_ROLL_P','MC_ROLLRATE_P','MC_ROLLRATE_I','MC_ROLLRATE_D','MC_ROLLRATE_K','MC_PITCHRATE_P','MC_PITCHRATE_D','MC_YAW_P','MC_YAWRATE_P','MC_YAWRATE_I','MC_YAWRATE_MAX','MPC_YAWRAUTO_MAX',
          'MPC_XY_VEL_MAX','MPC_XY_CRUISE','MPC_ACC_HOR','MPC_ACC_HOR_MAX','MPC_XY_P','MPC_XY_VEL_P_ACC','MPC_XY_VEL_I_ACC','MPC_XY_VEL_D_ACC','MPC_THR_HOVER','MPC_TILTMAX_AIR','MPC_MAN_TILT_MAX',
          'MC_AT_EN','MC_AT_START','MC_AT_APPLY','MC_AT_SYSID_AMP','CA_ROTOR0_KM','IMU_GYRO_CUTOFF','IMU_DGYRO_CUTOFF','SIM_GZ_EC_MIN1']:
    if k in P: print("  %-18s %s" % (k, P[k]))
arm = topic('actuator_armed'); t0 = 0.0
if arm is not None and (arm.data['armed'] > 0).any(): t0 = ts(arm)[np.argmax(arm.data['armed'] > 0)]
print("\narmed at t0=%.2f s (times below are relative to t0)" % t0)
print("\n== log messages ==")
for m in u.logged_messages:
    t = m.timestamp / 1e6 - t0
    if t_from <= t <= t_to: print("  %+8.2f  [%d] %s" % (t, m.log_level, m.message))
print("\n== vehicle_status transitions ==")
vs = topic('vehicle_status')
if vs is not None:
    prev = None
    for i, t in enumerate(ts(vs)):
        cur = (int(vs.data['nav_state'][i]), int(vs.data['arming_state'][i]), int(vs.data['failsafe'][i]), int(vs.data.get('failure_detector_status', np.zeros(len(ts(vs))))[i]))
        if cur != prev and t_from <= t - t0 <= t_to:
            print("  %+8.2f  nav_state=%d arming=%d failsafe=%d fd_status=%d" % (t - t0, *cur)); prev = cur
print("\n== failsafe_flags that were ever true (with first/last time) ==")
ff = topic('failsafe_flags')
if ff is not None:
    tt = ts(ff) - t0
    for k, v in ff.data.items():
        if k == 'timestamp' or v.dtype.kind not in 'biu': continue
        on = v.astype(bool)
        if on.any() and (~on).any():
            idx = np.where(on)[0]; print("  %-32s true %d/%d samples, first %+8.2f last %+8.2f" % (k, on.sum(), len(on), tt[idx[0]], tt[idx[-1]]))
        elif on.all(): print("  %-32s ALWAYS true" % k)
fd = topic('failure_detector_status')
if fd is not None:
    print("\n== failure_detector_status transitions ==")
    tt = ts(fd) - t0; prev = None
    for i in range(len(tt)):
        cur = tuple(int(fd.data[k][i]) for k in ['fd_roll','fd_pitch','fd_alt','fd_ext','fd_arm_escs','fd_battery','fd_imbalanced_prop','fd_motor'] if k in fd.data)
        if cur != prev and t_from <= tt[i] <= t_to: print("  %+8.2f  roll,pitch,alt,ext,escs,batt,imbal,motor = %s" % (tt[i], cur)); prev = cur
def euler(q0, q1, q2, q3):
    roll = np.degrees(np.arctan2(2*(q0*q1+q2*q3), 1-2*(q1*q1+q2*q2)))
    pitch = np.degrees(np.arcsin(np.clip(2*(q0*q2-q3*q1), -1, 1)))
    yaw = np.degrees(np.arctan2(2*(q0*q3+q1*q2), 1-2*(q2*q2+q3*q3)))
    return roll, pitch, yaw
att = topic('vehicle_attitude'); gt = topic('vehicle_attitude_groundtruth')
lp = topic('vehicle_local_position'); lpgt = topic('vehicle_local_position_groundtruth')
sp = topic('trajectory_setpoint'); asp = topic('vehicle_attitude_setpoint'); mot = topic('actuator_motors')
def at(t, d, key):
    if d is None or key not in d.data: return float('nan')
    return float(np.interp(t, ts(d) - t0, d.data[key]))
def eul_at(t, d):
    if d is None: return (float('nan'),)*3
    q = [at(t, d, 'q[%d]' % i) for i in range(4)]
    r, p, y = euler(*q); return float(r), float(p), float(y)
print("\n== timeline (0.5 s) : EKF pos / truth pos / traj sp / EKF rpy / truth rpy / att_sp yaw / motors ==")
tt_end = min(t_to, ts(lp)[-1] - t0) if lp is not None else t_to
t = max(t_from, 0.0)
print("   t      ekf x,y,z         truth x,y,z        sp x,y,z,yaw           ekf r,p,y        truth r,p,y    sp_yaw   motors")
while t <= tt_end:
    e = [at(t, lp, k) for k in 'xyz']; g = [at(t, lpgt, k) for k in 'xyz']
    s = [at(t, sp, 'position[%d]' % i) for i in range(3)] + [math.degrees(at(t, sp, 'yaw'))]
    er = eul_at(t, att); gr = eul_at(t, gt)
    sy = math.degrees(at(t, asp, 'yaw_body'))
    m = [at(t, mot, 'control[%d]' % i) for i in range(4)]
    print("  %6.1f  %+6.2f %+6.2f %+6.2f   %+6.2f %+6.2f %+6.2f   %+6.2f %+6.2f %+6.2f %+7.1f   %+6.1f %+6.1f %+7.1f   %+6.1f %+6.1f %+7.1f  %+7.1f   %.2f %.2f %.2f %.2f" % (
        t, *e, *g, *s, *er, *gr, sy, *m))
    t += 0.5
