#!/usr/bin/env python3
import asyncio
from mavsdk import System
from datetime import datetime

async def main():
    drone = System(mavsdk_server_address="localhost", port=50051)
    await drone.connect()
    async for state in drone.core.connection_state():
        if state.is_connected:
            break

    # 모든 EKF2 관련 param 백업
    critical_params = [
        "EKF2_EV_CTRL", "EKF2_GPS_CTRL", "EKF2_MAG_TYPE",
        "EKF2_HGT_REF", "EKF2_RNG_CTRL", "EKF2_MIN_RNG",
        "EKF2_RNG_A_HMAX", "EKF2_RNG_PITCH", "EKF2_RNG_DELAY",
        "EKF2_BARO_CTRL", "EKF2_AID_MASK", "EKF2_EV_DELAY",
        "EKF2_EV_NOISE_MD", "EKF2_EV_POS_X", "EKF2_EV_POS_Y", "EKF2_EV_POS_Z",
        "COM_OF_LOSS_T", "COM_RC_LOSS_T", "COM_DISARM_LAND",
        "MPC_LAND_SPEED", "MPC_TKO_SPEED", "MPC_XY_VEL_MAX", "MPC_Z_VEL_MAX_DN",
        "MPC_Z_P", "MPC_Z_VEL_P_ACC", "MPC_Z_VEL_I_ACC",
        "SYS_HAS_GPS", "SYS_HAS_MAG",
    ]

    backup_file = "/tmp/px4_params_backup_{}.txt".format(
        datetime.now().strftime("%Y%m%d_%H%M%S"))
    
    print("Backing up to: {}".format(backup_file))
    with open(backup_file, "w") as f:
        f.write("# PX4 Param Backup - {}\n".format(datetime.now()))
        for param in critical_params:
            for attempt in ["int", "float"]:
                try:
                    if attempt == "int":
                        val = await drone.param.get_param_int(param)
                        f.write("{}={} # int\n".format(param, val))
                        print("  {} = {} (int)".format(param, val))
                    else:
                        val = await drone.param.get_param_float(param)
                        f.write("{}={} # float\n".format(param, val))
                        print("  {} = {} (float)".format(param, val))
                    break
                except Exception:
                    if attempt == "float":
                        f.write("# {}=ERROR\n".format(param))
                        print("  {} = ERROR".format(param))
                    continue
    print("Backup complete: {}".format(backup_file))

asyncio.get_event_loop().run_until_complete(main())
