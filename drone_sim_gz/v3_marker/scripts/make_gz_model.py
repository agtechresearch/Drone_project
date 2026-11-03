"""Generate the new-Gazebo (gz Harmonic) starling2 model + qvix world + airframe into repo_export/gz/.
Visual meshes come from repo_export/airframe/starling2/meshes (CAD-derived, gz frame x fwd / y left / z up, metres)."""
import os, shutil, re
RE=os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PX4=os.path.expanduser("~/starling_sim/PX4-Autopilot")
out=f"{RE}/gz/models/starling2"; os.makedirs(f"{out}/meshes",exist_ok=True)
for f in ("starling2_body.stl","starling2_prop_a.stl","starling2_prop_b.stl"): shutil.copy(f"{RE}/airframe/starling2/meshes/{f}", f"{out}/meshes/{f}")

# sensors copied verbatim from PX4 x500_base so PX4's gz bridge finds the same sensor names
x500=open(f"{PX4}/Tools/simulation/gz/models/x500_base/model.sdf").read()
sensors=re.search(r'(\s*<sensor name="air_pressure_sensor".*?</sensor>\s*<sensor name="navsat_sensor".*?</sensor>)', x500, re.S).group(1)

# --- geometry from CAD (m) ---
MASS=0.285; IXX,IYY,IZZ=0.001103,0.000742,0.001502          # box estimate 0.155 x 0.198 x 0.085
ROT={0:( 0.066,-0.088,'a','ccw'),1:(-0.066, 0.088,'a','ccw'),2:( 0.066, 0.088,'b','cw'),3:(-0.066,-0.088,'b','cw')}   # PX4 order: FR, RL, FL, RR
ROT_Z=0.0174; SPAWN_Z=0.035                                    # legs bottom at -0.0296
# --- motor model: 1504 3000KV + 120 mm prop. Scaled 2026-10-27 to the real vehicle's hover point (handoff_2026-10-27):
#     MPC_THR_HOVER 0.34 with THR_MDL_FAC 0.9 -> hover command u = 0.5616 -> omega = 150 + 850*u = 627 rad/s must give m*g/4 = 0.699 N
#     -> motorConstant = 0.699 / 627^2 = 1.78e-6 (max 1.78 N/motor at 1000 rad/s, T/W 2.5; real estimate 2.0-2.4 N).
MOTOR=dict(maxRotVelocity=1000.0, motorConstant=1.78e-06, momentConstant=0.016, rotorDragCoefficient=1.5e-05, rollingMomentCoefficient=1e-06)

# front camera (added 2026-10-27): stands in for the Starling 2 front stereo/tracking cluster. FOV 68 x ~53 deg (voxl-vision-hub
# stereo_front x_fov 68), 640x480 @ 15 Hz, topic /front_camera. The CameraVideoRecorder plugin exposes /front_camera/record_video
# (gz.msgs.VideoRecord start/stop) so grid_fly.sh RECORD_CAM=1 can save what the camera sees to logs/grid_<ts>_cam.mp4.
CAMERA="""
      <sensor name="front_camera" type="camera">
        <pose>0.075 0 0.0 0 0 0</pose>
        <camera>
          <horizontal_fov>1.187</horizontal_fov>
          <image><width>640</width><height>480</height></image>
          <clip><near>0.05</near><far>50</far></clip>
          <visibility_mask>1</visibility_mask>
        </camera>
        <always_on>1</always_on>
        <update_rate>15</update_rate>
        <visualize>false</visualize>
        <topic>front_camera</topic>
        <plugin filename="gz-sim-camera-video-recorder-system" name="gz::sim::systems::CameraVideoRecorder">
          <service>front_camera/record_video</service>
          <use_sim_time>true</use_sim_time>
          <fps>15</fps>
        </plugin>
      </sensor>
"""

def collision(name,pose,geom):
    return f"""      <collision name="{name}">
        <pose>{pose}</pose>
        <geometry>{geom}</geometry>
        <surface><contact><ode><min_depth>0.001</min_depth><max_vel>0</max_vel></ode></contact><friction><ode/></friction></surface>
      </collision>
"""
rotor_links=""; plugins=""
for i,(x,y,ab,dirn) in ROT.items():
    rotor_links+=f"""    <link name="rotor_{i}">
      <gravity>true</gravity>
      <self_collide>false</self_collide>
      <velocity_decay/>
      <pose>{x} {y} {ROT_Z} 0 0 0</pose>
      <inertial>
        <mass>0.002</mass>
        <inertia><ixx>2e-07</ixx><ixy>0</ixy><ixz>0</ixz><iyy>2.4e-06</iyy><iyz>0</iyz><izz>2.5e-06</izz></inertia>
      </inertial>
      <visual name="rotor_{i}_visual">
        <pose>0 0 0 0 0 0</pose>
        <geometry><mesh><scale>1 1 1</scale><uri>model://starling2/meshes/starling2_prop_{ab}.stl</uri></mesh></geometry>
        <material><ambient>0.25 0.25 0.25 1</ambient><diffuse>0.35 0.35 0.35 1</diffuse><specular>0.2 0.2 0.2 1</specular></material>
      </visual>
{collision(f"rotor_{i}_collision","0 0 0 0 0 0","<cylinder><radius>0.06</radius><length>0.006</length></cylinder>")}    </link>
    <joint name="rotor_{i}_joint" type="revolute">
      <parent>base_link</parent>
      <child>rotor_{i}</child>
      <axis>
        <xyz>0 0 1</xyz>
        <limit><lower>-1e+16</lower><upper>1e+16</upper></limit>
        <dynamics><spring_reference>0</spring_reference><spring_stiffness>0</spring_stiffness></dynamics>
      </axis>
    </joint>
"""
    plugins+=f"""    <plugin filename="gz-sim-multicopter-motor-model-system" name="gz::sim::systems::MulticopterMotorModel">
      <jointName>rotor_{i}_joint</jointName>
      <linkName>rotor_{i}</linkName>
      <turningDirection>{dirn}</turningDirection>
      <timeConstantUp>0.0125</timeConstantUp>
      <timeConstantDown>0.025</timeConstantDown>
      <maxRotVelocity>{MOTOR['maxRotVelocity']}</maxRotVelocity>
      <motorConstant>{MOTOR['motorConstant']}</motorConstant>
      <momentConstant>{MOTOR['momentConstant']}</momentConstant>
      <commandSubTopic>command/motor_speed</commandSubTopic>
      <motorNumber>{i}</motorNumber>
      <rotorDragCoefficient>{MOTOR['rotorDragCoefficient']}</rotorDragCoefficient>
      <rollingMomentCoefficient>{MOTOR['rollingMomentCoefficient']}</rollingMomentCoefficient>
      <rotorVelocitySlowdownSim>10</rotorVelocitySlowdownSim>
      <motorType>velocity</motorType>
    </plugin>
"""
sdf=f"""<?xml version="1.0" encoding="UTF-8"?>
<!-- Generated by repo_export/scripts/make_gz_model.py — edit the script, not this file.
     ModalAI Starling 2 (C26) for PX4 SITL on new Gazebo (gz Harmonic). Meshes from ModalAI D0014 Starling 2-C26 STEP. -->
<sdf version='1.9'>
  <model name='starling2'>
    <pose>0 0 {SPAWN_Z} 0 0 0</pose>
    <self_collide>false</self_collide>
    <static>false</static>
    <link name="base_link">
      <inertial>
        <mass>{MASS}</mass>
        <inertia><ixx>{IXX}</ixx><ixy>0</ixy><ixz>0</ixz><iyy>{IYY}</iyy><iyz>0</iyz><izz>{IZZ}</izz></inertia>
      </inertial>
      <gravity>true</gravity>
      <velocity_decay/>
      <visual name="base_link_visual">
        <pose>0 0 0 0 0 0</pose>
        <geometry><mesh><scale>1 1 1</scale><uri>model://starling2/meshes/starling2_body.stl</uri></mesh></geometry>
        <material><ambient>0.08 0.08 0.08 1</ambient><diffuse>0.12 0.12 0.12 1</diffuse><specular>0.3 0.3 0.3 1</specular></material>
      </visual>
      <!-- GUI highlight (2026-10-28): translucent orange halo so the small dark vehicle stands out in overview recordings.
           visibility_flags 0x2 + the front camera's visibility_mask 0x1 -> the vehicle's own camera never sees it. -->
      <visual name="gui_halo">
        <pose>0 0 0.01 0 0 0</pose>
        <geometry><sphere><radius>0.17</radius></sphere></geometry>
        <material><ambient>1 0.5 0 0.28</ambient><diffuse>1 0.5 0 0.28</diffuse><emissive>0.6 0.3 0 0.28</emissive></material>
        <transparency>0.72</transparency>
        <cast_shadows>false</cast_shadows>
        <visibility_flags>2</visibility_flags>
      </visual>
{collision("base_link_collision","0 0 0.0125 0 0 0","<box><size>0.155 0.198 0.085</size></box>")}{sensors}{CAMERA}
    </link>
{rotor_links}{plugins}    <!-- ground-truth odometry -> PX4 vehicle_visual_odometry (stands in for VOXL VIO; gz_bridge subscribes to /model/<name>/odometry_with_covariance) -->
    <plugin filename="gz-sim-odometry-publisher-system" name="gz::sim::systems::OdometryPublisher">
      <dimensions>3</dimensions>
      <odom_publish_frequency>50</odom_publish_frequency>
    </plugin>
  </model>
</sdf>
"""
open(f"{out}/model.sdf","w").write(sdf)
open(f"{out}/model.config","w").write("""<?xml version="1.0"?>
<model>
  <name>starling2</name>
  <version>1.0</version>
  <sdf version="1.9">model.sdf</sdf>
  <author><name>Sejong agtechlab</name></author>
  <description>ModalAI Starling 2 (C26, HW Rev1) quadrotor for PX4 SITL on gz Harmonic. Visual meshes converted from ModalAI STEP; sensors/motor plugins follow PX4 x500.</description>
</model>
""")
# qvix module model (SDF 1.6 static mesh, works as-is in gz)
if os.path.isdir(f"{RE}/gz/models/qvix_r_module"): shutil.rmtree(f"{RE}/gz/models/qvix_r_module")
shutil.copytree(f"{RE}/qvix_integration/qvix_r_module", f"{RE}/gz/models/qvix_r_module")
# qvix world = PX4 default.sdf + QVIX R module + Seoul coordinates
w=open(f"{PX4}/Tools/simulation/gz/worlds/default.sdf", encoding="utf-8-sig").read()
w=w.replace('<world name="default">','<world name="qvix">')
w=w.replace('      <plugin name="Entity tree" filename="EntityTree"/>\n','      <!-- Entity tree removed 2026-10-28: it took 1/3 of the window and hid the 3D view in screen recordings -->\n')
w=w.replace("<magnetic_field>6e-06 2.3e-05 -4.2e-05</magnetic_field>","<magnetic_field>-4.4e-06 2.97e-05 -4.0e-05</magnetic_field>")
assert "-4.4e-06 2.97e-05" in w
w=w.replace("<latitude_deg>47.397971057728974</latitude_deg>","<latitude_deg>37.5</latitude_deg>").replace("<longitude_deg> 8.546163739800146</longitude_deg>","<longitude_deg>127.0</longitude_deg>")
w=w.replace("    <light name=\"sunUTC\"","""    <!-- QVIX R strawberry plant-factory module (Taebaek NextOn), same placement as the Classic qvix.world -->
    <include>
      <uri>model://qvix_r_module</uri>
      <name>qvix_r_module</name>
      <pose>0.50 -12.0 0.15 0 0 0</pose>
    </include>
    <light name="sunUTC\"""")
assert "qvix_r_module" in w and 'name="qvix"' in w
open(f"{RE}/gz/worlds/qvix.sdf","w").write(w)
# airframe
open(f"{RE}/gz/airframes/4201_gz_starling2","w").write("""#!/bin/sh
#
# @name ModalAI Starling 2 (gz SITL, CAD geometry)
#
# @type Quadrotor
# @class Copter
#
# @maintainer Starling SITL / agtechlab
#

. ${R}etc/init.d/rc.mc_defaults

PX4_SIMULATOR=${PX4_SIMULATOR:=gz}
PX4_GZ_WORLD=${PX4_GZ_WORLD:=default}   # NOTE: was qvix until 2026-10-16 -> "empty world" runs silently used the QVIX world (wall at x=0.5 next to the origin)
PX4_SIM_MODEL=${PX4_SIM_MODEL:=starling2}

param set-default SIM_GZ_EN 1

# Estimation like the real vehicle: VIO (ground-truth odometry from gz), no GPS.
# PX4's simulated GPS has hard-coded 0.2 m horizontal / 0.5 m vertical noise, which made height and position tracking useless indoors.
param set-default SENS_EN_GPSSIM 0
param set-default SENS_EN_BAROSIM 0
# Heading like the real vehicle: VIO only, no magnetometer. On the Starling 2 the VIO frame's x axis is the camera/forward direction
# at power-on, so "north" (yaw 0) = initial heading; run_sitl.sh spawns facing north to reproduce that (gz odometry is world-fixed).
param set-default SENS_EN_MAGSIM 0
param set-default EKF2_MAG_TYPE 5
param set-default SYS_HAS_MAG 0        # otherwise the preflight check 'Compass Sensor 0 missing' refuses to arm (2026-10-16 13:03 run)
param set-default EKF2_GPS_CTRL 0
param set-default EKF2_EV_CTRL 15
param set-default EKF2_HGT_REF 3
param set-default EKF2_EVP_NOISE 0.1       # real vehicle values (handoff_2026-10-27); real uses EKF2_EV_CTRL 7 (no EV yaw) - kept 15 here so EKF yaw = truth
param set-default EKF2_EVV_NOISE 0.1
param set-default EKF2_EVA_NOISE 0.1
param set-default EKF2_EV_DELAY 0
# (EKF2_EV_QMIN 1 on the real vehicle: NOT applied - gz odometry reports quality 0, EV fusion would be rejected -> no local position, arming denied 2026-10-27 18:38)
param set-default COM_ARM_WO_GPS 1
# (COM_MODE_ARM_CHK does not exist in v1.15.4; arming without GPS works because run_sitl.sh sends SET_GPS_GLOBAL_ORIGIN after boot.)

# Control allocation - rotor hubs measured from ModalAI D0014 Starling 2-C26 STEP (PX4 FRD frame, y right)
param set-default CA_AIRFRAME 0
param set-default CA_ROTOR_COUNT 4
param set-default CA_ROTOR0_PX 0.066
param set-default CA_ROTOR0_PY 0.088
param set-default CA_ROTOR0_KM 0.05
param set-default CA_ROTOR1_PX -0.066
param set-default CA_ROTOR1_PY -0.088
param set-default CA_ROTOR1_KM 0.05
param set-default CA_ROTOR2_PX 0.066
param set-default CA_ROTOR2_PY -0.088
param set-default CA_ROTOR2_KM -0.05
param set-default CA_ROTOR3_PX -0.066
param set-default CA_ROTOR3_PY 0.088
param set-default CA_ROTOR3_KM -0.05

param set-default SIM_GZ_EC_FUNC1 101
param set-default SIM_GZ_EC_FUNC2 102
param set-default SIM_GZ_EC_FUNC3 103
param set-default SIM_GZ_EC_FUNC4 104
param set-default SIM_GZ_EC_MIN1 150
param set-default SIM_GZ_EC_MIN2 150
param set-default SIM_GZ_EC_MIN3 150
param set-default SIM_GZ_EC_MIN4 150
param set-default SIM_GZ_EC_MAX1 1000
param set-default SIM_GZ_EC_MAX2 1000
param set-default SIM_GZ_EC_MAX3 1000
param set-default SIM_GZ_EC_MAX4 1000

# ---- Real Starling 2 parameters (handoff_2026-10-27: voxl-px4 1.14.0, params/starling2_sitl_param_set.txt). Applied 2026-10-27.
# thrust / hover
param set-default THR_MDL_FAC 0.9
param set-default MPC_THR_HOVER 0.34
param set-default MPC_THR_MIN 0.07
param set-default MPC_THR_MAX 0.6
# position / velocity control
param set-default MPC_POS_MODE 4
param set-default MPC_XY_P 3.5
param set-default MPC_Z_P 5.0
param set-default MPC_XY_VEL_P_ACC 3.0
param set-default MPC_XY_VEL_I_ACC 0.1
param set-default MPC_XY_VEL_D_ACC 0.0
param set-default MPC_Z_VEL_P_ACC 8.0
param set-default MPC_Z_VEL_I_ACC 2.0
param set-default MPC_XY_VEL_MAX 5.0
param set-default MPC_XY_CRUISE 5.0
param set-default MPC_Z_VEL_MAX_UP 3.0
param set-default MPC_Z_VEL_MAX_DN 1.5
param set-default MPC_Z_V_AUTO_UP 0.5
param set-default MPC_Z_V_AUTO_DN 0.5
param set-default MPC_ACC_HOR 3.0
param set-default MPC_ACC_HOR_MAX 1000.0
param set-default MPC_ACC_UP_MAX 3.0
param set-default MPC_ACC_DOWN_MAX 3.0
param set-default MPC_JERK_MAX 40.0
param set-default MPC_JERK_AUTO 4.0
param set-default MPC_TILTMAX_AIR 30.0
param set-default MPC_TILTMAX_LND 12.0
param set-default MPC_YAWRAUTO_MAX 45.0
param set-default MPC_TKO_SPEED 0.5
param set-default MPC_TKO_RAMP_T 1.0
param set-default MPC_LAND_SPEED 0.6
param set-default MPC_LAND_ALT1 10.0
param set-default MPC_LAND_ALT2 5.0
param set-default MPC_LAND_ALT3 1.0
param set-default MIS_TAKEOFF_ALT 0.5
# attitude / rate control (real tuning; sim inertia is a box estimate - revert this block if the sim oscillates)
param set-default MC_ROLL_P 16.0
param set-default MC_PITCH_P 16.0
param set-default MC_YAW_P 2.8
param set-default MC_ROLLRATE_P 0.072
param set-default MC_ROLLRATE_I 0.171
param set-default MC_ROLLRATE_D 0.0009
param set-default MC_ROLLRATE_MAX 130.0
param set-default MC_PITCHRATE_P 0.097
param set-default MC_PITCHRATE_I 0.228
param set-default MC_PITCHRATE_D 0.0011
param set-default MC_PITCHRATE_MAX 130.0
param set-default MC_YAWRATE_P 0.15
param set-default MC_YAWRATE_I 0.5
param set-default MC_YAWRATE_D 0.0
param set-default MC_YAWRATE_K 1.0
param set-default MC_YAWRATE_MAX 150.0
# commander / landing
param set-default COM_DISARM_LAND 0.1
param set-default COM_DISARM_PRFLT 20.0
param set-default COM_SPOOLUP_TIME 2.0
param set-default COM_OF_LOSS_T 1.0
param set-default LNDMC_ROT_MAX 30.0
param set-default LNDMC_TRIG_TIME 0.5
param set-default LNDMC_XY_VEL_MAX 1.5
param set-default LNDMC_Z_VEL_MAX 0.25
param set-default MIS_YAW_ERR 12.0
param set-default NAV_ACC_RAD 10.0
# battery (2S 3950 mAh) - sim battery is ideal, kept for parity
param set-default BAT1_N_CELLS 2
param set-default BAT1_CAPACITY 3950
param set-default BAT1_V_CHARGED 4.1
param set-default BAT1_V_EMPTY 2.8
param set-default BAT1_R_INTERNAL 0.022

# EKF2 - IMU offset (real EKF2_IMU_POS_*, handoff_2026-10-27)
param set-default EKF2_IMU_POS_X 0.0043
param set-default EKF2_IMU_POS_Y 0.0073
param set-default EKF2_IMU_POS_Z -0.016

# same as the real vehicle
param set-default IMU_GYRO_RATEMAX 800
param set-default MAV_TYPE 2
param set-default MC_AT_EN 1
""")
os.chmod(f"{RE}/gz/airframes/4201_gz_starling2",0o755)
print("generated:", os.listdir(f"{RE}/gz/models"), os.listdir(f"{RE}/gz/worlds"), os.listdir(f"{RE}/gz/airframes"))
