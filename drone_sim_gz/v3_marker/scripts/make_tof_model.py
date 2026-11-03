"""Generate the wall-keep model variant starling2_tof + airframe 4202 into v2_wallkeep/gz/ (2026-10-29).
Derived from the CURRENT generated starling2 model (repo_export/gz/models/starling2/model.sdf, written by
repo_export/scripts/make_gz_model.py) so both stay identical except for:
  1. a front depth sensor "front_tof" (gz depth_camera) approximating the VOXL ToF: 224 x 172 px, horizontal FOV 106.5 deg
     (voxl-vision-hub VOA tof entry), range 0.1-6 m, 15 Hz, gaussian noise 5 mm, topic /front_tof
     (gz.msgs.Image R_FLOAT32 depth along the optical axis) + /front_tof/points.
  2. the OdometryPublisher writes its covariance odometry to /wallkeep/vio_raw instead of
     /model/<name>/odometry_with_covariance; vio_relay.py republishes it to PX4's topic, optionally with injected
     VIO errors (frame rotation, drift). /model/<name>/odometry (truth, used by truth_sampler.py) is unchanged.
Run on the host:  python3 v2_wallkeep/scripts/make_tof_model.py   then  v2_wallkeep/scripts/install_tof.sh
"""
import os, re, shutil
V2 = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SRC = os.path.join(os.path.dirname(V2), "repo_export", "gz")
out = f"{V2}/gz/models/starling2_tof"
if os.path.isdir(out):
    shutil.rmtree(out)
shutil.copytree(f"{SRC}/models/starling2/meshes", f"{out}/meshes")

TOF_POSE = "0.075 0 0.0 0 0 0"        # same place as the front camera (front sensor cluster)
TOF = f"""
      <!-- wall-keep depth sensor (v2_wallkeep, 2026-10-29): VOXL ToF approximation, looks forward -->
      <sensor name="front_tof" type="depth_camera">
        <pose>{TOF_POSE}</pose>
        <camera>
          <horizontal_fov>1.8588</horizontal_fov>
          <image><width>224</width><height>172</height><format>R_FLOAT32</format></image>
          <clip><near>0.1</near><far>6.0</far></clip>
          <noise><type>gaussian</type><mean>0</mean><stddev>0.005</stddev></noise>
          <visibility_mask>1</visibility_mask>
        </camera>
        <always_on>1</always_on>
        <update_rate>15</update_rate>
        <visualize>false</visualize>
        <topic>front_tof</topic>
      </sensor>
"""
# Real-vehicle sensor set (2026-10-30, cameras_2026-10-30.zip: /etc/modalai/extrinsics.conf, opencv_tracking_*_intrinsics.yml,
# voxl-tag-detector.conf, voxl-camera-server.conf). Extra topics; the sensors above stay for the old runs and the videos.
# Body FRD offsets from extrinsics.conf -> gz FLU (x, -y, -z).
#   front_camera_real  tracking_front (AR0144 fisheye 1280x800) as the tag detector sees it: undistorted at undistort_scale 0.6
#                      -> pinhole fx 276.6 px on 1280x800 (hfov 2 atan(640/276.6) = 133.3 deg), body (0.0665, -0.0065, -0.0154)
#                      FRD, looking straight ahead. 15 Hz rendered; tag_detect.py WK_TAG_HZ=5 detects on every 3rd frame
#                      (camera 30 fps, skip_n_frames 5).
#   front_tof_real     PMD liow2 ToF, body (0.066, 0.009, -0.012) FRD on the real vehicle; in the SIM at x 0.080 (2026-10-31): at
#                      0.066 the sensor sits inside the CAD body mesh and, depending on the vehicle's world yaw (-40, 60 deg),
#                      renders ~9 % valid pixels (tof_render_probe_vehicle2.py: 0.066 -> 0.09, 0.075 / 0.080 -> 1.00). extrinsics RPY (0, 90, 180): the sensor x axis is body
#                      DOWN, so the wide 106.5 deg axis is vertical and the horizontal FOV is 85.1 deg (to confirm on the
#                      vehicle). 172 x 224 portrait, vfov then 100 deg. 10 Hz (configured; 2.0 fps measured -> WK_TOF_HZ=2).
#   down_camera        tracking_down (fisheye) undistorted like the front: fx 267.8 px on 1280x800 (hfov 134.4 deg),
#                      body (-0.0588, -0.011, 0.0109) FRD, optical axis down, image right = body backward. 5 Hz (landing pad).
REAL = """
      <!-- real-vehicle sensor set (v3_marker make_tof_model.py, 2026-10-30, cameras_2026-10-30.zip) -->
      <sensor name="front_camera_real" type="camera">
        <pose>0.0665 0.0065 0.0154 0 0 0</pose>
        <camera>
          <horizontal_fov>2.3268</horizontal_fov>
          <image><width>1280</width><height>800</height></image>
          <clip><near>0.05</near><far>30</far></clip>
          <visibility_mask>1</visibility_mask>
        </camera>
        <always_on>1</always_on>
        <update_rate>15</update_rate>
        <visualize>false</visualize>
        <topic>front_camera_real</topic>
      </sensor>
      <sensor name="front_tof_real" type="depth_camera">
        <pose>0.080 -0.009 0.012 0 0 0</pose>
        <camera>
          <horizontal_fov>1.4853</horizontal_fov>
          <image><width>172</width><height>224</height><format>R_FLOAT32</format></image>
          <clip><near>0.1</near><far>6.0</far></clip>
          <noise><type>gaussian</type><mean>0</mean><stddev>0.005</stddev></noise>
          <visibility_mask>1</visibility_mask>
        </camera>
        <always_on>1</always_on>
        <update_rate>10</update_rate>
        <visualize>false</visualize>
        <topic>front_tof_real</topic>
      </sensor>
      <sensor name="down_camera" type="camera">
        <pose>-0.0588 0.011 -0.0109 0 1.5708 -1.5708</pose>
        <camera>
          <horizontal_fov>2.3478</horizontal_fov>
          <image><width>1280</width><height>800</height></image>
          <clip><near>0.02</near><far>10</far></clip>
          <visibility_mask>1</visibility_mask>
        </camera>
        <always_on>1</always_on>
        <update_rate>5</update_rate>
        <visualize>false</visualize>
        <topic>down_camera</topic>
      </sensor>
"""
TOF = TOF + REAL
sdf = open(f"{SRC}/models/starling2/model.sdf").read()
sdf = sdf.replace("model://starling2/meshes/", "model://starling2_tof/meshes/")
sdf = sdf.replace("<model name='starling2'>", "<model name='starling2_tof'>")
sdf = sdf.replace("Generated by repo_export/scripts/make_gz_model.py", "Generated by v2_wallkeep/scripts/make_tof_model.py from repo_export make_gz_model.py output")
# depth sensor right after the front camera sensor
i = sdf.index('<sensor name="front_camera"'); j = sdf.index("</sensor>", i) + len("</sensor>")
sdf = sdf[:j] + "\n" + TOF + sdf[j:]
sdf = sdf.replace("<odom_publish_frequency>50</odom_publish_frequency>",
                  "<odom_publish_frequency>50</odom_publish_frequency>\n"
                  "      <!-- PX4 reads /model/<name>/odometry_with_covariance; vio_relay.py fills it from here (error injection) -->\n"
                  "      <odom_covariance_topic>/wallkeep/vio_raw</odom_covariance_topic>")
# 2026-11-02 (user: make m3 the default model; codex sim-total-mass-consistency / sim-imu-offset-consistency): same two changes as
# make_model_m3.py, here so starling2 / make_gz_model.py (instances 0 and 1) stay untouched:
#   base_link 0.285 -> 0.2895 kg (links sum to the measured 297.5 g incl. battery; motor constant kept, it already gives the real
#   ulog hover 0.356-0.357 at that mass), imu_sensor at the real EKF2_IMU_POS (FRD 0.0043 0.0073 -0.016 -> FLU below).
# The model before this change is kept as gz/variants/m1_before_20261102 (WK_MODEL_DIR to fly it again).
BASE_MASS, IMU_POSE = 0.2895, "0.0043 -0.0073 0.016 0 0 0"
mb = re.search(r'<link name="base_link">\s*<inertial>\s*<mass>([^<]+)</mass>', sdf)
assert mb and abs(float(mb.group(1)) - 0.285) < 1e-9, "make_tof_model: unexpected base_link mass in the starling2 source"
sdf = sdf[:mb.start(1)] + "%g" % BASE_MASS + sdf[mb.end(1):]
mi = re.search(r'(<sensor name="imu_sensor" type="imu">)(\s*)(?!<pose>)', sdf)
assert mi, "make_tof_model: imu_sensor not found or already has a pose"
sdf = sdf.replace(mi.group(0), mi.group(1) + mi.group(2) + "<pose>%s</pose>%s" % (IMU_POSE, mi.group(2)), 1)
assert abs(sum(float(m) for m in re.findall(r"<mass>([^<]+)</mass>", sdf)) - 0.2975) < 1e-9
assert "front_tof" in sdf and "/wallkeep/vio_raw" in sdf and "model://starling2/" not in sdf
open(f"{out}/model.sdf", "w").write(sdf)
open(f"{out}/model.config", "w").write("""<?xml version="1.0"?>
<model>
  <name>starling2_tof</name>
  <version>1.0</version>
  <sdf version="1.9">model.sdf</sdf>
  <author><name>Sejong agtechlab</name></author>
  <description>starling2 + front ToF depth sensor + relayed VIO odometry (wall-keep development, v2_wallkeep).</description>
</model>
""")
af = open(f"{SRC}/airframes/4201_gz_starling2").read()
af = af.replace("# @name ModalAI Starling 2 (gz SITL, CAD geometry)", "# @name ModalAI Starling 2 + front ToF (gz SITL, wall-keep development)")
af = af.replace("PX4_SIM_MODEL=${PX4_SIM_MODEL:=starling2}", "PX4_SIM_MODEL=${PX4_SIM_MODEL:=starling2_tof}")
assert "starling2_tof" in af
os.makedirs(f"{V2}/gz/airframes", exist_ok=True)
open(f"{V2}/gz/airframes/4202_gz_starling2_tof", "w").write(af)
os.chmod(f"{V2}/gz/airframes/4202_gz_starling2_tof", 0o755)
print("generated", out, f"{V2}/gz/airframes/4202_gz_starling2_tof")
