# Hardware Notes

Two sources of truth:
1. **Datasheet-derived facts** (below, §A/§B) — from vendor material, known before bring-up.
2. **Phase-1 introspection results** (§C) — filled in live once robots are on the network.
   These override any datasheet/analogy guess where they conflict.

---

## A. ROSPug (HiWonder, SKU HIWND-380) — quadruped

Source: ThinkRobotics product datasheet (`docs/hardware/HiWonder ROSPug ... ThinkRobotics.com.pdf`).
This is HiWonder's *Professional* ROS quadruped — distinct from the cheaper Raspberry-Pi
PuppyPi, though it ships the same PuppyPi-family PC action editor.

**Compute / board**
- NVIDIA **Jetson Nano** control system.
- HiWonder ros_robot_controller expansion board. Ports labeled on the datasheet:
  PWM Servo Port, **Serial Bus Servo Port**, IMU Sensor, Serial Port, GPIO, IIC,
  5V/3.3V power, Buzzer, Key, Power Port + Power Switch.
- OLED display module (controller info, battery voltage, customizable content).

**Actuators**
- **12× HTS-30HS intelligent serial-bus servos** (3 per leg: hip/shoulder/elbow).
- Spec: 0.10 s/60° @12 V, **30 kg·cm** stall torque @12 V, 0.3° accuracy, stall +
  over-temperature protection, **data readback: temperature, voltage, position**.
- ⚠️ Serial-bus protocol (not PuppyPi's PWM leg servos). Joint control routes through the
  board's serial-bus servo path — confirm exact ROS message in Phase 1.
- Datasheet PC-editor screenshot shows IDs 1–8 (that's the reused PuppyPi editor UI); the
  ROSPug has 12 DOF — verify real servo IDs from `/joint_states` in Phase 1.

**Sensing**
- **Oradar MS200** TOF lidar: dToF, 360°, 0.03–12 m @90% reflectivity, angular res
  0.8°@10 Hz, laser pitch 0.5–2°, Class-1 laser, 40 g, IP5X. Data: distance, angle,
  intensity, timestamp. → standard ROS `/scan` (LaserScan) via an oradar driver.
- HD **wide-angle** camera on the head.
- Built-in IMU (self-balancing + yaw correction).

**Power:** 11.1 V 3500 mAh 5C LiPo (72×55×19 mm), DC5.5×2.5 / SM-2P plug, 12.6 V charger.

**Software stack (vendor):** ROS + Gazebo sim; self-developed dynamic-balancing kinematics;
Walk / Amble / Trot gaits; inverse kinematics with coordinate-mode action editor; SLAM
(Gmapping / Hector / Karto), fixed-/multi-point nav, TEB path planning, dynamic obstacle
avoidance; OpenCV + MediaPipe demos (color/tag/face/KCF tracking, line following, ball
shooting, stairs). Cross-platform control: WonderPi-style app (iOS/Android), PC software,
webpage, PS2-style wireless handle. RViz `mapping.rviz` shows LaserScan/Odometry/TF/Map/
RobotModel → ROS1-style SLAM.

**Transport hypothesis (verify in Phase 1):** same as PuppyPi family — roslibpy →
rosbridge `ws://<ip>:9090`, MJPEG at `http://<ip>:8080`, `puppy_control`-style topics.
Trust the *pattern*; verify every name.

## B. TonyPi (HiWonder, standard) — humanoid

Authoritative source: the vendor tree `reference/TonyPi/` (verified 2026-07-17, still valid).
- **No ROS.** JSON-RPC 2.0 over HTTP `:9030` (RPCServer.py); MJPEG camera `:8080`
  (`/?action=snapshot`); WS relay `:7788`.
- Key RPC: `RunAction([code_string, times])` (numeric-string codes from ActionGroupDict.py),
  `SetPWMServo` (head: 1=tilt, 2=pan, 500–2500), `SetBusServoPulse` (body ids 1–16,
  0–1000), `StopActionGroup`, `StandUp`, `LoadFunc/StartFunc/StopFunc` (+7 s `Heartbeat`
  watchdog), `StopBusServo`.
- On-robot Python (SSH `pi`/`raspberrypi`): `hiwonder.ros_robot_controller_sdk`,
  `Controller`, `ActionGroupControl` (`.d6a` SQLite action groups in
  `/home/pi/TonyPi/ActionGroups/`). `AGC.runActionGroup` is blocking → thread it.

## C. Phase-1 introspection results — ROSPug (**VERIFIED LIVE 2026-07-17**)

**These findings are AUTHORITATIVE. Where they conflict with §A, §B, or the pre-bring-up
assumptions, these win.** Obtained read-only over rosbridge; nothing was published, nothing actuated.
Full dump: `docs/rospug_ros_graph.txt` (105 topics, 158 services, 26 nodes).

### Connection (what worked)
- **Wired**, PC as the robot's router: `nmcli con up robot-wired` (profile: `enp132s0`,
  `ipv4.method=shared`, `autoconnect=no` so it never fights the existing `g1-wired`).
  PC = `10.42.0.1`, robot DHCPs a lease. PC keeps Wi-Fi internet → Gemini API works
  concurrently, and the robot gets NAT'd internet too.
- **Robot: `<ROBOT_IP>`, mDNS `rospug.local`.** (DHCP lease, so re-check the IP each boot,
  or pin it.)
- **SSH banner: `OpenSSH_7.6p1 Ubuntu-4ubuntu0.7` → Ubuntu 18.04** (stock Jetson Nano
  JetPack base) ⇒ **ROS1 Melodic**. No Docker involved. Port `11311` is not exposed
  externally, but that doesn't matter — rosbridge is the interface.
- Open ports: **22** (SSH), **4000** (NoMachine remote desktop — this is the GUI path; no
  VNC), **8080** (web_video_server), **9090** (rosbridge). Closed: 5900/5901, 9030, 7788.

**Onboard Wi-Fi / AP mode (from the user manual, 2026-07-20).** On boot the ROSPug defaults
to **AP-direct mode**: it broadcasts its own Wi-Fi SSID **`HW-XXXXXXXX`** (8 hex chars,
device-unique, e.g. `HW-XXXXXXXX`), password **`<AP_PASSWORD>`** (the HiWonder factory default, not reproduced here). By HiWonder convention the
robot is **`192.168.149.1`** on its own AP. This is a *second*, independent path to the
robot (onboard `wlan0`), separate from the wired `eth0` we've been using — and it's the
only path that works while the robot is **walking untethered**. Trade-off: the PC has one
Wi-Fi radio, so joining `HW-…` means the PC drops the site network and loses internet → **no Gemini
API** unless the PC has wired internet. Also every HiWonder robot uses the same AP IP, so
two robots can't be on their own APs at once. Target end-state for locomotion demos is
therefore **STA mode** (robot joins a router that also carries the PC + internet), with AP
as the fallback / the gateway used to configure STA. See `docs/alternatives.md` §5.

### ⚠️ THE BIG ONE: it is `pug_control`, NOT `puppy_control`
Every quadruped topic name assumed before bring-up is wrong for this robot.
The real surface:

| Topic | Type | Fields |
|---|---|---|
| `/pug_control/velocity_move` | `pug_control/Velocity` | `x, y, yaw_rate` (float32) + **`stop` (bool)** ← PuppyPi's Velocity had no `stop` field |
| `/pug_control/pose` | `pug_control/Pose` | `roll, pitch, yaw, height, x_shift, stance_x, stance_y, run_time` (same shape as PuppyPi) |
| `/pug_control/gait` | `pug_control/Gait` | `overlap_time, swing_time, clearance_time, z_clearance` (same shape as PuppyPi) |
| `/pug_control/cmd_vel` | `geometry_msgs/Twist` | coarse |
| `/cmd_vel` | `geometry_msgs/Twist` | coarse |
| `/pug_control/web_controller` | `pug_control/WebController` | `gait, foot_height, period, …` |
| `/app_control/{velocity_move,pose,gait}` | same types | the app's mirror of the above |

**Low-level board (`ros_robot_controller`) — serial-bus servos, as the datasheet implied:**

| Topic | Type |
|---|---|
| `/ros_robot_controller/bus_servo/set_state` | `SetBusServoState{ state: BusServoState{present_id, target_id, position, offset, voltage, temperature, position_limit}, duration }` |
| `/ros_robot_controller/pwm_servo/set_state` | `SetPWMServoState` |
| `/ros_robot_controller/robot/{move,stop,set_gait,set_pose,set_joint_angle}` | `Move` / **`Stop`** / `SetGait` / `SetPose` / `SetJointAngle` |
| `/ros_robot_controller/enable_reception` | `std_msgs/Bool` ← same gotcha as TonyPi: **enable before servo readback** |
| `/ros_robot_controller/{battery,imu_raw}` | `std_msgs/UInt16` (mV) / `sensor_msgs/Imu` |
| `/ros_robot_controller/{set_buzzer,set_oled}` | `BuzzerState` / `OLEDState` |

**Services (verified types):**
- `/pug_control/go_home` → `std_srvs/Empty` (stand/reset)
- `/pug_control/run_action_group` → **`pug_control/SetActionName{ string name }`** (tricks;
  ⚠️ name list still unknown — needs an on-robot `ls`, see open items)
- `/pug_control/enable_sim` → `std_srvs/SetBool`
- `/self_balancing/set_running` → `std_srvs/SetBool`
- `/performance/{enter,exit}` → `std_srvs/Trigger`; `/performance/set_self_balancing` →
  `SetBool`; **`/performance/heartbeat` → `SetBool`** (watchdog, cf. TonyPi);
  `/performance/set_target` → `pug_app/SetTarget{string data}`
- `/ros_robot_controller/robot/set_leg_ik` → `SetLegIK{float32 position}`;
  `/set_leg_fk`; `/get_leg_position` → `GetLegPosition`
- `/ros_robot_controller/bus_servo/get_state` → `GetBusServoState{GetBusServoCmd cmd}`
- `/lidar_app/set_running` → `pug_control/SetInt64{int64 data}`
- Demo nodes each expose `set_running`: `/apriltag_detect`, `/face_detect`, `/hand_gesture`,
  `/object_tracking` (+`set_target_color`, `set_threshold`), `/visual_patrol`,
  `/lab_config_manager`.

**Gazebo/sim per-joint controllers (confirms 12 DOF):**
`/pug/{lf,lb,rf,rb}_{joint,thigh,calf}_position_controller/command` → `std_msgs/Float64`
(hip=`joint`, shoulder=`thigh`, elbow=`calf`).

**26 nodes running at boot:** `pug_control`, `ros_robot_controller`, `self_balancing`,
`joint_state_publisher`, `robot_state_publisher`, `imu_calib`, `imu_filter`, `oradar_scan`,
`front_laser`, `lidar_app`, `joystick`, `joystick_control`, `oled_display`, `performance`,
`web_video_server`, `rosbridge_websocket`, `rosapi`, `csi_camera/csi_camera_image_proc`,
`hand_trajectory`, + demos (`apriltag_detect`, `face_detect`, `hand_gesture`,
`object_tracking`, `visual_patrol`, `lab_config_manager`).

### Sensor liveness (12 s subscribe test)
| Topic | Status | Evidence |
|---|---|---|
| `/scan` (`LaserScan`) | ✅ **LIVE** | 451 beams, 219 valid, 0.23–11.98 m (matches MS200's 0.03–12 m) |
| `/joint_states` (`JointState`) | ✅ **LIVE** | **12 joints**: `lf/lb/rf/rb` × `joint/thigh/calf` — 12 DOF confirmed. ⚠️ all positions read exactly `0.0` → suspect servo readback needs `/ros_robot_controller/enable_reception` = true; verify before trusting |
| `/imu` (`Imu`) | ✅ LIVE | accel z≈10.54, quat w=0.715 (fused) |
| `/ros_robot_controller/imu_raw` | ✅ LIVE | accel z≈10.12, quat w=0 (unfused) |
| `/ros_robot_controller/battery` | ✅ LIVE | see battery warning below |
| `/tf` | ✅ LIVE | `base_link → joint_Link_N → thigh_Link_N → calf_Link_N` |
| `/csi_camera/image_raw` | ❌ **SILENT** | see camera fault below |
| `/compressed_image` | ❌ **SILENT** | camera-dependent |

### 🔴 OPEN FAULT: camera driver is not running
- `/csi_camera/image_raw` and `/csi_camera/camera_info` have **`publishers: NONE`**, and
  `/csi_camera/csi_camera_image_proc` is subscribed to them, waiting.
- ⇒ The `image_proc` pipeline is healthy, but the **CSI camera driver node never starts**,
  so every downstream topic (`image_color`, `image_rect_color`, `image_mono`, …) is
  advertised-but-empty. `web_video_server` lists them (it reads the master's topic list)
  but `/snapshot` and `/stream` hang forever returning 0 bytes. The `:8080` index itself
  returns HTTP 200 in 10 ms, so the server is fine — there is simply no image data.
- No camera errors on `/rosout_agg` in a 15 s window ⇒ not crash-looping; **absent**.
- Config expects: `/camera_type="CSI"`, `/csi_camera/camera_id=0`,
  `/csi_camera/image_width=640`, `image_height=360`, `frame_id="/csi_camera_link"`.
- **Suspects (brand-new robot):** unseated CSI ribbon cable; `nvargus-daemon` not running;
  camera node missing from / failing in the boot launch; no `/dev/video0`.
- **This blocks the whole project** — Gemini ER needs frames. Fix before Phase 3.
- Diagnosis needs shell/GUI: SSH `:22` or NoMachine `:4000` → check `ls /dev/video*`,
  `systemctl status nvargus-daemon`, the bringup launch file, and try starting the camera
  node by hand.

### 🔋 Battery warning
`/ros_robot_controller/battery` = **10345 mV (10.35 V)** on a 3S 11.1 V LiPo ≈ **3.45 V/cell**
— roughly 20–25% remaining, close to the ~9.9 V (3.3 V/cell) safe floor. **Charge before any
motion testing** (12.6 V charger, in the box). Running 12× 30 kg·cm servos at this voltage
risks brown-outs mid-motion and over-discharge damage to the pack. Treat ≥11.1 V as the
gate for movement tests, and add a pre-flight battery check to the safety layer.

## E. LIVE MOTION TEST RESULTS (2026-07-20) — **VERIFIED ON HARDWARE**

First supervised motion, wired, no translation. Robot on flat floor, human observing.
Script: `scripts/first_motion_test.py`.

### ✅ The motor path works end-to-end
`/pug_control/pose` → `pug_node.pose_callback` → `SetPose` → SDK → 12 servos. Commanded
poses produced the correct physical motion. `/pug_control/go_home` (std_srvs/Empty) works
as a safe init.

### ✅ Roll/pitch command path is CORRECT — the earlier swap alarm was a false alarm
Empirically, with a human watching (0.20 rad, repeated alternating cycles):
- commanding **roll** → body leans **sideways** ✅
- commanding **pitch** → body tips **nose up/down** ✅

So the SDK's `struct.pack("<7f", x, y, z, x_shift, pitch, roll, duration)` transposition is
what the **firmware expects**; it cancels out. **Do NOT "fix" it in the adapter.**

### 🔄 BUT THE IMU'S AXES ARE TRANSPOSED relative to the body frame
Measured while commanding pose (0.15 rad = 8.6°):
| Commanded | IMU same-axis Δ | IMU cross-axis Δ |
|---|---|---|
| roll +8.6° | roll −1.2° | **pitch +8.4°** |
| pitch +8.6° | pitch +1.2° | **roll −7.8°** |

The motion appears on the **opposite** IMU axis at the commanded magnitude.
⇒ **When reading `/imu` for feedback, swap roll and pitch back.** (Sign convention on the
cross-axis also looked inverted for pitch→roll; confirm the sign before closed-loop use.)

### ❌ `/joint_states` IS NOT REAL FEEDBACK — and here is exactly why
`pug_control.launch:7` starts the **stock ROS `joint_state_publisher` with no
`source_list`**:
```xml
<node name="joint_state_publisher" pkg="joint_state_publisher" type="joint_state_publisher" />
```
That node reads the URDF and emits **placeholder** joint values purely so
`robot_state_publisher` can build a complete TF tree for RViz. It is **never wired to the
servos**. Nothing is broken — it is visualisation scaffolding, not telemetry.

`enable_reception` was a red herring: the driver already calls `board.enable_reception()`
at startup (`ros_robot_controller_node.py:24`), so it was on the whole time, and it would
not have affected `joint_state_publisher` regardless.

⚠️ **COROLLARY: the TF leg transforms are equally fake.** `robot_state_publisher` consumes
those zeros, so `base_link -> joint_Link -> thigh_Link -> calf_Link` reflects the zero pose,
not reality. **Do not use TF for leg positions.**

### ✅ REAL joint feedback DOES exist — two working services (VERIFIED live)

**1. `/ros_robot_controller/robot/get_leg_position`** (`GetLegPosition`, empty request)
Returns `success` + **12 floats in METRES**, laid out **[X×4, Y×4, Z×4]**, leg order
**FR, FL, BR, BL** (per the SDK's `set_leg_ik_sim` docstring).
VERIFIED to track commands exactly:
| Commanded height | Reported Z ×4 |
|---|---|
| −0.13 m | `-0.13, -0.13, -0.13, -0.13` |
| −0.09 m | `-0.09, -0.09, -0.09, -0.09` |
| Δ | **+0.04 — exactly the commanded change** |
This is the best closed-loop channel for verifying a pose actually took effect.

**2. `/ros_robot_controller/bus_servo/get_state`** (`GetBusServoState{GetBusServoCmd[] cmd}`)
Per-servo readback, ids 1–12. Set only the flags you need:
| Flag | Status | Sample (12 servos) |
|---|---|---|
| `get_position` | ✅ works | `500, 314, 523, 503, 687, 490, 503, 314, 508, 501, 686, 477` (0–1000) |
| `get_temperature` | ✅ works | `39, 43, 43, 57, 53, 63, 55, 48, 59, 43, 36, 34` °C |
| `get_id` | ✅ works | `1..12` all respond |
| `get_voltage` | ❌ **VENDOR BUG** | `'Board' object has no attribute 'bus_servo_read_voltage'` |

🔴 **NEVER set `get_voltage=1`** — it raises inside the handler and fails the ENTIRE service
call, not just that field. Same class of bug as the PuppyPi `setServo` NameError.

💡 **Per-servo temperature is a safety channel we did not plan for** — real overheat/stall
detection. Observed 34–63 °C at rest (servo 6 hottest at 63 °C). HTS-30HS servos have
over-temperature protection; worth monitoring during sustained motion.

### ✅ Safety layer verified against live hardware
- **Battery gate**: read 11.98 V, correctly permitted motion (floor 11.1 V).
- **Clamps**: `roll 5.0 → 0.2`, `pitch −5.0 → −0.2`, `height −90 → −0.15`, `x_shift 50 → 0.03`.
- **E-stop**: latched, blocked *both* pose and action commands, reset cleanly.

### Feedback channels — what to actually use in the agent loop
| Channel | Status |
|---|---|
| `/csi_camera/image_raw` | ✅ live (autostarted this session; no manual gscam needed) |
| `/scan` (lidar) | ✅ live |
| `/imu` | ✅ live — **but swap roll/pitch** |
| `/ros_robot_controller/battery` | ✅ live, real millivolts |
| `/joint_states` | ❌ **fake zeros — do not use** (and TF legs are fake too) |
| `get_leg_position` (srv) | ✅ **REAL** — Cartesian feet, metres, tracks commands exactly |
| `bus_servo/get_state` (srv) | ✅ position + temperature + id. ❌ **never request voltage** |

### ROSPug — still open
- [ ] **Fix the camera** (blocking)
- [ ] SSH/NoMachine credentials (not in hand; Jetson image ⇒ NOT PuppyPi's `pi`/`raspberrypi`)
- [ ] Real action-group names for `/pug_control/run_action_group` (needs on-robot `ls`)
- [ ] Real velocity/pose/yaw **limits** — read from `pug_control` source on the robot; the
      §2.3 numbers (‖x‖≤35 cm/s, ‖yaw‖≤0.89 rad/s, height −15…−5) are UNVERIFIED for this robot
- [ ] Units for `pug_control/Velocity.x` (cm/s assumed by analogy — confirm in source)
- [ ] Whether `Velocity.stop=true` is the clean stop path vs zero-velocity
- [x] ~~enable_reception for /joint_states~~ ✅ answered: joint_states is FAKE regardless (§E)
- [x] ~~Test bus_servo/get_state~~ ✅ position/temp/id work; voltage is a vendor bug (§E)
- [ ] **No robot-side deadman exists** — build one before any untethered/wireless motion
- [ ] Pin the robot's IP (DHCP lease will move)

### TonyPi — the 2026-07-17 bring-up list, and what happened to it
- [x] Reachable IP; JSON-RPC on `:9030`; snapshot from `:8080` — **answered on the
  physical robot, 2026-07-28**: see the ten-sample snapshot-vs-stream measurement in
  `physicalagent/robots/tonypi_link.py` (`snapshot()`) and `tools/tonypi_view.py`.
- [ ] `ls /home/pi/TonyPi/ActionGroups/` — still not done, and no longer needed: the
  action codes in use are pinned from the vendor's own `ActionGroupDict.py`
  (`physicalagent/config.py`), never from a directory listing on the robot.


## F. LOCOMOTION + MECHANICAL NOTES (2026-07-20 session 2)

### ✅ Deadman PROVEN on a moving robot
Robot marching under gait; heartbeat cut → **PAUSED after 1.40 s** (timeout 1.5 s), robot
visibly stopped and held position; heartbeat restored → auto-resumed to ALIVE with no
restart. This is the safety prerequisite for untethered/wireless operation. Node lives at
`robot_nodes/physicalagent_deadman.py`, deployed to
`/home/hiwonder/physicalagent/physicalagent_deadman.py`.

### ❗ Walking does NOT translate yet — unresolved
At 0.05 m/s AND at 0.10 m/s (vendor max), with default gait AND with a carpet gait
(`z_clearance` 0.06, `swing_time` 0.25, `overlap_time` 0.30, `clearance_time` 0.25), the
robot **marches in place**: legs cycle through the gait but the body does not translate.
Also tried `/app_control/velocity_move` (which applies the vendor's CoM `x_shift`
correction) — same result. **Cause still unknown.** Candidates:
1. **A leg linkage was failing during these tests** (see below) — may invalidate ALL of the
   above locomotion results. Re-test after the repair before drawing conclusions.
2. Carpet friction/pile resisting translation.
3. Some enable/mode we have not found (the gait engine runs, but translation may need
   something else).

### 🔧 MECHANICAL FAILURE + REPAIR (Front Right leg)
Mid-test, the **push-rod/linkage on the FRONT RIGHT leg came out**. It has an adjustable
length, so re-seating it risks setting the wrong length — which would corrupt the IK for
that leg (the kinematics assume a fixed rod length).

**Verification after repair (all zero-motion):**
- **Servo positions vs pre-failure baseline**: max deviation 12/1000 counts (~1%), most
  within ±3. Mirror symmetry intact (front L+R = 999, back = 1001, expect ~1000).
  ⚠️ But this only proves the SERVO SHAFT angle is unchanged — it is structurally blind to
  a wrong rod length (servo can sit at the same angle with different leg geometry).
- **`get_leg_position`** reported all four feet at exactly −0.13 m. Also blind: it is
  forward kinematics from servo angles + ASSUMED geometry, not measured foot position.
- **IMU body tilt**: changed 4.42° from baseline (roll −2.90°, pitch +3.33°). Suggestive but
  CONFOUNDED — the robot was moved during the repair and sits on carpet, which compresses
  unevenly; 4.4° over a ~20 cm stance is only ~1.5 cm, well within carpet variation.
- **Rock/paper test (decisive)**: robot sits **solid on all four feet, no diagonal rock, no
  foot lets paper slide** ⇒ the four contact points are coplanar ⇒ linkage length looks
  correct. Physical evidence outranks the confounded IMU reading.

**Lesson for future repairs:** servo readback and `get_leg_position` CANNOT validate
mechanical geometry — both work from assumed kinematics. Use (a) the physical rock/paper
test and (b) an IMU comparison taken on a HARD FLAT surface, not carpet.

### Servo → leg map (decoded from `pug_node.id_to_kinematics` + `reshape((3,4))`)
Servos group in **threes per leg**, legs ordered **FR, FL, BR, BL**:
| Leg | servos (hip, thigh, calf) |
|---|---|
| Front Right | 1, 2, 3 |
| Front Left | 4, 5, 6 |
| Back Right | 7, 8, 9 |
| Back Left | 10, 11, 12 |
Resting pattern: hips all ~500; thighs mirror (right ~314, left ~686, summing ~1000);
calves ~480–525. Useful as a health baseline.

### Battery behaviour under load
Session drained **12.13 V → 11.08 V** across roughly a dozen short motion tests. Budget
~60–90 mV per short burst. The 11.1 V motion floor fired correctly and blocked further
testing — the gate works.

---

## G. WALKING RESOLVED (2026-07-20 session 3)

### ✅ THE ROBOT WALKS — the "no translation" problem was the broken leg
Re-tested after the Front-Right push-rod repair. **The robot translates normally.**
The entire §F "walking does not translate" finding was an artifact of the failing
linkage that was coming loose *during* those tests. It is now void.

| Test | Commanded | Predicted | Measured | Method |
|---|---|---|---|---|
| Stage 2 burst 1 | 0.05 m/s × 3 s | 15 cm | +16.1 cm | range-median (**inflated**) |
| Stage 2 burst 2 | 0.05 m/s × 3 s | 15 cm | +20.1 cm | range-median (**inflated**) |
| **Stage 3** | **0.10 m/s × 3 s** | **30 cm** | **+28.9 cm (96%)** | **wall line-fit (trusted)** |

Operator confirmed visually that it moved, but **less** than the Stage 2 lidar claimed —
which is what led to the odometer being rebuilt. Stage 3 agrees with the operator.

### Lidar odometry: use a YAW-INVARIANT measure, not median range
**Do not** use "median range of the forward sector" as an odometer. The robot's lidar is
body-mounted; a small heading change swings the beam onto a different part of a distant
surface, and at ~5 m range that produces a large apparent range change with **zero** actual
travel. This inflated the Stage 2 numbers.

**Use instead:** fit a line (total least squares) to the wall points in the forward ±30°,
then take the **perpendicular distance** from the robot to that line. Turning in place does
not change perpendicular distance to a wall; only translating toward it does.
Implementation: `wall_perp_distance()` in the stage-3 script. Result: 0.0 cm noise floor,
96% agreement with the commanded distance.

**Always establish the noise floor first** — take two stationary reads back to back and
report the difference. Any "travel" smaller than that number means nothing.

### ⚠️ IMU yaw drifts ~1.4°/s — DO NOT use it for heading
Measured while the robot was **stationary**: yaw went −31.4° → −35.8° → −41.7° across three
back-to-back reads (10.3° of pure drift). During a 3 s walk it moved 8.1°, i.e. ~1.0°/s —
**slower** than the stationary drift rate.

⇒ There is **no evidence the robot curves while walking**, and IMU yaw cannot be used to
detect curving, hold a heading, or close a navigation loop. It is a magnetometer-less
gyro-integration estimate and it wanders. Roll and pitch remain trustworthy (gravity
provides an absolute reference); **yaw does not**. If heading control is ever needed, it
must come from lidar scan-matching, not the IMU.

### Static load test after the repair (how to validate a leg repair)
Zero-velocity pose sequence (roll ±0.12, pitch +0.12, height −0.09 ↔ −0.13), all through
the real SafetyGuard, watching IMU response:
- every commanded pose produced a **matching IMU response** (~5–6° observed for a 6.9°
  command; the shortfall is carpet compliance)
- loading the repaired side mirrored the good side (roll +0.12 → +5.62°, −0.12 → −6.16°)
- **total drift across the whole sequence: 0.28° roll, 0.41° pitch** ← the key number.
  A loose or wrong-length push-rod shows hysteresis and walks away from baseline under
  repeated loading. It did not.

This is the recommended first stage after any mechanical repair: it loads all 12 servos
while the robot is planted and cannot walk away.

### Known-good axis transposition reconfirmed
Commanded **roll** appears in the IMU **pitch** field and vice versa, exactly as documented
in §E. Do **not** "fix" this — the SDK packs `(pitch, roll)` while the signature says
`(roll, pitch)`, and the two transpositions cancel. Reconfirmed across 8 pose commands.

### Session 3 operational notes
- Ethernet came up on the wrong NM profile (`g1-wired`, 192.168.123.x — the Go2/G1 robot).
  The Pug needs `robot-wired` (10.42.0.1/24 shared). Fix: `nmcli con up robot-wired ifname <dev>`.
- Robot's login shell is **zsh**, so `ssh host 'source /opt/ros/melodic/setup.bash'` fails
  with `no such file or directory: /home/hiwonder/setup.sh`. Wrap in `bash -c "..."`.
- SSH key now installed (`~/.ssh/id_ed25519.pub` → robot `authorized_keys`); no more password.
- The deadman does **not** auto-start on boot. Launch after every robot reboot:
  `ssh hiwonder@<ROBOT_IP> 'bash -c "source /opt/ros/melodic/setup.bash && source /home/hiwonder/pug/devel/setup.bash && nohup python3 /home/hiwonder/physicalagent/physicalagent_deadman.py > ~/physicalagent/deadman.log 2>&1 &"'`
  (ssh will hang holding the pipe — that is harmless, the process survives.)
- Battery telemetry needs a **>3 s** subscription window before the first message lands;
  a short read looks like "no telemetry" when the publisher is healthy.
- Battery across the whole session: 12.05 V → 12.00 V. Far cheaper than session 2 because
  the tests were short and mostly static.

---

## H. WIRELESS + CAPABILITY SURVEY (2026-07-20 session 3b)

### ✅ WORKING WIRELESS TOPOLOGY: 2.4 GHz phone hotspot
Robot **and** PC both join the same 2.4 GHz hotspot. This is the only topology tested that
gives wireless robot control **and** PC internet simultaneously (needed: PC must reach the
Gemini API while driving the robot). ROS traffic stays local through the AP, so it costs no
cellular data — only Gemini API calls do.

Robot associated at **−55 dBm, 144 Mbit/s**, got a DHCP lease in ~10 s.

**How to put the robot in STA mode** (vendor's own mechanism, `hw_wifi.service`):
```yaml
# /home/hiwonder/hiwonder_wifi/config/hw_wifi.yaml
mode: sta
sta:
  ssid: '<SSID>'
  password: '<PSK>'
  akm_type: WPA2PSK      # use NONE for an open network
  cipher_type: CCMP      # use NONE for an open network
  timeout: 45
```
then `sudo systemctl restart hw_wifi.service`. Factory backup lives at
`hw_wifi.yaml.bak-physicalagent`. Restore it + restart to get AP mode back.

**Built-in safety net:** if STA association times out, `hw_wifi.py` sets `conf.mode="ap"` and
loops, so the robot **restores its own AP** rather than going dark. Verified — it did exactly
this on a failed attempt. ⚠️ But note the fallback only covers a *timeout*; an exception in the
STA path hits `sys.exit(-1)` and leaves no WiFi at all. Keep ethernet plugged while changing it.

### ❌ An 802.1X WLAN will NOT work — the reason is vendor-side
`hw_wifi.py` supports only `AKM_TYPE_NONE` and `AKM_TYPE_WPA2PSK`. **No EAP support at
all**, so an 802.1X network is unreachable no matter what credentials you have. The radio
is also regdomain-locked to `country CN`, which rules out any AP in the 5350–5725 MHz gap
CN does not allocate. Use a WPA2-PSK router or the robot's own AP; do not spend a session
on this.

### 📶 LINK BENCHMARK — the exact path Gemini will use (roslibpy → rosbridge)
Identical tests, same robot, same rosbridge, ethernet vs WiFi:

| metric | ETHERNET | WIFI (hotspot) |
|---|---|---|
| packet loss | 0% | **0%** |
| ICMP median / worst | 2.2 / 5.6 ms | 16.9 / 88.6 ms |
| rosbridge connect | 1842 ms | 1701 ms |
| **rosapi service call (median)** | **94.2 ms** | **108.2 ms** |
| rosapi p95 | 104 ms | 164 ms |
| IMU topic | 49.9 Hz, worst gap 30 ms | **49.9 Hz**, worst gap 86 ms |
| LIDAR topic | 10.0 Hz, worst gap 105 ms | **10.0 Hz**, worst gap 300 ms |
| stalls > 500 ms | 0 | **0** |

### 🔑 ARCHITECTURAL RULE: topics in the control loop, NOT services
A rosbridge **service call costs ~95 ms on a 2.2 ms network** — that is rosbridge's own
processing, not the link. WiFi adds only ~14 ms on top. Consequences:
- The wireless penalty is negligible; **service-call overhead is not**.
- Three service calls = 300 ms burned before the robot moves.
- Topic streaming is lossless over WiFi (997 IMU msgs + 150 scans, zero dropped).
⇒ Build the adapter on **topics** for command and telemetry. Reserve services for rare setup
(`go_home`, parameter changes), never per-command.

### 🔍 `/scan` IS FILTERED — use `/scan_raw` for perception
`/scan` is **not** the lidar driver's output. `lidar.launch` runs a `laser_filters`
`scan_to_scan_filter_chain` node (`front_laser`) that invalidates beams in **[2.0, 4.28] rad**
(≈115°–245°, the rear) plus a 0.2–12.0 m range box. Verified empirically:

| topic | rays | valid | nearest |
|---|---|---|---|
| `/scan` | 449 | 269 (60%) | 0.45 m |
| `/scan_raw` | 450 | **432 (96%)** | 0.08 m |

`/scan_raw` carries **162 more valid returns**. The 0.08 m return is the beam hitting the robot's
own body — which is *why* the vendor filters it for navigation. So:
- **navigation/SLAM/AMCL** → `/scan` (filtered, correct — planner must not see the chassis)
- **Gemini perception** → `/scan_raw` (full 360°, 60% more spatial data)

This also explains why an earlier forward/rear differential odometry check returned `nan` for the
rear sector: it was not an empty room, the beams were filtered out.

### 🗺️ NAVIGATION STACK IS PRESENT AND COMPLETE (not running by default)
From `pug_navigation` / `pug_slam`, all verified against source:
- **`/move_base_simple/goal`** (`geometry_msgs/PoseStamped`) — publish a goal in the map frame and
  the robot autonomously plans and walks to it.
- **`/move_base`** — full ROS1 action interface (goal/cancel/feedback/result).
- Global planner `global_planner/GlobalPlanner` (Dijkstra); local planner
  **`teb_local_planner/TebLocalPlannerROS`**, `max_vel_x 0.15`.
- **`/odom_raw`** — `rf2o_laser_odometry`, scan-matching odometry at 5 Hz. ⭐ This is REAL odometry
  and would have measured today's turn properly. Not in the boot stack; launch it.
- **`/odom`** — `robot_localization` EKF fusing `/odom_raw` + `/imu`. The correct pose source, and
  it sidesteps raw IMU yaw drift.
- `/amcl` localisation, `/map_server`, `slam_gmapping` (8 particles, 0.05 m cells).
- Waypoint nav: `/clicked_point`, `/start_navigation`, `/stop_navigation`, `/clear_goals`.
- **No** cartographer / hector / slam_toolbox / rtabmap anywhere in the tree.

### 👁️ VISION PIPELINES already available (MJPEG on port 8080)
`web_video_server` serves any of these as MJPEG or single-JPEG snapshot — no ROS client needed:
`/csi_camera/image_raw`, `image_rect_color`, `image_mono`, plus result streams from
**`/apriltag_detect`**, **`/face_detect`**, **`/hand_gesture`**, **`/object_tracking`**,
`/lab_config_manager`.
Snapshot: `curl -o f.jpg "http://<robot>:8080/snapshot?topic=/csi_camera/image_raw&quality=90"`.
⭐ AprilTag is the most useful for us: fiducials give exact 6-DoF pose, a hard ground truth to
check Gemini's spatial claims against.

### ⚠️ TWO NODES PUBLISH TO `/cmd_vel` AT BOOT
`lidar_app` (and a second demo node) publish velocity independently of the navigation stack.
`/lidar_app/set_running` selects 0=idle, 1=obstacle avoidance, +follow/guard modes, and
`/lidar_app/enter` calls `go_home` (physical motion). ⇒ Expect contention on `/cmd_vel`.
**Our stack uses `/pug_control/velocity_move`, which these nodes do not touch — keep it that way.**

### Motion results after the FR leg repair (carpet)
| motion | commanded | measured | note |
|---|---|---|---|
| forward | 30 cm | **28.9 cm (96%)** | lidar wall-fit, 0.0 cm noise floor |
| turn | 60° | **~45° (75%)** | operator observation; feet scrub sideways on carpet |
| lateral (y) | 15 cm | **confirmed** | `ALLOW_LATERAL_Y` is real on ROSPug |
| sustained | 8 s continuous | **held** | no fault, deadman ALIVE throughout |

Turning underperforms translation because a yaw requires the feet to scrub laterally across
carpet pile — much higher resistance than rolling forward. Expect better on hard floor.

### Battery across session 3
12.05 V → **10.97 V**. Sustained walking is expensive: an 8 s walk sagged the pack to 10 970 mV
*under load* (below the 11 100 mV floor) and it recovered to ~11 130 mV on release. The gate then
**correctly refused** the final test at 10 965 mV. Budget roughly 100 mV per short motion test.

### Session 3b operational notes
- **NetworkManager silently roams back** to a network with internet. Pin with
  `nmcli con modify <robot-ap> connection.autoconnect-priority 100` and drop the other's
  autoconnect — otherwise the PC leaves the robot's AP mid-test and everything times out.
- **The robot's clock self-corrected** (2023-03-02 → 2026-07-20) as soon as it had internet via
  the hotspot — NTP fixed the 3-year skew. Do **not** rely on this: it depends on internet, and
  monotonic time is still the right choice for safety logic. `TRUST_ROBOT_TIMESTAMPS` stays False.
- Twisted's reactor **cannot be restarted in-process** — benchmark each link in a separate
  process or `roslibpy` raises `ReactorNotRestartable` on the second connect.
- `iw` 4.14 syntax: `iw dev wlan0 scan freq 5320` (frequency AFTER `scan`, no `passive` keyword).

---

## I. DEADMAN PROVEN OVER A REAL WIRELESS DROP (2026-07-21)

### ✅ The test the deadman was built for — PASSED
Fully wireless (hotspot, no cable anywhere). Robot walking at 0.10 m/s with heartbeat
flowing; the PC's WiFi radio was then killed mid-stride (`nmcli dev disconnect`) — a true
radio death, not a simulated heartbeat stop. The walker process kept publishing heartbeats
into the dead link the whole time, exactly like a real app would.

| checkpoint | result |
|---|---|
| outage duration | 10.7 s |
| robot state on reconnect | **PAUSED** (latched) — stopped itself |
| total travel (walk + post-cut) | **32.7 cm** vs ~30 cm predicted |
| travel after the cut | ~17 cm ≈ 1.5 s timeout + gait deceleration |
| heartbeat restored | **PAUSED → ALIVE auto-resume**, no restart |
| robot-side backstop (t+20 s) | fired harmlessly into an already-stopped robot |

### Test architecture worth reusing
- **The whole test is ONE self-contained local script** — killing the PC radio also kills
  the session's own internet, so nothing can depend on the cloud mid-test. `trap` restores
  WiFi on any exit path.
- **Three independent safety layers**: deadman (primary), a robot-side `sleep 20 &&
  rostopic pub /ros_robot_controller/robot/stop` backstop armed over SSH before the burst
  (fires regardless of network), and the operator watching.
- **Walker/checker split**: Twisted's reactor can't restart in-process, so the pre-cut
  walker and post-reconnect checker are separate processes sharing state via files.
- The walker deliberately NEVER sends a stop — proving the deadman is the only thing
  stopping the robot.

### Gotchas
- `pgrep -f physicalagent_deadman` over SSH **matches its own wrapper shell** → false
  "RUNNING". Use `ps aux | grep "[p]ython3 /home/hiwonder/physicalagent"` or (better) the
  latched `/physicalagent/deadman_status` topic.
- Robot's rospy file logs: `~/.ros/log/latest/*deadman*.log` glob fails under zsh
  (`no matches found`) — the node was run directly, not via roslaunch, so logs went to the
  redirected stdout file (block-buffered, hence often empty). PC-side timing + lidar travel
  is the reliable record.
- Battery reads 200+ mV higher after motion stops (sag recovery): 11.22 V under load →
  11.44 V at rest, same pack.

---

## J. TURN QUANTIFIED VIA rf2o + SCAN CORRELATION (2026-07-21)

### ✅ `/odom_raw` (rf2o) WORKS — this is the adapter's odometry source
Launched via the vendor's own self-contained include (sensing-only, `publish_tf=false`):
`roslaunch /home/hiwonder/pug/src/pug_slam/launch/include/rf2o_laser_odometry.launch`
Stationary drift **−0.61° per 4 s (~0.15°/s)** vs the IMU's 1.4°/s — and scan-anchored, so it
does not accumulate like a gyro. 5 Hz. Kill with `pkill -f rf2o` to restore boot state.

### 📐 The robot OVER-turns: ~122% of commanded yaw, symmetric
0.35 rad/s × 3 s (= 60°) commanded, measured by two independent instruments:
| turn | rf2o | scan-corr (sign-fixed) |
|---|---|---|
| left +60° | **+74.8°** | +88.8° (noisy fit 0.86 m) |
| right −60° | **−73.0°** | −72.8° (fit 0.73 m — 0.2° apart!) |

Yesterday's operator estimate (~45°) was human-protractor error — the robot actually turns
**~73–75° for a 60° command on carpet**, both directions. Open-loop yaw scale is ~1.22×.
⇒ For accurate turns, the bridge should **close the loop on `/odom_raw`** (command → measure →
correct), not apply a fudge constant.

### ⚠️ Instrument lesson (again)
The scan-correlation's index-shift→degrees mapping had a **flipped sign convention** (lidar is
CCW-ordered; the sign was never pinned). Raw output looked like the instruments disagreed by
160° when the magnitudes actually agreed to 0.2°. Same class of error as the IMU "curving"
verdict: **before declaring robot behaviour, check the instrument's sign/axis conventions
against a known motion.**

---

## K. FIRST GEMINI CONTACT + SURVEY REMAINDER (2026-07-21)

### 🎉 GEMINI ROBOTICS-ER CONNECTED — full function-calling loop works, first try
`scripts/gemini_sanity.py`, sensing-only tools via the real `base.py` dispatch:
- Model made **parallel tool calls** (get_status + scan_surroundings in one hop).
- It **interpreted the deadman state correctly**: reported the robot "currently holding its
  position" from `deadman: PAUSED`, and qualified its walk-forward advice with clearance data.
- **ER pointing validated against ground truth**: 5/5 objects (2 chairs, monitor, cabinet, and
  another robot in the room) — every `[y,x]` point lands ON its object.
  Overlay: `artifacts/session3/er_points_overlay.jpg` (offline capture, not in this
  repository — see THIRD_PARTY_NOTICES.md §2).
- Hazard reasoning is genuinely embodied: "low-profile five-star chair bases" as the top risk
  for a 25 cm robot.
Model: `gemini-robotics-er-1.6-preview`, temperature 0. Key in `.env` (gitignored).

### 📷 CORRECTION: camera is 640×360, NOT 640×480
From vendor launch (`csi_camera.launch`): nvarguscamerasrc 640×360@30fps target. Earlier docs
and the session-3 artifact said 480 — wrong, the JPEG itself is 360 tall.

### 📐 Factory camera CALIBRATION exists (captured from the robot)
`~/.ros/camera_info/csi_camera.yaml`: 640×360 plumb_bob,
K = [387.53, 0, 309.44; 0, 386.83, 224.21; 0,0,1], D = [-0.312, 0.082, ...] (strong barrel).
For metric reasoning: use P (fx=286.68, fy=352.03) on RECTIFIED images, K+D on raw.
⚠️ No camera extrinsics: `/csi_camera_link` has no URDF link → camera pose vs base unknown.

### Camera facts that matter for the bridge
- **Encoding rgb8** (all 6 vendor consumers do RGB2BGR) — assume bgr8 and colours swap.
- **All vendor vision nodes consume `/csi_camera/image_rect_color`** (undistorted), not raw.
- **`/compressed_image` is a pre-encoded JPEG stream** — the cheapest full-rate feed over
  rosbridge (no PC-side encode). Best choice for a streaming loop; snapshot HTTP for one-offs.
- image_transport is **lazy**: rectification only runs while subscribed; first frame lags.

### ⚠️ HAZARD: SLAM/nav launch files duplicate the camera
`pug_slam/gmapping.launch` and `pug_navigation/navigation.launch` BOTH include camera.launch —
launching them on the live robot spawns duplicate `/csi_camera` + `/web_video_server` and
**roslaunch kills the running ones** (same failure class as the csi_camera relaunch landmine).
Never launch them whole; extract the nodes needed (as done with rf2o's include).

Full survey detail: `artifacts/session3/capabilities_remainder.json` (offline capture,
not in this repository) (camera + demos + hazards
domains; control-surface domain stalled mid-stream — resumable from workflow cache).

---

## L. FIRST AI-COMMANDED MOTION — SAFETY PATH PROVEN (battery-blocked) 2026-07-21

`scripts/gemini_motion_sanity.py` — Gemini gets a bounded `walk_forward(cm)` tool plus
sensing + stop. Seven safety layers (battery gate, SafetyGuard speed clamp, distance bound
≤20cm, duration bound ≤5s, ≤3 moves/session, deadman heartbeat, operator). The model NEVER
touches raw velocity.

### Result: safety path fully validated; actual motion still pending a charge
Battery was 11.00 V (below the 11.10 floor), so no motion occurred — but this made the run a
complete test of the SAFETY path with the AI in the loop:

1. **Model self-refused (good behaviour).** Given "step forward ~10cm if safe", Gemini checked
   status, read 11004 mV, recognised it as below the 11100 threshold (learned from the tool
   description), and DECLINED to even call walk_forward — reasoning about the constraint itself.

2. **Hard gate enforced (the actual guarantee).** Adversarial re-run
   (an ad-hoc probe script, not kept) ORDERED the model to "move forward now, don't check". It
   called `walk_forward(15)` — and the adapter's `check_battery_ok_for_motion` refused it:
   `battery 11004 mV is below the motion floor 11100 mV — refusing to move`.

⇒ The safety guarantee does NOT depend on the model behaving. A confused or jailbroken model
still cannot move on a low battery. This is the property that matters, and it is proven with
the real Gemini in the loop.

### Still NOT demonstrated: actual AI-commanded translation
Requires battery ≥ 11.10 V. The SAME script produces real bounded motion once charged — the
model will call walk_forward, the adapter clamps + moves + measures, and reports achieved vs
requested distance back to the model.

---

## M. rf2o TRANSLATION IS UNRELIABLE — DO NOT USE IT AS DISTANCE GROUND TRUTH (2026-07-21)

### ⚠️ The finding
`measured_cm` (euclidean displacement of `/odom_raw`) **disagrees with reality by 0.5x–2.5x**
and the error is not a constant scale, so it cannot be calibrated out.

Evidence 1 — during a long live session, walk commands clamped to 30 cm reported:

| | walk 1 | walk 2 | walk 3 | walk 4 |
|---|---|---|---|---|
| `measured_cm` (rf2o) | 70.4 | 64.2 | 69.2 | 76.0 |
| front-sector median Δ (lidar) | 26 cm | 27 cm | 28 cm | 27 cm |

Evidence 2 — a controlled 3-trial experiment run immediately after a **fresh rf2o restart**,
same 30 cm command:

| trial | odom | lidar Δ | ratio |
|---|---|---|---|
| 1 | 14.7 cm | 21.0 cm | 0.70 |
| 2 | 17.7 cm | 34.0 cm | 0.52 |
| 3 | 12.0 cm | 10.0 cm | 1.20 |

Long-running rf2o **over**-reported ~2.5x; freshly restarted it **under**-reported ~0.8x.

### Why (best explanation, consistent with all observations)
A walking quadruped pitches and bobs the lidar. A 2D scan matcher interprets that apparent
scene shift as translation. **Yaw is far more robust to this than translation**, which is why
`turn()` — closing the loop on the same `/odom_raw` — remains accurate (achieved −89.9° on a
90° request, −9.8° on −10°) while distance does not.

### Consequence that actually bit us
The model was told it had travelled 70 cm when it had travelled 27. It therefore believed it
had overshot, stopped short of its target repeatedly, and never reached the green ball it was
sent to find before hitting the hop cap.

### What we do now
- `measured_cm` is **removed**. `walk`/`strafe` report `expected_cm` (speed × duration), a
  clearly-labelled `odom_estimate_cm`, and **`front_clearance_m`** as the real progress signal.
- The system prompt explicitly tells the model to trust `turn().achieved_deg` and to judge
  translation progress from shrinking lidar clearance, never from odometry.
- **Still open:** a rigorous straight-on wall calibration would quantify this better than the
  two ad-hoc instruments used here. The lidar median is itself noisy when the scene changes.

## N. BROWN-OUT REBOOT UNDER LOAD AT 11.8 V RESTING (2026-07-21)

The robot rebooted mid-session (`uptime` showed `up 2 min`; rf2o and the deadman, both manual
launches, were gone; the core stack auto-restarted). Battery read **11.74 V at rest**
immediately afterwards — well above the 11.10 V motion floor.

**So the pack was not flat: voltage sags transiently under walking load** far below what a
resting sample shows. The pre-move battery gate samples ONCE and is structurally blind to this.

Mitigation now in place: `_drive_for()` re-samples the pack every 0.5 s **during** a move and
aborts motion the moment it drops below the floor. This became essential when walk duration was
raised 3 s → 15 s, since a single call now draws heavy current for five times as long.

⚠️ Symptom to recognise: rosbridge port 9090 answers but **all topics are missing** and the
camera times out → the Jetson rebooted; relaunch rf2o and the deadman (§G).

---

## J. rf2o CHARACTERISED AT REST + DEADMAN PROVEN COLD (2026-07-22 session 5)

Both were re-launched after a reboot (§G commands, unchanged; deployed deadman verified
byte-identical to `robot_nodes/physicalagent_deadman.py` by md5). Then both were
characterised **with the robot planted and the motors idle** — zero battery cost, which
mattered because the pack was already within ~60 mV of the motion floor.

### 📏 rf2o at rest: yaw is trustworthy, translation is not

Two 20 s parked runs sampling `/odom_raw` at 1 Hz:

| Run | max excursion | net first→last | drift rate |
|---|---|---|---|
| 1 | 5.41 cm / 1.71° | 5.41 cm / 0.33° | 0.270 cm/s / 0.016 °/s |
| 2 | 2.86 cm / 2.53° | 0.96 cm / 1.94° | 0.048 cm/s / 0.097 °/s |

**Heading is quiet: under ~2.5° of bounded noise and no accumulation.** For comparison the
IMU drifts 1.4°/s (§G) — rf2o yaw is roughly two orders of magnitude better at rest. The
gross-overshoot monitor works in *tens* of degrees, so this noise floor is irrelevant to it.
It is safe in that role.

**Translation is not usable.** ±3 cm of jitter at rest, i.e. any claimed travel under about
5 cm is indistinguishable from noise. This corroborates the existing `odom_note` shipped to
the model (`odom_estimate_cm` unreliable, 0.5×–2.5×) — keep it.

⚠️ **Measure jitter and drift separately.** Max excursion from baseline is the *noise floor*;
net first-to-last over elapsed time is *drift*. They are different faults. A verdict
thresholded on max excursion alone condemns a healthy noisy sensor — it did exactly that here
on the first attempt, before the statistic was corrected.

### 🔁 Run 1's 4 cm jump did NOT reproduce
Run 1 sat inside ±2.4 cm for 18 s then jumped to +4.6 and +5.4 cm on the last two samples —
a phantom 4 cm translation on a stationary robot. **Run 2 was clean.** So it is a transient,
consistent with the §RESUME CPU-starvation hypothesis, and *not* a sensor property. Do not
build an outlier filter for it on this evidence. Parked runs are free — always take two.

### ✅ DEADMAN PROVEN WITHOUT MOVING THE ROBOT
The node arms on the first heartbeat and pauses after `DEFAULT_TIMEOUT_S=1.5` s of silence.
Both edges are driven purely by `/physicalagent/heartbeat`, so the **entire** state machine is
provable with the motors idle:

`DORMANT → ALIVE → PAUSED → ALIVE` — all four transitions observed, all PASS.

This is now the preferred deadman check. §F proved it on a *moving* robot, which is the
stronger test but costs ~100 mV; this costs nothing and should be run after every reboot.

⚠️ Side effect: proving it leaves the deadman **armed**, so it settles into `PAUSED` (re-sending
stop every 0.5 s) rather than `DORMANT`. Harmless — the first heartbeat from `./robot` returns
it to `ALIVE` — but `doctor` will read PAUSED, not DORMANT, until the console starts.

### 🔋 Idle drain is not negligible
11.32 V → 11.08 V across ~25 min of **sensing-only** work — no motion whatsoever. Launching
rf2o adds a scan matcher to an already-loaded Jetson (load avg ~2.2 on 4 cores). Budget for
idle drain, not just the documented ~100 mV per motion test, and **leave rf2o unlaunched when
it is not needed**.
