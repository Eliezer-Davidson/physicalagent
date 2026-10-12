"""Single source of truth: endpoints, model IDs, the robots' API surfaces, and limits.

PROVENANCE MARKERS — every constant carries one. Respect them.

    VERIFIED   Observed live on the real robot (2026-07-17 bring-up) or read from
               vendor source we hold. Trust it.
    INFERRED   Derived by sound reasoning from a datasheet/spec, but not observed.
    UNVERIFIED Assumed by analogy (usually to PuppyPi). MAY BE WRONG. Anything
               gating motion that is UNVERIFIED must stay behind MOTION_UNLOCKED.

The ROSPug is NOT a PuppyPi. Its namespace is `pug_control`, not `puppy_control`.
Every topic below marked VERIFIED was read from the live ROS graph over rosbridge;
see docs/rospug_ros_graph.txt and docs/hardware_notes.md §C.
"""
from __future__ import annotations

import os
from dataclasses import dataclass, field

from dotenv import load_dotenv

load_dotenv()

# ---------------------------------------------------------------------------
# Gemini models
# ---------------------------------------------------------------------------
GEMINI_API_KEY: str | None = os.getenv("GEMINI_API_KEY") or None

# VERIFIED (2026-07-17 research): ER 1.6 is standard Gemini API, text-out only,
# no Live API support -> voice needs the separate Live model below.
ER_MODEL = os.getenv("ER_MODEL", "gemini-robotics-er-1.6-preview")
LIVE_MODEL = os.getenv("LIVE_MODEL", "gemini-3.1-flash-live-preview")

# Thinking budgets: 0 is fastest and recommended for pointing; raise for planning.
THINKING_BUDGET_POINTING = 0
THINKING_BUDGET_PLANNING = 1024

# VERIFIED: spatial conventions. Points are [y, x] normalized 0-1000 (y FIRST).
# Boxes are [ymin, xmin, ymax, xmax] normalized 0-1000.
COORD_NORM = 1000


# ---------------------------------------------------------------------------
# ROSPug (HiWonder, SKU HIWND-380) — Jetson Nano quadruped
# ---------------------------------------------------------------------------
@dataclass(frozen=True)
class ROSPugConfig:
    # VERIFIED: mDNS works and survives DHCP changes; prefer it over a pinned IP.
    host: str = os.getenv("ROSPUG_HOST", "rospug.local")
    rosbridge_port: int = int(os.getenv("ROSPUG_ROSBRIDGE_PORT", "9090"))
    video_port: int = int(os.getenv("ROSPUG_VIDEO_PORT", "8080"))
    # SSH is used only by the read-only capture script. No credential default
    # ships here: set ROSPUG_SSH_USER / ROSPUG_SSH_PASS (the vendor documents
    # its own factory pair) or the capture script stops and says so.
    ssh_user: str = os.getenv("ROSPUG_SSH_USER", "")
    ssh_pass: str = os.getenv("ROSPUG_SSH_PASS", "")

    # --- Camera ---------------------------------------------------------
    # VERIFIED: it is a CSI/MIPI IMX219 at /csi_camera/*, NOT /usb_cam/image_raw.
    # We use image_raw deliberately: it comes straight from gscam. The rectified
    # topics depend on image_proc, which we observed being fragile (name-collision
    # churn), whereas image_raw published reliably.
    camera_topic: str = "/csi_camera/image_raw"
    camera_topic_fallbacks: tuple[str, ...] = (
        "/csi_camera/image_color",
        "/csi_camera/image_rect_color",
    )
    camera_width: int = 640   # VERIFIED live
    camera_height: int = 360  # VERIFIED live

    # VERIFIED THE HARD WAY: web_video_server BLOCKS FOREVER when a topic is
    # advertised but has no publisher. Never fetch a frame without a timeout or
    # the whole agent loop deadlocks silently. This bit us during bring-up.
    frame_timeout_s: float = 8.0

    # --- Control topics (ALL VERIFIED live over rosbridge) ---------------
    topic_velocity: str = "/pug_control/velocity_move"
    type_velocity: str = "pug_control/Velocity"        # {x, y, yaw_rate, stop}
    topic_pose: str = "/pug_control/pose"
    type_pose: str = "pug_control/Pose"                # {roll,pitch,yaw,height,x_shift,stance_x,stance_y,run_time}
    topic_gait: str = "/pug_control/gait"
    type_gait: str = "pug_control/Gait"                # {overlap_time,swing_time,clearance_time,z_clearance}
    topic_cmd_vel: str = "/cmd_vel"
    type_cmd_vel: str = "geometry_msgs/Twist"

    topic_bus_servo: str = "/ros_robot_controller/bus_servo/set_state"
    type_bus_servo: str = "ros_robot_controller/SetBusServoState"
    topic_robot_stop: str = "/ros_robot_controller/robot/stop"
    type_robot_stop: str = "ros_robot_controller/Stop"
    topic_enable_reception: str = "/ros_robot_controller/enable_reception"
    type_enable_reception: str = "std_msgs/Bool"

    # --- Telemetry topics (ALL VERIFIED streaming live) ------------------
    topic_battery: str = "/ros_robot_controller/battery"   # std_msgs/UInt16, millivolts
    topic_imu: str = "/imu"                                # fused
    topic_imu_raw: str = "/ros_robot_controller/imu_raw"
    topic_scan: str = "/scan"                              # sensor_msgs/LaserScan
    topic_joint_states: str = "/joint_states"              # sensor_msgs/JointState

    # --- Services (types VERIFIED live) ---------------------------------
    srv_go_home: str = "/pug_control/go_home"                     # std_srvs/Empty
    srv_run_action_group: str = "/pug_control/run_action_group"   # pug_control/SetActionName{name}
    type_run_action_group: str = "pug_control/SetActionName"
    srv_self_balancing: str = "/self_balancing/set_running"       # std_srvs/SetBool
    srv_heartbeat: str = "/performance/heartbeat"                 # std_srvs/SetBool
    srv_lidar_app: str = "/lidar_app/set_running"                 # pug_control/SetInt64

    # --- REAL joint feedback (VERIFIED 2026-07-20) -----------------------
    # /joint_states is FAKE (stock joint_state_publisher w/o source_list -> zeros,
    # for RViz/TF only). TF leg transforms are fake for the same reason. Use these:
    srv_leg_position: str = "/ros_robot_controller/robot/get_leg_position"
    type_leg_position: str = "ros_robot_controller/GetLegPosition"
    srv_bus_servo_state: str = "/ros_robot_controller/bus_servo/get_state"
    type_bus_servo_state: str = "ros_robot_controller/GetBusServoState"
    # get_leg_position returns 12 floats, METRES, laid out as [X x4, Y x4, Z x4],
    # leg order FR, FL, BR, BL. VERIFIED: commanding height -0.13 -> -0.09 moved
    # the four Z values by exactly +0.04.
    leg_order: tuple[str, ...] = ("FR", "FL", "BR", "BL")
    # bus_servo/get_state: position (0-1000) and temperature (degC) and id all WORK.
    # *** NEVER set get_voltage=1 *** -> vendor bug crashes the whole service call:
    #   'Board' object has no attribute 'bus_servo_read_voltage'
    BUS_SERVO_VOLTAGE_BROKEN: bool = True

    # VERIFIED: 12 DOF, these exact names, in this order, from live /joint_states.
    joint_names: tuple[str, ...] = (
        "lf_joint", "lf_thigh", "lf_calf",
        "lb_joint", "lb_thigh", "lb_calf",
        "rf_joint", "rf_thigh", "rf_calf",
        "rb_joint", "rb_thigh", "rb_calf",
    )

    # --- Action groups (VERIFIED 2026-07-20 from the robot) --------------
    # Files live at /home/hiwonder/software/action_editor/ActionGroups/*.d6a
    # (SQLite keyframe tables, 12 joints), NOT in the pug ROS workspace.
    #
    # *** NAME FORMAT: WITHOUT the extension. *** action_group_controller.py does
    #     actNum = os.path.join(path, 'ActionGroups', actNum + ".d6a")
    # so pass "stand", never "stand.d6a". (The pre-bring-up PuppyPi
    # assumption was to INCLUDE the extension — wrong for this robot.)
    #
    # Deliberately EXCLUDED as unknown-content scratch files someone left on the
    # robot — they could drive the servos anywhere: 111, 123, temp, test, 0_050.
    action_groups: tuple[str, ...] = (
        # posture / basics
        "stand", "sit", "lie_down", "up", "2_legs_stand", "stretch",
        # head / greeting
        "nod", "shake_head", "wave", "shake_hands", "shake_hands1",
        # locomotion extras
        "forward_little", "multi_turn", "roll_turn", "spacewalk",
        # stairs
        "up_stair_0", "up_stair_1", "up_stair_2", "up_stair_3", "up_stair_4",
        "up_stair_little",
        # ball
        "kick_ball_left", "kick_ball_right", "kick_ball_right_1",
        # tricks / demos
        "push-up", "boxing", "up_boxing", "pee",
    )

    # ⚠️ NOT IN USE, and NOT the allow-list. Decided in session 4 (reaffirmed
    # 2026-07-22): Gemini gets the FULL `action_groups` inventory above, within the
    # hardware/safety limits the guard enforces. Every live call site — cli.build_guard,
    # RospugAdapter, the scripts — passes `action_groups`, not this. Kept only as a
    # ready-made conservative list for bringing up an UNTESTED robot, whose tip
    # behaviour we do not yet know. If you wire it in, change the call sites
    # deliberately; do not assume this is already the default.
    #
    # Curated SAFE subset. Excludes
    # every dynamic tip/collision trajectory — two-leg stands, spacewalk, roll,
    # stair climbs, ball kicks, boxing, push-up, pee (leg-lift) — leaving static
    # postures and gentle in-place gestures. Locomotion is done via walk/turn,
    # not action groups. The full `action_groups` above stays the ground-truth
    # inventory; this is the allow-list.
    safe_action_groups: tuple[str, ...] = (
        "stand", "sit", "lie_down", "up", "stretch",
        "nod", "shake_head", "wave", "shake_hands", "shake_hands1",
    )


ROSPUG = ROSPugConfig()


# ---------------------------------------------------------------------------
# TonyPi (HiWonder, standard) — humanoid.  No ROS.
# Source of truth: reference/TonyPi (vendor source we hold) => VERIFIED.
# NOTE: not yet brought up on hardware; verified from source, not observation.
# ---------------------------------------------------------------------------
@dataclass(frozen=True)
class TonyPiConfig:
    # 192.168.149.1, not tonypi.local. The vendor manual is explicit that the
    # robot boots into AP-direct mode and raises its own "HW-xxxxxxxx" network
    # with itself as the gateway, so this address is correct by construction
    # whenever you are talking to the robot at all. `tonypi.local` needs mDNS,
    # which does not resolve over that AP link — and the failure is ugly: it
    # surfaces as `[Errno -2] Name or service not known` from inside a tool
    # call, so the model sees "vision is broken" rather than "wrong host".
    host: str = os.getenv("TONYPI_HOST", "192.168.149.1")
    rpc_port: int = int(os.getenv("TONYPI_RPC_PORT", "9030"))
    video_port: int = int(os.getenv("TONYPI_VIDEO_PORT", "8080"))
    # As above: no credential default ships here.
    ssh_user: str = os.getenv("TONYPI_SSH_USER", "")
    ssh_pass: str = os.getenv("TONYPI_SSH_PASS", "")

    frame_timeout_s: float = 8.0
    snapshot_path: str = "/?action=snapshot"

    # VERIFIED from reference/TonyPi/ActionGroupDict.py.
    # RunAction takes the numeric-string CODE, not the human name.
    action_codes: dict[str, str] = field(default_factory=lambda: {
        "stand": "0", "go_forward": "1", "back_fast": "2",
        "left_move_fast": "3", "right_move_fast": "4",
        "turn_left": "7", "turn_right": "8",
        "wave": "9", "bow": "10", "squat": "11",
        "chest": "13", "left_shot_fast": "14", "wing_chun": "15",
        "left_uppercut": "16", "right_uppercut": "17",
        "left_kick": "18", "right_kick": "19",
        "stand_up_front": "20", "stand_up_back": "21",
        "twist": "22", "stepping": "24",
    })

    # VERIFIED: head servos only. 1 = tilt, 2 = pan. Pulse 500-2500, neutral 1500.
    head_tilt_servo: int = 1
    head_pan_servo: int = 2
    pwm_pulse_min: int = 500
    pwm_pulse_max: int = 2500
    pwm_pulse_neutral: int = 1500
    # VERIFIED: body bus servos, ids 1-16, pulse 0-1000.
    bus_servo_id_min: int = 1
    bus_servo_id_max: int = 16
    bus_pulse_min: int = 0
    bus_pulse_max: int = 1000
    min_move_time_ms: int = 200
    # VERIFIED: 7s watchdog auto-unloads a loaded Func. Only beat while one is loaded.
    heartbeat_interval_s: float = 3.0


TONYPI = TonyPiConfig()


# ---------------------------------------------------------------------------
# MiniHexa (HiWonder) — hexapod. No ROS, no JSON-RPC: a raw wire protocol over
# CH340 serial OR BLE. Source of truth: HiWonder's published firmware
# (reference/MiniHexa) + a live bring-up capture => VERIFIED protocol, but the
# robot is not yet driven under our control (motion stays locked).
# ---------------------------------------------------------------------------
@dataclass(frozen=True)
class MiniHexaConfig:
    # VERIFIED: CH340 USB-serial, 115200 8N1, the full-control transport.
    device: str = os.getenv("MINIHEXA_DEVICE", "/dev/ttyUSB0")
    baud: int = int(os.getenv("MINIHEXA_BAUD", "115200"))
    # VERIFIED live: BLE device "miniHexa", GATT service ffe0, write char ffe1,
    # notify char ffe2. The BLE command set is REDUCED (no L, no K|2).
    # REQUIRED, no default: a BLE MAC is a persistent hardware identifier of one
    # specific robot, so it comes from MINIHEXA_BLE_ADDRESS and nowhere else.
    # Find yours with `bluetoothctl scan on` -- the device advertises "miniHexa".
    ble_address: str = os.getenv("MINIHEXA_BLE_ADDRESS", "")
    ble_service: str = "0000ffe0-0000-1000-8000-00805f9b34fb"
    ble_write_char: str = "0000ffe1-0000-1000-8000-00805f9b34fb"
    ble_notify_char: str = "0000ffe2-0000-1000-8000-00805f9b34fb"


MINIHEXA = MiniHexaConfig()


@dataclass(frozen=True)
class AiHandConfig:
    """The HiWonder AiHand mounted on the TonyPi's arm.

    OPT-IN, and off by default: a hand that is not physically bolted on must not
    be offered to the model. Set $AIHAND=1 once the hand is attached.

    TRANSPORT. The hand's controller is a micro:bit V2 (VID:PID 0d28:0204)
    presenting a USB CDC serial port on whichever computer it is plugged into.
    Plugged into this laptop, that is `serial` and /dev/ttyACM0. Plugged into
    the ROBOT's Raspberry Pi — the normal arrangement, since the hand rides on
    the arm — nothing on this machine can see it, so a small bridge process on
    the Pi forwards a TCP port to the serial device and the transport is `tcp`.
    See tools/aihand_bridge.py.

    FRAMING is UNVERIFIED. See aihand_codec: the fleet spec records `0x55 0x55`
    (lobot) for this hand but nothing has ever been sent to it, and the sibling
    MiniHexa speaks the ASCII form over a comparable link. $AIHAND_FRAMING picks
    which candidate to send so the probe can settle it empirically.
    """

    enabled: bool = os.getenv("AIHAND", "0").strip().lower() in ("1", "true", "yes")
    transport: str = os.getenv("AIHAND_TRANSPORT", "tcp")
    # tcp: the bridge on the robot's own computer. Defaults to the TonyPi's
    # address because that is the computer the hand is plugged into.
    host: str = os.getenv("AIHAND_HOST", os.getenv("TONYPI_HOST", "192.168.149.1"))
    port: int = int(os.getenv("AIHAND_PORT", "9031"))
    # serial: the micro:bit's CDC port on THIS machine.
    device: str = os.getenv("AIHAND_DEVICE", "/dev/ttyACM0")
    baud: int = int(os.getenv("AIHAND_BAUD", "115200"))
    framing: str = os.getenv("AIHAND_FRAMING", "lobot")


AIHAND = AiHandConfig()


# ---------------------------------------------------------------------------
# SAFETY LIMITS
# ---------------------------------------------------------------------------
# *** UNITS ARE NOW VERIFIED — SI: METRES and SECONDS. ***
# Resolved 2026-07-20 by reading the vendor source we copied off the robot
# (reference/rospug_capture_2026-07-20_125345/src). The evidence:
#
#   ros_robot_controller_sdk.py:
#       def move(x_move_amplitude, y_move_amplitude, angle_move_amplitude):
#           # 移动, 单位m            <- "movement, unit m"      => m/s
#       def stop(duration):
#           # 停止, 运行时间s        <- "stop, runtime s"       => SECONDS
#       def set_gait(...):
#           # z_clearance: 步高m     <- "step height m"         => METRES
#   pug_node.py cmd_vel_callback():  move_msg.x = msg.linear.x
#       -> a raw pass-through from geometry_msgs/Twist, whose ROS convention is m/s.
#   walking_param.yaml: height = -0.13, z_clearance = 0.04  (i.e. -13 cm, 4 cm)
#
# NOTE HOW CLOSE THIS WAS: the old "safe under either unit" guess was 0.2, but
# the vendor's OWN clamp is 0.1 m/s. Our conservative value was 2x the vendor
# maximum. MOTION_UNLOCKED=False is the only reason nothing was ever sent.
#
# *** AND THE CRITICAL FINDING: THE VENDOR DOES NOT CLAMP. ***
# pug_node.move_callback() publishes x/y/yaw straight through to
# /ros_robot_controller/robot/move with NO limiting. pose_callback() likewise.
# ONLY the /cmd_vel path is clamped (cmd_vel_limit_callback). So on the primary
# interface THIS SAFETY LAYER IS THE ONLY GUARD BETWEEN A BAD NUMBER AND 12
# SERVOS. Treat every limit below as load-bearing.
# ---------------------------------------------------------------------------

# Gate for the *first supervised motion*, not for units any more. Stays False
# until a human has watched this robot move under our control.
MOTION_UNLOCKED: bool = os.getenv("PHYSICALAGENT_MOTION_UNLOCKED", "0") == "1"

# --- Vendor's own limits (VERIFIED from source) -----------------------------
# cmd_vel_limit_callback(): x +/-0.1, y +/-0.1, yaw +/-0.4
# app_control_move_callback(): y fixed +/-0.05, yaw fixed +/-0.35
VENDOR_MAX_ABS_X_M_S = 0.1
VENDOR_MAX_ABS_Y_M_S = 0.1
VENDOR_MAX_ABS_YAW_RAD_S = 0.4
VENDOR_APP_ABS_Y_M_S = 0.05
VENDOR_APP_ABS_YAW_RAD_S = 0.35

# Pre-first-motion caps: half the vendor's app values. Deliberately timid.
UNVERIFIED_MAX_ABS_X = 0.05          # m/s
UNVERIFIED_MAX_ABS_Y = 0.025         # m/s
UNVERIFIED_MAX_ABS_YAW = 0.15        # rad/s
UNVERIFIED_MAX_DURATION_S = 1.0

# Post-verification caps: the vendor's own conservative (app-path) values.
# Never exceed VENDOR_MAX_* above.
VERIFIED_MAX_ABS_X_M_S = 0.1         # == vendor cmd_vel clamp
VERIFIED_MAX_ABS_Y_M_S = 0.1         # == vendor cmd_vel clamp (was the app's 0.05)
VERIFIED_MAX_ABS_YAW_RAD_S = 0.4     # == vendor cmd_vel clamp (was the app's 0.35)
# Raised 3.0 -> 15.0 so a single walk can cover ~1.5 m instead of 30 cm. Note
# x speed is ALREADY at the vendor ceiling (0.1 m/s): on this robot "faster"
# is only achievable as "longer", never as a higher commanded speed.
VERIFIED_MAX_DURATION_S = 15.0

# VERIFIED: lateral y IS implemented on the ROSPug (Move.msg has y; the app path
# drives it at +/-0.05 m/s). This differs from the PuppyPi, where y was a no-op.
ALLOW_LATERAL_Y = True

# --- Pose limits, in METRES and RADIANS ------------------------------------
# INFERRED: the vendor applies NO pose clamp at all, so these are ours alone.
# Anchored on walking_param.yaml defaults (height -0.13 m, x_shift 0.002-0.014 m)
# and the SDK's IK example (Z = -0.15 m). Conservative until exercised.
# OPENED UP in session 4 (decision: go wide open, the vendor applies
# no clamp). These were never vendor limits — pug_node applies NO pose clamp at
# all, so they were ours alone and were blocking requested postures (a 30 deg
# pitch request came back clamped to 11.5). They remain a finite sanity envelope
# rather than a protective one: an extreme pitch+height combination can still
# drive the leg IK to a pose the robot cannot hold, so change posture in steps.
# These were briefly set to +/-inf under the heading "full control". Restored,
# because infinity is not more control — it is the same control with the sanity
# check deleted. The firmware clamps regardless (a 46 deg pitch request came
# back as 11.5), so an unbounded value does not reach further; it only means the
# model is told its pose was accepted when it was not, and that a pitch+height
# combination the legs cannot hold gets published with nothing left to catch it.
# The values below ARE the widened, full-range setting, verified against the
# vendor stack. Widen them further only with a hardware observation to cite.
MAX_ABS_ROLL_RAD = 0.8               # ~46 deg (was 0.2)
MAX_ABS_PITCH_RAD = 0.8              # ~46 deg (was 0.2)
HEIGHT_MIN_M = -0.22                 # was -0.15 (SDK IK example floor)
HEIGHT_MAX_M = -0.04                 # was -0.08
DEFAULT_HEIGHT_M = -0.13             # walking_param.yaml
MAX_ABS_X_SHIFT_M = 0.08             # was 0.03
MAX_ABS_STANCE_M = 0.08              # was 0.03
DEFAULT_POSE_RUN_TIME_S = 0.5        # VERIFIED: this field is SECONDS, not ms

# --- Gait defaults (VERIFIED from walking_param.yaml; seconds + metres) -----
GAIT_OVERLAP_TIME_S = 0.2
GAIT_SWING_TIME_S = 0.18
GAIT_CLEARANCE_TIME_S = 0.02
GAIT_Z_CLEARANCE_M = 0.04

# Bounded actions: continuous motion requires re-invocation by the model.
MAX_STEPS_PER_CALL = 4

# VERIFIED (docs/hardware_notes.md section J): the robot rotates ~122% of
# the commanded yaw, symmetric. Open-loop turns divide by this.
TURN_OVER_TURN_FACTOR = 1.22

# --- Battery ---------------------------------------------------------------
# VERIFIED live: 3S LiPo; we measured 10345mV then ~10045mV while it drained.
# 4.20 V/cell = 12.6 V full | 3.70 = 11.1 nominal | 3.30 = 9.9 damage floor.
BATTERY_CELLS = 3
BATTERY_FULL_MV = 12600
BATTERY_NOMINAL_MV = 11100
BATTERY_FLOOR_MV = 9900
# Motion is gated on this. Below it, refuse to move: servos brown out mid-motion
# and the pack over-discharges.
BATTERY_MOTION_MIN_MV = 11100
BATTERY_WARN_MV = 10500

# --- Watchdog --------------------------------------------------------------
WATCHDOG_TIMEOUT_S = 1.0   # if the orchestrator stops polling, stop within 1s
RATE_LIMIT_HZ = 10.0       # max command rate per adapter

# --- Clock -----------------------------------------------------------------
# VERIFIED: the ROSPug's clock read 2023-03-02 (Jetson has no RTC battery).
# NEVER trust robot-side message timestamps. Stamp episodes PC-side.
TRUST_ROBOT_TIMESTAMPS = False
