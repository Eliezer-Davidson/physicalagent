"""RospugAdapter — the full HiWonder ROSPug tool surface for Gemini.

Composes the tested pieces: SafetyGuard (clamp/e-stop/whitelist), TurnController
(closed-loop yaw on /odom_raw), perception.summarize_scan (lidar sectors), and a
Transport (roslibpy in prod, FakeTransport in tests). The adapter itself is
therefore fully testable offline.

Design decisions (from the tool-surface design panel + verified hardware facts):
  * SIGNED single tools: walk(+fwd/-back), strafe(+left/-right), turn(+left/-right).
    One tool per axis, sign = direction — fewer tools for the model to confuse.
  * UNITS the model reasons well about: centimetres and degrees at the boundary;
    the adapter converts to the robot's SI m/s and rad internally.
  * turn() is CLOSED-LOOP (robot over-turns ~122% open-loop); walk/strafe are
    bounded-open-loop (safety is in the duration/speed/distance bounds).
  * Every motion tool shares _preflight(): battery gate, deadman-not-paused,
    motion budget. e-stop is enforced by base.execute(). Refusals are ToolResults,
    never exceptions — a malformed call must not pause the robot via the deadman.
"""
from __future__ import annotations

import math

from physicalagent import config
from physicalagent.robots.base import RobotAdapter, ToolSpec
from physicalagent.robots.perception import min_front_clearance, summarize_scan
from physicalagent.robots.transport import Clock, MonotonicClock, Transport
from physicalagent.robots.turn_control import TurnController
from physicalagent.safety import (
    Pose, SafetyError, Velocity, battery_state, clamp_velocity,
)

_R = config.ROSPUG


def _finite(value, name: str) -> float:
    """Coerce to a finite float or raise SafetyError. A NaN/inf slipping into
    the clamp maths would publish a garbage velocity (min(nan, cap) == nan),
    so non-finite numbers are refused at the boundary, before any motion."""
    f = float(value)  # raises ValueError/TypeError on garbage -> caught by execute()
    if f != f or f in (float("inf"), float("-inf")):
        raise SafetyError(f"{name} must be a finite number, got {value!r}")
    return f


class RospugAdapter(RobotAdapter):
    def __init__(self, transport: Transport, guard, *, clock: Clock | None = None,
                 robot_profile=None, frame_fetcher=None, action_groups=None,
                 max_step_cm: float = 200.0, max_strafe_cm: float = 100.0,
                 max_turn_deg: float = 180.0, max_duration_s: float = 20.0,
                 max_moves: int | None = None, settle_s: float = 0.3,
                 top_speed_ms: float = 0.30, yaw_rate: float = 0.4,
                 open_loop_turn: bool = True):
        super().__init__(guard=guard, robot_profile=robot_profile)
        self._t = transport
        self._clock = clock or MonotonicClock()
        self._frame_fetcher = frame_fetcher or self._http_frame
        # Full vendor inventory by default (session 4 decision): every built-in
        # action is exposed, the tip risk accepted (carpet, operator present). The
        # curated subset had silently blocked a directly requested move.
        # Pass action_groups=(...) to restrict again.
        self._actions = tuple(action_groups or _R.action_groups)
        self._max_step_cm = max_step_cm
        self._max_strafe_cm = max_strafe_cm
        self._max_turn_deg = max_turn_deg
        self._max_duration_s = max_duration_s
        self._max_moves = max_moves
        self._settle_s = settle_s
        self._top_speed = top_speed_ms
        self._moves_used = 0
        self.last_frame: bytes | None = None
        # The turn path publishes yaw_rate straight to the unclamped velocity
        # interface, so it must be pre-clamped to the SAME cap the guard applies
        # to walk/strafe — otherwise a locked turn would run at the unlocked rate.
        # The cap comes from the GUARD'S OWN PROFILE, not from the module-level
        # config: reaching around the guard would silently give a second robot
        # the ROSPug's yaw rate. (For the ROSPug the two are equal by
        # construction, which is exactly why the bypass went unnoticed.)
        clamped_yaw = abs(clamp_velocity(
            Velocity(yaw_rate=yaw_rate),
            guard.profile.velocity_limits(guard.motion_unlocked)).yaw_rate)
        self._turn = TurnController(
            transport, self._clock, vel_topic=_R.topic_velocity, vel_type=_R.type_velocity,
            odom_key="odom", yaw_rate=clamped_yaw, tol_deg=4.0,
            timeout_s=max_duration_s + 4.0,
            # OPEN LOOP by default: odometry under-reports rotation, so closing
            # the loop on it made the robot overshoot badly and refuse valid
            # turns. The gait is accurate; the sensor is not.
            open_loop=open_loop_turn, over_turn=config.TURN_OVER_TURN_FACTOR)

    # ---- tool surface ----------------------------------------------------
    def capabilities(self):
        return (
            ToolSpec("get_status",
                     "Read the robot's vital signs: battery (millivolts; motion is refused "
                     "below 11100, and is also aborted mid-move if the pack sags below it "
                     "under load), and safety deadman state (ALIVE/PAUSED/DORMANT).",
                     {"type": "object", "properties": {}}),
            ToolSpec("scan_surroundings",
                     "360-degree lidar summary: nearest and median obstacle distance in metres "
                     "for the front, left, right, and back sectors. Distances under 0.3 m are "
                     "collision risks. Call this before walking.",
                     {"type": "object", "properties": {}}),
            ToolSpec("look",
                     "Capture the robot's current camera view (floor-level, forward-facing). "
                     "The image is attached to your next turn so you can see what is ahead.",
                     {"type": "object", "properties": {}}),
            ToolSpec("walk",
                     "Walk straight. Positive distance_cm = forward, negative = backward. Top "
                     "speed is fixed at the hardware maximum (0.1 m/s), so a longer distance "
                     "simply walks for longer, up to ~150 cm per call. IMPORTANT: judge your "
                     "real progress from front_clearance_m (lidar), NOT from odom_estimate_cm, "
                     "which is unreliable on carpet. Only walk forward if the front is clear.",
                     {"type": "object", "properties": {"distance_cm": {
                         "type": "number", "description": "distance in cm; + forward, - backward"}},
                      "required": ["distance_cm"]}),
            ToolSpec("strafe",
                     "Step sideways without turning. Positive distance_cm = left, negative = "
                     "right. Bounded and clamped like walk.",
                     {"type": "object", "properties": {"distance_cm": {
                         "type": "number", "description": "distance in cm; + left, - right"}},
                      "required": ["distance_cm"]}),
            ToolSpec("turn",
                     "Turn in place. Positive angle_deg = left (counter-clockwise), negative = "
                     "right. The turn is timed and calibrated, and is accurate to roughly a few "
                     "degrees. odom_observed_deg is a rough cross-check only and usually reads "
                     "LOW — ignore it. To face something precisely, turn a little, look, repeat.",
                     {"type": "object", "properties": {"angle_deg": {
                         "type": "number", "description": "angle in degrees; + left, - right"}},
                      "required": ["angle_deg"]}),
            ToolSpec("set_gait",
                     "Configure the quadruped gait timings. Use to create faster/slower walking styles. "
                     "overlap_time_s: time all 4 feet touch ground (default 0.2). "
                     "swing_time_s: leg air time (default 0.18). "
                     "clearance_time_s: (default 0.02). "
                     "z_clearance_m: step height (default 0.04).",
                     {"type": "object", "properties": {
                         "overlap_time_s": {"type": "number"},
                         "swing_time_s": {"type": "number"},
                         "clearance_time_s": {"type": "number"},
                         "z_clearance_m": {"type": "number"}}}),
            ToolSpec("set_pose",
                     "Adjust body posture while standing in place (no travel). pitch_deg tilts "
                     "the nose (and the fixed camera) up/down (+up/-down, to ~46 deg); "
                     "roll_deg leans left/right; height_cm sets ride height (-22 lowest to -4 "
                     "highest, default -13); x_shift_cm shifts the body forward/back over the "
                     "feet. Combine pitch with a low height to sit back on the rear legs. "
                     "Extreme combinations can make the robot unstable — change posture in "
                     "steps and check the result.",
                     {"type": "object", "properties": {
                         "pitch_deg": {"type": "number"},
                         "roll_deg": {"type": "number"},
                         "height_cm": {"type": "number"},
                         "x_shift_cm": {"type": "number"}}}),
            ToolSpec("do_action",
                     "Run a built-in named action sequence. Available: stand, sit, lie_down, "
                     "up, 2_legs_stand, stretch, nod, shake_head, wave, shake_hands, "
                     "shake_hands1, forward_little, multi_turn, roll_turn, spacewalk, "
                     "up_stair_0 to up_stair_4, up_stair_little, kick_ball_left, "
                     "kick_ball_right, kick_ball_right_1, push-up, boxing, up_boxing, pee. "
                     "Some are dynamic and can tip the robot — make sure it has clear space.",
                     {"type": "object", "properties": {"name": {"type": "string"}},
                      "required": ["name"]}),
            ToolSpec("stop",
                     "Immediately halt all motion and hold position. Always available.",
                     {"type": "object", "properties": {}}),
        )

    def _handlers(self):
        return {"get_status": self._status, "scan_surroundings": self._scan,
                "look": self._look, "walk": self._walk, "strafe": self._strafe,
                "turn": self._turn_tool, "set_pose": self._set_pose,
                "set_gait": self._set_gait,
                "do_action": self._do_action, "stop": self._stop_tool}

    def _always_allowed_tools(self):
        return {"stop", "get_status", "scan_surroundings", "look"}

    def _motion_tool_names(self):
        """Everything that puts the body in motion. `stop` is NOT here: halting
        must stay available on a sensing-only robot, and it is the complement
        of _always_allowed_tools() above.

        set_gait is here even though publishing gait timings moves nothing by
        itself: it reconfigures how the NEXT walk executes, so on a robot whose
        motion tools are withheld it has no meaning, and offering it would put a
        motion-shaped tool in front of a model that cannot move."""
        return {"walk", "strafe", "turn", "set_pose", "set_gait", "do_action"}

    # ---- sensing ---------------------------------------------------------
    def _status(self):
        mv = self._t.latest("batt")
        return {"battery_mv": mv, "battery_volts": round(mv / 1000, 2) if mv else None,
                "battery_state": (battery_state(mv, self._guard.profile)
                                  if mv else "unknown"),
                "deadman": self._t.latest("dead") or "unknown",
                "moves_used": self._moves_used,
                "moves_remaining": self._moves_remaining()}

    def _scan(self):
        return {"sectors": summarize_scan(self._t.latest("scan"))}

    def _look(self):
        self.last_frame = self._frame_fetcher()
        return {"frame_captured": True, "bytes": len(self.last_frame or b"")}

    def _drive_for(self, duration: float, slice_s: float = 0.5):
        """Sleep for `duration` while watching the pack, in `slice_s` steps.

        Returns the sagging voltage if the battery dropped below the motion
        floor, else None. VERIFIED THE HARD WAY: the robot brown-out-rebooted
        mid-session while resting voltage still read 11.8 V — the sag happens
        under load, which a single pre-move sample cannot see. Now that one
        walk can drive for 15 s, that blind spot had to close.

        The floor is THIS robot's, taken from the guard's profile. A mains-
        powered robot declares no pack (None) and is not sampled at all — there
        is nothing to sag, and comparing against the ROSPug's 3S floor would
        abort every move a 2S (or unpowered) robot ever made.
        """
        floor = self._guard.profile.battery_motion_min_mv
        if floor is None:
            self._clock.sleep(duration)
            return None
        remaining = duration
        while remaining > 0:
            step = min(slice_s, remaining)
            self._clock.sleep(step)
            remaining -= step
            mv = self._t.latest("batt")
            if mv is None or mv != mv:
                continue          # a dropped sample mid-move is not evidence of sag
            if mv < floor:
                return mv
        return None

    def _front_clearance(self):
        """Nearest front-sector return, or None if the lidar is unavailable.

        Defensive: this only ENRICHES a completed move's report, so a missing or
        malformed scan must never turn a successful move into a failure.
        """
        try:
            return min_front_clearance(self._t.latest("scan"))
        except Exception:
            return None

    def _moves_remaining(self):
        """Remaining budget, or 'unlimited' when no cap is configured."""
        if self._max_moves is None:
            return "unlimited"
        return max(0, self._max_moves - self._moves_used)

    # ---- shared motion preflight ----------------------------------------
    def _preflight(self):
        self._guard.guard_battery(self._t.latest("batt"))
        # Fail CLOSED on the deadman, mirroring the battery gate: motion is only
        # safe while supervision is confirmed ALIVE. None (node down / telemetry
        # not arrived), DORMANT (never armed), PAUSED (link lost), or a stale/
        # unexpected value all refuse — never assume supervision we cannot see.
        dead = self._t.latest("dead")
        if dead != "ALIVE":
            raise SafetyError(
                f"deadman is {dead!r}, not ALIVE — motion refused (supervision not confirmed)")
        if self._max_moves is not None and self._moves_used >= self._max_moves:
            raise SafetyError(f"motion budget exhausted ({self._max_moves} moves used this session)")

    def _duration_cap(self) -> float:
        """Tighter of the adapter's own cap and THIS ROBOT'S per-call cap, so
        locked motion is genuinely short-and-check.

        The per-call cap comes from the guard's profile rather than config, so a
        robot whose safe move length differs from the ROSPug's actually gets it.
        """
        return min(self._max_duration_s,
                   self._guard.profile.max_duration_s(self._guard.motion_unlocked))

    def _linear_move(self, distance_cm, axis, max_cm):
        self._preflight()
        requested = _finite(distance_cm, "distance_cm")
        bounded = max(-max_cm, min(requested, max_cm))
        if abs(bounded) < 1e-6:
            return {"requested_cm": requested, "allowed_cm": 0.0, "note": "no motion"}
        vec = {"x": 0.0, "y": 0.0, "yaw_rate": 0.0}
        vec[axis] = math.copysign(self._top_speed, bounded)
        safe = self._guard.guard_velocity(Velocity(x=vec["x"], y=vec["y"]))
        speed = getattr(safe, axis)
        if abs(speed) < 1e-9:
            return {"requested_cm": requested, "allowed_cm": round(bounded, 1),
                    "note": f"{axis}-axis motion not available on this robot"}
        duration = min(abs(bounded) / 100.0 / abs(speed), self._duration_cap())
        self._moves_used += 1
        before = self._odom_xy()
        self._t.publish(_R.topic_velocity, _R.type_velocity,
                        {"x": float(safe.x), "y": float(safe.y), "yaw_rate": 0.0, "stop": False})
        sag = self._drive_for(duration)
        self._t.publish(_R.topic_velocity, _R.type_velocity,
                        {"x": 0.0, "y": 0.0, "yaw_rate": 0.0, "stop": True})
        self._clock.sleep(self._settle_s)
        if sag is not None:
            raise SafetyError(
                f"battery sagged to {sag} mV mid-move "
                f"(floor {self._guard.profile.battery_motion_min_mv}) "
                f"— motion stopped partway and the robot is halted. The pack cannot "
                f"sustain this load; recharge before moving again.")
        after = self._odom_xy()

        # MEASURED ON HARDWARE: rf2o scan-matching reports translation anywhere
        # from 0.5x to 2.5x of the true distance on carpet, because a walking
        # quadruped pitches and bobs the lidar and a 2D matcher reads that as
        # travel. (Yaw is far more robust, which is why the closed-loop turn
        # still trusts this same source.) Presenting it as `measured_cm` told
        # the model it had gone 70 cm when it had gone 27, so it consistently
        # stopped short of its targets. It is kept ONLY as a labelled estimate.
        odom_estimate = None
        if before and after:
            odom_estimate = round(
                math.hypot(after[0] - before[0], after[1] - before[1]) * 100, 1)

        out = {"requested_cm": requested, "allowed_cm": round(bounded, 1),
               "commanded_speed_ms": round(speed, 3), "duration_s": round(duration, 2),
               "expected_cm": round(abs(speed) * duration * 100, 1),
               "odom_estimate_cm": odom_estimate if odom_estimate is not None else "unavailable",
               "odom_note": "odom_estimate_cm is unreliable on carpet (0.5x-2.5x); "
                            "use front_clearance_m to judge real progress",
               "moves_remaining": self._moves_remaining()}
        clearance = self._front_clearance()
        if clearance is not None:
            out["front_clearance_m"] = clearance
        return out

    def _walk(self, distance_cm):
        return self._linear_move(distance_cm, "x", self._max_step_cm)

    def _strafe(self, distance_cm):
        return self._linear_move(distance_cm, "y", self._max_strafe_cm)

    def _turn_tool(self, angle_deg):
        self._preflight()
        requested = _finite(angle_deg, "angle_deg")
        bounded = max(-self._max_turn_deg, min(requested, self._max_turn_deg))
        res = self._turn.turn(bounded)
        if res.error:
            # refused before commanding motion (e.g. no odometry) — do NOT spend
            # a budget slot on a turn that never moved the robot.
            raise SafetyError(res.error)
        self._moves_used += 1
        return {"requested_deg": requested, "allowed_deg": round(bounded, 1),
                "achieved_deg": res.achieved_deg, "reached": res.reached,
                "timed_out": res.timed_out, "method": res.method,
                "odom_observed_deg": res.odom_observed_deg,
                "moves_remaining": self._moves_remaining()}

    def _set_pose(self, pitch_deg=0.0, roll_deg=0.0, height_cm=None, x_shift_cm=0.0):
        self._preflight()
        pitch_deg = _finite(pitch_deg, "pitch_deg")
        roll_deg = _finite(roll_deg, "roll_deg")
        x_shift_m = _finite(x_shift_cm, "x_shift_cm") / 100.0
        height_m = (_finite(height_cm, "height_cm") / 100.0) if height_cm is not None \
            else config.DEFAULT_HEIGHT_M
        # Roll is exposed again (session 4); the vendor applies no
        # pose clamp at all, so our envelope is a sanity bound, not a guarantee.
        safe = self._guard.guard_pose(Pose(
            roll=math.radians(roll_deg), pitch=math.radians(pitch_deg),
            height=height_m, x_shift=x_shift_m,
            run_time=config.DEFAULT_POSE_RUN_TIME_S))
        self._t.publish(_R.topic_pose, _R.type_pose, {
            "roll": float(safe.roll), "pitch": float(safe.pitch), "yaw": 0.0,
            "height": float(safe.height), "x_shift": float(safe.x_shift),
            "stance_x": float(safe.stance_x), "stance_y": float(safe.stance_y),
            "run_time": float(safe.run_time)})
        self._clock.sleep(self._settle_s)
        return {"pitch_deg": round(math.degrees(safe.pitch), 1),
                "roll_deg": round(math.degrees(safe.roll), 1),
                "height_cm": round(safe.height * 100, 1),
                "x_shift_cm": round(safe.x_shift * 100, 1)}

    def _set_gait(self, overlap_time_s: float = None, swing_time_s: float = None, clearance_time_s: float = None, z_clearance_m: float = None):
        self._preflight()
        overlap = float(overlap_time_s) if overlap_time_s is not None else config.GAIT_OVERLAP_TIME_S
        swing = float(swing_time_s) if swing_time_s is not None else config.GAIT_SWING_TIME_S
        clearance_time = float(clearance_time_s) if clearance_time_s is not None else config.GAIT_CLEARANCE_TIME_S
        z_clearance = float(z_clearance_m) if z_clearance_m is not None else config.GAIT_Z_CLEARANCE_M

        msg = {
            "overlap_time": overlap,
            "swing_time": swing,
            "clearance_time": clearance_time,
            "z_clearance": z_clearance,
        }
        self._t.publish(_R.topic_gait, _R.type_gait, msg)
        self._clock.sleep(self._settle_s)
        return msg

    def _do_action(self, name):
        self._preflight()
        # defense in depth: the adapter's own curated list is checked FIRST, so
        # even a permissively-configured guard cannot unlock a tip-risk action.
        if name not in self._actions:
            raise SafetyError(
                f"action '{name}' is not in the adapter's safe action list "
                f"({', '.join(self._actions)})")
        safe_name = self._guard.guard_action(name)  # raises if not whitelisted
        self._moves_used += 1
        self._t.call_service(_R.srv_run_action_group, _R.type_run_action_group,
                             {"name": safe_name})
        return {"action": safe_name, "moves_remaining": self._moves_remaining()}

    def _stop_tool(self):
        self.stop_all()
        return {"stopped": True}

    # ---- helpers ---------------------------------------------------------
    def _odom_xy(self):
        m = self._t.latest("odom")
        try:
            p = m["pose"]["pose"]["position"]
            return (p["x"], p["y"])
        except (KeyError, TypeError):
            return None

    def _http_frame(self):
        import os
        import urllib.request
        # precedence: explicit env override -> config mDNS host. There is no
        # last-resort literal: an address belongs in the environment.
        host = os.environ.get("ROSPUG_HOST") or getattr(_R, "host", None)
        if not host:
            raise RuntimeError("set ROSPUG_HOST (or config.ROSPUG.host) first")
        url = f"http://{host}:8080/snapshot?topic=/csi_camera/image_raw&quality=90"
        return urllib.request.urlopen(url, timeout=10).read()

    # ---- RobotAdapter required ------------------------------------------
    def get_frame(self):
        self.last_frame = self._frame_fetcher()
        return self.last_frame

    def telemetry(self):
        return self._status()

    def stop_all(self):
        try:
            self._t.publish(_R.topic_velocity, _R.type_velocity,
                            {"x": 0.0, "y": 0.0, "yaw_rate": 0.0, "stop": True})
        except Exception:
            pass
