"""ZmqSonicLink — the real transport to NVIDIA's SONIC policy, over ZMQ.

VERIFIED 2026-07-31 by source audit of gear_sonic_deploy (file:line below). Not
yet verified by a robot moving, which is why sonic_link.VERIFIED_COMMAND_PATH
stays None until a human watches it and dates the promotion.

THE WIRE FORMAT
    [topic bytes][1280-byte null-padded JSON header][concatenated LE buffers]

    HEADER_SIZE is 1280 (zmq_packed_message_subscriber.hpp:99). Two example
    senders shipped in that repo pad to 1024 and are silently never parsed —
    the correct reference is tests/test_zmq_manager.py. Getting this wrong
    produces no error and no motion, so the constant is pinned by a test.

    The deploy binary's socket is SUB and CONNECTs, so WE are the PUB and we
    bind (zmq_packed_message_subscriber.hpp:190,201-202).

TWO TOPICS
    command : start(u8) stop(u8) planner(u8)          input_command.hpp:38-47
    planner : mode(i32) movement(f32[3]) facing(f32[3])
              speed(f32) height(f32)                  input_command.hpp:71-107

WHY THERE IS NO vx/vy/wz
    SONIC's planner does not take a twist. Its signature is
    (mode, target_vel, target_height, movement_direction[3], facing_direction[3])
    — localmotion_kplanner.hpp:688-693. Velocity is a MODE plus a unit DIRECTION
    plus a scalar SPEED, and yaw rate is expressed as a facing vector that
    changes over time. This class does that conversion so the tool surface above
    it can stay in the plain terms a language model reasons well about.
    (A literal [vx, vy, wz] exists only on the ROS2 path, which is compiled out
    of the binary on this machine.)

WHY A BACKGROUND PUBLISHER
    A planner message older than ~1 s is treated as stale and the robot falls
    back to IDLE (input_command.hpp:104-106). So a command is not a single send;
    it is a stream. The thread here re-publishes the current intent at 20 Hz and
    stops publishing when the command expires — which makes expiry the DEFAULT
    behaviour rather than something the caller must remember. A dead caller
    leaves a robot that stops, not one that keeps walking.

WORLD FRAME
    movement and facing are WORLD-frame unit vectors (planner_onnx.md: MuJoCo
    Z-up, X forward, Y left). Body-relative requests are rotated by our own
    heading estimate, which is dead-reckoned from the yaw we have commanded.
    That estimate is good enough to steer a demo and is NOT a substitute for
    reading the robot's real heading; it is labelled as an estimate everywhere
    it surfaces.
"""
from __future__ import annotations

import json
import math
import struct
import threading
import time

from physicalagent.robots import sonic_modes, sonic_upper_body
from physicalagent.robots.sonic_upper_body import NUM_UPPER_BODY_JOINTS
from physicalagent.robots.sonic_link import (
    MAX_VX_M_S,
    SonicLink,
    SonicState,
    clamp_duration,
    clamp_velocity,
)

HEADER_SIZE = 1280          # zmq_packed_message_subscriber.hpp:99 — NOT 1024
PUBLISH_HZ = 20.0           # planner staleness is ~1 s; 20 Hz is comfortable
DEFAULT_PORT = 5556

# LocomotionMode, localmotion_kplanner.hpp:78-106. The full table of 27 lives in
# sonic_modes.py; these four are named here because the velocity path picks
# between them by speed rather than by name.
MODE_IDLE = 0
MODE_SLOW_WALK = 1          # 0.1-0.8 m/s
MODE_WALK = 2               # 0.8-2.5 m/s
MODE_SQUAT = 4              # IDEL_SQUAT

SLOW_WALK_MAX_M_S = 0.8

# How fast the upper-body reference is allowed to move, radians per second per
# joint. 1.2 rad/s takes about 1.2 s for a 70-degree shoulder swing, which reads
# as deliberate rather than as a flinch. SONIC rate-limits the Dex3 hands but NOT
# `upper_body_position`, so without this the reference is a step change and the
# policy chases it as hard as its gains allow.
UPPER_BODY_SLEW_RAD_S = 1.2


def _pack(topic: bytes, fields: list[dict], payload: bytes) -> bytes:
    """One packed message. Raises if the header would overflow its fixed slot —
    a truncated header is a message that is silently never parsed."""
    header = json.dumps({"v": 1, "endian": "le", "count": 1,
                         "fields": fields}).encode()
    if len(header) > HEADER_SIZE:
        raise ValueError(f"header {len(header)}B exceeds the {HEADER_SIZE}B slot")
    return topic + header + b"\x00" * (HEADER_SIZE - len(header)) + payload


def encode_command(start: bool, stop: bool, planner: bool) -> bytes:
    return _pack(b"command",
                 [{"name": n, "dtype": "u8", "shape": [1]}
                  for n in ("start", "stop", "planner")],
                 struct.pack("BBB", int(start), int(stop), int(planner)))


def encode_planner(mode: int, movement: tuple[float, float, float],
                   facing: tuple[float, float, float],
                   speed: float, height: float,
                   upper_body: "list[float] | None" = None) -> bytes:
    """The planner message, optionally carrying an upper-body joint reference.

    UPPER BODY IS AN OPTIONAL FIELD ON THIS SAME TOPIC
        `upper_body_position`, 17 f32 (zmq_manager.hpp:787,885-913). When present
        the deploy binary sets has_upper_body_control_ and feeds the values to
        the policy as `ref_upper_dof_pos`, which its observation config already
        lists. Absent, the policy uses its own `loco_upper_body_dof_pos` default.

        This is why arm control does NOT require streaming poses, switching the
        policy out of planner mode, or touching its keyboard: the legs stay on
        the planner, balance is never interrupted, and all 27 locomotion modes
        remain available. It was easy to miss -- the field appears in the
        vendor's own test sender (tests/test_zmq_manager.py:126) and in no
        documentation.

        The field must be sent on EVERY message while it is wanted, not once:
        has_upper_body_control_ is recomputed from the latest planner message
        (zmq_manager.hpp:259,588), so omitting it hands the arms straight back
        to the policy.

        `upper_body` arrives in OUR serial order and is permuted to the policy's
        IsaacLab order here -- this is the one and only wire boundary, so it is
        the one and only place that permutation is applied. See the
        sonic_upper_body docstring for what shipping it unpermuted did.
    """
    fields = [{"name": "mode", "dtype": "i32", "shape": [1]},
              {"name": "movement", "dtype": "f32", "shape": [3]},
              {"name": "facing", "dtype": "f32", "shape": [3]},
              {"name": "speed", "dtype": "f32", "shape": [1]},
              {"name": "height", "dtype": "f32", "shape": [1]}]
    payload = (struct.pack("<i", int(mode))
               + struct.pack("<fff", *movement)
               + struct.pack("<fff", *facing)
               + struct.pack("<f", float(speed))
               + struct.pack("<f", float(height)))
    if upper_body is not None:
        if len(upper_body) != NUM_UPPER_BODY_JOINTS:
            raise ValueError(f"upper_body must be {NUM_UPPER_BODY_JOINTS} values, "
                             f"got {len(upper_body)}")
        fields.append({"name": "upper_body_position", "dtype": "f32",
                       "shape": [NUM_UPPER_BODY_JOINTS]})
        payload += struct.pack(f"<{NUM_UPPER_BODY_JOINTS}f",
                               *sonic_upper_body.to_wire(upper_body))
    return _pack(b"planner", fields, payload)


class ZmqSonicLink(SonicLink):
    """Publishes SONIC's planner intent at 20 Hz until each command expires."""

    def __init__(self, *, host: str = "*", port: int = DEFAULT_PORT,
                 settle_s: float = 0.5, auto_start: bool = True):
        import zmq

        self._ctx = zmq.Context.instance()
        self._pub = self._ctx.socket(zmq.PUB)
        self._pub.bind(f"tcp://{host}:{port}")
        # PUB/SUB drops messages sent before the subscriber attaches; without
        # this pause the start command is lost and nothing ever moves.
        time.sleep(settle_s)

        self._lock = threading.Lock()
        self._yaw = 0.0                  # dead-reckoned heading ESTIMATE
        self._mode = MODE_IDLE
        self._speed = 0.0
        self._yaw_rate = 0.0
        self._height = -1.0              # -1 means "mode default"
        self._expires_at = 0.0
        self._hold = False               # static poses hold; travel expires
        # None means "the policy owns the upper body". Once set, it must go out
        # on EVERY planner message: the binary recomputes has_upper_body_control_
        # from the latest message, so one omission returns the arms to the policy
        # mid-motion.
        self._upper_body: list[float] | None = None
        # Where the arms are being asked to END UP. `_upper_body` is what is
        # actually on the wire this tick and walks toward this at `_ub_slew`.
        self._ub_target: list[float] | None = None
        self._ub_slew = UPPER_BODY_SLEW_RAD_S
        self._body_vel = (0.0, 0.0)
        # (move_heading, facing_heading) in the POLICY's frame, or None to build
        # movement/facing from the dead-reckoned _yaw as set_velocity does. A
        # closed-loop navigator must not go through _yaw -- see set_course.
        self._course: tuple[float, float] | None = None
        self._stop_flag = threading.Event()
        self._started = False
        self._publish_error_reported = False

        self._thread = threading.Thread(target=self._publish_loop, daemon=True)
        self._thread.start()
        if auto_start:
            self.start_control()

    # -- lifecycle ---------------------------------------------------------
    def start_control(self) -> dict:
        """Start the control system and enter planner mode. Equivalent to the
        keyboard's ']' followed by Enter."""
        self._pub.send(encode_command(start=True, stop=False, planner=True))
        self._started = True
        return {"started": True, "planner_mode": True}

    def emergency_stop(self) -> dict:
        self._pub.send(encode_command(start=False, stop=True, planner=True))
        with self._lock:
            self._expires_at = 0.0
            self._hold = False
            self._mode = MODE_IDLE
        return {"emergency_stop": True}

    def close(self) -> None:
        self._stop_flag.set()
        self._thread.join(timeout=1.0)
        self._pub.close(linger=0)

    # -- SonicLink ---------------------------------------------------------
    def set_velocity(self, vx, vy, wz, duration_s) -> dict:
        vx, vy, wz = clamp_velocity(vx, vy, wz)
        duration_s = clamp_duration(duration_s)
        speed = math.hypot(vx, vy)
        mode = MODE_IDLE if speed < 1e-3 and abs(wz) < 1e-3 else (
            MODE_SLOW_WALK if speed <= SLOW_WALK_MAX_M_S else MODE_WALK)
        with self._lock:
            self._body_vel = (vx, vy)
            self._course = None          # a body-relative request ends any course
            self._speed = speed
            self._yaw_rate = wz
            self._mode = mode
            self._hold = False           # travel always expires
            self._expires_at = time.monotonic() + duration_s
        return {"kind": "velocity", "vx": vx, "vy": vy, "wz": wz,
                "duration_s": duration_s, "mode": mode}

    def set_course(self, move_heading: float, facing_heading: float,
                   speed: float, duration_s: float) -> dict:
        """Steer by ABSOLUTE heading in the policy's own frame.

        WHY THIS EXISTS ALONGSIDE set_velocity
            set_velocity takes a body-relative request and rotates it by
            `self._yaw`, which is dead-reckoned by integrating the yaw rate we
            COMMANDED (see _publish_loop). Nothing corrects it, so it drifts from
            the robot's real heading -- fine for "walk forward for two seconds",
            useless for closing a loop on position, where the drift is precisely
            the error being corrected.

            set_course writes movement and facing straight through. The caller is
            expected to compute them from a MEASURED pose each cycle, which makes
            the drift irrelevant rather than merely small.

        FRAME
            movement/facing go from the wire into the planner untransformed
            (g1_deploy_onnx_ref.cpp:3712-3742), so they are in the policy's
            heading frame -- anchored to the base quaternion captured at init
            (HeadingState, robot_parameters.hpp:55-61), NOT to the body and NOT
            to odometry. They coincide only if the robot started at yaw 0.
            sonic_navigate measures the offset rather than assuming it.
        """
        speed = max(0.0, min(float(speed), MAX_VX_M_S))
        duration_s = clamp_duration(duration_s)
        # Zero speed still means SLOW_WALK, never IDLE. A course with speed 0 is
        # "turn on the spot to face this way", and IDLE does not turn -- the
        # robot would stand still while the navigator waited for a heading change
        # that could not arrive, and abort on no-progress. This mirrors turn(),
        # which reaches the same state through set_velocity(0, 0, wz).
        mode = MODE_SLOW_WALK if speed <= SLOW_WALK_MAX_M_S else MODE_WALK
        with self._lock:
            self._course = (float(move_heading), float(facing_heading))
            self._speed = speed
            self._mode = mode
            self._yaw_rate = 0.0
            self._hold = False
            self._expires_at = time.monotonic() + duration_s
        return {"kind": "course", "move_heading_rad": float(move_heading),
                "facing_heading_rad": float(facing_heading), "speed": speed,
                "duration_s": duration_s, "mode": mode}

    def stop(self) -> dict:
        with self._lock:
            self._body_vel = (0.0, 0.0)
            self._course = None
            self._speed = 0.0
            self._yaw_rate = 0.0
            self._mode = MODE_IDLE
            self._hold = False           # releases a held pose too
            self._expires_at = 0.0
        # one explicit IDLE so the robot settles now rather than on staleness
        self._send_current(force_idle=True)
        return {"kind": "stop"}

    def select_mode(self, name: str, *, duration_s: float | None = None,
                    height_m: float | None = None, speed: float | None = None) -> dict:
        """Enter one of SONIC's 27 locomotion modes.

        HOLD vs EXPIRE, and why they differ
            A travelling mode expires: if the agent loop dies mid-walk the robot
            must stop, so `duration_s` bounds it exactly as set_velocity does.
            A static mode HOLDS: it is a posture, not motion, and a squat that
            silently stood back up after five seconds would be a worse surprise
            than one that stayed put. A held mode still ends on stop(), on
            emergency_stop(), and when the process exits and the stream goes
            stale — nothing here can outlive the console.
        """
        key = str(name).strip().lower()
        if key == "damp":
            return self.emergency_stop()
        mode = sonic_modes.resolve(key)
        height = (sonic_modes.clamp_height(height_m, mode)
                  if height_m is not None else mode.height)

        # An unnamed speed is NOT zero. Zero means "stand still" on the wire, and
        # sending it to a styled mode is what made jump_forward look like a big
        # step; the mode's own operating point is in sonic_modes, copied from the
        # vendor's gamepad. A named speed still wins and is still clamped.
        spd = mode.speed if speed is None else clamp_velocity(float(speed), 0.0, 0.0)[0]

        # A mode-default speed is -1, so "does this mode travel" is "is the speed
        # not exactly zero", not "is it positive". Getting that wrong would hold a
        # walking mode forever with no expiry, which is the one thing the duration
        # bound exists to prevent.
        travel = mode.takes_speed and abs(spd) > 1e-3
        with self._lock:
            self._mode = mode.value
            self._speed = spd
            # -1 is "ask the mode", not "reverse at 1 m/s". Dead reckoning cannot
            # integrate a speed nobody has stated, so it integrates nothing;
            # goto() and calibrate() read measured pose and are unaffected.
            self._body_vel = (spd if spd > 0.0 else 0.0, 0.0)
            self._yaw_rate = 0.0
            self._height = height
            if travel:
                self._hold = False
                self._expires_at = time.monotonic() + clamp_duration(duration_s)
            else:
                # Static and styled poses hold until something changes them.
                self._hold = mode.kind != sonic_modes.STANDING or duration_s is None
                self._expires_at = (time.monotonic() + clamp_duration(duration_s)
                                    if not self._hold else 0.0)
        return {"kind": "mode", "name": mode.name, "mode_id": mode.value,
                "mode_kind": mode.kind, "held": self._hold, "speed": spd,
                "height_m": None if height < 0 else height,
                "tested_on_this_robot": mode.tested}

    def set_height(self, height_m: float | None) -> dict:
        """Ask the planner to hold a different pelvis height. Applies to whatever
        mode is current, and survives until changed."""
        h = sonic_modes.clamp_height(height_m)
        with self._lock:
            self._height = h
        self._send_current()
        return {"height_m": None if h < 0 else h,
                "note": "-1 means the mode's own default height" if h < 0 else None}

    def read_state(self) -> SonicState:
        """What we can honestly say without a lowstate subscription: the intent
        we are publishing and our dead-reckoned heading. Height and tilt are
        NOT reported, because we do not measure them — reporting a plausible
        constant is the failure this project has already shipped once."""
        with self._lock:
            active = self._hold or time.monotonic() < self._expires_at
            moving = active and not self._hold
            return SonicState(
                fresh=True,
                root_height_m=None,
                tilt_rad=None,
                velocity=(self._body_vel[0], self._body_vel[1], self._yaw_rate)
                if moving else (0.0, 0.0, 0.0),
                mode=self._mode_name(),
                note=("commanded intent and a dead-reckoned heading estimate "
                      f"({math.degrees(self._yaw):.0f} deg); height and tilt are "
                      "not measured on this link"),
            )

    # -- internals ---------------------------------------------------------
    def _mode_name(self) -> str:
        mode = sonic_modes.BY_VALUE.get(self._mode)
        return mode.name if mode else str(self._mode)

    def set_upper_body(self, values: "list[float] | None", *,
                       slew_rad_s: float | None = None) -> dict:
        """Take (or hand back) the waist and both arms.

        None returns them to the policy's own reference. Anything else is
        clamped to the robot's joint limits and republished on every planner
        message until changed. It does NOT expire on a timer the way travel
        does: a held arm pose cannot walk into anything, and an arm that
        silently dropped back to neutral mid-task would be the more surprising
        failure.

        THE REFERENCE IS RAMPED, NOT STEPPED
            What goes on the wire walks toward the requested pose at
            `slew_rad_s` rather than jumping to it. SONIC applies no rate limit
            of its own to `upper_body_position` -- the only slew clamp in the
            deploy binary is on the Dex3 hands (dex3_hands.hpp:21) -- so a step
            change in the reference is chased as hard as the policy's gains
            allow, which is what made every arm command look like a flinch.
            Ramping in this layer is also the only place it CAN be done without
            rebuilding the vendor binary.
        """
        if values is None:
            with self._lock:
                self._upper_body = None
                self._ub_target = None
            self._send_current()
            return {"upper_body": "released to the policy"}
        clamped = sonic_upper_body.clamp(values)
        rate = UPPER_BODY_SLEW_RAD_S if slew_rad_s is None else max(0.05,
                                                                    float(slew_rad_s))
        with self._lock:
            if self._upper_body is None:
                # Taking control: start from the pose the policy is already
                # holding, so the first command ramps from where the arms
                # ACTUALLY are rather than snapping to neutral first.
                self._upper_body = list(sonic_upper_body.NEUTRAL)
            self._ub_target = clamped
            self._ub_slew = rate
            travel = max(abs(t - c) for t, c in zip(clamped, self._upper_body))
        # Send immediately rather than waiting for the next 20 Hz tick, so a
        # command issued while parked in IDLE (which publishes nothing) still
        # lands.
        self._send_current()
        return {"upper_body": clamped, "slew_rad_s": rate,
                "travel_time_s": round(travel / rate, 2)}

    def upper_body_travel_time(self) -> float:
        """Seconds until the ramp reaches the target, from right now. The
        readback in the adapter waits this out -- measuring mid-ramp and
        reporting the gap as tracking error is how a working arm gets called
        broken."""
        with self._lock:
            if self._ub_target is None or self._upper_body is None:
                return 0.0
            gap = max(abs(t - c) for t, c in zip(self._ub_target,
                                                 self._upper_body))
            return gap / max(self._ub_slew, 1e-6)

    def _advance_upper_body(self, dt: float) -> None:
        """Walk the published reference one tick toward the target. Caller holds
        no lock; this takes it."""
        with self._lock:
            target, cur = self._ub_target, self._upper_body
            if target is None or cur is None:
                return
            step = self._ub_slew * dt
            self._upper_body = [
                c + max(-step, min(step, t - c)) for c, t in zip(cur, target)]

    def _send_current(self, force_idle: bool = False) -> None:
        with self._lock:
            yaw = self._yaw
            mode = MODE_IDLE if force_idle else self._mode
            speed = 0.0 if force_idle else self._speed
            vx, vy = (0.0, 0.0) if force_idle else self._body_vel
            course = None if force_idle else self._course
            height = self._height
            upper = list(self._upper_body) if self._upper_body else None
        if course is not None:
            # Already absolute headings in the policy's frame: send them through
            # untouched. Rotating these by the dead-reckoned _yaw would inject
            # exactly the drift set_course exists to avoid.
            move_h, face_h = course
            facing = (math.cos(face_h), math.sin(face_h), 0.0)
            movement = (math.cos(move_h), math.sin(move_h), 0.0)
        else:
            facing = (math.cos(yaw), math.sin(yaw), 0.0)
            if speed > 1e-3:
                # body-relative request rotated into the world frame
                wx = vx * math.cos(yaw) - vy * math.sin(yaw)
                wy = vx * math.sin(yaw) + vy * math.cos(yaw)
                norm = math.hypot(wx, wy) or 1.0
                movement = (wx / norm, wy / norm, 0.0)
            else:
                movement = facing
        self._pub.send(encode_planner(mode, movement, facing, speed, height,
                                      upper_body=upper))

    def _publish_loop(self) -> None:
        dt = 1.0 / PUBLISH_HZ
        while not self._stop_flag.is_set():
            try:
                self._publish_beat(dt)
            except Exception as exc:
                # This thread is the ONLY continuous command channel to the
                # robot. Unguarded, one raise ended it permanently while every
                # link call kept returning a success dict and the planner
                # quietly dropped to IDLE after ~1 s — motion commands that
                # report success and do nothing. Report once, keep publishing.
                if not self._publish_error_reported:
                    self._publish_error_reported = True
                    try:
                        print(f"*** SONIC PUBLISH BEAT FAILED: {exc!r} — the "
                              "command stream is DEGRADED. Commands may report "
                              "success and never reach the robot. ***",
                              flush=True)
                    except Exception:
                        pass
            time.sleep(dt)

    def _publish_beat(self, dt: float) -> None:
        """One beat, split out so the loop above can guard it without burying
        the logic inside a try block."""
        now = time.monotonic()
        with self._lock:
            moving = now < self._expires_at
            # An upper-body reference keeps the stream alive on its own. It
            # has to: the planner drops any message older than ~1 s, so an
            # arm pose held while the robot stands still would sag back to
            # the policy's default a second after it was commanded.
            active = moving or self._hold or self._upper_body is not None
            if moving:
                self._yaw += self._yaw_rate * dt
        # Advance the arm ramp before publishing, so each message carries a
        # slightly further-along reference instead of the same step.
        self._advance_upper_body(dt)
        # Publishing only while a command is live is what makes expiry the
        # default. When it lapses we send IDLE once and then go quiet.
        # A held pose keeps publishing on purpose: the planner treats a
        # message older than ~1 s as stale and drops to IDLE, so "hold this
        # squat" is a stream, not a latch we could forget to release.
        if active:
            self._send_current()
