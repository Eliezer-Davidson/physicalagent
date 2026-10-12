"""TonyPiAdapter — the HiWonder TonyPi humanoid tool surface for Gemini.

Composes the tested pieces: SafetyGuard (e-stop + the action whitelist), the
tiered servo clamp (tonypi_servo.clamp_servo), and a TonyLink (HttpTonyLink in
prod, FakeTonyLink in tests). Like the ROSPug adapter it is therefore fully
testable offline.

Design decisions (from the TonyPi facts in reference/TonyPi + the tier design):
  * The tool surface is DISCRETE, self-terminating commands — action groups that
    stop on their own, and servo POSITION moves — not continuous walk. That is
    why the profile's supervision class is COMMAND_EXPIRY: each command stops by
    default, so there is no PC-side deadman to keep the body from running away.
  * Every servo command (bus and head PWM) passes through clamp_servo(tier). The
    TIER is the safety envelope on this robot: LOCKED by default, widened only by
    an explicit operator choice. There is no cmd_vel and no guard_velocity here —
    the tier clamp is what stands between a bad number and the servos.
  * do_action is TWO-GATED: the adapter's own curated list is checked first, then
    the guard's whitelist. Neither a permissive guard nor a permissive adapter
    list alone can run a tip-risk trajectory.
  * A biped must never be left frozen mid-pose: stop halts the action group and
    the bus servos, THEN commands a stable standing stance (SETTLE_TO_STANCE).
  * Refusals are ToolResults, never exceptions — execute() turns a bad tool call
    into a message the model can read and correct, not a robot fault.

Camera live-frames
  Fresh frames need the vision function loaded and its 7-second deadman fed:
  start_camera() loads function 9 (lab_adjust — camera-only, NO motion) and
  starts it; heartbeat() feeds the deadman. The PERIODIC driving of heartbeat()
  is left to the caller (cli owns a small daemon thread), so this adapter spawns
  no thread of its own and stays deterministic offline. look() itself only pulls
  the latest snapshot and does not depend on any thread.
"""
from __future__ import annotations

from physicalagent.robots import aihand_codec as codec
from physicalagent.robots.aihand import AiHand
from physicalagent.robots.base import RobotAdapter, ToolSpec
from physicalagent.robots.tonypi_link import rpc_result
from physicalagent.robots.tonypi_servo import ServoDomain, ServoTier, clamp_servo
from physicalagent.robots.transport import Clock, MonotonicClock
from physicalagent.safety import SafetyError, TONYPI_SAFE_ACTION_GROUPS

# VERIFIED from reference/TonyPi/RPCServer.py: SetBusServoPulse validates ids
# 1..16; SetPWMServo validates ids 1..2 (1 = head tilt, 2 = head pan).
BUS_SERVO_ID_MIN = 1
BUS_SERVO_ID_MAX = 16
HEAD_TILT_ID = 1
HEAD_PAN_ID = 2
# The stable stance a stop settles the biped into (a real .d6a action group).
SETTLE_ACTION = "stand"
# Function 9 = lab_adjust: opens the camera, runs NO motion. The right choice for
# a keepalive whose only job is to keep frames flowing.
CAMERA_FUNC = 9


def dedupe_frame(jpeg: bytes) -> bytes:
    """Drop the duplicated bottom half of a TonyPi frame, if it is duplicated.

    MEASURED 2026-07-28: this robot's camera publishes 640x960 in which rows
    480..959 are a BYTE-EXACT copy of rows 0..479 — mean absolute difference
    0.00 across the whole half, at zero offset in both axes. It is duplication,
    not a stereo pair, so there is no disparity and no depth to recover.

    That matters because get_frame() feeds a vision model: half of every image
    is paid for in tokens and carries nothing, and a duplicated scene invites
    the model to report two of every object.

    Deliberately conservative. It crops ONLY when the image is exactly twice as
    tall as it is... duplicated, verified by comparing the halves on this frame,
    never assumed from the dimensions alone. Any doubt, or no Pillow installed,
    and the original bytes pass through untouched — a slightly wasteful image
    beats a silently mangled one.
    """
    try:
        import io

        from PIL import Image, ImageChops, ImageStat
    except ImportError:
        return jpeg                      # no Pillow: pass through, never fail
    try:
        im = Image.open(io.BytesIO(jpeg))
        w, h = im.size
        if h % 2:
            return jpeg
        top, bottom = im.crop((0, 0, w, h // 2)), im.crop((0, h // 2, w, h))
        if ImageStat.Stat(ImageChops.difference(
                top.convert("L"), bottom.convert("L"))).mean[0] > 0.5:
            return jpeg                  # genuinely different halves: keep both
        out = io.BytesIO()
        top.save(out, format="JPEG", quality=90)
        return out.getvalue()
    except Exception:
        return jpeg                      # a frame we cannot parse is still a frame


class TonyPiAdapter(RobotAdapter):
    def __init__(self, link, guard, *, clock: Clock | None = None,
                 robot_profile=None, tier: ServoTier = ServoTier.LOCKED,
                 action_groups=None, hand: AiHand | None = None):
        super().__init__(guard=guard, robot_profile=robot_profile)
        self._link = link
        self._clock = clock or MonotonicClock()
        self._tier = tier
        self.hand = hand
        # Curated safe action groups. Defaults to the module-level safe set
        # (standing, in-place gestures); pass action_groups=(...) to narrow it.
        # This is gate #1; the guard's whitelist is gate #2.
        self._actions = tuple(action_groups or TONYPI_SAFE_ACTION_GROUPS)
        self.last_frame: bytes | None = None

    # ---- tool surface ----------------------------------------------------
    def capabilities(self):
        tools = (
            ToolSpec("get_status",
                     "Read the robot's state: the current pulse of each bus servo "
                     "(head-to-toe joint positions), which vision function is running, "
                     "and the active motion tier (locked/normal/full, which bounds how "
                     "far and fast joints may move). Always available.",
                     {"type": "object", "properties": {}}),
            ToolSpec("look",
                     "Capture the robot's current camera view (forward-facing, on the "
                     "pan/tilt head). The image is attached to your next turn so you can "
                     "see what is in front of you.",
                     {"type": "object", "properties": {}}),
            ToolSpec("do_action",
                     "Run a built-in, self-terminating gesture that keeps the robot "
                     "standing on both feet. Available: " + ", ".join(TONYPI_SAFE_ACTION_GROUPS) +
                     ". These are safe, in-place motions — walks, kicks, squats and "
                     "get-ups are deliberately not offered.",
                     {"type": "object", "properties": {"name": {"type": "string"}},
                      "required": ["name"]}),
            ToolSpec("set_joint",
                     "Move ONE body bus servo to a pulse position. servo_id is 1-16; pulse "
                     "is 0-1000 with 500 the neutral centre; move_time_ms is how long the "
                     "move takes (longer = gentler). The active tier clamps both the pulse "
                     "range and the minimum time — the result reports what was actually "
                     "commanded, which may be tighter than you asked.",
                     {"type": "object", "properties": {
                         "servo_id": {"type": "integer", "description": "bus servo id 1-16"},
                         "pulse": {"type": "integer", "description": "target pulse 0-1000 (centre 500)"},
                         "move_time_ms": {"type": "number", "description": "move duration in ms"}},
                      "required": ["servo_id", "pulse"]}),
            ToolSpec("set_joints",
                     "Move several body bus servos together in one timed motion. joints is "
                     "a list of {servo_id, pulse} (ids 1-16, pulse 0-1000, centre 500); "
                     "move_time_ms applies to the whole group. Every pulse and the time "
                     "are clamped by the active tier. If any servo_id is invalid the whole "
                     "batch is refused — nothing is sent partway.",
                     {"type": "object", "properties": {
                         "joints": {"type": "array", "items": {
                             "type": "object", "properties": {
                                 "servo_id": {"type": "integer"},
                                 "pulse": {"type": "integer"}},
                             "required": ["servo_id", "pulse"]}},
                         "move_time_ms": {"type": "number"}},
                      "required": ["joints"]}),
            ToolSpec("set_head",
                     "Aim the camera by moving the head PWM servos. pan turns the head "
                     "left/right (servo 2), tilt nods it up/down (servo 1); each is a pulse "
                     "500-2500 with 1500 centred. Give either or both. Clamped by the "
                     "active tier.",
                     {"type": "object", "properties": {
                         "pan": {"type": "number", "description": "head pan pulse 500-2500 (centre 1500)"},
                         "tilt": {"type": "number", "description": "head tilt pulse 500-2500 (centre 1500)"},
                         "move_time_ms": {"type": "number"}}}),
            ToolSpec("stop",
                     "Immediately halt the running action and the servos, then settle into "
                     "a stable standing stance. Because the robot balances on two legs, "
                     "stopping leaves it upright rather than frozen mid-pose. Always available.",
                     {"type": "object", "properties": {}}),
            ToolSpec("emergency_stop",
                     "Same as stop — halt everything and settle to a stable stance. "
                     "Always available, including when motion is otherwise locked out.",
                     {"type": "object", "properties": {}}),
        )
        if self.hand:
            tools += self._hand_capabilities()
        return tools

    def _hand_capabilities(self):
        """Tool declarations for the AiHand on the arm. Present only when a hand
        is attached, so the model is never offered a hand it does not have."""
        finger = {"type": "integer",
                  "description": f"pulse {codec.PULSE_MIN}-{codec.PULSE_MAX}; "
                                 f"lower is open, higher is closed"}
        return (
            ToolSpec("set_hand",
                     "Move individual fingers of the hand on the arm. Each finger "
                     f"takes a pulse from {codec.PULSE_MIN} (open) to "
                     f"{codec.PULSE_MAX} (closed); omit a finger to leave it "
                     "where it is. 'base' rotates the whole hand at the wrist. "
                     "Every value is clamped by the active tier. The hand has NO "
                     "sensors — it cannot tell you what it is holding or whether "
                     "it gripped anything, so look with the camera to check.",
                     {"type": "object", "properties": {
                         "thumb": finger, "index": finger, "middle": finger,
                         "ring": finger, "pinky": finger, "base": finger,
                         "move_time_ms": {
                             "type": "number",
                             "description": "how long the move takes, in ms"}}}),
            ToolSpec("open_hand",
                     "Open every finger of the hand — the hand's rest pose. Use "
                     "before reaching for something, and to let go of it.",
                     {"type": "object", "properties": {
                         "move_time_ms": {"type": "number"}}}),
            ToolSpec("close_hand",
                     "Close every finger into a fist or a grip. The hand has no "
                     "force sensing and will keep squeezing whatever is between "
                     "the fingers, so close on soft or sturdy objects only.",
                     {"type": "object", "properties": {
                         "move_time_ms": {"type": "number"}}}),
        )

    def _handlers(self):
        handlers = {"get_status": self._status, "look": self._look,
                    "do_action": self._do_action, "set_joint": self._set_joint,
                    "set_joints": self._set_joints, "set_head": self._set_head,
                    "stop": self._stop_tool, "emergency_stop": self._stop_tool}
        if self.hand:
            handlers["set_hand"] = self._set_hand
            handlers["open_hand"] = self._open_hand
            handlers["close_hand"] = self._close_hand
        return handlers

    def _always_allowed_tools(self):
        """Tools that run even while e-stopped: the reads, and the two halts.
        Halting a biped must never be refused — it is the safe direction."""
        return {"get_status", "look", "stop", "emergency_stop"}

    def _motion_tool_names(self):
        """Everything that physically moves a servo. The halts are NOT here (a
        stop must stay available on a sensing-only robot), nor are the reads."""
        tools = {"do_action", "set_joint", "set_joints", "set_head"}
        if self.hand:
            tools |= {"set_hand", "open_hand", "close_hand"}
        return tools

    # ---- sensing ---------------------------------------------------------
    def _status(self):
        # Read-only and never refused: swallow link errors into a flag rather
        # than raising, so status still returns something the model can read.
        positions, read_ok = None, False
        try:
            ok, data = rpc_result(self._link.call("GetBusServosPulse", ["angularReadback"]))
            positions, read_ok = (data, True) if ok else (None, False)
        except Exception:
            positions, read_ok = None, False
        running = None
        try:
            rok, rdata = rpc_result(self._link.call("GetRunningFunc", []))
            if rok and isinstance(rdata, (list, tuple)) and rdata:
                running = rdata[0]
        except Exception:
            running = None
        return {"servo_positions": positions, "servo_read_ok": read_ok,
                "running_func": running, "tier": self._tier.name.lower(),
                "safe_state": "settle_to_stance"}

    def _look(self):
        # Delegates to get_frame() rather than calling snapshot() itself, so
        # there is exactly ONE camera path. It used to have its own, and that
        # is how dedupe_frame came to be applied on the path production never
        # took: get_frame() deduped, `look` did not, and `look` is the tool the
        # model calls (brain/er.py attaches adapter.last_frame after it). A
        # second entry point to one camera is a second thing to keep in sync.
        frame = self.get_frame()
        return {"frame_captured": True, "bytes": len(frame or b"")}

    # ---- motion ----------------------------------------------------------
    def _do_action(self, name):
        # gate #1: the adapter's own curated list, checked FIRST so even a
        # permissively-configured guard cannot run a tip-risk trajectory.
        if name not in self._actions:
            raise SafetyError(
                f"action '{name}' is not in the adapter's safe action list "
                f"({', '.join(self._actions)})")
        safe_name = self._guard.guard_action(name)   # gate #2: raises if not whitelisted
        self._link.call("RunAction", [safe_name, 1])
        return {"action": safe_name, "times": 1}

    def _one_servo(self, servo_id, pulse, move_time_ms):
        """Validate a bus servo id and clamp its command to the active tier.
        Returns (id, clamped_pulse, clamped_time). Raises SafetyError on a bad id
        BEFORE anything is sent."""
        sid = int(servo_id)
        if not (BUS_SERVO_ID_MIN <= sid <= BUS_SERVO_ID_MAX):
            raise SafetyError(
                f"servo_id {sid} out of range {BUS_SERVO_ID_MIN}-{BUS_SERVO_ID_MAX}")
        # BUS domain: body servos are 0-1000, centre 500 — a different band from
        # the head PWM servos, so the domain is passed explicitly.
        p, t = clamp_servo(pulse, move_time_ms, self._tier, ServoDomain.BUS)
        return sid, p, t

    def _set_joint(self, servo_id, pulse, move_time_ms=None):
        sid, p, t = self._one_servo(servo_id, pulse, move_time_ms)
        # SetBusServoPulse(time_ms, count, id, pulse) — count=1 for a single joint.
        self._link.call("SetBusServoPulse", [t, 1, sid, p])
        return {"servo_id": sid, "pulse": p, "move_time_ms": t, "tier": self._tier.name.lower()}

    def _set_joints(self, joints, move_time_ms=None):
        joints = list(joints or [])
        if not joints:
            raise SafetyError("set_joints needs at least one {servo_id, pulse}")
        # Validate + clamp EVERY joint before sending anything: the batch is
        # all-or-nothing, so one bad id refuses the whole call and writes nothing.
        clamped = []
        shared_time = None
        for j in joints:
            sid, p, t = self._one_servo(j["servo_id"], j["pulse"], move_time_ms)
            clamped.append((sid, p))
            shared_time = t   # identical across joints (same move_time_ms + tier)
        params = [shared_time, len(clamped)]
        for sid, p in clamped:
            params += [sid, p]
        self._link.call("SetBusServoPulse", params)
        return {"joints": [{"servo_id": s, "pulse": p} for s, p in clamped],
                "move_time_ms": shared_time, "tier": self._tier.name.lower()}

    def _set_head(self, pan=None, tilt=None, move_time_ms=None):
        if pan is None and tilt is None:
            raise SafetyError("set_head needs at least one of pan or tilt")
        # Tilt (id 1) first, then pan (id 2), so the wire order is stable. Both
        # are clamped by the same tier; the firmware applies the servo2 config
        # offset for the pan servo robot-side, so 1500 is the LOGICAL centre here.
        servos = []
        if tilt is not None:
            _, p, t = self._clamp_head(tilt, move_time_ms)
            servos.append((HEAD_TILT_ID, p))
            time_ms = t
        if pan is not None:
            _, p, t = self._clamp_head(pan, move_time_ms)
            servos.append((HEAD_PAN_ID, p))
            time_ms = t
        # SetPWMServo(time_ms, count, id, pulse, ...).
        params = [time_ms, len(servos)]
        for sid, p in servos:
            params += [sid, p]
        self._link.call("SetPWMServo", params)
        return {"servos": [{"id": s, "pulse": p} for s, p in servos],
                "move_time_ms": time_ms, "tier": self._tier.name.lower()}

    def _clamp_head(self, pulse, move_time_ms):
        # PWM domain: head servos are 500-2500, centre 1500.
        p, t = clamp_servo(pulse, move_time_ms, self._tier, ServoDomain.PWM)
        return None, p, t

    # ---- the hand on the arm ---------------------------------------------
    # Clamping lives in aihand_codec, against the HAND's own electrical range
    # and tier ladder — not the body's. The two are different servo classes and
    # conflating them is the same bug that once drove bus servos with head-servo
    # values.
    def _require_hand(self):
        if not self.hand:
            raise SafetyError(
                "no hand is attached to this robot — check the AiHand link")
        return self.hand

    def _set_hand(self, move_time_ms=None, **fingers):
        hand = self._require_hand()
        named = {name: value for name, value in fingers.items()
                 if value is not None}
        if not named:
            raise SafetyError(
                "set_hand needs at least one finger position "
                f"({', '.join(sorted(codec.FINGER_IDS))})")
        return hand.set_fingers(named, move_time_ms=move_time_ms)

    def _open_hand(self, move_time_ms=None):
        return self._require_hand().rest(move_time_ms=move_time_ms)

    def _close_hand(self, move_time_ms=None):
        return self._require_hand().close_fist(move_time_ms=move_time_ms)

    def _stop_tool(self):
        self.stop_all()
        return {"stopped": True, "settled_to": SETTLE_ACTION}

    # ---- camera keepalive (no thread here; caller drives heartbeat) ------
    def start_camera(self):
        """Load the camera-only vision function (9 = lab_adjust) and start it, so
        the MJPEG snapshot stream has fresh frames. NO motion is involved."""
        self._link.call("LoadFunc", [CAMERA_FUNC])
        self._link.call("StartFunc", [])
        return {"camera_started": True, "func": CAMERA_FUNC}

    def heartbeat(self):
        """Feed the vision function's 7-second deadman. The caller pings this on a
        timer; the adapter spawns no thread so tests stay deterministic."""
        self._link.call("Heartbeat", [])

    # ---- RobotAdapter required ------------------------------------------
    def get_frame(self):
        self.last_frame = dedupe_frame(self._link.snapshot())
        return self.last_frame

    def telemetry(self):
        return self._status()

    def stop_all(self):
        # Halt FIRST (action group + bus servos), THEN settle to a stable stance:
        # a biped left frozen mid-pose is a fall. Each step is guarded so one
        # link failure cannot skip the settle — halting is the safe direction and
        # must be best-effort, never a raise out of stop_all.
        for method, params in (("StopActionGroup", ["stopActionGroup"]),
                               ("StopBusServo", ["stopAction"])):
            try:
                self._link.call(method, params)
            except Exception:
                pass
        try:
            self._link.call("RunAction", [SETTLE_ACTION, 1])
        except Exception:
            pass
        # The hand is part of the robot, so a stop has to reach it: a fist that
        # keeps squeezing through an emergency stop is exactly what this layer
        # exists to prevent. Opening is the safe direction (it drops whatever is
        # held rather than crushing it), and like every step above it is
        # best-effort — a dead hand link must never stop the biped settling.
        if self.hand:
            try:
                self.hand.rest()
            except Exception:
                pass
