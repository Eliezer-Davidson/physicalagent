"""MiniHexaAdapter — the HiWonder MiniHexa hexapod tool surface for Gemini.

Composes the tested pieces: SafetyGuard (e-stop + battery gate + the action
whitelist), the tiered motion clamp (minihexa_motion), the wire codec
(minihexa_codec), and a MiniHexaLink (Serial/BLE in prod, Fake in tests). Like
every other adapter it is therefore fully testable offline.

THE CENTRAL HAZARD, AND HOW THIS ADAPTER ANSWERS IT
    The firmware has NO watchdog and its walk command LATCHES a velocity: the
    robot walks forever until a C|0|0|0& stop arrives. It does not self-stop on
    silence or link loss. So `walk` here is not a command — it is a BOUNDED
    BURST: battery preflight, clamp to the tier, send the move, block for the
    tier's burst duration, then ALWAYS send the stop. The invariant that the LAST
    frame of any successful walk is the stop is the single most important
    property of this class (test I2). Safe operation additionally requires a human
    operator present and able to cut power — see SupervisionClass.OPERATOR_BOUNDED.

THE SERIAL / BLE CAPABILITY SPLIT
    Per-joint control (L) and action-stop (K|2) exist over USB only; the BLE
    firmware silently ignores them. So set_servo / set_servos refuse over a link
    whose supports_per_joint is False, and stop emits K|2 only on a serial link.

Other design decisions, mirrored from the TonyPi adapter:
  * Every motion tool runs the battery preflight FIRST (guard.guard_battery),
    which refuses on an unreadable or too-low pack. stop is the sole exception —
    it must work regardless of battery.
  * do_action is TWO-GATED (adapter curated list, then guard whitelist). Both
    default to the EMPTY MINIHEXA_SAFE_ACTION_GROUPS, so do_action always refuses
    until a group is characterized on hardware — correct and intended.
  * safe_state is FREEZE: a hexapod is statically stable, so a stop cuts motion
    and holds — it never commands a new pose (unlike the biped's settle-to-stance).
  * Refusals are ToolResults, never exceptions — execute() turns a bad tool call
    into a message the model can read and correct, not a robot fault.

There is no camera in this path, so get_frame() returns None.
"""
from __future__ import annotations

import time as _time

from physicalagent.robots import minihexa_codec as codec
from physicalagent.robots.base import RobotAdapter, ToolSpec
from physicalagent.robots.minihexa_motion import (
    MotionTier, burst_duration_s, clamp_pose_bytes, clamp_servo_duty, clamp_walk,
)
from physicalagent.safety import MINIHEXA_SAFE_ACTION_GROUPS, SafetyError, battery_state


class MiniHexaAdapter(RobotAdapter):
    def __init__(self, link, guard, *, clock=None, robot_profile=None,
                 tier: MotionTier = MotionTier.LOCKED, sleep=None,
                 action_groups=None):
        super().__init__(guard=guard, robot_profile=robot_profile)
        self._link = link
        self._clock = clock            # accepted for a uniform factory signature
        self._tier = tier
        # The walk burst blocks for burst_duration then stops. sleep is injected
        # so tests drive it without real time passing (mirrors brain.er's sleep).
        self._sleep = sleep if sleep is not None else _time.sleep
        # Curated safe action groups — gate #1. Defaults to the module-level EMPTY
        # set, so do_action refuses everything until a group is characterized.
        self._actions = tuple(action_groups if action_groups is not None
                              else MINIHEXA_SAFE_ACTION_GROUPS)

    # ---- tool surface ----------------------------------------------------
    def capabilities(self):
        return (
            ToolSpec("get_status",
                     "Read the robot's state: pack voltage (millivolts) and its "
                     "health (ok/warn/critical/unreadable), the active motion tier "
                     "(locked/normal/full, which bounds how fast and far you move), "
                     "the link transport, and whether per-joint control is available "
                     "(USB only). Always available.",
                     {"type": "object", "properties": {}}),
            ToolSpec("walk",
                     "Take a SINGLE bounded walking burst, then stop automatically. "
                     "vx is forward/back and vy is left/right strafe (each -50..50, "
                     "clamped tighter by the active tier); turn is 0 (straight), 1 "
                     "(veer left) or 2 (veer right). The robot walks for a short "
                     "fixed time set by the tier and then halts on its own — there "
                     "is no continuous walk, so re-issue walk to keep going.",
                     {"type": "object", "properties": {
                         "vx": {"type": "integer", "description": "forward/back -50..50"},
                         "vy": {"type": "integer", "description": "left/right strafe -50..50"},
                         "turn": {"type": "integer", "description": "0 straight, 1 left, 2 right"}},
                      "required": ["vx", "vy", "turn"]}),
            ToolSpec("set_pose",
                     "Hold a body pose without walking. yaw/pitch/roll tilt the "
                     "body and x/y shift it (each -50..50, clamped by the tier); "
                     "height raises/lowers the body 0..30. The legs stay planted; "
                     "this is posture, not locomotion.",
                     {"type": "object", "properties": {
                         "yaw": {"type": "integer"}, "pitch": {"type": "integer"},
                         "roll": {"type": "integer"}, "x": {"type": "integer"},
                         "y": {"type": "integer"},
                         "height": {"type": "integer", "description": "body height 0..30"}},
                      "required": ["yaw", "pitch", "roll", "x", "y", "height"]}),
            ToolSpec("do_action",
                     "Run a built-in action group by numeric id. NOTE: no action "
                     "group is currently whitelisted (they are uncharacterized and "
                     "could walk the robot off an edge), so this refuses every id "
                     "until groups are validated on hardware one at a time.",
                     {"type": "object", "properties": {
                         "group_id": {"type": "integer", "description": "built-in action group id"}},
                      "required": ["group_id"]}),
            ToolSpec("set_servo",
                     "Move ONE leg joint servo to a duty position (USB only; refused "
                     "over BLE). servo_id is 1-18; duty is 500-2500 us with 1500 "
                     "centred; move_time_ms is how long the move takes. The active "
                     "tier clamps both the duty band and the minimum time — the "
                     "result reports what was actually commanded.",
                     {"type": "object", "properties": {
                         "servo_id": {"type": "integer", "description": "leg joint servo id 1-18"},
                         "duty": {"type": "integer", "description": "target duty 500-2500 us (centre 1500)"},
                         "move_time_ms": {"type": "number", "description": "move duration in ms"}},
                      "required": ["servo_id", "duty"]}),
            ToolSpec("set_servos",
                     "Move several leg joint servos together (USB only; refused over "
                     "BLE). servos is a list of {servo_id, duty} (ids 1-18, duty "
                     "500-2500, centre 1500); move_time_ms applies to the whole "
                     "group. Every duty and the time are clamped by the active tier. "
                     "If any servo_id is invalid the whole batch is refused — nothing "
                     "is sent partway.",
                     {"type": "object", "properties": {
                         "servos": {"type": "array", "items": {
                             "type": "object", "properties": {
                                 "servo_id": {"type": "integer"},
                                 "duty": {"type": "integer"}},
                             "required": ["servo_id", "duty"]}},
                         "move_time_ms": {"type": "number"}},
                      "required": ["servos"]}),
            ToolSpec("stop",
                     "Immediately halt the latched walk (and, over USB, any running "
                     "action group). The hexapod is statically stable, so stopping "
                     "leaves it standing where it is. Always available.",
                     {"type": "object", "properties": {}}),
            ToolSpec("emergency_stop",
                     "Same as stop — halt everything now. Always available, "
                     "including when motion is otherwise locked out.",
                     {"type": "object", "properties": {}}),
        )

    def _handlers(self):
        return {"get_status": self._status, "walk": self._walk,
                "set_pose": self._set_pose, "do_action": self._do_action,
                "set_servo": self._set_servo, "set_servos": self._set_servos,
                "stop": self._stop_tool, "emergency_stop": self._stop_tool}

    def _always_allowed_tools(self):
        """Tools that run even while e-stopped: the read, and the two halts.
        Halting a robot that never self-stops must never be refused."""
        return {"get_status", "stop", "emergency_stop"}

    def _motion_tool_names(self):
        """Everything that physically moves the robot. The halts are NOT here (a
        stop must stay available on a sensing-only robot), nor is the read."""
        return {"walk", "set_pose", "do_action", "set_servo", "set_servos"}

    # ---- battery preflight ----------------------------------------------
    def _preflight(self):
        """Battery gate run FIRST by every motion tool. guard.guard_battery raises
        SafetyError on an unreadable or too-low pack; execute() turns that into a
        refusal. If read_battery_mv itself raises, the exception likewise becomes a
        refusal at the execute() boundary — fail closed, nothing sent."""
        self._guard.guard_battery(self._link.read_battery_mv())

    # ---- sensing ---------------------------------------------------------
    def _status(self):
        # Read-only and never refused: swallow a link error into a flag rather
        # than raising, so status still returns something the model can read.
        battery_mv, read_ok = None, False
        try:
            battery_mv = self._link.read_battery_mv()
            read_ok = battery_mv is not None
        except Exception:
            battery_mv, read_ok = None, False
        state = (battery_state(battery_mv, self._guard.profile)
                 if battery_mv is not None else "unreadable")
        kind, per_joint = None, None
        try:
            kind = self._link.kind
        except Exception:
            kind = None
        try:
            per_joint = self._link.supports_per_joint
        except Exception:
            per_joint = None
        return {"battery_mv": battery_mv, "battery_read_ok": read_ok,
                "battery_state": state, "tier": self._tier.name.lower(),
                "link_kind": kind, "supports_per_joint": per_joint,
                "safe_state": "freeze"}

    # ---- motion ----------------------------------------------------------
    def _walk(self, vx, vy, turn):
        self._preflight()
        cvx, cvy, cturn = clamp_walk(vx, vy, turn, self._tier)
        burst = burst_duration_s(self._tier)
        # Send the move OUTSIDE the try: if this raises, nothing is latched and we
        # do not want to send a spurious stop. Once it succeeds, the stop is
        # guaranteed-attempted in the finally even if the sleep is interrupted —
        # this is the I2 invariant: a successful walk never leaves the robot moving.
        self._link.send(codec.encode_move(cvx, cvy, cturn))
        try:
            self._sleep(burst)
        finally:
            self._link.send(codec.encode_stop())
        return {"vx": cvx, "vy": cvy, "turn": cturn, "burst_s": burst,
                "tier": self._tier.name.lower()}

    def _set_pose(self, yaw, pitch, roll, x, y, height):
        self._preflight()
        yb, pb, rb, xb, yy, hb = clamp_pose_bytes(yaw, pitch, roll, x, y, height,
                                                  self._tier)
        self._link.send(codec.encode_pose(yb, pb, rb, xb, yy, hb))
        return {"yaw": yb, "pitch": pb, "roll": rb, "x": xb, "y": yy,
                "height": hb, "tier": self._tier.name.lower()}

    def _do_action(self, group_id):
        self._preflight()
        gid = str(int(group_id))
        # gate #1: the adapter's own curated list (EMPTY by default), checked FIRST
        # so even a permissively-configured guard cannot run an uncharacterized
        # trajectory. With the empty list this always raises, sending nothing.
        if gid not in self._actions:
            raise SafetyError(
                f"action group {gid} is not in the adapter's safe action list "
                f"({', '.join(self._actions) or 'empty — no group is characterized yet'})")
        safe = self._guard.guard_action(gid)   # gate #2: raises if not whitelisted
        self._link.send(codec.encode_action(int(safe)))
        return {"group_id": int(safe)}

    def _clamp_one_servo(self, servo_id, duty, move_time_ms):
        """Validate a servo id and clamp its command to the active tier. Returns
        (id, clamped_duty, clamped_time). Raises SafetyError on a bad id BEFORE
        anything is sent."""
        sid = int(servo_id)
        if not (codec.SERVO_ID_MIN <= sid <= codec.SERVO_ID_MAX):
            raise SafetyError(
                f"servo_id {sid} out of range {codec.SERVO_ID_MIN}-{codec.SERVO_ID_MAX}")
        d, t = clamp_servo_duty(duty, move_time_ms, self._tier)
        return sid, d, t

    def _require_per_joint(self):
        if not self._link.supports_per_joint:
            raise SafetyError(
                "per-joint servo control is USB-only; not available over BLE")

    def _set_servo(self, servo_id, duty, move_time_ms=None):
        self._require_per_joint()          # USB-only gate first — nothing sent over BLE
        self._preflight()
        sid, d, t = self._clamp_one_servo(servo_id, duty, move_time_ms)
        self._link.send(codec.encode_servo([(sid, d)], t))
        return {"servo_id": sid, "duty": d, "move_time_ms": t,
                "tier": self._tier.name.lower()}

    def _set_servos(self, servos, move_time_ms=None):
        self._require_per_joint()
        self._preflight()
        servos = list(servos or [])
        if not servos:
            raise SafetyError("set_servos needs at least one {servo_id, duty}")
        # Validate + clamp EVERY entry before sending anything: the batch is
        # all-or-nothing, so one bad id refuses the whole call and writes nothing.
        moves, shared_time = [], None
        for s in servos:
            sid, d, t = self._clamp_one_servo(s["servo_id"], s["duty"], move_time_ms)
            moves.append((sid, d))
            shared_time = t   # identical across entries (same move_time_ms + tier)
        self._link.send(codec.encode_servo(moves, shared_time))
        return {"servos": [{"servo_id": s, "duty": d} for s, d in moves],
                "move_time_ms": shared_time, "tier": self._tier.name.lower()}

    def _stop_tool(self):
        self.stop_all()
        return {"stopped": True, "safe_state": "freeze"}

    # ---- RobotAdapter required ------------------------------------------
    def get_frame(self):
        # No camera in this path.
        return None

    def telemetry(self):
        return self._status()

    def stop_all(self):
        # Best-effort and NEVER raises: send the velocity stop (works over BLE and
        # USB), then the action-stop (K|2) ONLY on a link that parses it (serial).
        # Each send is guarded so one failure cannot skip the other. FREEZE safe
        # state: we command NO new pose — the hexapod is statically stable.
        try:
            per_joint = self._link.supports_per_joint
        except Exception:
            per_joint = False
        try:
            self._link.send(codec.encode_stop())
        except Exception:
            pass
        if per_joint:
            try:
                self._link.send(codec.encode_action_stop())
            except Exception:
                pass
