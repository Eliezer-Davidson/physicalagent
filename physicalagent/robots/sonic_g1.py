"""SonicG1Adapter — the tool surface a vision-language model is offered for a
Unitree G1 driven by NVIDIA's SONIC whole-body policy, in MuJoCo simulation.

Composes the tested pieces the way TonyPiAdapter does: a link (FakeSonicLink in
tests, a real transport in the sim), optional telemetry, an optional simulator
sidecar, and the SafetyGuard's e-stop. Fully testable offline: every one of
those is optional and every tool that needs a missing one refuses in a way the
model can read.

THREE SOURCES, KEPT SEPARATE ON PURPOSE

    link       what we COMMAND        ZMQ -> SONIC planner
    telemetry  what the ROBOT reports  Unitree SDK2 DDS: rt/lowstate, rt/odostate
    sim        what the WORLD contains simulator sidecar: camera, scan, contacts

    The split is not tidiness. `telemetry` speaks the protocol the physical G1
    speaks, so it ports to hardware untouched; `sim` cannot exist on hardware at
    all and would be replaced by a RealSense and a Livox. Blurring them would
    produce a tool surface that silently stops working the day the robot is
    real. Every sensing result therefore names its own source.

DESIGN DECISIONS, and why each one

  * The model commands VELOCITY, MODE and POSE, never body joint angles. SONIC's
    whole job is to turn intent into 29 coordinated joint targets at 50 Hz while
    staying upright. Offering body joints would bypass the only thing keeping
    the robot standing. Fingers ARE offered directly, because they are on a
    separate channel that does not fight balance.

  * Every travelling command is BOUNDED IN TIME and expires. There is no latched
    walk. A stalled agent loop leaves the robot standing still rather than
    walking into a wall. Static POSES hold instead, because a posture cannot run
    away and a squat that stood up by itself after five seconds would be a worse
    surprise than one that stayed put.

  * Velocities are clamped to the sim envelope (sonic_link.clamp_velocity). In
    simulation this protects nothing — it is a rehearsal, so that the same tool
    surface can be pointed at hardware later without redesigning the limits.

  * `stop` and the reads stay available while e-stopped. Halting must never be
    refused; that is the safe direction.

  * Nothing invents a reading. A sensing tool with no source raises, and the
    refusal says which source is missing. A model shown a black frame will
    reason about the darkness; a model told "no camera is attached" will not.

WHAT THIS IS NOT
    Not hardware. Nothing here has driven a physical G1, and the G1 tier
    table governs that path, not this one.
"""
from __future__ import annotations

import os
import threading

from physicalagent.robots import inspire_grasps
from physicalagent.robots import sonic_fall
from physicalagent.robots import sonic_link as link_mod
from physicalagent.robots import sonic_modes
from physicalagent.robots import sonic_arm_kin
from physicalagent.robots import sonic_navigate
from physicalagent.robots import sonic_upper_body as upper_body
from physicalagent.robots import sonic_world
from physicalagent.robots.base import RobotAdapter, ToolSpec
from physicalagent.robots.sonic_link import RealClock

# Poses still reachable by the old short names. Superseded by set_mode's full
# menu of 27; kept because 'stand' and 'sit' are what a model reaches for first
# and breaking them would be a regression in behaviour, not just in API.
SAFE_POSES = ("stand", "sit", "squat", "damp")


class SonicG1Adapter(RobotAdapter):
    def __init__(self, link, guard=None, *, view=None, clock=None,
                 robot_profile=None, telemetry=None, sim=None,
                 auto_damp=True):
        super().__init__(guard=guard, robot_profile=robot_profile)
        self._link = link
        self._view = view
        self._telemetry = telemetry
        self._sim = sim
        self._clock = clock or RealClock()
        # Fall management (detect, reflex, recover). SONIC itself has
        # none: no detector, no reflex, no get-up. The detector is mode-aware
        # (sonic_fall.ENVELOPES covers all 27 modes; sources are per-entry:
        # measured-in-sim, vendor operating point, or plausible — see the
        # table) and reads rt/odostate + the IMU. Whether the PHYSICAL G1
        # publishes rt/odostate is an assumption, not a measurement —
        # verify on the bench before calling this hardware-ready. On a
        # CONFIRMED fall the watchdog damps: kp=0, kd=8 on every motor — a
        # controlled collapse instead of the policy fighting the floor and
        # wrecking actuators. The latch then marks every result until
        # reset_fall_latch(), an operator act.
        self._fall = sonic_fall.FallDetector()
        self._auto_damp = bool(auto_damp)
        # Mode allowlist (§4e step 4). Unset = every mode, which is only
        # acceptable in simulation; the G1 tier table governs hardware,
        # where day one starts from sonic_fall.HARDWARE_DAY_ONE_ALLOWLIST.
        raw = os.environ.get("SONIC_MODE_ALLOWLIST", "").strip()
        self._mode_allowlist = (frozenset(
            sonic_modes.resolve(n).name for n in raw.split(",") if n.strip())
            if raw else None)
        self._fall_alarm_fired = False
        self._lowstate_alarm_fired: str | None = None   # last label announced
        self._watchdog_error_reported = False
        self._watchdog_good_beats = 0
        self._fall_gate = threading.Lock()
        self._watchdog_stop = threading.Event()
        self._watchdog_thread: threading.Thread | None = None
        self._watchdog_sim = None   # dedicated sidecar client, lazily built
        self.last_frame: bytes | None = None
        # Built on first goto and kept, so a measured frame offset survives
        # across calls. The offset is fixed for the life of a policy process.
        self._navigator = None
        # The upper-body reference last sent, so a partial command merges onto
        # the current pose rather than snapping the other joints to neutral.
        # None means the policy still owns the arms. Declared here rather than
        # sprung into existence by the first arm command: it is read by
        # where_are_my_hands and reach_to, which a model may well call first.
        self._last_upper_body: list[float] | None = None

    # ---- tool surface ----------------------------------------------------
    def capabilities(self):
        return (
            ToolSpec("get_status",
                     "Read the robot's real state: where it is in the world, which "
                     "way it is facing, how tall it is standing, how far it is "
                     "tilted, how fast it is moving, and what it was last told to "
                     "do. Always available.",
                     {"type": "object", "properties": {}}),
            ToolSpec("look",
                     "Capture the robot's forward camera view. The image is attached "
                     "to your next turn so you can see what is in front of the robot.",
                     {"type": "object", "properties": {}}),
            ToolSpec("scan",
                     "Sweep for obstacles all the way around the robot and get the "
                     "distance and name of whatever is in each direction. Bearing 0 "
                     "is straight ahead and increases to the left. Use this before "
                     "moving somewhere you have not looked.",
                     {"type": "object", "properties": {
                         "beams": {"type": "integer",
                                   "description": "how many directions to sample, 4-180"}}}),
            ToolSpec("describe_scene",
                     "List the objects around the robot. For each one: range_m, "
                     "bearing_deg and height_m to the CENTRE of the space the "
                     "object actually occupies (not to its model origin, which "
                     "can be metres away from it); top_m, how high its top is; "
                     "size_m, how big it is as [x, y, z] metres; and "
                     "nearest_edge_m, how far the robot is from walking into "
                     "it. Heights are relative to the pelvis, so top_m near 0 "
                     "is about waist height and a large negative height_m is "
                     "near the floor — that is how to tell whether something "
                     "is on the floor or standing on a table. Names come from "
                     "the scene file. Each object also carries world_xy — the "
                     "world-frame point goto() accepts — whenever the robot's "
                     "odometry is fresh.",
                     {"type": "object", "properties": {}}),
            ToolSpec("get_contacts",
                     "Find out what is physically touching the robot right now and "
                     "how hard. This is the ONLY way to tell whether the hands are "
                     "actually holding something — closed fingers on an empty hand "
                     "look identical to closed fingers on a bottle.",
                     {"type": "object", "properties": {}}),
            ToolSpec("get_joints",
                     "Read joint angles, speeds and torques. Give a limb to keep the "
                     "answer short: left_leg, right_leg, waist, left_arm, right_arm. "
                     "Omit it to get all 29.",
                     {"type": "object", "properties": {
                         "limb": {"type": "string",
                                  "description": "left_leg, right_leg, waist, left_arm or right_arm"}}}),
            ToolSpec("walk",
                     "Walk with the PLAIN, ordinary gait by commanding a body "
                     "velocity for a short time. vx is forward (positive) or "
                     "backward (negative) in metres per second; vy is sideways, "
                     "positive to the robot's left. duration_s is how long to hold "
                     "that velocity before stopping automatically — the robot "
                     "always stops on its own, so issue this again to keep going. "
                     "Speeds are clamped; the result tells you what was actually "
                     "commanded.\n\n"
                     "THIS TOOL HAS ONE GAIT AND CANNOT CHANGE STYLE. Slowing it "
                     "down does not make it a sneak, and speeding it up does not "
                     "make it a run or a jump. If the request names a MANNER of "
                     "moving — sneak, creep, limp, stagger, tiptoe, march, "
                     "shuffle, prowl, bounce, dance, box, punch, jump, hop, crawl, "
                     "run, or moving 'like a' anything — that is a locomotion mode "
                     "and belongs to set_mode. Use set_mode even if you could "
                     "approximate the request by choosing a speed here: the "
                     "approximation looks obedient and is not the behaviour that "
                     "was asked for.",
                     {"type": "object", "properties": {
                         "vx": {"type": "number", "description": "forward speed m/s, +forward"},
                         "vy": {"type": "number", "description": "sideways speed m/s, +left"},
                         "duration_s": {"type": "number", "description": "seconds to move"}},
                      "required": ["vx"]}),
            ToolSpec("turn",
                     "Turn in place by commanding a yaw rate for a short time. "
                     "yaw_rate is radians per second, positive turns left. Like walk, "
                     "it expires on its own and is clamped.",
                     {"type": "object", "properties": {
                         "yaw_rate": {"type": "number", "description": "rad/s, +left"},
                         "duration_s": {"type": "number", "description": "seconds to turn"}},
                      "required": ["yaw_rate"]}),
            ToolSpec("goto",
                     "Walk to a POINT and stop there. Prefer this over walk() "
                     "whenever you know where you want to end up: walk() is open "
                     "loop and its error grows, while this steers on the robot's "
                     "measured position every 0.1 s and stops within "
                     f"{sonic_navigate.POS_TOL_M} m of the target.\n\n"
                     "x and y are metres in the world frame get_status() reports "
                     "as position — so read your position first, then name a "
                     "target in the same numbers. Optional yaw_deg is the "
                     "direction to face on arrival.\n\n"
                     "IT WALKS THE STRAIGHT LINE AND DOES NOT AVOID OBSTACLES. "
                     "Use scan() first if you are not sure the path is clear, and "
                     "break a long route into legs around anything in the way. "
                     f"Refuses targets beyond {sonic_navigate.MAX_RANGE_M} m for "
                     "that reason. Blocks until it arrives or gives up, and the "
                     "result always says which — read 'ok' and 'reason', and "
                     "'final_error_m' for how close it actually got.",
                     {"type": "object", "properties": {
                         "x": {"type": "number", "description": "target x in metres, world frame"},
                         "y": {"type": "number", "description": "target y in metres, world frame"},
                         "yaw_deg": {"type": "number",
                                     "description": "optional heading to face on arrival"},
                         "timeout_s": {"type": "number",
                                       "description": "give up after this long (default 60)"}},
                      "required": ["x", "y"]}),
            ToolSpec("goto_object",
                     "Walk toward a NAMED object from describe_scene() and stop "
                     "at a standoff short of it, facing it. Furniture is movable "
                     "— walking into it shoves it — so this never targets a "
                     "point inside the object. Use goto() only when you need an "
                     "exact coordinate instead of an object.",
                     {"type": "object", "properties": {
                         "name": {"type": "string",
                                  "description": "exact name from describe_scene()"},
                         "standoff_m": {"type": "number",
                                        "description": "stop this many metres short (default 0.6, min 0.4)"},
                         "timeout_s": {"type": "number",
                                       "description": "give up after this long (default 60)"}},
                      "required": ["name"]}),
            ToolSpec("calibrate_heading",
                     "Measure the offset between the direction the robot is "
                     "told to walk and the direction it actually goes. Walks "
                     "about 0.8 m straight forward to do it, so make sure there "
                     "is room.\n\n"
                     "Run this ONCE before your first goto if you can. Without "
                     "it goto assumes the offset is zero, which is usually right "
                     "but is an assumption — goto says so in its result. If a "
                     "goto reported that warning and ended up somewhere odd, "
                     "run this and retry.",
                     {"type": "object", "properties": {}}),
            ToolSpec("set_mode",
                     "Switch the robot's whole-body behaviour. This is the most "
                     "powerful control you have: the policy has 27 trained modes and "
                     "each one changes how the entire body moves.\n\n"
                     "REACH FOR THIS FIRST whenever the request describes HOW to "
                     "move rather than only where. Anything of the form 'walk like "
                     "a ...', 'move quietly', 'jump', 'crawl', 'throw a punch', "
                     "'dance' is a mode, and walk() cannot produce any of them — "
                     "walk() has exactly one gait at whatever speed you pick. "
                     "Picking a slow speed on walk() to represent 'sneak' is a "
                     "wrong answer that looks like a right one.\n\n"
                     + sonic_modes.menu() +
                     "\n\nModes marked [untested] are accepted by the policy and have "
                     "never been observed running on this robot — try them, but check "
                     "get_status() afterwards. 'static' and 'ground' modes hold until "
                     "you change them. Getting back UP from a ground mode is not "
                     "guaranteed.\n\n"
                     "LEAVE speed UNSET unless the user asked for a particular pace. "
                     "Each mode has a trained speed and setting your own makes the mode "
                     "look broken rather than making it slower.\n\n"
                     "crawl and elbow_crawl are entered in STAGES automatically: ask "
                     "for the mode you want and the adapter walks the vendor's ladder "
                     "(kneel_two_legs -> crawl -> elbow_crawl), waiting for the BODY "
                     "to reach each stage before advancing, and reverses it on the "
                     "way out. Do NOT stage manually with multiple set_mode calls — "
                     "your pacing is faster than the robot's descent and the chain "
                     "collapses. One call, the target mode, done.",
                     {"type": "object", "properties": {
                         "name": {"type": "string", "description": "a mode name from the list"},
                         "speed": {"type": "number",
                                   "description": "m/s. OMIT to use the mode's own trained speed"},
                         "duration_s": {"type": "number",
                                        "description": "seconds; only meaningful when travelling"}},
                      "required": ["name"]}),
            ToolSpec("set_pose",
                     "Change the robot's whole-body pose. Available: " + ", ".join(SAFE_POSES) +
                     ". 'damp' relaxes the joints and the robot sinks to the ground — "
                     "it is the last resort, not a normal stop. set_mode covers all of "
                     "these and 23 more.",
                     {"type": "object", "properties": {
                         "name": {"type": "string", "description": "one of: " + ", ".join(SAFE_POSES)}},
                      "required": ["name"]}),
            ToolSpec("set_height",
                     "Ask the robot to hold its body higher or lower while keeping "
                     "whatever it is doing — crouch under something, or stand tall. "
                     f"Between {sonic_modes.HEIGHT_MIN_M} and {sonic_modes.HEIGHT_MAX_M} "
                     "metres at the pelvis.",
                     {"type": "object", "properties": {
                         "height_m": {"type": "number", "description": "pelvis height in metres"}},
                      "required": ["height_m"]}),
            ToolSpec("set_arms",
                     "Move the waist and both arms by naming a pose. Available: "
                     + ", ".join(sorted(upper_body.POSES)) +
                     ". The legs are untouched — the robot keeps balancing and "
                     "can still walk while holding the pose. Use 'neutral' to "
                     "hand the arms back to the balance controller.",
                     {"type": "object", "properties": {
                         "pose": {"type": "string",
                                  "description": "one of: " + ", ".join(sorted(upper_body.POSES))}},
                      "required": ["pose"]}),
            ToolSpec("set_joints",
                     "Set individual upper-body joints by name, in radians. Only "
                     "the joints you name change; the rest hold their current "
                     "reference. This is direct joint control of the waist and "
                     "arms — the legs stay under the balance controller, so the "
                     "robot will not fall over doing it.\n\n" + upper_body.describe() +
                     "\n\nThis sets a REFERENCE. The controller may deviate from "
                     "it to stay upright. Read get_joints() afterwards to see "
                     "where the arm actually went.",
                     {"type": "object", "properties": {
                         "joints": {"type": "object",
                                    "description": "e.g. {\"right_shoulder_pitch\": -1.2, "
                                                   "\"right_elbow\": 0.8}"}},
                      "required": ["joints"]}),
            ToolSpec("reach_to",
                     "Put a hand at a POINT, instead of choosing seven joint "
                     "angles yourself. Give x, y, z and this works out the arm "
                     "angles and commands them.\n\n"
                     "COORDINATES ARE RELATIVE TO THE ROBOT'S PELVIS, not the "
                     "world: x forward, y left, z up, origin at the hips, all "
                     "metres. This is NOT the frame goto() uses — goto takes "
                     "world coordinates, this takes body-relative ones. So "
                     "'0.4, 0.2, 0.1' means 40 cm in front, 20 cm to the left, "
                     "10 cm above the hips, wherever the robot happens to be "
                     "standing. Bearings from scan() and describe_scene() are "
                     "in this same body-relative sense.\n\n"
                     "The arm can reach about "
                     f"{sonic_arm_kin.reach_limits('left')[1]:.2f} m from the "
                     "shoulder. A point outside that is refused with the "
                     "distance it could actually get to, rather than the arm "
                     "stretching at it. hand may be 'left', 'right', or "
                     "'either' to use whichever reaches better.\n\n"
                     "This is a REFERENCE like the other arm tools: the "
                     "balance controller may not track it exactly. The result "
                     "reports where the solution put the hand AND the measured "
                     "joint angles afterwards — judge by those.",
                     {"type": "object", "properties": {
                         "x": {"type": "number", "description": "metres forward of the pelvis"},
                         "y": {"type": "number", "description": "metres to the robot's left"},
                         "z": {"type": "number", "description": "metres above the pelvis"},
                         "hand": {"type": "string",
                                  "description": "left, right, or either"}},
                      "required": ["x", "y", "z"]}),
            ToolSpec("where_are_my_hands",
                     "Where both hands are right now, in the same pelvis-"
                     "relative frame reach_to uses, worked out from the "
                     "measured joint angles. Use this to check whether a reach "
                     "actually arrived, and to find out what is in range before "
                     "asking for it.",
                     {"type": "object", "properties": {}}),
            ToolSpec("release_arms",
                     "Hand the waist and arms back to the balance controller, "
                     "which will use them however it needs to stay upright.",
                     {"type": "object", "properties": {}}),
            ToolSpec("set_grasp",
                     "Shape a hand into a named grasp. Choose the style from what you "
                     "are trying to hold:\n" + inspire_grasps.describe() +
                     "\n\nsize is small, medium or large (how big the object is); "
                     "strength is light, firm or tight. This sets the fingers — it "
                     "does NOT confirm you are holding anything. Check get_contacts().",
                     {"type": "object", "properties": {
                         "grasp": {"type": "string", "description": "a grasp name from the list"},
                         "hand": {"type": "string", "description": "left, right or both"},
                         "size": {"type": "string", "description": "small, medium or large"},
                         "strength": {"type": "string", "description": "light, firm or tight"}},
                      "required": ["grasp"]}),
            ToolSpec("get_hands",
                     "Read where the fingers ACTUALLY are, and how far that is "
                     "from where they were told to go. A finger stopped short of "
                     "its target is being blocked by something. set_grasp only "
                     "tells you what you asked for; this tells you what happened.",
                     {"type": "object", "properties": {}}),
            ToolSpec("stop",
                     "Stop moving immediately and stand still. Always available, "
                     "including while motion is otherwise locked out.",
                     {"type": "object", "properties": {}}),
            ToolSpec("emergency_stop",
                     "Same as stop — halt all motion and hold a standing pose.",
                     {"type": "object", "properties": {}}),
            ToolSpec("recover_from_fall",
                     "Try to stand the robot back up AFTER a confirmed fall, by "
                     "replaying the one measured get-up route "
                     "(lie_face_down -> crawl -> kneel_two_legs -> idle). "
                     "Use it ONLY when the robot is actually down: every step "
                     "commands it toward the floor, so on a standing robot this "
                     "would lie it down — it refuses in that case rather than "
                     "obeying. It is not a fix for a stuck or confused task. "
                     "The result reports each stage's outcome and whether the "
                     "robot ended up standing; believe that, not the attempt.",
                     {"type": "object", "properties": {}}),
        )

    def _handlers(self):
        return {"get_status": self._status, "look": self._look,
                "scan": self._scan, "describe_scene": self._describe_scene,
                "get_contacts": self._contacts, "get_joints": self._joints,
                "get_hands": self._hands, "walk": self._walk, "turn": self._turn,
                "goto": self._goto, "goto_object": self._goto_object,
                "calibrate_heading": self._calibrate_heading,
                "set_mode": self._set_mode, "set_pose": self._set_pose,
                "set_height": self._set_height, "set_grasp": self._set_grasp,
                "set_arms": self._set_arms, "set_joints": self._set_joints,
                "reach_to": self._reach_to,
                "where_are_my_hands": self._where_are_my_hands,
                "release_arms": self._release_arms,
                "stop": self._stop_tool, "emergency_stop": self._stop_tool,
                "recover_from_fall": self._recover_from_fall}

    def _always_allowed_tools(self):
        """Reads and halts survive an e-stop. Stopping a biped is never refused."""
        return {"get_status", "look", "scan", "describe_scene", "get_contacts",
                "get_joints", "get_hands", "where_are_my_hands", "stop",
                "emergency_stop"}

    def _motion_tool_names(self):
        return {"walk", "turn", "goto", "goto_object", "calibrate_heading",
                "set_mode", "set_pose", "set_height", "set_grasp", "set_arms",
                "set_joints", "reach_to", "release_arms", "recover_from_fall"}

    # ---- fall watchdog ---------------------------------------------------
    def _check_lowstate(self) -> None:
        """One-shot LOW_STATE_ABSENT alarm. SONIC drops
        control PERMANENTLY if rt/lowstate goes 500 ms stale, so a silent
        stream is a live hazard the operator has to hear about.

        Guarded exactly like pose() below, and for the same reason: a telemetry
        source that cannot answer this question must not take the fall watchdog
        down with it. A raise means we do not KNOW whether the robot is
        publishing — which is a weaker claim than LOW_STATE_ABSENT and gets its
        own label, because telling an operator the robot stopped publishing
        when in fact our own reader broke sends them to the wrong fault.
        """
        try:
            status = self._telemetry.lowstate()
            active = status.get("active")
            if active is None:
                # A missing key is NOT a False. Collapsing it would announce
                # "the robot stopped publishing" because our own reader got an
                # unexpected shape — the wrong-fault error this split exists
                # to prevent.
                note = ("telemetry answered lowstate() with no 'active' field: "
                        f"{status!r}")
            else:
                active = bool(active)
                note = status.get("note")
        except Exception as exc:
            active, note = None, f"telemetry cannot report lowstate: {exc!r}"

        label = ("LOW_STATE_ABSENT ALARM" if active is False
                 else "LOW_STATE_UNKNOWN")
        # The one-shot is keyed BY LABEL. One shared flag let a transient
        # UNKNOWN (our reader hiccuped) permanently suppress the ABSENT alarm
        # that follows — and ABSENT is the one that means SONIC is about to
        # drop control for good.
        with self._fall_gate:
            if active:
                self._lowstate_alarm_fired = None
                return
            if self._lowstate_alarm_fired == label:
                return
            self._lowstate_alarm_fired = label
        # The print stays OUTSIDE the gate: it shares that lock with
        # reset_fall_latch(), and blocking a console reset behind stdout is a
        # bad trade for a message nobody is racing on.
        print(f"*** {label}: {note} ***", flush=True)

    def _fall_poll_once(self) -> dict | None:
        """One watchdog beat, separated from the thread so tests can drive it
        synchronously. Reads pose, feeds the detector, damps on a NEWLY
        confirmed fall.

        Readings must be available AND fresh. pose() keeps returning the last
        sample forever with available=True; gating on that alone means a
        frozen stream either replays one bad frame into five 'consecutive'
        strikes, or holds 'standing' forever while the robot lies on the
        floor — the exact failure sonic_telemetry's header warns about.
        """
        if self._telemetry is None:
            return None

        self._check_lowstate()

        try:
            pose = self._telemetry.pose()
        except Exception:
            pose = {}
        ok = pose.get("available") and pose.get("fresh", True)
        height = pose.get("height_m") if ok else None
        tilt = pose.get("tilt_deg") if ok else None
        contacts = self._watchdog_contacts()
        verdict = self._fall.update(height, tilt, contacts=contacts)
        # The alarm decision and reset_fall_latch() share this lock: without
        # it a reset landing between update() and this check can re-damp a
        # just-reset robot and leave _fall_alarm_fired True with the latch
        # clear — auto-damp then never fires again for the session.
        with self._fall_gate:
            if not (self._fall.fallen and not self._fall_alarm_fired):
                return verdict
            self._fall_alarm_fired = True
        if self._auto_damp:
            try:
                self._link.emergency_stop()
            except Exception as exc:
                print(f"*** FALL DETECTED and AUTO-DAMP FAILED: {exc} — "
                      "power the robot down manually ***", flush=True)
            else:
                print(f"*** FALL DETECTED ({self._fall.reason}) — motors "
                      "auto-damped. recover_from_fall can try the measured "
                      "get-up route; if it reports not standing, reset the "
                      "robot by hand, then type 'reset-fall'. ***",
                      flush=True)
        else:
            print(f"*** FALL DETECTED ({self._fall.reason}); auto-damp is "
                  "OFF ***", flush=True)
        return verdict

    def _watchdog_contacts(self):
        """Ground-contact body names for the detector, or None when no source
        exists. Sim only today: MuJoCo's own contact list via the sidecar, on
        a DEDICATED client — the tool handlers use self._sim from the console
        thread, and a ZMQ REQ socket shared across threads wedges its state
        machine. On hardware this returns None until foot/torso sensing
        exists, and the contact channel stays dormant."""
        if self._sim is None:
            return None
        if self._watchdog_sim is None:
            try:
                from physicalagent.robots.sonic_sim_client import SimClient
                self._watchdog_sim = SimClient()
            except Exception:
                return None
        try:
            r = self._watchdog_sim.body_pose(["pelvis"])
            return r.get("ground_contact") if r.get("ok") else None
        except Exception:
            return None

    # A degraded watchdog re-arms its report after this many consecutive good
    # beats (~5 s at 10 Hz). Never re-arming let one transient hiccup silence
    # every later, permanent failure; re-arming on the very next good beat
    # would let a flapping reader spam the operator at 5 Hz.
    WATCHDOG_REARM_BEATS = 50

    def _fall_watchdog_loop(self) -> None:
        while not self._watchdog_stop.wait(0.1):
            try:
                self._fall_poll_once()
                self._watchdog_good_beats += 1
                if (self._watchdog_error_reported
                        and self._watchdog_good_beats >= self.WATCHDOG_REARM_BEATS):
                    self._watchdog_error_reported = False
                    print("*** fall watchdog is beating again after a failure "
                          "— it will report the next one. ***", flush=True)
                continue
            except Exception as exc:
                self._watchdog_good_beats = 0
                # An unguarded raise in a beat used to kill this thread
                # outright, which left start_fall_watchdog()'s True standing as
                # a lie for the rest of the session: the console reports the
                # safety net as up while nothing is watching. Report once and
                # keep beating — the next sample may succeed, and a detector
                # that says it is degraded beats one that is silently gone.
                if not self._watchdog_error_reported:
                    self._watchdog_error_reported = True
                    # Guarded: this print lives INSIDE the handler that exists
                    # to keep the thread alive, so a broken stdout here would
                    # kill the watchdog — the exact bug being fixed.
                    try:
                        print(f"*** FALL WATCHDOG BEAT FAILED: {exc!r} — the "
                              "fall detector is DEGRADED. Treat the robot as "
                              "unmonitored until this is fixed. ***", flush=True)
                    except Exception:
                        pass

    def start_fall_watchdog(self) -> bool:
        """Start the 10 Hz watchdog thread. Explicit rather than a side effect
        of construction, so tests and probes get a thread-free adapter and the
        live console states its safety posture in one greppable line. Returns
        whether a watchdog is running."""
        if self._telemetry is None:
            return False
        if self._watchdog_thread is None or not self._watchdog_thread.is_alive():
            self._watchdog_stop.clear()
            self._watchdog_thread = threading.Thread(
                target=self._fall_watchdog_loop,
                name="sonic-fall-watchdog", daemon=True)
            self._watchdog_thread.start()
        return True

    def stop_fall_watchdog(self) -> None:
        """Stop the watchdog before tearing down the link/telemetry it reads —
        otherwise a fall confirmed during shutdown damps on a closed socket."""
        self._watchdog_stop.set()
        t = self._watchdog_thread
        if t is not None and t.is_alive():
            t.join(timeout=1.0)

    def reset_fall_latch(self) -> None:
        """Operator act, deliberately NOT a model-visible tool: the model must
        never be able to clear a safety latch and carry on as if the fall had
        not happened. Reached via the console's 'reset-fall' command."""
        with self._fall_gate:
            self._fall.reset()
            self._fall_alarm_fired = False

    # ---- posture, attached to every motion result -------------------------
    def _annotate(self, name, payload):
        """Every command that moves the robot comes back saying what posture the
        robot is ACTUALLY in.

        Observed 2026-08-01: the model repeatedly believed it had stood the
        robot back up when the robot was flat on the floor -- pelvis at 0.09 m
        with the torso and both wrists in contact with the ground. Nothing it
        received contradicted that belief. `set_mode("idle")` on a fallen robot
        returns {"kind": "mode", "name": "idle"} and reads exactly like success,
        because the wire accepted the message; whether the robot got up is a
        different question and nothing was answering it.

        get_status() has carried height, tilt and `upright` all along. That is
        not enough: it is a tool the model must CHOOSE to call, and a belief
        that everything is fine is precisely the state in which nothing prompts
        you to check. So the check is attached to the result instead of offered
        alongside it.
        """
        if name not in self._motion_tool_names() and name != "stop":
            return payload
        body = self._posture()
        return {**payload, "body": body} if body else payload

    # Standing is BOTH high enough and upright enough. Height alone says a
    # deep squat has fallen; tilt alone says a robot lying flat on its back at
    # 0.1 m is fine.
    STANDING_HEIGHT_M = 0.55
    UPRIGHT_TILT_DEG = 30.0

    def _posture(self) -> dict | None:
        """Measured height and tilt, with a verdict the model cannot misread.

        Reads rt/odostate, which is the robot's own estimator on hardware, so
        this is not a simulator-only capability.

        The verdict is MODE-AWARE. squat, kneel, crawl and lie_face_down put the
        robot low on purpose, and calling those a fall is the same error this
        project made five separate times in one day -- a bound calibrated on a
        standing robot applied to a mode that is not standing.
        """
        if self._telemetry is None:
            return None
        try:
            pose = self._telemetry.pose()
        except Exception:
            pose = {}

        # The latch outranks the pose read: a fallen robot with stale or
        # missing telemetry must still be reported fallen.
        if self._fall.fallen:
            damped = ("Motors were auto-damped. " if self._auto_damp else
                      "Auto-damp is OFF — the policy may still be fighting "
                      "the floor. ")
            return {"height_m": pose.get("height_m"),
                    "tilt_deg": pose.get("tilt_deg"), "standing": False,
                    "note": (f"FALL DETECTED and latched ({self._fall.reason}). "
                             + damped +
                             "recover_from_fall replays the one measured "
                             "get-up route; its reliability is stated in its "
                             "own result. If it reports not standing, ask the "
                             "operator to reset the robot by hand.")}

        return self._measured_posture(pose)

    def _measured_posture(self, pose: dict | None = None) -> dict | None:
        """Posture from the LIVE reading alone, with no latch short-circuit.

        Split out because the latch outranking the pose read is right for
        reporting and wrong for judging a recovery: after the get-up route the
        question is "is the robot standing NOW", and answering it with a latch
        set on the way up makes a successful recovery report failure (measured
        2026-08-06, 2/2 prone trials at 0.79 m).
        """
        if pose is None:
            if self._telemetry is None:
                return None
            try:
                pose = self._telemetry.pose()
            except Exception:
                pose = {}
        # Stale telemetry must not be presented as a measured posture.
        if not (pose.get("available") and pose.get("fresh", True)):
            return None
        height, tilt = pose.get("height_m"), pose.get("tilt_deg")
        if height is None or tilt is None:
            return None
        standing = height >= self.STANDING_HEIGHT_M and tilt < self.UPRIGHT_TILT_DEG

        mode = None
        try:
            mode = sonic_modes.BY_NAME.get(self._link.read_state().as_dict().get("mode"))
        except Exception:
            pass
        # Judged against the CURRENT mode's envelope, not a flat standing
        # bound: stealth_walk_2 walks perfectly at 0.533, under the 0.55
        # standing threshold — the flat test alone re-commits the §2 bug in
        # the very layer written alongside the fix.
        env = sonic_fall.ENVELOPES.get(mode.name) if mode is not None else None
        in_envelope = (env is not None and height >= env.min_height_m
                       and tilt <= env.max_tilt_deg)

        if standing:
            note = "standing"
        elif in_envelope:
            note = (f"low ({height:.2f} m, tilt {tilt:.0f} deg) — within the "
                    f"expected envelope for {mode.name} ({mode.kind} mode)")
        else:
            note = (f"NOT STANDING — pelvis {height:.2f} m, tilt {tilt:.0f} deg. "
                    "The robot is on the ground. Commanding an ordinary mode "
                    "will NOT stand it back up, and repeated attempts keep "
                    "appearing to succeed. recover_from_fall is the only thing "
                    "that tries, and it reports its own outcome; if it says "
                    "the robot is not standing, ask the operator to reset it.")
        return {"height_m": height, "tilt_deg": tilt,
                "standing": standing, "note": note}

    # ---- sensing ---------------------------------------------------------
    def _status(self):
        """Never raises: a status call that fails must still return something the
        model can read, and it must say the state is unavailable rather than
        inventing one."""
        try:
            state = self._link.read_state()
            payload = state.as_dict()
        except Exception as exc:
            payload = {"state_available": False, "note": f"state read failed: {exc}"}
        payload["commanded"] = {"mode": payload.pop("mode", None),
                                "velocity": payload.pop("velocity", None)}
        payload["command_path_verified"] = link_mod.VERIFIED_COMMAND_PATH is not None

        if self._telemetry is None:
            payload["measured"] = {
                "available": False,
                "note": "no telemetry link — height, tilt and position are NOT "
                        "measured; everything above is only what was commanded"}
            return payload
        try:
            pose = self._telemetry.pose()
            imu = self._telemetry.imu()
            controller = self._telemetry.controller()
        except Exception as exc:
            payload["measured"] = {"available": False, "note": f"telemetry read failed: {exc}"}
            return payload

        # FIRST field, ahead of anything measured. If the balance controller has
        # stopped, every number below is describing a robot that is falling over
        # and being reset, and no command can reach it -- so the model must see
        # that before it reads a pose and concludes things are fine.
        payload["controller"] = controller
        payload["measured"] = pose
        if pose.get("available"):
            payload["measured"]["upright"] = pose["tilt_deg"] < 30.0
        payload["imu"] = imu

        # A fall is followed by an automatic reset that stands the robot back up
        # at the origin, so the reading above describes a FRESH robot and looks
        # exactly like a manoeuvre that worked. Reported here because get_status
        # is where the model looks to decide whether something succeeded, and it
        # is the one place the omission actively misleads.
        if self._sim is not None:
            try:
                payload["episode"] = self._sim.episode()
            except Exception:
                pass
        return payload

    def _need_sim(self, what: str):
        if self._sim is None:
            raise RuntimeError(
                f"{what} needs the simulator sidecar and none is attached. "
                "Launch through scripts/sonic_live.sh, which starts it.")
        return self._sim

    def _look(self):
        """A frame from the robot's own camera. Prefers the sidecar (the LIVE
        simulation) over a MujocoView (a separate model instance that would show
        a robot standing still in an empty world -- a convincing, wrong answer)."""
        if self._sim is not None:
            self.last_frame = self._sim.frame_jpeg()
            return {"frame_captured": True, "bytes": len(self.last_frame or b""),
                    "source": "the robot's head camera in the live simulation"}
        if self._view is None:
            raise RuntimeError("no camera is attached to this robot")
        self.last_frame = self._view.frame_jpeg()
        return {"frame_captured": True, "bytes": len(self.last_frame or b"")}

    def _scan(self, beams=36):
        return self._need_sim("scan").scan(beams=int(beams))

    def _describe_scene(self):
        out = self._need_sim("describe_scene").objects()
        pose = self._telemetry.pose() if self._telemetry is not None else None
        return sonic_world.augment_objects(out, pose)

    def _contacts(self):
        return self._need_sim("get_contacts").contacts()

    def _hands(self):
        return self._need_sim("get_hands").hand_state()

    def _joints(self, limb=None):
        if self._telemetry is None:
            raise RuntimeError(
                "no telemetry link is attached, so joint angles cannot be read")
        return self._telemetry.joints(limb)

    # ---- motion ----------------------------------------------------------
    def _require_allowed(self, mode_name: str) -> None:
        """The allowlist must gate every path that changes the wire mode, not
        just set_mode — set_pose, walk, turn and goto all select modes too,
        and a gate with side doors is not a gate (adversarial review,
        2026-08-01: all three bypasses were found and demonstrated)."""
        if self._mode_allowlist is not None and mode_name not in self._mode_allowlist:
            raise ValueError(
                f"mode '{mode_name}' is not on this deployment's allowlist. "
                f"Allowed: {', '.join(sorted(self._mode_allowlist))}. The list "
                "is an operator decision ($SONIC_MODE_ALLOWLIST), not a "
                "negotiation — do not retry with a synonym or another tool.")

    def _move(self, vx, vy, wz, duration_s):
        """One clamped, bounded motion command. Shared by walk and turn so the
        clamp and the expiry cannot drift apart between the two tools."""
        vx, vy, wz = link_mod.clamp_velocity(vx, vy, wz)
        duration_s = link_mod.clamp_duration(duration_s)
        # set_velocity switches the wire mode to a walking mode; the detector
        # and the allowlist must both see that, or a walk() after a ground
        # mode leaves the detector judging a walking robot by an
        # accept-everything envelope (and the allowlist gated nothing).
        self._require_allowed("slow_walk")
        self._fall.note_mode("slow_walk")
        self._link.set_velocity(vx, vy, wz, duration_s)
        self._clock.sleep(duration_s)
        # Stop explicitly at the end. Relying on the far side to expire the
        # command would be trusting an unverified behaviour with a walking robot.
        self._link.stop()
        return {"vx": vx, "vy": vy, "yaw_rate": wz, "duration_s": duration_s,
                "stopped": True}

    def _walk(self, vx, vy=0.0, duration_s=None):
        return self._move(vx, vy or 0.0, 0.0, duration_s)

    def _turn(self, yaw_rate, duration_s=None):
        return self._move(0.0, 0.0, yaw_rate, duration_s)

    def _goto(self, x, y, yaw_deg=None, timeout_s=None):
        """Walk to a point, closed on measured position.

        Refuses without telemetry rather than falling back to open-loop walking:
        a goto that silently degrades into "walk roughly that way for a while"
        would report arrival it never checked, which is the failure this whole
        tool exists to remove.
        """
        import math

        if self._telemetry is None:
            raise ValueError(
                "goto needs the robot's measured position and no telemetry is "
                "attached. Use walk() and turn(), and check get_status() as you "
                "go — they are open loop, so verify rather than assume.")
        self._require_allowed("slow_walk")   # the navigator drives slow_walk
        self._fall.note_mode("slow_walk")
        if self._navigator is None:
            self._navigator = sonic_navigate.Navigator(
                self._link, self._telemetry, self._clock)
        goal = sonic_navigate.Goal(
            float(x), float(y),
            None if yaw_deg is None else math.radians(float(yaw_deg)))
        return self._navigator.go(
            goal, timeout_s=float(timeout_s) if timeout_s else 60.0)

    def _goto_object(self, name, standoff_m=0.6, timeout_s=None):
        """Resolve a describe_scene name to a world point and goto a STANDOFF
        short of it — walking INTO furniture shoves it (2.9 kg free bodies)."""
        scene = self._describe_scene()
        pose = self._telemetry.pose() if self._telemetry is not None else None
        if not (pose and pose.get("available")):
            raise ValueError("goto_object needs the robot's measured position "
                             "and no telemetry is attached.")
        rxy = (pose["position_m"]["x"], pose["position_m"]["y"])
        x, y, yaw_deg = sonic_world.resolve_goto_object(
            scene, str(name), float(standoff_m), robot_xy=rxy)
        res = self._goto(x, y, yaw_deg=yaw_deg, timeout_s=timeout_s)
        obj = next(o for o in scene["objects"] if o["name"] == name)
        res.update(object=name, object_world_xy=obj["world_xy"],
                   standoff_m=max(sonic_world.MIN_STANDOFF_M, float(standoff_m)))
        return res

    def _chain_values(self, side):
        """The 10 chain angles for one arm, preferring MEASURED over commanded.

        The waist matters as much as the arm here: solving with the waist at
        zero while the torso is actually turned puts the whole arm frame in the
        wrong place, and the reach misses by the turn angle. Measured first,
        last commanded second, neutral last.
        """
        base = list(self._last_upper_body) if self._last_upper_body else \
            list(upper_body.NEUTRAL)
        measured = None
        if self._telemetry is not None:
            try:
                rows = self._telemetry.joints()
                if rows.get("available") and rows.get("fresh"):
                    import math
                    measured = {k: math.radians(v["angle_deg"])
                                for k, v in rows["joints"].items()}
            except Exception:
                measured = None

        out = []
        for link in sonic_arm_kin.LINKS[side]:
            if measured is not None and link.joint in measured:
                out.append(measured[link.joint])
            else:
                out.append(base[upper_body.INDEX[link.joint]])
        return out, (measured is not None)

    def _where_are_my_hands(self):
        out = {"frame": "pelvis-relative metres: x forward, y left, z up, "
                        "origin at the hips — the same frame reach_to takes"}
        for side in ("left", "right"):
            chain, measured = self._chain_values(side)
            state = sonic_arm_kin.fk(chain, side)
            out[side] = {
                "palm_m": [round(v, 3) for v in state["palm_pos"]],
                "wrist_m": [round(v, 3) for v in state["wrist_pos"]],
                "from": "measured joint angles" if measured
                        else "the last commanded pose — NOT measured",
            }
        out["reach_from_shoulder_m"] = round(
            sonic_arm_kin.reach_limits("left")[1], 3)
        return out

    def _reach_to(self, x, y, z, hand="either"):
        target = (float(x), float(y), float(z))
        want = str(hand).strip().lower()
        if want in ("either", "any", "closest", "best"):
            sides = ["left", "right"]
        elif want in ("left", "right"):
            sides = [want]
        else:
            raise ValueError(f"hand must be left, right or either, not {hand!r}")

        best = None
        for side in sides:
            chain, _ = self._chain_values(side)
            waist = tuple(chain[:sonic_arm_kin.WAIST_LINKS])
            arm_now = chain[sonic_arm_kin.WAIST_LINKS:]
            sol = sonic_arm_kin.ik(target, side, waist=waist, seed=arm_now)
            sol["side"] = side
            if best is None or sol["error_m"] < best["error_m"]:
                best = sol

        if not best["ok"]:
            return {
                "ok": False,
                "reason": f"cannot reach that point — the closest the "
                          f"{best['side']} hand can get is "
                          f"{best['error_m']:.2f} m away. The arm reaches about "
                          f"{best['reach_span_m']:.2f} m from the shoulder, so "
                          f"walk closer or pick a nearer point.",
                "target_m": [round(v, 3) for v in target],
                "closest_reachable_m": [round(v, 3) for v in best["palm_pos"]],
                "hand": best["side"],
                "nothing_was_commanded": True,
            }

        values, unknown = upper_body.from_named_joints(
            best["joints"], base=list(self._last_upper_body)
            if self._last_upper_body else None)
        assert not unknown, unknown
        self._link.set_upper_body(values)
        return self._arm_result(
            values,
            hand=best["side"],
            target_m=[round(v, 3) for v in target],
            solution_palm_m=[round(v, 3) for v in best["palm_pos"]],
            solution_error_m=round(best["error_m"], 4),
            note_on_frame="target was pelvis-relative; use where_are_my_hands() "
                          "to see where the hand ended up in the same frame")

    def _calibrate_heading(self):
        if self._telemetry is None:
            raise ValueError(
                "calibrating the heading frame needs the robot's measured "
                "position and no telemetry is attached.")
        if self._navigator is None:
            self._navigator = sonic_navigate.Navigator(
                self._link, self._telemetry, self._clock)
        return self._navigator.calibrate()

    # The vendor's own driver never jumps into or out of a crawl: it
    # stages kneel_two_legs -> crawl -> elbow_crawl with a dwell between, and
    # reverses the ladder to stand back up (beginCrawlingTransition,
    # gamepad_manager.hpp:468-510). Gemini cannot be trusted with that timing —
    # its per-prompt latency and the 12-hop cap broke every manual attempt on
    # 2026-08-01 — so the adapter owns it.
    CRAWL_LADDER = ("kneel_two_legs", "crawl", "elbow_crawl")
    STAGE_DWELL_S = 2.5     # blind fallback when no telemetry is attached
    # Pelvis band that says the BODY actually reached the stage, from the
    # vendor heights and the measured settles (kneel 0.407-0.471, clean crawl
    # 0.382-0.502, elbow bouts 0.15-0.32). A fixed dwell is NOT enough: the
    # first live ladder run advanced every 1.5-2.5 s while the descent takes
    # 4-6 s, and the policy never committed — the robot stayed standing
    # through the whole chain (measured 2026-08-01 evening).
    STAGE_TARGETS = {"kneel_two_legs": (0.44, 0.12), "crawl": (0.44, 0.12),
                     "elbow_crawl": (0.30, 0.12)}
    STAGE_MIN_DWELL_S = 2.0   # the vendor's own minimum between stages
    STAGE_TIMEOUT_S = 10.0

    def _wait_for_stage(self, stage: str) -> str:
        """Block until the body reaches the stage's posture band.
        Returns 'reached', 'timeout' (continued anyway, honestly reported),
        or 'blind' (no telemetry — vendor-style fixed dwell)."""
        target, tol = self.STAGE_TARGETS[stage]
        return self._wait_for_band(target, tol, self.STAGE_TIMEOUT_S)

    def _wait_for_band(self, target: float, tol: float, timeout_s: float) -> str:
        """The waiting half of _wait_for_stage, with the band passed in so the
        recovery route can wait on postures that are not crawl-ladder stages
        (prone, and standing at the end)."""
        if self._telemetry is None:
            self._clock.sleep(self.STAGE_DWELL_S)
            return "blind"
        start = self._clock.time()
        while True:
            self._clock.sleep(0.2)
            elapsed = self._clock.time() - start
            height = None
            try:
                pose = self._telemetry.pose()
                if pose.get("available") and pose.get("fresh", True):
                    height = pose.get("height_m")
            except Exception:
                pass
            if (elapsed >= self.STAGE_MIN_DWELL_S and height is not None
                    and abs(height - target) <= tol):
                return "reached"
            if elapsed >= timeout_s:
                return "timeout"

    def _ladder_stages(self, current: str | None, target: str) -> list[str]:
        """Intermediate modes to pass through, in order, excluding the target.
        Empty when the transition is adjacent or outside the crawl family."""
        ladder = self.CRAWL_LADDER
        cur = ladder.index(current) if current in ladder else -1
        tgt = ladder.index(target) if target in ladder else -1
        if cur == tgt or (cur == -1 and tgt == -1):
            return []
        step = 1 if tgt > cur else -1
        return [ladder[p] for p in range(cur + step, tgt, step)]

    def _set_mode(self, name, speed=None, duration_s=None, stage=True):
        """stage=False sends the mode with NO generic crawl ladder in front of
        it. Reserved for callers replaying a route that was measured as a
        whole — today only the get-up route, whose order the ladder would
        actively corrupt (see RECOVERY_ROUTE). It is not a speed optimisation
        and must not be used to skip staging on an ordinary transition."""
        mode = sonic_modes.resolve(name)      # raises with the menu if unknown
        self._require_allowed(mode.name)

        current = None
        try:
            current = self._link.read_state().as_dict().get("mode")
        except Exception:
            pass
        stages = self._ladder_stages(current, mode.name) if stage else []
        # Every stage must be allowed BEFORE the first one is sent — refusing
        # halfway through would strand the robot mid-ladder.
        for stage in stages:
            self._require_allowed(stage)
        stage_report = []
        for stage in stages:
            self._fall.note_mode(stage)
            self._link.select_mode(stage)
            stage_report.append(f"{stage}:{self._wait_for_stage(stage)}")

        self._fall.note_mode(mode.name)
        result = self._link.select_mode(mode.name, speed=speed,
                                        duration_s=duration_s)
        if stages and isinstance(result, dict):
            result = {**result, "staged_through": stage_report,
                      "note": ("entered via the vendor's staged ladder, each "
                               "stage waiting for the body to arrive "
                               f"({' -> '.join(stage_report)} -> {mode.name}); "
                               "a stage marked 'timeout' means the body never "
                               "reached that posture — check the body field "
                               "before claiming the mode worked. Exit by "
                               "asking for idle; the ladder reverses "
                               "automatically")}
        if isinstance(result, dict) and duration_s and mode.takes_speed and speed:
            # A travelling mode was asked for with a duration: sleep it out and
            # halt, exactly as walk() does, so the same expiry guarantee holds
            # whichever tool the model reached for.
            self._clock.sleep(link_mod.clamp_duration(duration_s))
            self._link.stop()
            result = {**result, "stopped": True}
        return result

    def _set_pose(self, name):
        key = str(name).strip().lower()
        if key not in SAFE_POSES:
            raise ValueError(
                f"pose {name!r} is not available; choose from "
                f"{', '.join(SAFE_POSES)} — or use set_mode for all 27 modes")
        if key == "damp":
            # Commanded damp: going to the floor ON PURPOSE. Without this the
            # watchdog confirms a "fall" on the safety pose itself. And damp
            # is never allowlist-gated — refusing the safe-collapse command
            # would be the one refusal that can hurt someone.
            self._fall.note_damp()
        else:
            resolved = sonic_modes.resolve(key).name
            self._require_allowed(resolved)
            self._fall.note_mode(resolved)
        self._link.select_mode(key)
        return {"pose": key}

    def _set_height(self, height_m):
        result = self._link.set_height(height_m)
        # The fall envelopes assume each mode's default height; a commanded
        # height moves the whole operating point, and the detector must follow
        # or a lawful crouch reads as a fall (review item 5).
        try:
            self._fall.note_height(result.get("height_m")
                                   if isinstance(result, dict) else None)
        except Exception:
            pass
        return result

    # ---- upper body ------------------------------------------------------
    # The waist and both arms ride on the planner topic as `upper_body_position`,
    # so the legs never leave the balance controller. That is the whole reason
    # joint-level control is offered here and refused for the legs.
    # The policy blends toward an upper-body reference over about a second
    # rather than snapping to it -- it is a learned tracker weighing balance,
    # not a servo. Long enough to have converged, short enough not to stall the
    # agent loop.
    ARM_SETTLE_S = 1.5

    def _arm_result(self, values, **extra) -> dict:
        """Set, wait, and report what ACTUALLY happened.

        The settle-and-read-back is not politeness. The first live run of this
        tool returned immediately, the model read the joints mid-transit and
        reported the arm had failed to move -- while it was still on its way and
        eventually got most of the way there. A tool that returns before its
        effect exists forces the model to guess when to look, and it guesses
        wrong in the direction of declaring failure.
        """
        self._last_upper_body = list(values)
        # Wait out the RAMP as well as the policy's own blend. Measuring while
        # the reference is still travelling reports the gap as tracking error,
        # which is how a correct arm gets called broken -- this project has
        # already done that once with a settle time that was simply too short.
        settle = self.ARM_SETTLE_S
        try:
            settle += float(self._link.upper_body_travel_time())
        except (AttributeError, TypeError):
            pass
        out = {"commanded_deg": upper_body.as_degrees(values),
               "legs": "still under the balance controller — the robot can walk "
                       "while holding this"}
        out.update(extra)

        self._clock.sleep(settle)
        if self._telemetry is None:
            out["note"] = ("no telemetry attached, so this is what was COMMANDED, "
                           "not what the arm did. The controller may deviate from "
                           "the reference to stay upright.")
            return out
        try:
            reached = self._telemetry.joints()["joints"]
        except Exception:
            return out

        gaps = {}
        for i, name in enumerate(upper_body.JOINT_NAMES):
            row = reached.get(name)
            if row is None:
                continue
            want = out["commanded_deg"][name]
            gaps[name] = {"commanded_deg": want, "actual_deg": row["angle_deg"],
                          "error_deg": round(row["angle_deg"] - want, 1)}
        worst = max(gaps.values(), key=lambda g: abs(g["error_deg"]), default=None)
        out["reached"] = gaps
        out["note"] = (
            f"measured {settle:.1f}s after commanding (the reference is ramped, "
            "not stepped, so this waits for the ramp too). The controller "
            "tracks this reference but weighs balance too, so it will not land "
            "exactly on it — judge success by 'actual_deg', never by what you "
            "asked for."
            + (f" Largest gap: {abs(worst['error_deg'])} deg." if worst else ""))
        return out

    def _set_arms(self, pose):
        values = upper_body.pose(pose)       # raises with the menu if unknown
        self._link.set_upper_body(values)
        return self._arm_result(values, pose=pose)

    def _set_joints(self, joints):
        if not isinstance(joints, dict) or not joints:
            raise ValueError(
                "joints must be a mapping of joint name to radians, e.g. "
                '{"right_shoulder_pitch": -1.2}. Names: '
                + ", ".join(upper_body.JOINT_NAMES))
        # Merge onto what is currently commanded, not onto neutral: a model
        # adjusting one joint of a pose it just set must not have the other
        # sixteen silently snap back.
        base = self._last_upper_body or upper_body.NEUTRAL
        values, unknown = upper_body.from_named_joints(joints, base=base)
        self._link.set_upper_body(values)
        extra = {"changed": sorted(set(joints) - set(unknown))}
        if unknown:
            # Named and returned, never dropped in silence -- a model whose
            # typo vanished concludes the arm is broken.
            extra["ignored_unknown_joints"] = unknown
            extra["valid_joint_names"] = list(upper_body.JOINT_NAMES)
        return self._arm_result(values, **extra)

    def _release_arms(self):
        self._last_upper_body = None
        return self._link.set_upper_body(None)

    def _set_grasp(self, grasp, hand="both", size="medium", strength="firm"):
        sim = self._need_sim("set_grasp")
        side = str(hand).strip().lower()
        sides = ("left", "right") if side in ("both", "all") else (side,)
        payload = {s: inspire_grasps.expand(grasp, size=size, strength=strength,
                                            side=s) for s in sides}
        reply = sim.set_hands(**payload)
        if not reply.get("ok", True):
            # The sidecar refuses on a rigid-hand scene. Pass the refusal
            # through verbatim rather than dressing it as a success -- a grasp
            # the model believes happened is worse than one it knows failed.
            raise RuntimeError(reply.get("error", "hands unavailable"))
        out = inspire_grasps.grasp_result(grasp, size, strength, side)
        out["applied"] = reply.get("applied")
        return out

    def _stop_tool(self):
        self.stop_all()
        return {"stopped": True}

    # The get-up route, and the ONLY one with evidence behind it: probe-measured
    # 2026-08-02 from a real shoulder-drop fall, n=1, pelvis
    # 0.225 -> 0.151 -> 0.431 -> 0.410 -> 0.787 m, ending standing.
    #
    # The ORDER is load-bearing, and it is deliberately NOT the crawl ladder:
    #  - lie_face_down first: it normalises a twisted fall into the clean prone
    #    the rest of the route assumes.
    #  - crawl SECOND, because crawl's own ENTRY TRANSITION is the thing that
    #    lifts a prone body to a ~0.449 m quadruped. _set_mode's generic ladder
    #    inserts kneel_two_legs here instead, which asks a PRONE robot to kneel
    #    — a transition nothing has ever measured. Hence stage=False.
    #  - crawl at speed 0: vendor speeds dropped a shoulder 4 times in 5.
    #  - idle LAST and only from kneel_two_legs. idle-from-prone is the
    #    catastrophic shortcut: it thrashed, collapsed to 0.078 m and dragged
    #    the robot 1.99 m past the band's origin guard (measured 2026-08-02).
    #    This route refuses it by construction — idle appears once, at the end.
    # (name, speed, pelvis target m, tolerance m)
    RECOVERY_ROUTE = (
        ("lie_face_down", None, 0.19, 0.12),   # (m) 0.151-0.225 prone
        ("crawl",         0.0,  0.44, 0.12),   # (m) clean crawl 0.382-0.502
        ("kneel_two_legs", None, 0.44, 0.12),  # (m) kneel 0.407-0.471
        ("idle",          None, 0.78, 0.12),   # (m) idle stands at 0.786
    )
    RECOVERY_STAGE_TIMEOUT_S = 12.0   # entering lie_face_down alone takes ~6 s

    def _recover_from_fall(self):
        """Stand the robot back up by replaying the one measured get-up route.

        REFUSES on a robot that is verifiably standing. Every step here is a
        command to go to the floor, so calling this by mistake on a healthy
        robot would deliberately lie it down — the tool description reads as a
        rescue, and a model will reach for it when a task merely looks stuck.
        With no telemetry the refusal is the same unless a fall is latched:
        lying a possibly-standing robot down on a guess is the worse error.

        Each stage waits for the BODY to arrive rather than sleeping a guess.
        Fixed dwells were measured failing on this exact chain: the first live
        ladder run advanced every 1.5-2.5 s while the descent takes 4-6 s and
        the policy never committed. Every stage outcome is reported.
        """
        latched = self._fall.fallen
        posture = self._posture()
        if not latched:
            if posture is None:
                return {"recovered": False, "refused": "posture unknown",
                        "note": ("No fall is latched and there is no live "
                                 "posture reading, so I cannot tell whether "
                                 "the robot is down. Every step of this "
                                 "recovery commands the robot to the floor, so "
                                 "I will not run it on a guess. Restore "
                                 "telemetry, or reset the robot by hand.")}
            if posture.get("standing"):
                return {"recovered": False, "refused": "already standing",
                        "posture": posture,
                        "note": ("The robot is standing, so there is nothing "
                                 "to recover from. Running this would lie it "
                                 "down on purpose.")}

        # Every mode in the route must be allowed BEFORE the first one is sent:
        # refusing halfway leaves the robot stranded on the floor mid-route.
        for name, _, _, _ in self.RECOVERY_ROUTE:
            self._require_allowed(name)

        stage_report = []
        self._fall.begin_recovery()
        try:
            for name, speed, target, tol in self.RECOVERY_ROUTE:
                self._set_mode(name, speed=speed, stage=False)
                if name == "lie_face_down":
                    # Ordering matters: _set_mode has just switched the
                    # detector's envelope to lie_face_down, where being on the
                    # floor is lawful. Clearing the latch before that would
                    # re-trip the alarm on the very next beat.
                    self.reset_fall_latch()
                outcome = self._wait_for_band(target, tol,
                                              self.RECOVERY_STAGE_TIMEOUT_S)
                stage_report.append(f"{name}:{outcome}")
        finally:
            self._fall.end_recovery()

        # Judged on the LIVE reading, not on _posture()'s latch short-circuit:
        # a latch set on the way up would report a robot standing at 0.79 m as
        # a failed recovery, which is exactly what 2/2 measured trials did.
        posture = self._measured_posture()
        recovered = bool(posture and posture.get("standing"))
        relatched = self._fall.fallen
        if recovered and relatched:
            # Standing is measured, so the latch is stale — and leaving it set
            # marks every later result "FALL DETECTED" and blocks the operator.
            # Cleared on the MEASUREMENT, never on the mere fact that the route
            # ran: a success flag the route can set by itself is the farmable
            # scalar this project has already been burned by twice.
            self.reset_fall_latch()
        note = ("The robot is standing again and the fall latch is clear."
                if recovered else
                "The route ran to the end and the robot is NOT standing. "
                "Do not re-run it blindly — check the stage outcomes: a "
                "'timeout' names the posture the body never reached.")
        if relatched:
            note += (" NOTE: the fall detector latched again during the route"
                     + (f" ({self._fall.reason})" if self._fall.reason else "")
                     + ". Worth reporting — the guard may be firing on the "
                       "recovery itself.")
        return {"recovered": recovered, "route": stage_report,
                "posture": posture, "relatched_during_route": relatched,
                "note": note}

    # ---- RobotAdapter required ------------------------------------------
    def get_frame(self) -> bytes:
        if self._sim is not None:
            try:
                self.last_frame = self._sim.frame_jpeg()
                return self.last_frame
            except Exception:
                return b""
        if self._view is None:
            return b""
        self.last_frame = self._view.frame_jpeg()
        return self.last_frame

    def telemetry(self) -> dict:
        return self._status()

    def stop_all(self) -> None:
        # Best-effort and never raises: a failing stop must not prevent the rest
        # of the halt sequence, and stop_all is called from the agent loop's exit
        # path where an exception would leave the robot moving.
        #
        # MODE-AWARE, and this is load-bearing: the brain calls stop_all at the
        # end of EVERY task (er.py), and sending idle@height to a robot on its
        # hands and knees is a stand-up command it cannot execute — measured
        # 2026-08-01, it collapses. "Halted" means different things per kind:
        #   GROUND  -> same mode at speed 0: stop travelling, hold the posture
        #   STATIC  -> already stationary; leave the planner alone
        #   else    -> idle, as before
        current = None
        try:
            current = self._link.read_state().as_dict().get("mode")
        except Exception:
            pass
        mode = sonic_modes.BY_NAME.get(current) if current else None
        try:
            if mode is not None and mode.kind == sonic_modes.GROUND:
                self._fall.note_mode(mode.name)
                self._link.select_mode(mode.name, speed=0.0)
            elif mode is not None and mode.kind == sonic_modes.STATIC:
                self._fall.note_mode(mode.name)
            else:
                self._fall.note_mode("idle")
                self._link.stop()
        except Exception:
            pass
