"""Safety layer — pure policy that wraps every robot command.

Design note: this module NEVER touches hardware or the network. It takes proposed
commands and returns clamped/validated ones (or raises). That keeps it fully
unit-testable and makes it the one place all limits live. Adapters translate the
validated typed commands into ROS/RPC messages afterwards.
"""
from __future__ import annotations

import time as _time
from dataclasses import dataclass, replace

from physicalagent import config


class SafetyError(Exception):
    """Raised when a command violates a safety rule and must not be sent."""


def _clamp(value: float, limit: float) -> float:
    """Clamp value to [-limit, +limit]."""
    return max(-limit, min(limit, value))


# ---------------------------------------------------------------------------
# Velocity
# ---------------------------------------------------------------------------
@dataclass(frozen=True)
class Velocity:
    x: float = 0.0
    y: float = 0.0
    yaw_rate: float = 0.0
    stop: bool = False


@dataclass(frozen=True)
class VelocityLimits:
    max_abs_x: float
    max_abs_yaw: float
    allow_lateral_y: bool
    max_abs_y: float = 0.0


def clamp_velocity(vel: Velocity, limits: VelocityLimits) -> Velocity:
    """Return a Velocity clamped to the given limits. Units are m/s and rad/s.

    Lateral y IS implemented on the ROSPug (unlike the PuppyPi), so when allowed
    it is clamped to max_abs_y rather than passed through; when disallowed it is
    forced to 0. The stop flag passes through untouched.
    """
    return replace(
        vel,
        x=_clamp(vel.x, limits.max_abs_x),
        y=_clamp(vel.y, limits.max_abs_y) if limits.allow_lateral_y else 0.0,
        yaw_rate=_clamp(vel.yaw_rate, limits.max_abs_yaw),
    )


def velocity_limits_for(motion_unlocked: bool) -> VelocityLimits:
    """Pick velocity limits based on whether motion has been unlocked.

    Units are VERIFIED SI (m/s, rad/s) — see config.py for the source evidence.
    LOCKED (default): half the vendor's own app-path values; the timid setting
    used until a human has watched this robot move under our control.
    UNLOCKED: the vendor's own conservative limits (0.1 m/s / 0.35 rad/s).
    Neither ever exceeds config.VENDOR_MAX_*.
    """
    if motion_unlocked:
        return VelocityLimits(
            max_abs_x=config.VERIFIED_MAX_ABS_X_M_S,
            max_abs_y=config.VERIFIED_MAX_ABS_Y_M_S,
            max_abs_yaw=config.VERIFIED_MAX_ABS_YAW_RAD_S,
            allow_lateral_y=config.ALLOW_LATERAL_Y,
        )
    return VelocityLimits(
        max_abs_x=config.UNVERIFIED_MAX_ABS_X,
        max_abs_y=config.UNVERIFIED_MAX_ABS_Y,
        max_abs_yaw=config.UNVERIFIED_MAX_ABS_YAW,
        allow_lateral_y=config.ALLOW_LATERAL_Y,
    )


# ---------------------------------------------------------------------------
# Action-group whitelist
# ---------------------------------------------------------------------------
def check_action_allowed(name: str, whitelist) -> str:
    """Return name if it is in the whitelist; otherwise raise SafetyError.

    An empty whitelist rejects everything — the safe default. The ROSPug's real
    names live in config.ROSPUG.action_groups (verified off the robot); pass them
    WITHOUT the .d6a extension, which the vendor controller appends itself.
    """
    if name not in whitelist:
        raise SafetyError(
            f"action {name!r} is not in the whitelist {tuple(whitelist)!r} — refusing"
        )
    return name


# ---------------------------------------------------------------------------
# Bounded actions — continuous motion must be re-chosen by the model each call
# ---------------------------------------------------------------------------
def bound_steps(n: int, max_steps: int = config.MAX_STEPS_PER_CALL) -> int:
    """Clamp a step count to [0, max_steps]."""
    return max(0, min(int(n), max_steps))


def bound_duration(seconds: float, max_s: float) -> float:
    """Clamp a duration to [0, max_s]."""
    return max(0.0, min(float(seconds), max_s))


def max_duration_for(motion_unlocked: bool) -> float:
    """Per-call duration cap; short while motion is locked, larger once verified."""
    return (
        config.VERIFIED_MAX_DURATION_S
        if motion_unlocked
        else config.UNVERIFIED_MAX_DURATION_S
    )


# ---------------------------------------------------------------------------
# Pose
# ---------------------------------------------------------------------------
@dataclass(frozen=True)
class Pose:
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0
    height: float = config.DEFAULT_HEIGHT_M
    x_shift: float = 0.0
    stance_x: float = 0.0
    stance_y: float = 0.0
    run_time: float = config.DEFAULT_POSE_RUN_TIME_S


@dataclass(frozen=True)
class PoseLimits:
    max_abs_roll: float
    max_abs_pitch: float
    height_min: float
    height_max: float
    max_abs_x_shift: float
    max_abs_stance: float


def clamp_pose(pose: Pose, limits: PoseLimits) -> Pose:
    """Clamp a pose to safe ranges. yaw is forced to 0 (not steerable via pose)."""
    return replace(
        pose,
        roll=_clamp(pose.roll, limits.max_abs_roll),
        pitch=_clamp(pose.pitch, limits.max_abs_pitch),
        yaw=0.0,
        height=max(limits.height_min, min(limits.height_max, pose.height)),
        x_shift=_clamp(pose.x_shift, limits.max_abs_x_shift),
        stance_x=_clamp(pose.stance_x, limits.max_abs_stance),
        stance_y=_clamp(pose.stance_y, limits.max_abs_stance),
    )


def default_pose_limits() -> PoseLimits:
    """Pose limits from config, in METRES/RADIANS.

    INFERRED, not vendor-verified: pug_node.pose_callback applies NO clamp at all,
    so these bounds are ours alone. Anchored on walking_param.yaml (height
    -0.13 m) and the SDK IK example (Z = -0.15 m).
    """
    return PoseLimits(
        max_abs_roll=config.MAX_ABS_ROLL_RAD,
        max_abs_pitch=config.MAX_ABS_PITCH_RAD,
        height_min=config.HEIGHT_MIN_M,
        height_max=config.HEIGHT_MAX_M,
        max_abs_x_shift=config.MAX_ABS_X_SHIFT_M,
        max_abs_stance=config.MAX_ABS_STANCE_M,
    )


# ---------------------------------------------------------------------------
# SafetyProfile — every limit for ONE robot, in one frozen object
# ---------------------------------------------------------------------------
# Explicit "this robot has no battery to gate on". Pass this, never a bare
# None: the point is that the profile author had to TYPE something. See the
# battery fields on SafetyProfile below for why.
NO_BATTERY = None


@dataclass(frozen=True)
class SafetyProfile:
    """All safety limits for a single robot.

    Replaces the module-level constants that hard-coded the ROSPug. The vendor
    ceiling is checked in __post_init__ rather than only in tests: on the
    ROSPug's primary interface the vendor applies NO clamp of its own, so a
    profile declaring a limit above the ceiling would be a live hazard, not a
    latent test failure. Making it unconstructable is the stronger guarantee.

    The battery fields have NO DEFAULT, deliberately. They are Optional because
    not every robot has a pack -- a mains-powered arm has nothing to gate on --
    but "not every robot has one" must never collapse into "forgetting to say
    is the same as saying no". A default of None meant a profile author who
    simply omitted the field got NO battery gate, silently, in the one layer
    standing between a language model and 12 servos. That is fail-open.

    So absence is now unrepresentable: state a floor in millivolts, or state
    NO_BATTERY. Omitting either is a TypeError at construction, which is a
    failure the author sees immediately rather than one the robot discovers by
    over-discharging its pack. This is the same reasoning as the vendor-ceiling
    check in __post_init__: for a safety limit, unconstructable beats untested.
    """

    name: str
    locked_velocity: VelocityLimits
    unlocked_velocity: VelocityLimits
    vendor_ceiling: VelocityLimits
    pose: PoseLimits
    locked_max_duration_s: float
    unlocked_max_duration_s: float
    max_steps_per_call: int
    # Required. int millivolts, or NO_BATTERY for a robot with no pack.
    # Placed ahead of the defaulted fields so the dataclass enforces it.
    battery_motion_min_mv: int | None
    battery_warn_mv: int | None
    action_whitelist: tuple[str, ...] = ()
    rate_limit_hz: float = 10.0
    watchdog_timeout_s: float = 1.0

    def __post_init__(self):
        for label, limits in (("locked", self.locked_velocity),
                              ("unlocked", self.unlocked_velocity)):
            for field_name in ("max_abs_x", "max_abs_y", "max_abs_yaw"):
                declared = getattr(limits, field_name)
                ceiling = getattr(self.vendor_ceiling, field_name)
                if declared > ceiling:
                    raise SafetyError(
                        f"profile {self.name!r}: {label} {field_name}={declared} "
                        f"exceeds the vendor ceiling {ceiling} — refusing to build"
                    )

        # A profile can state a floor without a warn level (or vice versa) and
        # still build: NO_BATTERY is a valid value for either field on its
        # own. That makes a profile "half battery-aware" -- e.g. a floor with
        # NO_BATTERY as the warn level builds fine, looks like it gates
        # motion, but battery_state() can never return "critical" no matter
        # how flat the pack (see battery_state below: warn is only consulted
        # when it is not None). The mirror direction -- NO_BATTERY floor with
        # a real warn level -- builds fine too, but guard_battery() no-ops on
        # a None floor, so there is no motion gate at all despite the profile
        # looking battery-aware. Both fields must be NO_BATTERY together, or
        # both must be real values together.
        if (self.battery_motion_min_mv is None) != (self.battery_warn_mv is None):
            raise SafetyError(
                f"profile {self.name!r}: battery_motion_min_mv="
                f"{self.battery_motion_min_mv!r} and battery_warn_mv="
                f"{self.battery_warn_mv!r} are inconsistent — both must be "
                f"NO_BATTERY, or both must be stated — refusing to build"
            )

    def velocity_limits(self, motion_unlocked: bool) -> VelocityLimits:
        return self.unlocked_velocity if motion_unlocked else self.locked_velocity

    def max_duration_s(self, motion_unlocked: bool) -> float:
        return (self.unlocked_max_duration_s if motion_unlocked
                else self.locked_max_duration_s)


# The ROSPug is profile #1, built from the constants it used to own outright.
# Provenance for every value below is recorded in config.py.
ROSPUG_SAFETY = SafetyProfile(
    name="rospug",
    locked_velocity=VelocityLimits(
        max_abs_x=config.UNVERIFIED_MAX_ABS_X,
        max_abs_y=config.UNVERIFIED_MAX_ABS_Y,
        max_abs_yaw=config.UNVERIFIED_MAX_ABS_YAW,
        allow_lateral_y=config.ALLOW_LATERAL_Y,
    ),
    unlocked_velocity=VelocityLimits(
        max_abs_x=config.VERIFIED_MAX_ABS_X_M_S,
        max_abs_y=config.VERIFIED_MAX_ABS_Y_M_S,
        max_abs_yaw=config.VERIFIED_MAX_ABS_YAW_RAD_S,
        allow_lateral_y=config.ALLOW_LATERAL_Y,
    ),
    # VERIFIED: pug_node cmd_vel_limit_callback. The only clamp the vendor has.
    vendor_ceiling=VelocityLimits(
        max_abs_x=config.VENDOR_MAX_ABS_X_M_S,
        max_abs_y=config.VENDOR_MAX_ABS_Y_M_S,
        max_abs_yaw=config.VENDOR_MAX_ABS_YAW_RAD_S,
        allow_lateral_y=config.ALLOW_LATERAL_Y,
    ),
    pose=default_pose_limits(),
    locked_max_duration_s=config.UNVERIFIED_MAX_DURATION_S,
    unlocked_max_duration_s=config.VERIFIED_MAX_DURATION_S,
    max_steps_per_call=config.MAX_STEPS_PER_CALL,
    action_whitelist=tuple(config.ROSPUG.action_groups),
    battery_motion_min_mv=config.BATTERY_MOTION_MIN_MV,
    battery_warn_mv=config.BATTERY_WARN_MV,
    rate_limit_hz=config.RATE_LIMIT_HZ,
    watchdog_timeout_s=config.WATCHDOG_TIMEOUT_S,
)


# ---------------------------------------------------------------------------
# TonyPi — profile #2, the first HUMANOID. No cmd_vel interface exists: its
# safety is enforced by the per-servo TIER clamps (robots/tonypi_servo.py) and
# the action whitelist below, NOT by guard_velocity/guard_pose. SafetyProfile
# still requires the ROS-shaped velocity/pose fields, so they are honest zero
# placeholders here — the TonyPi adapter never calls guard_velocity or
# guard_pose, and a zero ceiling makes that explicit (any velocity would clamp
# to nothing). Battery: the JSON-RPC API (reference/TonyPi/RPCServer.py) exposes
# NO voltage method — GetBusServosPulse returns positions only — so a floor
# would be invented, not measured. NO_BATTERY is the honest declaration.
# ---------------------------------------------------------------------------

# Curated SAFE action groups: standing, in-place gestures that keep the biped on
# both feet. Deliberately EXCLUDES every walk / turn / kick / squat / get-up
# trajectory until the robot is characterised on hardware. These are .d6a base
# names passed straight to RunAction (reference/TonyPi/ActionGroups/*.d6a).
TONYPI_SAFE_ACTION_GROUPS = ("stand", "wave", "bow", "chest", "twist")

_TONYPI_NO_VEL = VelocityLimits(
    max_abs_x=0.0, max_abs_yaw=0.0, allow_lateral_y=False, max_abs_y=0.0)

TONYPI_SAFETY = SafetyProfile(
    name="tonypi",
    locked_velocity=_TONYPI_NO_VEL,
    unlocked_velocity=_TONYPI_NO_VEL,
    vendor_ceiling=_TONYPI_NO_VEL,
    pose=default_pose_limits(),      # placeholder; TonyPi never calls guard_pose
    locked_max_duration_s=1.0,
    unlocked_max_duration_s=1.0,
    max_steps_per_call=1,
    action_whitelist=TONYPI_SAFE_ACTION_GROUPS,
    # No pack telemetry in the API -> NO_BATTERY, both fields together (the
    # __post_init__ consistency rule). Honest absence, not a forgotten field.
    battery_motion_min_mv=NO_BATTERY,
    battery_warn_mv=NO_BATTERY,
)


# ---------------------------------------------------------------------------
# MiniHexa — profile #3, the first HEXAPOD. Like the TonyPi it has no cmd_vel
# interface: its safety is enforced by the per-tier motion clamps
# (robots/minihexa_motion.py) plus the wire codec, NOT by guard_velocity/
# guard_pose. So the ROS-shaped velocity/pose fields are honest zero placeholders
# (the adapter never calls guard_velocity or guard_pose; a zero ceiling makes
# that explicit). What DIFFERS from the TonyPi: the firmware has a real battery
# telemetry path (USB 'A|<mV>&', BLE '$<mV>$..$'), so this profile gates motion on
# a measured floor rather than declaring NO_BATTERY.
# ---------------------------------------------------------------------------

# EMPTY on purpose. The built-in K|1|N action groups are UNCHARACTERIZED — an
# unknown one could walk the robot off an edge — so the whitelist starts empty,
# which is the framework's safe default (check_action_allowed refuses everything
# against an empty whitelist). Each group earns its place here only after it is
# characterized on hardware, one at a time. Until then do_action always refuses,
# which is correct and intended.
MINIHEXA_SAFE_ACTION_GROUPS: tuple[str, ...] = ()

_MINIHEXA_NO_VEL = VelocityLimits(
    max_abs_x=0.0, max_abs_yaw=0.0, allow_lateral_y=False, max_abs_y=0.0)

MINIHEXA_SAFETY = SafetyProfile(
    name="minihexa",
    locked_velocity=_MINIHEXA_NO_VEL,
    unlocked_velocity=_MINIHEXA_NO_VEL,
    vendor_ceiling=_MINIHEXA_NO_VEL,
    pose=default_pose_limits(),      # placeholder; hexapod pose clamped in the
                                     # byte domain (minihexa_motion), never guard_pose
    # Match the LOCKED / FULL walk-burst caps in minihexa_motion (0.4 s / 1.5 s):
    # these are the same envelope stated in the duration domain.
    locked_max_duration_s=0.4,
    unlocked_max_duration_s=1.5,
    max_steps_per_call=1,
    action_whitelist=MINIHEXA_SAFE_ACTION_GROUPS,
    # 2S LiPo. Provenance: the firmware's battery sample window is centred on a
    # 7400 mV nominal pack; a live healthy read was ~8127 mV. 3.5 V/cell = 7000 mV
    # is the motion floor (below it the servos brown out mid-burst and the pack
    # over-discharges); 3.3 V/cell = 6600 mV is the critical/warn level.
    battery_motion_min_mv=7000,
    battery_warn_mv=6600,
)


# ---------------------------------------------------------------------------
# Battery gate
# ---------------------------------------------------------------------------
def check_battery_ok_for_motion(
    millivolts: int, min_mv: int = config.BATTERY_MOTION_MIN_MV
) -> None:
    """Raise SafetyError if the pack is too low to drive motion safely.

    Below this, the 12 servos brown out mid-motion and the LiPo over-discharges.
    An unreadable (None) or non-finite (NaN) reading is treated as unsafe and
    REFUSED — never assume the pack is fine when we cannot see it. (NaN is
    specifically dangerous: `nan < min` is False, so a naive check would permit
    motion on garbage telemetry.)
    """
    if millivolts is None:
        raise SafetyError("battery reading unreadable (no telemetry) — refusing to move")
    if millivolts != millivolts:  # NaN
        raise SafetyError("battery reading is NaN — refusing to move")
    if millivolts < min_mv:
        raise SafetyError(
            f"battery {millivolts} mV is below the motion floor {min_mv} mV — refusing to move"
        )


def battery_state(millivolts: int, profile: SafetyProfile) -> str:
    """Classify pack voltage against THIS robot's thresholds.

    Returns 'ok' | 'warn' | 'critical', or 'no_battery' for a mains-powered
    robot. The profile is required rather than defaulted: these thresholds are
    a property of the pack's chemistry and cell count, and reading them from
    the ROSPug's module-level constants would report a 2S robot resting at a
    perfectly healthy 7.4 V as 'critical' -- alarming the model into refusing
    work on a robot that is fine.
    """
    if profile.battery_motion_min_mv is None:
        return "no_battery"
    if profile.battery_warn_mv is not None and millivolts < profile.battery_warn_mv:
        return "critical"
    if millivolts < profile.battery_motion_min_mv:
        return "warn"
    return "ok"


# ---------------------------------------------------------------------------
# Rate limiter (stateful)
# ---------------------------------------------------------------------------
class RateLimiter:
    """Allow at most `max_hz` commands per second. Clock is injectable for tests."""

    def __init__(self, max_hz: float = config.RATE_LIMIT_HZ, clock=_time.monotonic):
        self._min_interval = 1.0 / max_hz
        self._clock = clock
        self._last: float | None = None

    def allow(self) -> bool:
        now = self._clock()
        if self._last is None or (now - self._last) >= self._min_interval:
            self._last = now
            return True
        return False


# ---------------------------------------------------------------------------
# Watchdog deadman
# ---------------------------------------------------------------------------
class Watchdog:
    """Deadman timer. The orchestrator must kick() faster than timeout_s, or the
    adapter should treat the robot as unsupervised and stop it."""

    def __init__(self, timeout_s: float = config.WATCHDOG_TIMEOUT_S, clock=_time.monotonic):
        self.timeout_s = timeout_s
        self._clock = clock
        self._last_kick = clock()

    def kick(self) -> None:
        self._last_kick = self._clock()

    def expired(self) -> bool:
        return (self._clock() - self._last_kick) > self.timeout_s


# ---------------------------------------------------------------------------
# SafetyGuard — per-robot command validator tying the pieces together
# ---------------------------------------------------------------------------
class SafetyGuard:
    """Validates/clamps every command for one robot and holds the latching e-stop.

    Composition note: this guard owns the per-command policy — e-stop latch,
    clamping, and the action whitelist. The battery gate (check_battery_ok_for_
    motion) is called by the adapter's motion preflight.

    RateLimiter and Watchdog are separate, independently-tested pieces available
    for a high-frequency command loop. The ROSPug adapter does NOT tick them:
    its motion commands are self-spacing (each tool call sleeps for the move's
    duration, and the agent loop pauses for model inference between calls), and
    the *robot-side* deadman node is the real watchdog — a PC-side one would be
    on the wrong side of a link break. They are wired only if a future
    high-rate control path needs them.
    """

    def __init__(
        self,
        profile: SafetyProfile,
        motion_unlocked: bool = False,
        action_whitelist=None,
    ):
        # motion_unlocked defaults to False rather than to the module-level
        # config.MOTION_UNLOCKED: the unlock decision is per-robot and belongs
        # to the RobotProfile (see cli.motion_unlocked_for). A guard built with
        # no opinion gets the timid limits, which is the safe direction.
        # profile is required -- this guard has no knowledge of which robot it
        # is guarding beyond what the caller supplies. action_whitelist=None
        # means "use the profile's own list"; an explicit tuple (including
        # ()) overrides it.
        self._profile = profile
        self.motion_unlocked = motion_unlocked
        self.action_whitelist = (tuple(profile.action_whitelist)
                                 if action_whitelist is None
                                 else tuple(action_whitelist))
        self._vel_limits = profile.velocity_limits(motion_unlocked)
        self._pose_limits = profile.pose
        self._estopped = False

    @property
    def profile(self) -> SafetyProfile:
        return self._profile

    def guard_battery(self, millivolts) -> None:
        """Battery gate, using this robot's floor. No-op for mains-powered robots."""
        if self._profile.battery_motion_min_mv is None:
            return
        check_battery_ok_for_motion(millivolts, self._profile.battery_motion_min_mv)

    @property
    def is_estopped(self) -> bool:
        return self._estopped

    def estop(self) -> None:
        """Latch the emergency stop. All motion commands raise until reset()."""
        self._estopped = True

    def reset(self) -> None:
        """Clear the e-stop latch."""
        self._estopped = False

    def _check_estop(self) -> None:
        if self._estopped:
            raise SafetyError("e-stop engaged — command refused until reset()")

    def guard_velocity(self, vel: Velocity) -> Velocity:
        self._check_estop()
        return clamp_velocity(vel, self._vel_limits)

    def guard_pose(self, pose: Pose) -> Pose:
        self._check_estop()
        return clamp_pose(pose, self._pose_limits)

    def guard_action(self, name: str) -> str:
        self._check_estop()
        return check_action_allowed(name, self.action_whitelist)


class G1SafetyGuard(SafetyGuard):
    """Safety guard for dynamic humanoids like the Unitree G1.
    
    Includes dynamic stability checks based on IMU data (pitch/roll).
    """

    def check_dynamic_stability(self, pitch: float, roll: float) -> None:
        """Check if the robot is falling."""
        # Simple thresholds for demonstration
        if abs(pitch) > 0.5 or abs(roll) > 0.5:
            self.estop()
            raise SafetyError(f"G1 is falling! pitch={pitch}, roll={roll}. E-stop engaged.")
