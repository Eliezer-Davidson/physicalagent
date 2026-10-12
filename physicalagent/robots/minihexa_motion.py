"""MiniHexa tiered motion clamp — pure policy, no I/O, no hardware.

This is the MiniHexa analogue of tonypi_servo.clamp_servo: the one place that
decides how far and how fast the hexapod may move, given an explicitly-selected
TIER. It is the safety-critical heart of the adapter, and it stands alone — the
hardware constants below are facts read off HiWonder's firmware, stated here with
provenance rather than imported, exactly as minihexa_codec does.

WHY THIS MATTERS MORE HERE THAN ON A BIPED
    The MiniHexa firmware has NO WATCHDOG and its walk command is VELOCITY-
    LATCHED: a C|vx|vy|turn& frame sets a velocity the robot holds forever, until
    a C|0|0|0& stop arrives. It never self-stops on silence or link loss. So the
    tier does two jobs the TonyPi tier does not: it caps the velocity BYTE (how
    fast the latch runs) AND it sets the BURST DURATION (how long the host lets
    the latch run before it is contractually required to send the stop). A wider
    tier is a faster, longer, further burst.

THERE IS NO TORQUE CHANNEL IN THE WIRE PROTOCOL.
    The C/F/L opcodes carry velocity bytes, pose bytes, and per-servo duty +
    move-time — and nothing else. "Aggressiveness" is therefore expressed ONLY
    through SPEED (the velocity byte, the servo move-time) and RANGE (the
    velocity cap, the duty band, the pose cap). "Override torque" in this project
    means widen the range and shorten the move time — nothing more. There is no
    torque knob to turn, and this module deliberately exposes none: raising the
    tier can never disable a servo's on-board protection because no such control
    crosses the wire.

The tier ladder (LOCKED < NORMAL < FULL):

    velocity byte cap (|vx|,|vy|):  LOCKED 15    NORMAL 30    FULL 50   (50 = codec MOVE_BYTE_LIMIT)
    walk burst duration (s):        LOCKED 0.4   NORMAL 0.8   FULL 1.5
    per-servo duty band (us):       LOCKED 1200-1800  NORMAL 900-2100  FULL 500-2500  (500/2500 electrical)
    per-servo move-time floor (ms): LOCKED 800   NORMAL 400   FULL 120
    pose byte cap (yaw..y):         LOCKED 15    NORMAL 30    FULL 50   (height always codec 0..30)

Each band is strictly nested inside the next; each floor strictly lower. FULL is
the deliberate override of HiWonder's conservative defaults up to the firmware /
electrical maximum — reachable ONLY by selecting the tier explicitly; the default
everywhere is LOCKED.
"""
from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum

# --- hardware facts, echoed from minihexa_codec (stated, not imported) --------
# velocity/pose bytes: codec MOVE_BYTE_LIMIT / POSE_BYTE_LIMIT = 50; height 0..30.
VELOCITY_BYTE_MAX = 50
POSE_BYTE_MAX = 50
POSE_HEIGHT_MIN, POSE_HEIGHT_MAX = 0, 30
# per-servo duty: hiwonder_servo.h electrical range 500..2500 us.
SERVO_DUTY_MIN, SERVO_DUTY_MAX = 500, 2500
# firmware's move-time is a two-byte field (BYTE_TO_HW); cap so it cannot wrap.
TIME_MAX_MS = 0xFFFF


class MotionTier(IntEnum):
    """How much of the hexapod's speed and range the operator has unlocked.

    IntEnum so the ladder orders naturally: LOCKED < NORMAL < FULL. Selected
    explicitly (profile field / $PHYSICALAGENT_MINIHEXA_TIER / adapter arg /
    --tier); the default everywhere is LOCKED, the timid setting used until a
    human has decided this robot may burst further, faster and longer under our
    control. FULL is the deliberate override of HiWonder's conservative range /
    speed defaults up to the firmware / electrical maximum.
    """

    LOCKED = 0
    NORMAL = 1
    FULL = 2

    @classmethod
    def resolve(cls, name: str | None) -> "MotionTier":
        """Map a case-insensitive name (or None/blank) to a tier. None or blank
        is LOCKED — absence is the safe direction. An unknown name raises rather
        than silently falling back, so a typo cannot quietly relax the clamp."""
        if name is None or not name.strip():
            return cls.LOCKED
        try:
            return cls[name.strip().upper()]
        except KeyError:
            raise ValueError(
                f"unknown motion tier {name!r}; choose from "
                f"{', '.join(t.name.lower() for t in cls)}"
            ) from None


@dataclass(frozen=True)
class MotionLimits:
    """One tier's envelope: the walk/pose byte caps, the burst duration, and the
    per-servo duty band + move-time floor."""

    velocity_cap: int       # |vx|,|vy| byte cap
    burst_s: float          # how long a latched walk runs before the stop
    duty_min: int           # per-servo duty band low edge (us)
    duty_max: int           # per-servo duty band high edge (us)
    min_move_time_ms: int   # per-servo move-time floor
    pose_cap: int           # yaw/pitch/roll/x/y byte cap


# The ladder. Each band is strictly nested inside the next; each floor strictly
# lower. Changing a number here changes what every MiniHexa motion tool of that
# tier can command.
_TIER_LIMITS: dict[MotionTier, MotionLimits] = {
    MotionTier.LOCKED: MotionLimits(
        velocity_cap=15, burst_s=0.4, duty_min=1200, duty_max=1800,
        min_move_time_ms=800, pose_cap=15),
    MotionTier.NORMAL: MotionLimits(
        velocity_cap=30, burst_s=0.8, duty_min=900, duty_max=2100,
        min_move_time_ms=400, pose_cap=30),
    MotionTier.FULL: MotionLimits(
        velocity_cap=VELOCITY_BYTE_MAX, burst_s=1.5,
        duty_min=SERVO_DUTY_MIN, duty_max=SERVO_DUTY_MAX,
        min_move_time_ms=120, pose_cap=POSE_BYTE_MAX),
}


def _clampi(value, lo: int, hi: int) -> int:
    return max(lo, min(hi, int(value)))


def clamp_walk(vx, vy, turn, tier: MotionTier) -> tuple[int, int, int]:
    """Clamp a walk command to the tier's velocity-byte cap.

    vx and vy are clamped to +/- the tier's velocity cap (LOCKED 15, NORMAL 30,
    FULL 50 = the codec MOVE_BYTE_LIMIT). `turn` passes through untouched — it is
    a discrete {0,1,2} the codec validates, not a magnitude the tier bounds.
    Returns (vx, vy, turn) as ints. The codec applies its own -50..50 clamp too,
    so this is a tightening, never a widening, of what finally reaches the wire.
    """
    cap = _TIER_LIMITS[tier].velocity_cap
    return _clampi(vx, -cap, cap), _clampi(vy, -cap, cap), turn


def burst_duration_s(tier: MotionTier) -> float:
    """How long a latched walk is allowed to run before the mandatory stop.

    The firmware never self-stops, so the burst duration IS the safety envelope
    in the time domain: the adapter sends the move, sleeps for exactly this long,
    then sends the stop. LOCKED 0.4 s, NORMAL 0.8 s, FULL 1.5 s.
    """
    return _TIER_LIMITS[tier].burst_s


def clamp_servo_duty(duty, move_time_ms, tier: MotionTier) -> tuple[int, int]:
    """Clamp one per-servo command to the tier's duty band and move-time floor.

    `duty` is clamped to the tier band (LOCKED 1200-1800, NORMAL 900-2100, FULL
    the full electrical 500-2500) — NOT to the full electrical range unless the
    tier is FULL. `move_time_ms` may be None, meaning 'use the tier's safe
    default' (the floor); otherwise a faster-than-floor move is slowed to the
    floor, and the time is capped at the firmware's two-byte ceiling so an absurd
    value cannot wrap. Returns (duty, move_time_ms) as ints. Per-joint control is
    a USB-only capability (the BLE firmware does not parse L) — that gate lives in
    the adapter; this function is pure clamp policy.
    """
    limits = _TIER_LIMITS[tier]
    d = _clampi(duty, limits.duty_min, limits.duty_max)
    if move_time_ms is None:
        t = limits.min_move_time_ms
    else:
        t = max(limits.min_move_time_ms, int(move_time_ms))
    t = min(t, TIME_MAX_MS)
    return d, t


def clamp_pose_bytes(yaw, pitch, roll, x, y, height,
                     tier: MotionTier) -> tuple[int, int, int, int, int, int]:
    """Clamp an F-frame body pose to the tier's byte caps.

    The first five bytes (yaw, pitch, roll, x, y) are clamped to +/- the tier's
    pose cap (LOCKED 15, NORMAL 30, FULL 50 = the codec POSE_BYTE_LIMIT). `height`
    is ALWAYS clamped to the codec's 0..30 range regardless of tier — it is a
    body-height byte with a fixed firmware domain, not an aggressiveness axis, so
    the tier never widens or narrows it. Returns six ints.
    """
    cap = _TIER_LIMITS[tier].pose_cap
    return (
        _clampi(yaw, -cap, cap),
        _clampi(pitch, -cap, cap),
        _clampi(roll, -cap, cap),
        _clampi(x, -cap, cap),
        _clampi(y, -cap, cap),
        _clampi(height, POSE_HEIGHT_MIN, POSE_HEIGHT_MAX),
    )
