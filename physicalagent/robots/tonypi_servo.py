"""TonyPi tiered servo clamp — pure policy, no I/O, no hardware.

This is the TonyPi analogue of safety.clamp_velocity: the one place that decides
how far and how fast a single servo may move, given an explicitly-selected TIER
and which physical servo class (DOMAIN) it is. Mirrors minihexa_codec's
discipline — the hardware constants below are facts read off the vendor stack,
stated here with provenance rather than imported, so this module stands alone.

TWO DOMAINS, TWO ELECTRICAL RANGES — they must never be confused:

    BUS  body serial bus servos, SetBusServoPulse, ids 1-16
         pulse 0-1000, centre 500   (VERIFIED config.py + bus_servo/get_state
                                      readback; the RPCServer "0-500" docstring
                                      is a vendor typo, the get_state proves 0-1000)
    PWM  head pan/tilt servos, SetPWMServo, ids 1-2
         pulse 500-2500, centre 1500

The tier ladder (LOCKED < NORMAL < FULL) is the safety design, PER DOMAIN:

    BUS   LOCKED 350-650    NORMAL 200-800    FULL 0-1000     (0-1000 electrical)
    PWM   LOCKED 1200-1800  NORMAL 700-2300   FULL 500-2500   (500-2500 electrical)
    time floors (both):  LOCKED 800 ms   NORMAL 400 ms   FULL 120 ms

Each rung is strictly more permissive than the one below within its domain: a
wider pulse band AND a lower move-time floor. FULL is the deliberate override of
HiWonder's conservative speed/range limits, up to the servo's true electrical
maximum — reachable ONLY by selecting the tier explicitly; the default is LOCKED.

Commanding a bus servo a PWM-scale value (e.g. 1500) drives it hard past its
0-1000 range, which is why clamp_servo REQUIRES the domain — there is no default,
so a caller cannot accidentally clamp against the wrong band.

WHAT THIS DOES NOT DO — and must never do:
    It does not touch the servos' on-board thermal / overload / voltage
    protection. Clamping range and speed is not the same as disabling
    protection, and the two must never be conflated. See PROTECTION_OVERRIDE_*.
"""
from __future__ import annotations

from dataclasses import dataclass
from enum import Enum, IntEnum

# --- servo electrical facts (per domain) --------------------------------------
# BUS: VERIFIED config.py:215 (body bus servos ids 1-16, pulse 0-1000) and the
# bus_servo/get_state readback. PWM: head servos, standard RC pulse 500-2500.
BUS_PULSE_MIN, BUS_PULSE_MAX, BUS_PULSE_CENTER = 0, 1000, 500
PWM_PULSE_MIN, PWM_PULSE_MAX, PWM_PULSE_CENTER = 500, 2500, 1500
# Move time is a firmware two-byte field; cap it so an absurd value cannot wrap.
TIME_MAX_MS = 0xFFFF


class ServoDomain(str, Enum):
    """Which physical servo class a command targets — and therefore which
    electrical range and tier band it is clamped against."""

    BUS = "bus"   # body serial bus servos (SetBusServoPulse), 0-1000, centre 500
    PWM = "pwm"   # head pan/tilt servos (SetPWMServo), 500-2500, centre 1500


class ServoTier(IntEnum):
    """How much of a servo's range and speed the operator has unlocked.

    IntEnum so the ladder orders naturally: LOCKED < NORMAL < FULL. Selected
    explicitly (profile field / $PHYSICALAGENT_TONYPI_TIER / adapter arg /
    --tier); the default everywhere is LOCKED, the timid setting used until a
    human has decided this robot may move further and faster under our control.
    """

    LOCKED = 0
    NORMAL = 1
    FULL = 2

    @classmethod
    def resolve(cls, name: str | None) -> "ServoTier":
        """Map a case-insensitive name (or None/blank) to a tier. None or blank
        is LOCKED — absence is the safe direction. An unknown name raises rather
        than silently falling back, so a typo cannot quietly relax the clamp."""
        if name is None or not name.strip():
            return cls.LOCKED
        try:
            return cls[name.strip().upper()]
        except KeyError:
            raise ValueError(
                f"unknown servo tier {name!r}; choose from "
                f"{', '.join(t.name.lower() for t in cls)}"
            ) from None


@dataclass(frozen=True)
class ServoLimits:
    """One tier's clamp: the allowed pulse band and the minimum move time."""

    pulse_min: int
    pulse_max: int
    min_move_time_ms: int


# The ladders, one per domain. Each band is strictly nested inside the next; each
# floor strictly lower. Changing a number here changes what every TonyPi servo
# tool of that domain can command.
_TIER_LIMITS: dict[ServoDomain, dict[ServoTier, ServoLimits]] = {
    ServoDomain.BUS: {
        ServoTier.LOCKED: ServoLimits(pulse_min=350, pulse_max=650, min_move_time_ms=800),
        ServoTier.NORMAL: ServoLimits(pulse_min=200, pulse_max=800, min_move_time_ms=400),
        ServoTier.FULL: ServoLimits(pulse_min=BUS_PULSE_MIN, pulse_max=BUS_PULSE_MAX,
                                    min_move_time_ms=120),
    },
    ServoDomain.PWM: {
        ServoTier.LOCKED: ServoLimits(pulse_min=1200, pulse_max=1800, min_move_time_ms=800),
        ServoTier.NORMAL: ServoLimits(pulse_min=700, pulse_max=2300, min_move_time_ms=400),
        ServoTier.FULL: ServoLimits(pulse_min=PWM_PULSE_MIN, pulse_max=PWM_PULSE_MAX,
                                    min_move_time_ms=120),
    },
}


def clamp_servo(pulse, move_time_ms, tier: ServoTier,
                domain: ServoDomain) -> tuple[int, int]:
    """Clamp one servo command to the (domain, tier) band, and raise its move time
    to the tier's floor.

    `domain` is REQUIRED (no default): BUS servos are 0-1000 and PWM servos are
    500-2500, so clamping against the wrong band would drive a servo past its
    range — the caller must always say which it is. Returns (pulse, move_time_ms)
    as ints. `move_time_ms` may be None, meaning 'use the tier's safe default' —
    the adapter passes None when the model does not specify a time. Faster-than-
    floor moves are slowed to the floor; the time is also capped at the firmware's
    two-byte ceiling so an absurd value cannot wrap. This returns exactly two
    values and never touches servo protection — see PROTECTION_OVERRIDE_* below.
    """
    limits = _TIER_LIMITS[domain][tier]
    p = max(limits.pulse_min, min(limits.pulse_max, int(pulse)))
    if move_time_ms is None:
        t = limits.min_move_time_ms
    else:
        t = max(limits.min_move_time_ms, int(move_time_ms))
    t = min(t, TIME_MAX_MS)
    return p, t


# --- PROTECTION OVERRIDE SEAM — deliberately unimplemented --------------------
# HiWonder's serial bus servos have on-board thermal / overload / voltage
# protection that halts the motor before it cooks itself. clamp_servo and every
# normal tool leave that protection ENABLED, unconditionally — there is no code
# path in this module, or in the adapter's normal tools, that turns it off.
#
# Disabling protection is a SEPARATE, dangerous capability that is intentionally
# NOT built in this pass. If it is ever added it MUST be its own flag, default
# OFF, wired nowhere near clamp_servo or the range/speed tiers above, guarded by
# an explicit operator action — never a side effect of selecting a higher tier.
#
# WARNING: disabling a servo's thermal/overload protection can PERMANENTLY
# destroy the servo — a stalled motor with protection off burns its winding or
# strips its gears within seconds. Never disable it to push through a stall; fix
# the mechanical cause instead. Raising the tier is about RANGE and SPEED only.
PROTECTION_OVERRIDE_SUPPORTED = False
