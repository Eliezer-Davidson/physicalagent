"""AiHand codec — pure framing and clamping for the HiWonder AiHand. No I/O.

WHAT IS VERIFIED (fleet spec 2026-07-21, section 2, from the vendor product):

    Hiwonder AiHand, ONE product: a 6-DOF hand whose controller IS a micro:bit V2
    (USB VID:PID 0d28:0204, DAPLink — see probe/usb.py).
    Actuators: 5 x PWM finger servos (1.8 kgf-cm) + 1 base rotation = 6 DOF.
    Feedback: NONE AT ALL. Power: 7.4 V 3 A. Mass: 0.73 kg (191 g hand only).
    Transport line in the spec table reads: USB-TTL `0x55 0x55`.

WHAT IS NOT VERIFIED — and this module refuses to pretend otherwise:

    1. THE FRAMING. The spec's `0x55 0x55` is the classic LOBOT/HiWonder binary
       servo-controller header, but nobody has yet sent a byte to THIS hand's
       micro:bit firmware. A sibling robot in this fleet (MiniHexa) speaks the
       ASCII `L|n|time|id|pos...&` form over the same kind of link, and an
       earlier draft of the hand integration assumed exactly that. Both are
       plausible; neither is confirmed. So BOTH are implemented, `encode()`
       dispatches between them, and VERIFIED_FRAMING stays None until a probe
       moves a real finger. Guessing here is how you get an ack with no motion.

    2. THE PULSE RANGE. PWM hobby servos are 500-2500 us (the TonyPi head servos
       in tonypi_servo.py are exactly that), so that is the electrical range
       assumed here. The earlier draft used 500-1500 with the comment "roughly".
       "Roughly" is not a range: 500-2500 is the standard and is what the tier
       ladder is cut from, but the hand's MECHANICAL travel inside that range is
       unknown, which is why LOCKED is narrow and is the default.

Because the hand has NO feedback of any kind, the clamp in this module is not
one protection among several — it is the ONLY protection. A finger driven past
its mechanical stop will stall and cook silently; nothing will report it. That is
the whole argument for the tier ladder below and for LOCKED being the default.
"""
from __future__ import annotations

from physicalagent.robots.tonypi_servo import ServoTier

# --- servo electrical facts ---------------------------------------------------
# Standard RC PWM microseconds, same domain as the TonyPi's head servos.
PULSE_MIN, PULSE_MAX, PULSE_CENTER = 500, 2500, 1500
# Move time is a firmware two-byte field in the LOBOT framing; cap it so an
# absurd value cannot wrap around to a near-instant move.
TIME_MAX_MS = 0xFFFF

# --- LOBOT binary framing constants -------------------------------------------
LOBOT_HEADER = b"\x55\x55"
CMD_SERVO_MOVE = 3          # LOBOT servo-controller "move N servos" opcode

# --- the hand's six degrees of freedom ----------------------------------------
# Servo ids 1-5 are the fingers thumb..pinky; 6 is the base rotation. The
# ORDER is the vendor's; which physical finger each id drives is confirmed
# one id at a time by the probe, not assumed from this table.
FINGER_IDS: dict[str, int] = {
    "thumb": 1,
    "index": 2,
    "middle": 3,
    "ring": 4,
    "pinky": 5,
    "base": 6,
}


def finger_id(name: str) -> int:
    """Servo id for a finger name, case-insensitively. Unknown names raise
    rather than resolving to a default — commanding the wrong finger on a hand
    with no feedback is invisible."""
    try:
        return FINGER_IDS[name.strip().lower()]
    except KeyError:
        raise ValueError(
            f"unknown finger {name!r}; choose from "
            f"{', '.join(sorted(FINGER_IDS))}"
        ) from None


# --- the tier ladder ----------------------------------------------------------
# Same LOCKED < NORMAL < FULL ladder as every other body in this fleet, cut for
# a feedback-free PWM hand. Each band is strictly nested inside the next and
# each move-time floor strictly lower, so raising a tier can only ever widen
# range and allow faster motion — never anything else.
#
# LOCKED is deliberately a small excursion either side of centre: enough to see
# a finger move and confirm the wiring, not enough to drive one into its stop
# before anyone has watched this hand move once.
TIER_BANDS: dict[ServoTier, tuple[int, int]] = {
    ServoTier.LOCKED: (1200, 1800),
    ServoTier.NORMAL: (700, 2300),
    ServoTier.FULL: (PULSE_MIN, PULSE_MAX),
}

TIER_MIN_MOVE_MS: dict[ServoTier, int] = {
    ServoTier.LOCKED: 800,
    ServoTier.NORMAL: 400,
    ServoTier.FULL: 120,
}


def tier_band(tier: ServoTier) -> tuple[int, int]:
    """The (min, max) pulse this tier permits."""
    return TIER_BANDS[tier]


def clamp_finger(pulse, tier: ServoTier,
                 move_time_ms=None) -> tuple[int, int]:
    """Clamp one finger command to its tier band and raise its move time to the
    tier floor. Returns (pulse, move_time_ms) as ints.

    `move_time_ms=None` means "use this tier's safe default" — the adapter
    passes None when the model does not specify a time. The time is capped at
    the firmware's two-byte ceiling so an absurd value cannot wrap.
    """
    lo, hi = tier_band(tier)
    p = max(lo, min(hi, int(pulse)))
    floor = TIER_MIN_MOVE_MS[tier]
    t = floor if move_time_ms is None else max(floor, int(move_time_ms))
    return p, min(t, TIME_MAX_MS)


# --- framings -----------------------------------------------------------------
def _ordered(servos: dict[int, int]) -> list[tuple[int, int]]:
    """Servos as (id, pulse) sorted by id.

    Sorting matters: the same logical command must produce the same bytes
    regardless of dict insertion order, or a probe result cannot be reproduced.
    """
    if not servos:
        raise ValueError("refusing to encode a frame with no servos in it")
    return sorted(servos.items())


def encode_ascii(servos: dict[int, int], move_time_ms: int) -> bytes:
    """The MiniHexa-style text frame: `L|n|time|id|pos|id|pos...&`."""
    items = _ordered(servos)
    body = "".join(f"|{sid}|{pos}" for sid, pos in items)
    return f"L|{len(items)}|{int(move_time_ms)}{body}&".encode("ascii")


def _u16le(value: int, what: str) -> bytes:
    if not 0 <= int(value) <= 0xFFFF:
        raise ValueError(f"{what} {value} is out of range for a 16-bit field")
    return int(value).to_bytes(2, "little")


def encode_lobot(servos: dict[int, int], move_time_ms: int) -> bytes:
    """The LOBOT binary frame the fleet spec records for this hand.

        55 55 LEN CMD COUNT TIME_L TIME_H  (ID POS_L POS_H) * COUNT

    LEN counts every byte after the two header bytes. Values too wide for their
    field raise rather than truncating — a truncated position is a finger sent
    somewhere nobody asked for, and this hand cannot report where it went.
    """
    items = _ordered(servos)
    payload = bytearray()
    payload.append(CMD_SERVO_MOVE)
    payload.append(len(items))
    payload += _u16le(move_time_ms, "move time")
    for sid, pos in items:
        if not 0 <= int(sid) <= 0xFF:
            raise ValueError(f"servo id {sid} is out of range for a byte field")
        payload.append(int(sid))
        payload += _u16le(pos, "position")
    # LEN counts the payload plus its own byte.
    return LOBOT_HEADER + bytes([len(payload) + 1]) + bytes(payload)


FRAMINGS = {
    "lobot": encode_lobot,
    "ascii": encode_ascii,
}

# Flipped to a framing name ONLY when a probe has moved a real finger on real
# hardware. Until then every send is explicitly a candidate, and the operator
# is told so. Promoting this is a human act with a name and a date.
VERIFIED_FRAMING: str | None = None

# The framing sent when nobody chooses one. It is the fleet spec's recorded
# transport, so it is the best-evidenced guess — but it is still a guess, which
# is exactly why VERIFIED_FRAMING above is a separate, still-None fact.
DEFAULT_FRAMING = "lobot"


def encode(servos: dict[int, int], move_time_ms: int,
           framing: str = DEFAULT_FRAMING) -> bytes:
    """Encode a multi-servo move in the named candidate framing."""
    try:
        encoder = FRAMINGS[framing]
    except KeyError:
        raise ValueError(
            f"unknown framing {framing!r}; choose from "
            f"{', '.join(sorted(FRAMINGS))}"
        ) from None
    return encoder(servos, move_time_ms)
