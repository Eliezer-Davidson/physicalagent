"""MiniHexa wire-protocol codec — pure, no I/O, no hardware.

Reimplemented from HiWonder's published firmware (reference/MiniHexa/.../
uart_server.cpp, ble_server.cpp, remote.ino). Their source has no LICENSE, so
none of it is copied — only the wire *protocol* (a fact, not expression) is
reproduced here. Frames end in '&', fields split on '|', the first field is a
one-letter opcode; the firmware parses each field with (uint8_t)atoi(...).

Opcodes we emit:
    C|vx|vy|turn&                     move   (vx,vy signed -50..50; turn 0/1/2)
    F|yaw|pitch|roll|x|y|height&      body pose (6 signed bytes)
    K|1|N&                            run built-in action group N
    K|2&                              stop action group          (USB only)
    L|n|t_lo|t_hi|[id|d_lo|d_hi]*n&   direct per-servo control   (USB only)
    M&                                read servo positions       (USB only)
Decoded (robot -> PC push, ~1 Hz):
    A|<millivolts>&                   battery
"""
from __future__ import annotations

import re

# --- hardware limits, straight from the firmware -------------------------
MOVE_BYTE_LIMIT = 50          # remote.ino fmap domain for vx/vy; beyond this it extrapolates
POSE_BYTE_LIMIT = 50          # yaw/pitch/roll/x/y byte domain
POSE_HEIGHT_MIN = 0           # F height byte -> fmap(0..30)
POSE_HEIGHT_MAX = 30
SERVO_MIN_DUTY = 500          # hiwonder_servo.h min_duty
SERVO_MAX_DUTY = 2500         # hiwonder_servo.h max_duty
SERVO_CENTER_DUTY = 1500
SERVO_ID_MIN = 1              # 6 legs x 3 joints = ids 1..18
SERVO_ID_MAX = 18
TIME_MAX_MS = 0xFFFF          # BYTE_TO_HW two-byte field

_VALID_TURN = (0, 1, 2)
_BATT = re.compile(rb"A\|(-?\d+)&")
# BLE notify telemetry: '$<mv>$<field2>$' (ble_server.cpp), a DIFFERENT framing
# from the USB 'A|<mV>&'. First field is the pack millivolts; second sits at
# 0xFFFF (unpopulated sensor slot) in live capture.
_BLE_TELEM = re.compile(rb"\$(-?\d+)\$(-?\d+)\$")
# A sane resting/charged window for a 2S..3S LiPo pack. A parsed value outside
# it is treated as unreadable so a mis-framed line fails the battery gate CLOSED
# (0xFFFF must never masquerade as a healthy 65 V pack).
BLE_BATTERY_MV_SANE_MIN = 3000
BLE_BATTERY_MV_SANE_MAX = 13000


def _clampi(value, lo: int, hi: int) -> int:
    return max(lo, min(hi, int(value)))


def _lo_hi(value: int) -> tuple[int, int]:
    """Split a uint16 into (low byte, high byte) as the firmware's BYTE_TO_HW expects."""
    return value & 0xFF, (value >> 8) & 0xFF


def encode_move(vx: int, vy: int, turn: int) -> bytes:
    """C|vx|vy|turn& — vx/vy clamped to +-50 (the fmap domain), turn in {0,1,2}."""
    if turn not in _VALID_TURN:
        raise ValueError(f"turn must be 0 (straight), 1 (left) or 2 (right), got {turn!r}")
    vx = _clampi(vx, -MOVE_BYTE_LIMIT, MOVE_BYTE_LIMIT)
    vy = _clampi(vy, -MOVE_BYTE_LIMIT, MOVE_BYTE_LIMIT)
    return f"C|{vx}|{vy}|{turn}&".encode()


def encode_stop() -> bytes:
    """C|0|0|0& — zero velocity. The firmware's move is velocity-latched, so this halts it."""
    return b"C|0|0|0&"


def encode_action(group_id: int) -> bytes:
    """K|1|N& — run built-in action group N (1..5 named, >=6 stored)."""
    n = int(group_id)
    if n < 1:
        raise ValueError(f"action group id must be >= 1, got {n}")
    return f"K|1|{n}&".encode()


def encode_action_stop() -> bytes:
    """K|2& — stop a running action group (parsed on the USB firmware only)."""
    return b"K|2&"


def encode_servo(moves, time_ms: int) -> bytes:
    """L|count|t_lo|t_hi|[id|d_lo|d_hi]*count& — direct per-servo control (USB only).

    `moves` is a sequence of (servo_id, duty_us). duty is hard-clamped to the
    servo's electrical range 500..2500us; id must be a real joint 1..18. The
    per-move `time_ms` eases the servos to position (0 = as fast as possible).
    """
    moves = list(moves)
    time_ms = _clampi(time_ms, 0, TIME_MAX_MS)
    t_lo, t_hi = _lo_hi(time_ms)
    parts = ["L", str(len(moves)), str(t_lo), str(t_hi)]
    for servo_id, duty in moves:
        sid = int(servo_id)
        if not (SERVO_ID_MIN <= sid <= SERVO_ID_MAX):
            raise ValueError(f"servo id must be {SERVO_ID_MIN}..{SERVO_ID_MAX}, got {sid}")
        duty = _clampi(duty, SERVO_MIN_DUTY, SERVO_MAX_DUTY)
        d_lo, d_hi = _lo_hi(duty)
        parts += [str(sid), str(d_lo), str(d_hi)]
    return ("|".join(parts) + "&").encode()


def encode_read_servos() -> bytes:
    """M& — request current servo positions."""
    return b"M&"


def encode_pose(yaw: int, pitch: int, roll: int, x: int, y: int, height: int) -> bytes:
    """F|yaw|pitch|roll|x|y|height& — body pose, byte domain (remote.ino data[0..5]).

    yaw/pitch/roll/x/y are signed -50..50; height is 0..30 (all pre-fmap bytes).
    """
    return (
        f"F|{_clampi(yaw, -POSE_BYTE_LIMIT, POSE_BYTE_LIMIT)}"
        f"|{_clampi(pitch, -POSE_BYTE_LIMIT, POSE_BYTE_LIMIT)}"
        f"|{_clampi(roll, -POSE_BYTE_LIMIT, POSE_BYTE_LIMIT)}"
        f"|{_clampi(x, -POSE_BYTE_LIMIT, POSE_BYTE_LIMIT)}"
        f"|{_clampi(y, -POSE_BYTE_LIMIT, POSE_BYTE_LIMIT)}"
        f"|{_clampi(height, POSE_HEIGHT_MIN, POSE_HEIGHT_MAX)}&"
    ).encode()


def parse_battery_frame(line) -> int | None:
    """Parse the one telemetry frame the USB robot pushes: A|<mV>&. Returns
    millivolts, or None for any non-battery / malformed line (never raises)."""
    if not line:
        return None
    if isinstance(line, str):
        line = line.encode("utf-8", "replace")
    m = _BATT.search(line)
    if not m:
        return None
    try:
        return int(m.group(1))
    except ValueError:
        return None


def parse_ble_battery(line) -> int | None:
    """Pack millivolts from the BLE notify telemetry frame '$<mv>$<field2>$'.

    The BLE server (ble_server.cpp) frames telemetry differently from the USB
    server's 'A|<mV>&' — so parse_battery_frame reads nothing off BLE. Live
    capture off the real robot showed '$8127$65535$' (~8.13 V 2S pack; field2 at
    0xFFFF, an unpopulated sensor slot); the FIRST field is the millivolts.

    Fail-closed: a first field outside a sane 2S..3S LiPo window (or a malformed
    / non-BLE / empty line) returns None, which the battery gate treats as
    unreadable and REFUSES motion — so a mis-framed line can never read as a
    healthy pack. Never raises."""
    if not line:
        return None
    if isinstance(line, str):
        line = line.encode("utf-8", "replace")
    m = _BLE_TELEM.search(line)
    if not m:
        return None
    try:
        mv = int(m.group(1))
    except ValueError:
        return None
    if not (BLE_BATTERY_MV_SANE_MIN <= mv <= BLE_BATTERY_MV_SANE_MAX):
        return None
    return mv
