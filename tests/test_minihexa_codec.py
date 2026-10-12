"""Tests for the MiniHexa wire-protocol codec.

The grammar was reverse-engineered from HiWonder's published firmware
(reference/MiniHexa/.../uart_server.cpp + remote.ino) and REIMPLEMENTED here —
their unlicensed C++ is never copied. Frames end in '&', fields split on '|',
first field is the opcode. These tests pin every encoder to the exact bytes the
firmware's read_data() parser expects, and pin the decoder to the one telemetry
frame the robot pushes unprompted (A|<mV>&).
"""
import math

import pytest

from physicalagent.robots import minihexa_codec as codec


class TestMove:
    def test_forward_is_C_frame(self):
        assert codec.encode_move(50, 0, 0) == b"C|50|0|0&"

    def test_negative_values_are_signed_decimal(self):
        # firmware does (uint8_t)atoi(str) then casts to int8, so "-50" round-trips
        assert codec.encode_move(-30, 0, 0) == b"C|-30|0|0&"

    def test_vx_vy_clamp_to_the_firmware_fmap_domain_50(self):
        # beyond +-50 the firmware's fmap EXTRAPOLATES; 50 is the hard wire ceiling
        assert codec.encode_move(999, -999, 0) == b"C|50|-50|0&"

    def test_turn_must_be_0_1_or_2(self):
        assert codec.encode_move(0, 0, 1) == b"C|0|0|1&"
        assert codec.encode_move(0, 0, 2) == b"C|0|0|2&"
        with pytest.raises(ValueError):
            codec.encode_move(0, 0, 3)

    def test_stop_is_all_zeros(self):
        assert codec.encode_stop() == b"C|0|0|0&"


class TestAction:
    def test_run_action_group_is_K1N(self):
        assert codec.encode_action(3) == b"K|1|3&"

    def test_action_id_must_be_positive(self):
        with pytest.raises(ValueError):
            codec.encode_action(0)

    def test_stop_action_is_K2(self):
        assert codec.encode_action_stop() == b"K|2&"


class TestServo:
    def test_single_servo_splits_duty_into_lo_hi_bytes(self):
        # duty 1500 = 0x05DC -> lo=220, hi=5 ; time 500 = 0x01F4 -> lo=244, hi=1
        # frame: L|count|time_lo|time_hi|id|duty_lo|duty_hi&
        assert codec.encode_servo([(1, 1500)], 500) == b"L|1|244|1|1|220|5&"

    def test_multi_servo(self):
        # duty 600 = 0x0258 -> lo=88, hi=2 ; time 0 -> lo=0,hi=0
        assert codec.encode_servo([(1, 1500), (2, 600)], 0) == b"L|2|0|0|1|220|5|2|88|2&"

    def test_duty_clamps_to_500_2500(self):
        # 9000 -> 2500 = 0x09C4 -> lo=196, hi=9 ; 10 -> 500 = 0x01F4 -> lo=244,hi=1
        assert codec.encode_servo([(5, 9000)], 0) == b"L|1|0|0|5|196|9&"
        assert codec.encode_servo([(5, 10)], 0) == b"L|1|0|0|5|244|1&"

    def test_servo_id_must_be_1_through_18(self):
        with pytest.raises(ValueError):
            codec.encode_servo([(0, 1500)], 0)
        with pytest.raises(ValueError):
            codec.encode_servo([(19, 1500)], 0)

    def test_read_servos_is_M(self):
        assert codec.encode_read_servos() == b"M&"


class TestPose:
    def test_pose_is_F_frame_six_signed_bytes(self):
        # order: yaw|pitch|roll|x|y|height (remote.ino data[0..5])
        assert codec.encode_pose(0, 0, 0, 0, 0, 0) == b"F|0|0|0|0|0|0&"

    def test_pose_bytes_clamp_to_50(self):
        assert codec.encode_pose(999, -999, 0, 0, 0, 40) == b"F|50|-50|0|0|0|30&"


class TestBatteryDecode:
    def test_parses_millivolts(self):
        assert codec.parse_battery_frame(b"A|7600&") == 7600

    def test_ignores_non_battery_lines(self):
        assert codec.parse_battery_frame(b"[initImpl] FW Version") is None

    def test_malformed_payload_is_none_not_crash(self):
        assert codec.parse_battery_frame(b"A|abc&") is None
        assert codec.parse_battery_frame(b"") is None


class TestBleTelemetryDecode:
    """The BLE server pushes a DIFFERENT telemetry framing from the USB
    'A|<mV>&'. Live capture off the real robot (BLE address redacted) over notify
    char ffe2 showed '$8127$65535$' — first field the 2S pack millivolts
    (~8.13 V), second field pinned at 0xFFFF (an unpopulated sensor slot). The
    USB A| parser reads NOTHING from these, so a BLE battery gate needs its own
    decoder — and that decoder must fail CLOSED on a mis-framed line."""

    def test_parses_first_field_as_millivolts(self):
        assert codec.parse_ble_battery(b"$8127$65535$") == 8127
        assert codec.parse_ble_battery(b"$8135$65535$") == 8135

    def test_accepts_str_or_bytes(self):
        assert codec.parse_ble_battery("$7400$0$") == 7400

    def test_out_of_range_reads_as_unreadable_fail_closed(self):
        # If field order were ever wrong and 0xFFFF were read as "mV", it must
        # NOT look like a healthy 65 V pack — a value outside a sane 2S/3S LiPo
        # window is None, so the battery gate refuses rather than permits.
        assert codec.parse_ble_battery(b"$65535$0$") is None
        assert codec.parse_ble_battery(b"$0$0$") is None

    def test_the_usb_A_frame_is_not_a_ble_frame(self):
        assert codec.parse_ble_battery(b"A|7600&") is None

    def test_malformed_or_empty_is_none_not_crash(self):
        assert codec.parse_ble_battery(b"$abc$1$") is None
        assert codec.parse_ble_battery(b"") is None
        assert codec.parse_ble_battery(None) is None
