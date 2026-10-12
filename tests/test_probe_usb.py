"""USB identification — how `./robot probe` knows what got plugged in.

VID:PID values are VERIFIED from the spec's bring-up kit section (12).
"""
from physicalagent.probe.usb import UsbDevice, identify, parse_lsusb

LSUSB = """\
Bus 002 Device 001: ID 1d6b:0003 Linux Foundation 3.0 root hub
Bus 001 Device 004: ID 0d28:0204 NXP ARM mbed
Bus 001 Device 005: ID 1a86:7523 QinHeng Electronics CH340 serial converter
Bus 001 Device 006: ID 10c4:ea60 Silicon Labs CP210x UART Bridge
Bus 001 Device 007: ID 0403:6001 Future Technology Devices International FT232
Bus 001 Device 001: ID 1d6b:0002 Linux Foundation 2.0 root hub
"""


class TestParseLsusb:
    def test_it_finds_every_device(self):
        assert len(parse_lsusb(LSUSB)) == 6

    def test_it_extracts_vid_and_pid(self):
        d = parse_lsusb(LSUSB)[1]
        assert (d.vid, d.pid) == ("0d28", "0204")

    def test_it_keeps_the_description(self):
        assert "mbed" in parse_lsusb(LSUSB)[1].description

    def test_empty_input_yields_nothing(self):
        assert parse_lsusb("") == ()

    def test_junk_lines_are_skipped_not_crashed_on(self):
        assert len(parse_lsusb("total garbage\n\nID nope\n")) == 0


class TestIdentify:
    def test_microbit_daplink(self):
        """AiHand's controller IS a micro:bit V2."""
        assert "micro:bit" in identify("0d28", "0204")

    def test_ch340(self):
        assert "CH340" in identify("1a86", "7523")

    def test_cp2102(self):
        assert "CP2102" in identify("10c4", "ea60")

    def test_ftdi(self):
        assert "FTDI" in identify("0403", "6001")

    def test_unknown_is_reported_as_unknown_not_guessed(self):
        """Spec rule: UNKNOWN, never guess."""
        assert identify("dead", "beef") == "UNKNOWN"

    def test_identification_is_case_insensitive(self):
        assert "CH340" in identify("1A86", "7523")


class TestUsbDeviceLabelling:
    def test_parsed_devices_carry_their_label(self):
        labels = {d.label for d in parse_lsusb(LSUSB)}
        assert "micro:bit V2 (DAPLink)" in labels

    def test_unknown_devices_are_still_returned(self):
        """A robot we have no VID:PID for must still show up in the capture."""
        devices = parse_lsusb("Bus 001 Device 002: ID abcd:1234 Mystery Co Widget\n")
        assert devices[0].label == "UNKNOWN"
        assert devices[0].vid == "abcd"
