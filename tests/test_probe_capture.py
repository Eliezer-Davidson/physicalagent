"""Capture folder writing. Uses tmp_path -- nothing lands in the repo."""
import json

from physicalagent.probe.capture import CaptureWriter, capture_dir


class TestCaptureDir:
    def test_the_name_carries_robot_and_timestamp(self, tmp_path):
        d = capture_dir(tmp_path, "tonypi", "2026-07-22_101500")
        assert d.name == "tonypi_2026-07-22_101500"

    def test_it_is_created_under_the_base(self, tmp_path):
        assert capture_dir(tmp_path, "tonypi", "s").parent == tmp_path


class TestCaptureWriter:
    def test_write_text_creates_the_file(self, tmp_path):
        w = CaptureWriter(tmp_path / "cap")
        p = w.write_text("lsusb.txt", "Bus 001 ...")
        assert p.read_text() == "Bus 001 ..."

    def test_it_creates_the_directory_if_missing(self, tmp_path):
        w = CaptureWriter(tmp_path / "deep" / "cap")
        w.write_text("a.txt", "x")
        assert (tmp_path / "deep" / "cap" / "a.txt").exists()

    def test_manifest_is_valid_json(self, tmp_path):
        w = CaptureWriter(tmp_path / "cap")
        p = w.write_manifest({"robot": "tonypi", "ports": [9030]})
        assert json.loads(p.read_text())["robot"] == "tonypi"

    def test_manifest_is_named_manifest_json(self, tmp_path):
        w = CaptureWriter(tmp_path / "cap")
        assert w.write_manifest({}).name == "manifest.json"

    def test_manifest_is_human_readable(self, tmp_path):
        """Indented: these get read by a person during bring-up."""
        w = CaptureWriter(tmp_path / "cap")
        assert "\n  " in w.write_manifest({"a": 1, "b": 2}).read_text()

    def test_writing_twice_overwrites_cleanly(self, tmp_path):
        w = CaptureWriter(tmp_path / "cap")
        w.write_text("a.txt", "first")
        assert w.write_text("a.txt", "second").read_text() == "second"


class TestRunProbe:
    def test_it_writes_a_manifest_and_returns_the_folder(self, tmp_path):
        from physicalagent.probe import run_probe
        out = run_probe(
            host="10.0.0.1", robot="tonypi", base_dir=tmp_path,
            stamp="2026-07-22_101500",
            lsusb_text="Bus 001 Device 004: ID 0d28:0204 NXP ARM mbed",
            serial=("/dev/ttyUSB0",),
            port_probe=lambda h, p, t: p == 9030,
        )
        manifest = json.loads((out / "manifest.json").read_text())
        assert manifest["robot"] == "tonypi"
        assert manifest["host"] == "10.0.0.1"
        assert manifest["serial_ports"] == ["/dev/ttyUSB0"]

    def test_it_records_the_identified_usb_devices(self, tmp_path):
        from physicalagent.probe import run_probe
        out = run_probe(
            host="h", robot="aihand", base_dir=tmp_path, stamp="s",
            lsusb_text="Bus 001 Device 004: ID 0d28:0204 NXP ARM mbed",
            serial=(), port_probe=lambda h, p, t: False,
        )
        m = json.loads((out / "manifest.json").read_text())
        assert m["usb"][0]["label"] == "micro:bit V2 (DAPLink)"

    def test_it_records_the_conclusions(self, tmp_path):
        from physicalagent.probe import run_probe
        out = run_probe(
            host="h", robot="tonypi", base_dir=tmp_path, stamp="s",
            lsusb_text="", serial=(), port_probe=lambda h, p, t: p == 9030,
        )
        m = json.loads((out / "manifest.json").read_text())
        assert any("TonyPi" in n for n in m["conclusions"])

    def test_the_ros_graph_is_populated_when_a_ros_call_is_supplied(self, tmp_path):
        """REGRESSION: cmd_probe never passed ros_call, so every real capture
        recorded {"skipped": "no ROS link"} -- while the same probe printed
        "rosbridge is up, dump the graph next" and held an unused graph dumper.
        The plumbing from a live rosapi caller through to the manifest was
        never exercised end to end."""
        from physicalagent.probe import run_probe
        responses = {
            "/rosapi/topics": {"topics": ["/scan_raw", "/odom_raw"]},
            "/rosapi/nodes": {"nodes": ["/pug_control"]},
            "/rosapi/services": {"services": ["/pug_control/run_action_group"]},
        }
        out = run_probe(
            host="h", robot="rospug", base_dir=tmp_path, stamp="s",
            lsusb_text="", serial=(), port_probe=lambda h, p, t: p == 9090,
            ros_call=lambda name, type_, req: responses[name],
        )
        graph = json.loads((out / "manifest.json").read_text())["ros_graph"]
        assert "skipped" not in graph, "a live ROS link still recorded 'no ROS link'"
        assert graph["topics"] == ["/odom_raw", "/scan_raw"]     # sorted
        assert graph["nodes"] == ["/pug_control"]
        assert graph["services"] == ["/pug_control/run_action_group"]

    def test_without_a_ros_call_the_graph_records_that_it_was_skipped(self, tmp_path):
        from physicalagent.probe import run_probe
        out = run_probe(host="h", robot="x", base_dir=tmp_path, stamp="s",
                        lsusb_text="", serial=(), port_probe=lambda h, p, t: False)
        graph = json.loads((out / "manifest.json").read_text())["ros_graph"]
        assert graph == {"skipped": "no ROS link"}

    def test_a_failing_ros_call_still_produces_a_capture(self, tmp_path):
        """Bring-up rule: a partial capture beats a traceback."""
        from physicalagent.probe import run_probe

        def _boom(name, type_, req):
            raise RuntimeError("rosapi not installed")

        out = run_probe(host="h", robot="x", base_dir=tmp_path, stamp="s",
                        lsusb_text="", serial=(),
                        port_probe=lambda h, p, t: p == 9090, ros_call=_boom)
        graph = json.loads((out / "manifest.json").read_text())["ros_graph"]
        assert "rosapi not installed" in graph["error"]
        assert graph["topics"] == []

    def test_raw_lsusb_is_kept_verbatim(self, tmp_path):
        """Capture first, analyse later -- the raw text must survive."""
        from physicalagent.probe import run_probe
        out = run_probe(
            host="h", robot="x", base_dir=tmp_path, stamp="s",
            lsusb_text="RAW TEXT HERE", serial=(),
            port_probe=lambda h, p, t: False,
        )
        assert (out / "lsusb.txt").read_text() == "RAW TEXT HERE"
