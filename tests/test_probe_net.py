"""Network probing. The socket is injected, so these tests touch no network."""
from physicalagent.probe.net import (
    INTERESTING_PORTS, PortResult, classify, scan_ports,
)


def fake_probe(open_ports):
    return lambda host, port, timeout: port in open_ports


class TestScanPorts:
    def test_it_reports_every_port_it_was_asked_about(self):
        r = scan_ports("h", probe=fake_probe(set()))
        assert len(r) == len(INTERESTING_PORTS)

    def test_open_ports_are_marked_open(self):
        r = scan_ports("h", ports=((9090, "rosbridge"),), probe=fake_probe({9090}))
        assert r[0] == PortResult(port=9090, open=True, meaning="rosbridge")

    def test_closed_ports_are_marked_closed(self):
        r = scan_ports("h", ports=((9090, "rosbridge"),), probe=fake_probe(set()))
        assert r[0].open is False

    def test_a_probe_that_raises_counts_as_closed(self):
        """An unreachable host must not crash the whole capture."""
        def boom(host, port, timeout):
            raise OSError("no route to host")
        r = scan_ports("h", ports=((9090, "rosbridge"),), probe=boom)
        assert r[0].open is False


class TestKnownPorts:
    def test_rosbridge_is_probed(self):
        assert 9090 in [p for p, _ in INTERESTING_PORTS]

    def test_tonypi_jsonrpc_is_probed(self):
        """VERIFIED: RPCServer.py runs run_simple('', 9030, application)."""
        assert 9030 in [p for p, _ in INTERESTING_PORTS]

    def test_mjpeg_video_is_probed(self):
        assert 8080 in [p for p, _ in INTERESTING_PORTS]

    def test_ssh_is_probed(self):
        assert 22 in [p for p, _ in INTERESTING_PORTS]


class TestClassify:
    def test_rosbridge_open_suggests_a_ros_robot(self):
        c = classify((PortResult(9090, True, "rosbridge"),))
        assert any("ROS" in s for s in c)

    def test_9030_open_suggests_tonypi(self):
        c = classify((PortResult(9030, True, "TonyPi JSON-RPC"),))
        assert any("TonyPi" in s for s in c)

    def test_nothing_open_says_so_plainly(self):
        c = classify((PortResult(9090, False, "rosbridge"),))
        assert any("no known" in s.lower() for s in c)
