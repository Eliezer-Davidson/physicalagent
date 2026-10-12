"""Tests for RosbridgeTransport — the production roslibpy shim.

roslibpy is injected so the message-shaping and subscription wiring are testable
offline against a fake. The actual network behaviour is verified on hardware.
"""
from physicalagent.robots.transport import RosbridgeTransport


class FakeTopic:
    def __init__(self, ros, name, msg_type, **kw):
        self.name = name; self.msg_type = msg_type; self.kw = kw
        self.published = []
        self._cb = None

    def publish(self, message):
        self.published.append(message)

    def subscribe(self, cb):
        self._cb = cb

    def feed(self, message):  # test helper: simulate an incoming message
        if self._cb:
            self._cb(message)


class FakeService:
    def __init__(self, ros, name, srv_type):
        self.name = name; self.srv_type = srv_type
        self.response = {"success": True}

    def call(self, request, timeout=10):
        return self.response


class FakeRos:
    def __init__(self, host, port):
        self.host = host; self.port = port; self.ran = False

    def run(self):
        self.ran = True


class FakeRoslib:
    """Stands in for the roslibpy module."""
    def __init__(self):
        self.topics = []

    def Ros(self, host, port):
        self.ros = FakeRos(host, port)
        return self.ros

    def Topic(self, ros, name, msg_type, **kw):
        t = FakeTopic(ros, name, msg_type, **kw)
        self.topics.append(t)
        return t

    def Service(self, ros, name, srv_type):
        self.service = FakeService(ros, name, srv_type)
        return self.service

    def Message(self, d):
        return {"__msg__": d}

    def ServiceRequest(self, d):
        return {"__req__": d}


class TestRosbridgePublish:
    def test_publish_wraps_message_and_sends(self):
        rl = FakeRoslib()
        tr = RosbridgeTransport("10.0.0.9", roslib=rl)
        tr.publish("/pug_control/velocity_move", "pug_control/Velocity", {"x": 0.05, "stop": False})
        pub = next(t for t in rl.topics if t.name == "/pug_control/velocity_move")
        assert pub.published == [{"__msg__": {"x": 0.05, "stop": False}}]

    def test_publisher_is_reused_across_calls(self):
        rl = FakeRoslib()
        tr = RosbridgeTransport("h", roslib=rl)
        tr.publish("/v", "T", {"n": 1})
        tr.publish("/v", "T", {"n": 2})
        pubs = [t for t in rl.topics if t.name == "/v"]
        assert len(pubs) == 1              # one publisher, two messages
        assert len(pubs[0].published) == 2


class TestRosbridgeSubscribe:
    def test_subscription_stores_latest_under_key(self):
        rl = FakeRoslib()
        tr = RosbridgeTransport("h", roslib=rl)
        tr.subscribe("/ros_robot_controller/battery", "std_msgs/UInt16", "batt")
        topic = next(t for t in rl.topics if t.name == "/ros_robot_controller/battery")
        assert tr.latest("batt") is None
        topic.feed({"data": 11800})
        assert tr.latest("batt") == 11800  # unwraps std_msgs .data

    def test_non_data_message_stored_whole(self):
        rl = FakeRoslib()
        tr = RosbridgeTransport("h", roslib=rl)
        tr.subscribe("/scan_raw", "sensor_msgs/LaserScan", "scan")
        topic = next(t for t in rl.topics if t.name == "/scan_raw")
        topic.feed({"ranges": [1, 2, 3]})
        assert tr.latest("scan") == {"ranges": [1, 2, 3]}


class TestRosbridgeService:
    def test_call_service_wraps_request_and_returns_response(self):
        rl = FakeRoslib()
        tr = RosbridgeTransport("h", roslib=rl)
        out = tr.call_service("/pug_control/go_home", "std_srvs/Empty", {})
        assert out == {"success": True}
        assert rl.service.name == "/pug_control/go_home"
