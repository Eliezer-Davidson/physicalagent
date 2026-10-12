"""Transport — the seam between adapter logic and roslibpy.

The RobotAdapter never imports roslibpy. It talks to a Transport:
  * publish(topic, type, message)      — fire a ROS topic message
  * call_service(name, type, request)  — call a ROS service, get the response
  * latest(key)                        — most recent value of a subscribed topic
  * subscribe(topic, type, key, ...)   — start tracking a topic under `key`

Production uses RosbridgeTransport (roslibpy). Tests use FakeTransport, which
records everything published and serves canned/ scripted telemetry — so the
adapter's clamping, bounding, and closed-loop logic run fully offline.

Clock is injectable for the same reason: the closed-loop turn controller
sleeps and polls odometry, and tests must drive that deterministically without
real time passing (mirrors the pattern in safety.RateLimiter/Watchdog).
"""
from __future__ import annotations

import time
from abc import ABC, abstractmethod
from typing import Any


# ---------------------------------------------------------------------------
# Clocks
# ---------------------------------------------------------------------------
class Clock(ABC):
    @abstractmethod
    def now(self) -> float: ...

    @abstractmethod
    def sleep(self, seconds: float) -> None: ...


class MonotonicClock(Clock):
    """Production clock. Monotonic so NTP jumps cannot corrupt elapsed maths."""

    def now(self) -> float:
        return time.monotonic()

    def sleep(self, seconds: float) -> None:
        time.sleep(seconds)


class ScriptedClock(Clock):
    """Test clock. now() advances only when sleep() is called — no real waiting."""

    def __init__(self):
        self._t = 0.0
        self.total_slept = 0.0

    def now(self) -> float:
        return self._t

    def sleep(self, seconds: float) -> None:
        self._t += seconds
        self.total_slept += seconds


# ---------------------------------------------------------------------------
# Transport
# ---------------------------------------------------------------------------
class Transport(ABC):
    @abstractmethod
    def publish(self, topic: str, msg_type: str, message: dict) -> None: ...

    @abstractmethod
    def call_service(self, name: str, srv_type: str, request: dict, timeout: float = 10.0) -> dict: ...

    @abstractmethod
    def latest(self, key: str) -> Any: ...

    @abstractmethod
    def subscribe(self, topic: str, msg_type: str, key: str, throttle_rate: int = 0) -> None: ...


class FakeTransport(Transport):
    """In-memory transport for tests. Records publishes + service calls, serves
    seeded and scripted telemetry."""

    def __init__(self, telemetry: dict | None = None, services: dict | None = None):
        self.published: list[tuple[str, str, dict]] = []
        self.service_calls: list[tuple[str, str, dict]] = []
        self.subscriptions: list[tuple[str, str, str]] = []
        self._telemetry: dict[str, Any] = dict(telemetry or {})
        self._services: dict[str, Any] = dict(services or {})
        self._scripts: dict[str, list] = {}

    # -- Transport interface ------------------------------------------------
    def publish(self, topic, msg_type, message):
        self.published.append((topic, msg_type, dict(message)))

    def call_service(self, name, srv_type, request, timeout=10.0):
        self.service_calls.append((name, srv_type, dict(request)))
        if name not in self._services:
            raise KeyError(f"no canned response for service {name}")
        return self._services[name]

    def latest(self, key):
        if key in self._scripts:
            seq = self._scripts[key]
            if len(seq) > 1:
                return seq.pop(0)
            return seq[0] if seq else None  # hold final value
        return self._telemetry.get(key)

    def subscribe(self, topic, msg_type, key, throttle_rate=0):
        self.subscriptions.append((topic, msg_type, key))

    # -- test helpers -------------------------------------------------------
    def last_published(self, topic: str) -> dict | None:
        for t, _, msg in reversed(self.published):
            if t == topic:
                return msg
        return None

    def set_telemetry(self, key: str, value: Any) -> None:
        self._telemetry[key] = value

    def script_telemetry(self, key: str, sequence: list) -> None:
        """Serve these values in order across successive latest(key) reads;
        the final value is held once the sequence is exhausted."""
        self._scripts[key] = list(sequence)


class RosbridgeTransport(Transport):
    """Production transport over roslibpy (rosbridge WebSocket). roslibpy is
    injected so the shaping/wiring is unit-testable; network is verified on
    hardware. Subscriptions unwrap std_msgs `.data` and stash the latest value
    per key."""

    def __init__(self, host: str, port: int = 9090, roslib=None):
        self._roslib = roslib or __import__("roslibpy")
        self._ros = self._roslib.Ros(host=host, port=port)
        self._latest: dict[str, Any] = {}
        self._pubs: dict[str, Any] = {}
        self._subs: list = []

    def connect(self) -> None:
        self._ros.run()

    def is_connected(self) -> bool:
        return getattr(self._ros, "is_connected", True)

    def terminate(self) -> None:
        try:
            self._ros.terminate()
        except Exception:
            pass

    def _pub(self, topic: str, msg_type: str):
        if topic not in self._pubs:
            self._pubs[topic] = self._roslib.Topic(self._ros, topic, msg_type)
        return self._pubs[topic]

    def publish(self, topic, msg_type, message):
        self._pub(topic, msg_type).publish(self._roslib.Message(dict(message)))

    def subscribe(self, topic, msg_type, key, throttle_rate=0):
        t = self._roslib.Topic(self._ros, topic, msg_type,
                               throttle_rate=throttle_rate, queue_length=1)

        def _cb(msg, k=key):
            self._latest[k] = msg["data"] if isinstance(msg, dict) and set(msg) == {"data"} else msg

        t.subscribe(_cb)
        self._subs.append(t)

    def latest(self, key):
        return self._latest.get(key)

    def call_service(self, name, srv_type, request, timeout=10.0):
        svc = self._roslib.Service(self._ros, name, srv_type)
        return svc.call(self._roslib.ServiceRequest(dict(request)), timeout=timeout)
