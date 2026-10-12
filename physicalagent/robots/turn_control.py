"""Closed-loop turn controller — the ROSPug over-turns ~122% open-loop and its
IMU yaw drifts 1.4 deg/s, so turns close the loop on rf2o /odom_raw.

Command a fixed yaw rate in the target direction, poll odometry yaw, integrate
the (wrap-safe) change, and stop the instant the accumulated turn reaches the
target within tolerance — or on timeout. Report the achieved angle either way.
Never turns blind: if odometry is unavailable it refuses without commanding
motion.

Pure logic: Transport + Clock are injected, so this whole controller runs
offline against FakeTransport with a scripted odom sequence.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

from physicalagent.robots.transport import Clock, Transport


@dataclass(frozen=True)
class TurnResult:
    achieved_deg: float
    reached: bool
    timed_out: bool = False
    error: str | None = None
    odom_observed_deg: float | None = None   # cross-check only, NOT authoritative
    method: str = "closed_loop"


def _yaw_deg(odom_msg: dict) -> float | None:
    """Extract yaw (degrees) from a nav_msgs/Odometry-shaped dict."""
    try:
        q = odom_msg["pose"]["pose"]["orientation"]
        w, x, y, z = q["w"], q["x"], q["y"], q["z"]
    except (KeyError, TypeError):
        return None
    return math.degrees(math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))


def _wrap180(d: float) -> float:
    while d > 180:
        d -= 360
    while d < -180:
        d += 360
    return d


class TurnController:
    def __init__(self, transport: Transport, clock: Clock, *,
                 vel_topic: str, vel_type: str, odom_key: str,
                 yaw_rate: float, tol_deg: float, timeout_s: float,
                 poll_dt: float = 0.1, stall_s: float = 1.0,
                 wrong_way_deg: float = 8.0, open_loop: bool = False,
                 over_turn: float = 1.22, overshoot_factor: float = 2.0):
        self._t = transport
        self._clock = clock
        self._vel_topic = vel_topic
        self._vel_type = vel_type
        self._odom_key = odom_key
        self._yaw_rate = abs(yaw_rate)
        self._tol = tol_deg
        self._timeout = timeout_s
        self._poll_dt = poll_dt
        self._stall_s = stall_s      # no odom progress for this long => stop
        self._wrong_way_deg = wrong_way_deg   # rotation away from target => stop
        self._open_loop = open_loop
        self._over_turn = over_turn           # measured: robot turns ~122% of command
        self._overshoot_factor = overshoot_factor

    def _command(self, yaw_rate: float, stop: bool = False) -> None:
        self._t.publish(self._vel_topic, self._vel_type,
                        {"x": 0.0, "y": 0.0, "yaw_rate": float(yaw_rate), "stop": bool(stop)})

    def _stop(self) -> None:
        self._command(0.0, stop=True)

    def _odom_now(self) -> float | None:
        return _yaw_deg(self._t.latest(self._odom_key) or {})

    def turn_open_loop(self, target_deg: float) -> TurnResult:
        """Drive a calibrated duration; do NOT let odometry decide when to stop.

        VERIFIED ON HARDWARE: rf2o under-reports rotation, so closing the loop on
        it made the robot keep turning while convinced it had not moved — the
        operator watched it "spin and spin", overshooting badly, and saw turns
        refused outright with "odometry stalled" while the robot was rotating
        perfectly well. Since the GAIT is accurate and only the sensor is wrong,
        a timed command calibrated for the measured ~122% over-turn is both more
        accurate and incapable of that runaway.

        Odometry is kept solely as a gross-overshoot monitor. That role is safe
        precisely because it under-reports: if it still claims a huge rotation,
        the rotation is certainly real.
        """
        rate_deg_s = max(math.degrees(self._yaw_rate), 1e-6)
        duration = abs(target_deg) / rate_deg_s / self._over_turn
        direction = 1.0 if target_deg > 0 else -1.0
        start = self._odom_now()
        accumulated, prev = 0.0, start
        cutoff = abs(target_deg) * self._overshoot_factor

        self._command(direction * self._yaw_rate)
        remaining = duration
        while remaining > 0:
            step = min(self._poll_dt, remaining)
            self._clock.sleep(step)
            remaining -= step
            cur = self._odom_now()
            if cur is not None and prev is not None:
                accumulated += _wrap180(cur - prev)
            if cur is not None:
                prev = cur
            if abs(accumulated) >= cutoff:
                self._stop()
                return TurnResult(
                    achieved_deg=round(accumulated, 1), reached=False,
                    odom_observed_deg=round(accumulated, 1), method="open_loop",
                    error=f"stopped early: rotation reached {accumulated:.0f} deg, "
                          f"far past the requested {target_deg:.0f}")
        self._stop()
        return TurnResult(
            achieved_deg=round(target_deg, 1), reached=True, method="open_loop",
            odom_observed_deg=round(accumulated, 1) if start is not None else None)

    def turn(self, target_deg: float) -> TurnResult:
        if abs(target_deg) < 1e-6:
            return TurnResult(achieved_deg=0.0, reached=True)
        if self._open_loop:
            return self.turn_open_loop(target_deg)

        start = _yaw_deg(self._t.latest(self._odom_key) or {})
        if start is None:
            return TurnResult(achieved_deg=0.0, reached=False,
                              error="odometry (/odom_raw) unavailable — refusing to turn blind")

        direction = 1.0 if target_deg > 0 else -1.0
        accumulated = 0.0
        prev = start

        # Time budget comes from the ANGLE, not from an unrelated walk-duration
        # cap. Tying it to max_duration_s meant raising the walk limit silently
        # raised this to 24 s — ~550 deg of blind rotation on a stuck turn.
        rate_deg_s = max(math.degrees(self._yaw_rate), 1e-6)
        budget = abs(target_deg) / rate_deg_s * 2.0 + 3.0
        effective_timeout = min(self._timeout, budget)

        t0 = self._clock.now()
        last_progress_t = t0
        last_acc = 0.0

        self._command(direction * self._yaw_rate)
        while True:
            self._clock.sleep(self._poll_dt)
            cur = _yaw_deg(self._t.latest(self._odom_key) or {})
            if cur is not None:
                accumulated += _wrap180(cur - prev)
                prev = cur

            now = self._clock.now()
            # progress watchdog: if the accumulated angle has not advanced for
            # stall_s, the odometry has frozen or died (rf2o is fragile) — stop
            # rather than spin blind at the yaw rate until the hard timeout.
            if abs(accumulated - last_acc) > 0.5:
                last_acc = accumulated
                last_progress_t = now

            if abs(accumulated) >= abs(target_deg) - self._tol:
                self._stop()
                return TurnResult(achieved_deg=round(accumulated, 1), reached=True)

            if now - last_progress_t >= self._stall_s:
                self._stop()
                return TurnResult(achieved_deg=round(accumulated, 1), reached=False,
                                  error="odometry stalled mid-turn (no rotation progress) — stopped")

            # Rotating AWAY from the target means the odometry has lost tracking
            # or the robot is being moved. Observed live: commanded +45, odom
            # reported -27.8, and the controller kept driving into a spin.
            if accumulated * direction <= -self._wrong_way_deg:
                self._stop()
                return TurnResult(
                    achieved_deg=round(accumulated, 1), reached=False,
                    error=f"rotation is going the wrong direction "
                          f"({accumulated:.1f} deg while turning "
                          f"{'left' if direction > 0 else 'right'}) — odometry is "
                          f"unreliable here; stopped rather than spin")

            if now - t0 >= effective_timeout:
                self._stop()
                return TurnResult(achieved_deg=round(accumulated, 1),
                                  reached=False, timed_out=True)
