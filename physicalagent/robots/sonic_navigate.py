"""Go to a place, rather than walk for a while.

WHAT THIS IS
    `goto(x, y)` closed on the robot's MEASURED pose: read rt/odostate, compute
    the heading to the goal, send it as an absolute course, repeat at 10 Hz,
    stop inside a tolerance. Nav2's NavigateToPose semantics without ROS 2 --
    the shape a model actually wants (a goal, a loop, a terminal status), which
    is most of what ROS 2 would have been imported for.

WHY IT CANNOT USE walk()
    walk() is open loop twice over. Its heading comes from `_yaw`, which is the
    integral of yaw rates we COMMANDED and nothing corrects, and its distance is
    speed x time with no check that the robot moved. Both errors grow without
    bound. Closing on odometry makes the drift irrelevant instead of small.

THE FRAME PROBLEM, AND WHY IT IS MEASURED AND NOT ASSUMED
    movement/facing go from the wire into the planner untransformed
    (g1_deploy_onnx_ref.cpp:3712-3742), so they are in the policy's heading
    frame -- anchored to the base quaternion captured at policy init
    (HeadingState, robot_parameters.hpp:55-61). Odometry is in the simulator's
    world frame. The two coincide only if the robot happened to start at yaw 0.

    Assuming they coincide would produce a navigator that converges on this
    machine, on this spawn pose, and silently spirals anywhere else. So the
    offset is a parameter, `frame_offset`, and calibrate() MEASURES it: walk a
    short known course, see which way the robot actually went, take the
    difference. An uncalibrated Navigator says so in its result.

WHAT IT DOES NOT DO
    No obstacle avoidance. It walks the straight line to the goal and will walk
    into anything on it. The sidecar's scan() can see obstacles and a later
    version can use it; until then this is honest dead-simple point-to-point and
    the tool description says so.
"""
from __future__ import annotations

import math

# Stop inside this and call it arrived. The G1's own footfall wanders several
# centimetres per step, so a tighter goal than this is not a goal, it is a
# limit cycle -- the robot shuffling forever around a point it cannot hold.
POS_TOL_M = 0.15
YAW_TOL_RAD = math.radians(12)

# Below this we are close enough that steering hard at the goal overshoots more
# than it corrects; walk slowly and let the tolerance catch it.
SLOW_RADIUS_M = 0.6
MIN_SPEED_M_S = 0.15
MAX_SPEED_M_S = 0.5

# Turn in place rather than crab sideways when the goal is far off the nose.
# The policy CAN walk sideways, but it is slower and less stable than turning,
# and a model asking to "go to the kitchen" means facing the kitchen.
TURN_FIRST_RAD = math.radians(60)

# A goal further than this is refused rather than attempted. Not a safety limit
# -- an honesty one: without obstacle avoidance, a 20 m open-loop straight line
# is a promise this cannot keep.
MAX_RANGE_M = 8.0


def wrap(a: float) -> float:
    """Angle to (-pi, pi]. Every heading error must go through this: the
    difference of two headings near +/-pi is otherwise ~2pi, and the robot spins
    the long way round a 1-degree error."""
    return math.atan2(math.sin(a), math.cos(a))


class Goal:
    __slots__ = ("x", "y", "yaw")

    def __init__(self, x: float, y: float, yaw: float | None = None):
        self.x, self.y, self.yaw = float(x), float(y), yaw


def step(pose: tuple[float, float, float], goal: Goal, *,
         frame_offset: float = 0.0) -> dict:
    """One control cycle. PURE -- no clock, no link, no telemetry.

    pose is (x, y, yaw) measured, in odometry frame. Returns what to command and
    whether we are there. Keeping this pure is what lets the tests drive it to
    convergence over thousands of cycles with no simulator.
    """
    x, y, yaw = pose
    dx, dy = goal.x - x, goal.y - y
    dist = math.hypot(dx, dy)

    if dist <= POS_TOL_M:
        if goal.yaw is None:
            return {"done": True, "reason": "arrived", "distance_m": dist,
                    "speed": 0.0}
        yaw_err = wrap(goal.yaw - yaw)
        if abs(yaw_err) <= YAW_TOL_RAD:
            return {"done": True, "reason": "arrived", "distance_m": dist,
                    "yaw_error_rad": yaw_err, "speed": 0.0}
        # In position, wrong heading: turn on the spot. Facing is what the
        # policy turns toward; movement is irrelevant at zero speed but must
        # still be a unit vector, so point it the same way.
        target = goal.yaw + frame_offset
        return {"done": False, "reason": "turning to final heading",
                "distance_m": dist, "yaw_error_rad": yaw_err, "speed": 0.0,
                "move_heading": target, "facing_heading": target}

    course = math.atan2(dy, dx)               # odometry frame
    heading_err = wrap(course - yaw)

    # Slow down inside the approach radius, and slow down when badly misaligned
    # so the robot turns rather than arcs wide.
    speed = MAX_SPEED_M_S * min(1.0, dist / SLOW_RADIUS_M)
    if abs(heading_err) > TURN_FIRST_RAD:
        speed = 0.0                           # turn in place first
    else:
        speed = max(MIN_SPEED_M_S, speed) * max(0.3, math.cos(heading_err))

    target = course + frame_offset
    return {"done": False,
            "reason": "turning toward goal" if speed == 0.0 else "walking",
            "distance_m": dist, "heading_error_rad": heading_err,
            "speed": speed, "move_heading": target, "facing_heading": target}


class Navigator:
    """Runs `step` against a real link and telemetry until the goal or an abort.

    Every exit is a named reason. A navigator that returns without saying why it
    stopped leaves the model to infer success from the absence of an error, and
    it will infer wrong.
    """

    # Sent slightly longer than the cycle so the stream never lapses to IDLE
    # between cycles -- the planner drops a message older than ~1 s, but a gap
    # shorter than that still shows up as a stutter in the gait.
    CYCLE_S = 0.1
    COMMAND_TTL_S = 0.5

    # If the robot has not closed the distance by this much over NO_PROGRESS_S,
    # something is holding it: a wall, a fallen policy, a bad frame offset.
    NO_PROGRESS_M = 0.10
    NO_PROGRESS_S = 4.0

    # Below this the robot is not standing, whatever else is true.
    FLOOR_HEIGHT_M = 0.55

    def __init__(self, link, telemetry, clock=None, *, frame_offset: float = 0.0,
                 calibrated: bool = False):
        from physicalagent.robots.sonic_link import RealClock

        self._link = link
        self._telemetry = telemetry
        self._clock = clock if clock is not None else RealClock()
        self.frame_offset = float(frame_offset)
        self.calibrated = bool(calibrated)

    def _pose(self):
        p = self._telemetry.pose()
        if not p.get("available") or not p.get("fresh"):
            return None, p
        pos = p["position_m"]
        return (pos["x"], pos["y"], math.radians(p["heading_deg"])), p

    def go(self, goal: Goal, *, timeout_s: float = 60.0) -> dict:
        start, praw = self._pose()
        if start is None:
            return {"ok": False, "reason": "no pose — cannot navigate blind",
                    "pose": praw}
        dist0 = math.hypot(goal.x - start[0], goal.y - start[1])
        if dist0 > MAX_RANGE_M:
            return {"ok": False, "reason":
                    f"goal is {dist0:.1f} m away; this walks a straight line "
                    f"with no obstacle avoidance and refuses beyond "
                    f"{MAX_RANGE_M} m. Break it into legs.",
                    "distance_m": round(dist0, 2)}

        t0 = self._clock.time()
        best = dist0
        best_t = t0
        cycles = 0
        while True:
            now = self._clock.time()
            if now - t0 > timeout_s:
                return self._finish(goal, start, dist0, cycles,
                                    "timed out still walking")

            pose, praw = self._pose()
            if pose is None:
                self._link.stop()
                return self._finish(goal, start, dist0, cycles,
                                    "pose went stale mid-run — stopped rather "
                                    "than keep walking blind")
            if praw.get("height_m", 1.0) < self.FLOOR_HEIGHT_M:
                self._link.stop()
                return self._finish(goal, start, dist0, cycles,
                                    "robot went down — stopped")

            out = step(pose, goal, frame_offset=self.frame_offset)
            cycles += 1
            if out["done"]:
                self._link.stop()
                return self._finish(goal, start, dist0, cycles, out["reason"])

            if out["distance_m"] < best - self.NO_PROGRESS_M:
                best, best_t = out["distance_m"], now
            elif now - best_t > self.NO_PROGRESS_S:
                self._link.stop()
                return self._finish(
                    goal, start, dist0, cycles,
                    f"no progress for {self.NO_PROGRESS_S:.0f}s — blocked, or "
                    "the heading frame is wrong (run calibrate)")

            self._link.set_course(out["move_heading"], out["facing_heading"],
                                  out["speed"], self.COMMAND_TTL_S)
            self._clock.sleep(self.CYCLE_S)

    def _finish(self, goal, start, dist0, cycles, reason) -> dict:
        self._link.stop()
        end, praw = self._pose()
        out = {
            "reason": reason,
            "start_m": {"x": round(start[0], 2), "y": round(start[1], 2)},
            "goal_m": {"x": round(goal.x, 2), "y": round(goal.y, 2)},
            "distance_at_start_m": round(dist0, 2),
            "cycles": cycles,
            "frame_offset_deg": round(math.degrees(self.frame_offset), 1),
            "frame_offset_measured": self.calibrated,
        }
        if end is None:
            out["ok"] = False
            out["note"] = "final pose unreadable, so the result is unverified"
            return out
        err = math.hypot(goal.x - end[0], goal.y - end[1])
        out["end_m"] = {"x": round(end[0], 2), "y": round(end[1], 2)}
        out["final_error_m"] = round(err, 2)
        out["ok"] = err <= POS_TOL_M
        if goal.yaw is not None:
            out["final_heading_error_deg"] = round(
                math.degrees(wrap(goal.yaw - end[2])), 1)
        if not self.calibrated:
            out["warning"] = (
                "frame offset was assumed to be zero, not measured. If the "
                "robot walked off at an angle, that is why — call "
                "calibrate_heading() once per session and try again.")
        return out

    def calibrate(self, *, distance_m: float = 0.8, speed: float = 0.3) -> dict:
        """Measure the offset between the policy's heading frame and odometry.

        Walk a short leg on a commanded course, see which way the robot actually
        travelled, and take the difference. One measurement per policy start;
        the frame is anchored at init and does not move after that.
        """
        start, _ = self._pose()
        if start is None:
            return {"ok": False, "reason": "no pose — cannot calibrate"}

        commanded = 0.0                       # policy frame, straight ahead
        duration = distance_m / speed
        t_end = self._clock.time() + duration
        while self._clock.time() < t_end:
            self._link.set_course(commanded, commanded, speed, self.COMMAND_TTL_S)
            self._clock.sleep(self.CYCLE_S)
        self._link.stop()
        self._clock.sleep(1.0)                # let it settle before measuring

        end, _ = self._pose()
        if end is None:
            return {"ok": False, "reason": "pose lost during calibration"}
        dx, dy = end[0] - start[0], end[1] - start[1]
        travelled = math.hypot(dx, dy)
        if travelled < 0.15:
            return {"ok": False, "travelled_m": round(travelled, 2),
                    "reason": "robot barely moved, so the direction it moved in "
                              "means nothing — is the policy controlling?"}
        actual = math.atan2(dy, dx)           # odometry frame
        # commanded(policy) = actual(odometry) + offset
        self.frame_offset = wrap(commanded - actual)
        self.calibrated = True
        return {"ok": True, "travelled_m": round(travelled, 2),
                "commanded_heading_deg": round(math.degrees(commanded), 1),
                "actual_heading_deg": round(math.degrees(actual), 1),
                "frame_offset_deg": round(math.degrees(self.frame_offset), 1),
                "note": "add this offset to an odometry-frame heading to get "
                        "the heading to put on the wire"}
