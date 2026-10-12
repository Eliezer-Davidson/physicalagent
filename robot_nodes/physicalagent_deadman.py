#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""PhysicalAgent deadman — runs ON the ROSPug (Jetson), not on the PC.

WHY THIS EXISTS
    pug_node.py has NO timeout anywhere in its command path. Publish a velocity
    and the robot executes it until something else tells it otherwise. Over a
    cable that is fine. Over Wi-Fi it is not: if the link drops mid-walk, the PC
    cannot send a stop (there is no link to send it over), and the robot keeps
    walking into whatever is in front of it.

    A watchdog on the PC cannot solve this — it is on the wrong side of the
    break. The stop has to be decided by something that is still running when
    the network is gone. That is this node.

BEHAVIOUR (temporary pause, NOT a kill)
    PC publishes a heartbeat a few times a second. If this node stops hearing
    it, the robot STOPS and HOLDS its position. When the heartbeat comes back,
    the node simply un-pauses and normal commanding resumes — no restart, no
    re-launch, nothing to recover by hand.

WHY NOT ENABLE self_balancing ON PAUSE
    It looks like the obvious "balance yourself" call, but read
    pug_app/scripts/self_balancing.py:51 — enabling it PUBLISHES A POSE
    (0,0,0,-0.13,0,0,0,1) and then runs PID corrections off the IMU. That is
    active motion, while unsupervised, which is exactly what a deadman is meant
    to prevent. A quadruped standing on four legs is already statically stable:
    halt the gait and the servos hold their angles, so it stays upright and in
    place. Stopping IS the safe state here.
"""
from __future__ import print_function

import threading
import time

import rospy
from std_msgs.msg import Bool, String

from ros_robot_controller.msg import Stop

# --- Tunables (overridable as ROS private params) ---------------------------
DEFAULT_HEARTBEAT_TOPIC = "/physicalagent/heartbeat"
DEFAULT_STATUS_TOPIC = "/physicalagent/deadman_status"
DEFAULT_STOP_TOPIC = "/ros_robot_controller/robot/stop"
DEFAULT_TIMEOUT_S = 1.5      # silence longer than this => pause
DEFAULT_CHECK_HZ = 10.0      # how often we evaluate liveness
DEFAULT_REASSERT_S = 0.5     # while paused, re-send stop this often

# State names (published so the PC can see what the robot thinks).
DORMANT = "DORMANT"   # never heard a heartbeat yet -> not our business
ALIVE = "ALIVE"       # heartbeat healthy -> commands flow normally
PAUSED = "PAUSED"     # heartbeat lost (or PC asked) -> robot stopped, holding


class Deadman(object):
    def __init__(self):
        rospy.init_node("physicalagent_deadman")

        self.timeout = float(rospy.get_param("~timeout", DEFAULT_TIMEOUT_S))
        self.check_hz = float(rospy.get_param("~check_hz", DEFAULT_CHECK_HZ))
        self.reassert_s = float(rospy.get_param("~reassert", DEFAULT_REASSERT_S))
        hb_topic = rospy.get_param("~heartbeat_topic", DEFAULT_HEARTBEAT_TOPIC)

        # _last_beat is a MONOTONIC timestamp, deliberately not ROS/wall time.
        # This robot's clock is ~3 years behind, and we have given it internet,
        # so NTP may jump the wall clock by years mid-run. A monotonic clock
        # cannot jump, so our elapsed-time maths stays correct regardless.
        self._last_beat = None      # None => never seen a heartbeat
        self._paused = False
        self._armed = False         # becomes True on the first heartbeat, ever
        self._last_stop_sent = 0.0
        self._lock = threading.Lock()

        self._stop_pub = rospy.Publisher(
            rospy.get_param("~stop_topic", DEFAULT_STOP_TOPIC), Stop, queue_size=1)
        # latch=True so a PC that subscribes later immediately learns the state.
        self._status_pub = rospy.Publisher(
            rospy.get_param("~status_topic", DEFAULT_STATUS_TOPIC),
            String, queue_size=1, latch=True)

        rospy.Subscriber(hb_topic, Bool, self._on_heartbeat, queue_size=1)

        self._publish_status(DORMANT)
        rospy.loginfo(
            "[deadman] up. listening on %s, timeout %.2fs. "
            "DORMANT until the first heartbeat (so app/joystick use is unaffected).",
            hb_topic, self.timeout)

    # -- heartbeat ---------------------------------------------------------
    def _on_heartbeat(self, msg):
        """Called for every heartbeat the PC sends.

        msg.data == True  -> "I am alive and in control"
        msg.data == False -> "pause now, deliberately" (a soft e-stop that
                             rides the same channel, so the PC can pause the
                             robot without waiting for a timeout).
        """
        now = time.monotonic()
        with self._lock:
            if not self._armed:
                self._armed = True
                rospy.loginfo("[deadman] ARMED — first heartbeat received.")
                # Announce it. Without this the status topic still reads DORMANT
                # while we are actually armed and monitoring, so an observer
                # cannot tell "not watching" from "watching, all healthy".
                self._publish_status(ALIVE)
            self._last_beat = now

            if not msg.data:
                # Explicit pause request from the PC.
                if not self._paused:
                    rospy.logwarn("[deadman] PC requested pause (heartbeat=False)")
                    self._enter_pause_locked()
                return

            # Healthy beat: if we were paused, this is the resume edge.
            if self._paused:
                self._paused = False
                rospy.loginfo("[deadman] RESUMED — heartbeat restored, commands allowed.")
                self._publish_status(ALIVE)

    # -- pause / stop ------------------------------------------------------
    def _send_stop(self):
        """Publish a stop to the low-level controller.

        Stop.msg is a single float32 `duration`; the SDK comment documents it as
        'stop, runtime s'. 0.0 means stop immediately. This halts the gait
        engine; the legs stay where they are, holding the robot up.
        """
        try:
            self._stop_pub.publish(Stop(duration=0.0))
            self._last_stop_sent = time.monotonic()
        except Exception as exc:                      # never let the loop die
            rospy.logerr("[deadman] stop publish failed: %s", exc)

    def _enter_pause_locked(self):
        """Transition into PAUSED. Caller must already hold self._lock."""
        self._paused = True
        self._send_stop()
        self._publish_status(PAUSED)

    def _publish_status(self, state):
        try:
            self._status_pub.publish(String(data=state))
        except Exception:
            pass

    # -- main loop ---------------------------------------------------------
    def spin(self):
        rate = rospy.Rate(self.check_hz)
        while not rospy.is_shutdown():
            with self._lock:
                # Never interfere before the PC has ever talked to us. Without
                # this, the node would spam stop at boot and fight the phone app
                # or the wireless handle during normal standalone use.
                if self._armed and self._last_beat is not None:
                    silent_for = time.monotonic() - self._last_beat

                    if not self._paused and silent_for > self.timeout:
                        rospy.logwarn(
                            "[deadman] HEARTBEAT LOST (%.2fs > %.2fs) — stopping and holding.",
                            silent_for, self.timeout)
                        self._enter_pause_locked()

                    elif self._paused:
                        # Re-assert the stop periodically while paused, in case
                        # anything else re-commands motion while we are blind.
                        if time.monotonic() - self._last_stop_sent > self.reassert_s:
                            self._send_stop()
            rate.sleep()

        # On clean shutdown, leave the robot stopped rather than mid-gait.
        rospy.logwarn("[deadman] shutting down — sending final stop.")
        self._send_stop()


if __name__ == "__main__":
    try:
        Deadman().spin()
    except rospy.ROSInterruptException:
        pass
