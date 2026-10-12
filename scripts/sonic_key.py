#!/usr/bin/env python
"""Send a keystroke to the MuJoCo viewer window via XTEST.

The viewer's only control surface is its keyboard handler
(base_sim.py:handle_keyboard_button), which is fine for a human and useless to
an automated run. xdotool is not installed here, so python-xlib's XTEST
extension synthesises the press instead.

Keys that matter:
    9   release the elastic band (send ONLY after the policy reports Init Done
        and has been sent command{start:1} -- otherwise the robot just falls)
    v   toggle the camera between mjCAMERA_TRACKING and mjCAMERA_FREE. The
        default is TRACKING on the pelvis, which keeps the robot centred in
        frame no matter where it goes -- so a recording cannot distinguish
        walking from marching in place until this is pressed.
    backspace   reset the robot

Usage:  sonic_key.py 9 v
"""
from __future__ import annotations

import sys
import time

from Xlib import X, XK, display
from Xlib.ext import xtest

KEYSYMS = {"9": "9", "v": "v", "backspace": "BackSpace", "space": "space"}


def find_mujoco_window(dpy):
    """The viewer is a large XWayland surface with no name and no _NET_WM_PID,
    so it cannot be found by title or pid -- it is identified by being the
    largest client window that is not the full-screen compositor guard."""
    root = dpy.screen().root
    rg = root.get_geometry()
    best, best_area = None, 0
    for w in root.query_tree().children:
        try:
            g = w.get_geometry()
        except Exception:
            continue
        area = g.width * g.height
        if 400_000 < area < 0.95 * rg.width * rg.height and area > best_area:
            best, best_area = w, area
    return best


def main() -> int:
    keys = sys.argv[1:] or ["9"]
    dpy = display.Display()
    win = find_mujoco_window(dpy)
    if win is None:
        print("no viewer-sized window found -- is the sim running with a window?")
        return 1

    g = win.get_geometry()
    print(f"[key] target window {hex(win.id)} {g.width}x{g.height}")
    win.set_input_focus(X.RevertToParent, X.CurrentTime)
    dpy.sync()
    time.sleep(0.3)

    for k in keys:
        code = dpy.keysym_to_keycode(XK.string_to_keysym(KEYSYMS.get(k, k)))
        xtest.fake_input(dpy, X.KeyPress, code)
        dpy.sync()
        time.sleep(0.05)
        xtest.fake_input(dpy, X.KeyRelease, code)
        dpy.sync()
        print(f"[key] sent {k!r}")
        time.sleep(0.4)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
