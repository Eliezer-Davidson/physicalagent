#!/usr/bin/env python
"""Press '9' in the MuJoCo window to release the elastic band, via XTEST.

The simulator suspends the robot on a virtual spring until someone presses 9 in
the viewer (unitree_sdk2py_bridge.py handles the key). That is fine for a human
at the keyboard and impossible for an automated run, and xdotool is not
installed here — so this synthesises the keystroke with python-xlib's XTEST
extension instead.

Order matters: release the band only AFTER the policy reports "Init Done" and
has been sent command{start:1}. Drop the robot before the policy is holding it
and it simply falls, which is the failure this whole sequence exists to avoid.
"""
from __future__ import annotations

import sys
import time

from Xlib import X, XK, display
from Xlib.ext import xtest


def find_mujoco_window(dpy):
    """The viewer window is a large XWayland surface with no name and no
    _NET_WM_PID, so it cannot be found by title or pid — it is identified by
    being the only client window of roughly viewer size."""
    root = dpy.screen().root
    best, best_area = None, 0
    for w in root.query_tree().children:
        try:
            g = w.get_geometry()
        except Exception:
            continue
        area = g.width * g.height
        # big, but not the full-screen compositor guard window
        if 400_000 < area < 0.95 * root.get_geometry().width * root.get_geometry().height:
            if area > best_area:
                best, best_area = w, area
    return best


def main() -> int:
    dpy = display.Display()
    win = find_mujoco_window(dpy)
    if win is None:
        print("no viewer-sized window found — is the sim running with a window?")
        return 1

    g = win.get_geometry()
    print(f"[band] target window {hex(win.id)} {g.width}x{g.height}")

    # Focus it, then send the keypress. Both are needed: GLFW delivers key
    # events to the focused window only.
    win.set_input_focus(X.RevertToParent, X.CurrentTime)
    dpy.sync()
    time.sleep(0.3)

    keycode = dpy.keysym_to_keycode(XK.string_to_keysym("9"))
    for _ in range(3):                      # a couple of presses; the toggle is idempotent-ish
        xtest.fake_input(dpy, X.KeyPress, keycode)
        dpy.sync()
        time.sleep(0.05)
        xtest.fake_input(dpy, X.KeyRelease, keycode)
        dpy.sync()
        time.sleep(0.4)

    print("[band] sent '9' — the robot should now be off the spring")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
