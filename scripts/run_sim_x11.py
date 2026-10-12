"""Launch NVIDIA's gear_sonic MuJoCo sim with a WORKING on-screen window.

WHY THIS WRAPPER EXISTS
    `run_sim_loop.py --enable-onscreen` silently produces no window on this
    machine. mujoco.viewer.launch_passive() returns a live Handle,
    `is_running()` is True, no error is logged — and no X window is ever
    created. That is the worst kind of failure: a success-shaped one.

    Root cause: MuJoCo's viewer uses the Python `glfw` package, and this GLFW
    build advertises "X11 GLX Null EGL OSMesa" — it has a **Null** platform,
    a headless stub that accepts window creation and shows nothing. On this
    GNOME/Wayland session GLFW does not land on X11 by itself, so the viewer
    ends up on a platform that renders into the void.

    Forcing PLATFORM_X11 *before* MuJoCo touches GLFW fixes it. Verified: with
    the hint the window appears (1726x1121, Map State IsViewable, titled
    "MuJoCo : g1_29dof_inspire_rigid scene"); without it, nothing.

    Unsetting WAYLAND_DISPLAY is NOT sufficient on its own — that only stops
    the earlier, LOUD failure ("Wayland: Failed to create XKB compose table"
    then "OpenGL error 0x502 in or before mjr_makeContext"). Silence after
    that is not success.

USAGE
    XDG_SESSION_TYPE=x11 DISPLAY=:0 MUJOCO_GL=glfw \
        .venv_sim/bin/python scripts/run_sim_x11.py [--enable-onscreen ...]

    Arguments are passed straight through to run_sim_loop.py.
"""
from __future__ import annotations

import os
import runpy
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from physicalagent.paths import WBC_ROOT  # noqa: E402

SIM = str(WBC_ROOT / "gear_sonic" / "scripts" / "run_sim_loop.py")

# THE ONE THAT MATTERS. glfw/library.py:219-225 picks which libglfw.so to load:
#     if PYGLFW_LIBRARY_VARIANT in (wayland, x11): use it
#     elif XDG_SESSION_TYPE == 'wayland':          use the WAYLAND build
#     else:                                        use the X11 build
# This session is XDG_SESSION_TYPE=wayland, so without this the Wayland-only
# build loads — and it cannot open an X window at all, which is why the viewer
# silently rendered nowhere. Unsetting WAYLAND_DISPLAY does NOT affect this
# choice; only these two variables do. Assign, never setdefault: XDG_SESSION_TYPE
# is already set to "wayland", so setdefault is a no-op that looks like a fix.
os.environ["PYGLFW_LIBRARY_VARIANT"] = "x11"
os.environ["XDG_SESSION_TYPE"] = "x11"
os.environ.pop("WAYLAND_DISPLAY", None)
os.environ.setdefault("DISPLAY", ":0")
os.environ["MUJOCO_GL"] = "glfw"

import glfw  # noqa: E402  (must follow the env setup above)

build = glfw.get_version_string().decode()
print(f"[x11-wrapper] glfw build = {build}", flush=True)
if "X11" not in build:
    sys.exit(f"loaded a GLFW without X11 support ({build}) — no window is possible")
if not glfw.init():
    sys.exit("glfw.init() failed — cannot open a window on this display")
print("[x11-wrapper] glfw initialised on X11", flush=True)

sys.argv = [SIM] + sys.argv[1:]
runpy.run_path(SIM, run_name="__main__")
