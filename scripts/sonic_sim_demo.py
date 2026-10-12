#!/usr/bin/env python
"""Type a prompt, watch a simulated Unitree G1 try to do it.

    python scripts/sonic_sim_demo.py "walk forward a couple of metres, then turn left"

Pipeline:
    your prompt
      -> Gemini Robotics-ER 2            (physicalagent.brain.er, already tested)
      -> tool calls: look/walk/turn/set_pose/stop
      -> SonicG1Adapter                  (clamps, bounds, refuses)
      -> SonicLink                       (--link fake, or the real SONIC policy)
      -> G1 in MuJoCo

`--link fake` needs nothing running: the link integrates the commanded velocity
so the whole prompt->decision->motion path is exercisable with no simulator and
no policy. The camera view is real either way — MuJoCo renders the G1's on-board
camera and the frame is handed to the model when it calls look().

Rendering: headless uses EGL, an interactive viewer needs GLX on this Wayland
box. Passed explicitly rather than guessed, because getting it backwards looks
exactly like a broken simulator.
"""
from __future__ import annotations

import argparse
import os
import pathlib
import sys
import time

REPO = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))

from physicalagent.paths import G1_MODEL_DIR  # noqa: E402

G1_SCENE = str(G1_MODEL_DIR / "scene_29dof_inspire_rigid.xml")

SYSTEM = """You control a Unitree G1 humanoid robot in a physics simulation.

The robot balances itself — you never command joints. You command where the body
should go and which pose it should hold, and a whole-body controller works out
the 29 joint angles that keep it upright while doing what you asked.

Your tools:
  look()                  see through the robot's forward camera
  walk(vx, vy, duration_s)  vx forward m/s (+forward), vy sideways (+left)
  turn(yaw_rate, duration_s)  rad/s, + turns left
  set_pose(name)          stand, sit, squat, damp
  get_status()            height, tilt, velocity, whether state is readable
  stop()                  halt and stand still

Important behaviours:
  * Every walk and turn EXPIRES on its own after duration_s and the robot stops.
    To keep moving, call it again. This is deliberate.
  * Speeds are clamped. The result tells you what was ACTUALLY commanded, which
    may be slower than you asked for. Read it and adapt rather than assuming.
  * Prefer several short moves with a look() or get_status() between them over
    one long blind move.
  * 'damp' relaxes every joint and the robot sinks to the ground. It is a last
    resort, not a way to stop. Use stop() to stop.
  * If a tool refuses, the refusal explains why. Adjust and continue; don't repeat
    the identical call.

Say what you observe and what you intend before acting. When the task is done,
give a short plain-English summary of what happened.
"""


from physicalagent.apikey import load_gemini_key as load_api_key  # noqa: E402


def build_link(kind: str, port: int):
    if kind == "fake":
        from physicalagent.robots.sonic_link import FakeSonicLink
        return FakeSonicLink(), "FakeSonicLink (no simulator; commands integrated)"
    if kind == "sonic":
        from physicalagent.robots.sonic_zmq import ZmqSonicLink
        link = ZmqSonicLink(port=port)
        return link, (f"ZmqSonicLink bound on tcp://*:{port} — the policy must be "
                      f"running with --input-type zmq_manager")
    raise SystemExit(f"unknown link {kind!r}")


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("prompt", nargs="*", help="what the robot should do")
    p.add_argument("--model", default="gemini-robotics-er-2-preview")
    p.add_argument("--link", default="fake", choices=["fake", "sonic"])
    p.add_argument("--scene", default=G1_SCENE)
    p.add_argument("--no-camera", action="store_true",
                   help="skip MuJoCo rendering (no look() tool)")
    p.add_argument("--zmq-port", type=int, default=5556)
    p.add_argument("--max-hops", type=int, default=12)
    args = p.parse_args()

    task = " ".join(args.prompt).strip()
    if not task:
        task = input("What should the robot do? ").strip()
    if not task:
        return 2

    os.environ.setdefault("MUJOCO_GL", "egl")   # headless render

    from physicalagent.brain.er import Brain, GeminiClient
    from physicalagent.robots.sonic_g1 import SonicG1Adapter
    from physicalagent.robots.sonic_link import MujocoView

    view = None
    if not args.no_camera:
        try:
            view = MujocoView(model_path=args.scene).open()
            print(f"[sim]  camera ready from {pathlib.Path(args.scene).name}")
        except Exception as exc:
            print(f"[sim]  no camera ({exc}); continuing without look()")

    link, link_desc = build_link(args.link, args.zmq_port)
    print(f"[link] {link_desc}")

    class _Guard:
        is_estopped = False

    adapter = SonicG1Adapter(link, _Guard(), view=view)
    brain = Brain(adapter, GeminiClient(load_api_key(), model=args.model), SYSTEM,
                  max_hops=args.max_hops)

    print(f"[brain] {args.model}")
    print(f"[task] {task}\n" + "-" * 68)

    # Signature must match Brain.run's call exactly -- it passes six arguments
    # (name, args, ok, error, payload, elapsed_s) and wraps the callback in a
    # bare `except Exception: pass` so a broken display cannot abort a task that
    # is physically underway. A five-argument version therefore printed NOTHING
    # for every tool call, silently, while still reporting tools=7. Bit once on
    # 2026-07-31; the missing lines were the only gap in the evidence chain.
    def on_step(name, tool_args, ok, error, result, elapsed_s=None):
        mark = "ok " if ok else "REF"
        detail = error if error else ", ".join(f"{k}={v}" for k, v in (result or {}).items())
        stamp = f"{time.time():.3f} "
        print(f"  {stamp}[{mark}] {name}({', '.join(f'{k}={v}' for k, v in tool_args.items())})"
              f"{' -> ' + detail if detail else ''}"
              f"{f' [{elapsed_s}s]' if elapsed_s is not None else ''}", flush=True)

    result = brain.run(task, on_step=on_step)
    print("-" * 68)
    print(result.final_text or "(no final answer)")
    print(f"[brain] hops={result.hops} complete={result.complete} "
          f"tools={len(result.trace)}"
          + (f" think={result.think_s:.1f}s" if result.think_s else ""))

    state = link.read_state()
    if getattr(link, "position", None) is not None:
        print(f"\n[final] position={link.position} yaw={link.yaw:.2f} rad "
              f"mode={state.mode}")
    else:
        print(f"\n[final] mode={state.mode}  {state.note or ''}")
    if hasattr(link, "close"):
        link.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
