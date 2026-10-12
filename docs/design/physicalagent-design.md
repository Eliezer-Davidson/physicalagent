# PhysicalAgent — Design Spec

**Date:** 2026-07-17
**Status:** Frozen scope (Phase 0). The design document the first robot adapter was built from; kept as written, apart from removing references to files that are not part of this repository.

---

## 1. Mission

One program, **PhysicalAgent**, that connects Google's **Gemini Robotics-ER 1.6**
(`gemini-robotics-er-1.6-preview`) to two HiWonder robots, so they can be
driven by natural-language **voice or text**:

| Robot | Type | Onboard stack | Transport to PC |
|---|---|---|---|
| **ROSPug** (SKU HIWND-380, "the ROS pug") | pro quadruped, **NVIDIA Jetson Nano** | ROS (version TBD — detect in Phase 1) | roslibpy → rosbridge `:9090`, MJPEG `:8080` *(hypothesis, verify)* |
| **TonyPi** (standard) | humanoid, Raspberry Pi | **no ROS** — Python SDK + JSON-RPC | JSON-RPC `:9030`, MJPEG `:8080` |

Gemini does the reasoning (spatial pointing/boxes/trajectories, task planning, success
detection); the robots supply cameras + actuators; this bridge supplies the tool
vocabulary and the loop. Intelligence in the cloud, realtime control onboard.

## 2. Corrected hardware reality (the load-bearing change from the pre-bring-up assumptions)

The quadruped is a HiWonder **ROSPug**, **not** a PuppyPi. Confirmed from the ThinkRobotics
datasheet (`docs/hardware/` PDF, extracted into `docs/hardware_notes.md`). It ships
HiWonder's PuppyPi-family PC action editor ("PuppyPi V1.0"), so the two robots are
cousins, but the differences invalidate half of the original hardware assumption:

| Aspect | pre-bring-up assumption (PuppyPi) | ROSPug reality |
|---|---|---|
| Compute | Raspberry Pi | NVIDIA Jetson Nano |
| Actuators | 8× PWM leg servos | **12× HTS-30HS serial-bus servos** (30 kg·cm, 12 V, 0.3°, temp/voltage/position readback) |
| Joint control path | `/ros_robot_controller/pwm_servo/set_state` | **serial-bus servo** port on the ros_robot_controller board (different message) |
| DOF | 8 | 12 (hip/shoulder/elbow × 4 legs) |
| Sensing | camera only | camera + IMU + **Oradar MS200 TOF lidar** (dToF, 360°/12 m, `/scan`) |
| Navigation | none | ROS **SLAM/nav stack** (Gmapping/Hector/Karto, TEB path planning) |
| Extras | — | OLED display, buzzer, key, 11.1 V 3500 mAh LiPo |

**Consequence for the design:** trust the *transport pattern* from the pre-bring-up assumptions
(rosbridge + MJPEG + `puppy_control`-style topics are a strong first guess because the same
vendor stack ships here), but treat every concrete topic/service/message/servo-ID/action
name as **unverified until Phase 1 introspects the real robot**. The
`reference/PuppyPi-ros1|ros2` clones are kept as analogy references, not ground truth for
this robot.

## 3. Architecture

```
                 ┌──────────────────── Ubuntu PC (this repo) ────────────────────┐
 mic/speaker ⇄ Live API session (gemini-3.1-flash-live-preview, sync tools)      │  ← Phase 5
      │            │ tool calls                                                   │
 text CLI ───► Agent Orchestrator ──► ER 1.6 (gemini-robotics-er-1.6-preview)     │  ← Phase 4
                   │      ▲   frames    generate_content(frame, prompt)           │
                   ▼      │             → points/boxes/trajectory/plan/success     │
             Skill Library (typed function declarations, per robot)               │
                   ▼      │                                                        │
             SAFETY LAYER (clamps · whitelist · bounded actions · watchdog · stop) │
                   ▼                                                               │
      ┌── ROSPugAdapter ───────────┐   ┌── TonyPiAdapter ───────────────┐         │
      │ roslibpy → ws://pug:9090   │   │ JSON-RPC → http://tony:9030    │         │
      │ frames: http://pug:8080    │   │ frames: http://tony:8080       │         │
      │ vocab discovered in Phase 1│   │methods per the pre-bring-up assumptions         │         │
      └────────────────────────────┘   └────────────────────────────────┘         │
```

- **One `RobotAdapter` ABC**, two implementations. Common core:
  `get_frame() -> np.ndarray`, `capabilities() -> [ToolSpec]`, `execute(tool_call) -> Result`,
  `stop_all()`, `telemetry() -> dict`.
- **Two-brain split** (unchanged from the pre-bring-up assumptions): ER 1.6 is the ~1–3 s deliberative
  layer (decides *what* and *where*, verifies outcomes); the robots' onboard gait engine /
  action groups own realtime. Never stream ER into a servo loop. Every skill = fire a
  bounded behavior → recapture frame → let ER judge → next step.
- **Topology:** plain in-process Python agent loop first (lowest latency, simplest to debug
  on hardware). MCP-server wrapper deferred to Phase 6, so other agents can drive PhysicalAgent.

## 4. Skill library (v1 scope: parity-first)

Both robots expose a symmetric skill shape; the ROSPug's lidar/SLAM **navigation is a
Phase-6 stretch**, not v1.

- **ROSPug:** `walk(direction, speed, duration_s)`, `turn(angle_deg)`,
  `set_pose(pitch, roll, height)`, `run_trick(name)` *(names from Phase-1 inventory)*,
  `stand()`, `stop()`, `look_and_point(query)`, `walk_to_object(query)` (visual-servo:
  point → turn toward x-offset → bounded walk → re-check), `check_success(criterion)`.
- **TonyPi:** `do_action(name, times)`, `walk(direction, steps)`, `turn(direction, steps)`,
  `look(pan_deg, tilt_deg)`, `find_object(query)`, `track_object(query)` (head servos follow
  ER points), `set_joint(servo_id, pulse, ms)` (safety-gated), `check_success(criterion)`.
- **Voice (Phase 5):** the Live session's tool list = the skill library + `see(question) -> str`
  (snapshots the active robot camera, calls ER 1.6, returns speakable text).
- **Routing:** commands may address "the dog"/"the humanoid"; orchestrator routes; both
  adapters can run simultaneously.
- **Stretch (Phase 6), ROSPug only:** `navigate_to(map_point)` over the SLAM nav stack and
  a lidar-obstacle-stop reflex — added once the base loop is proven on hardware.

## 5. Safety layer (non-negotiable, wraps every command)

1. **Clamps** mirroring firmware limits *before send*. ROSPug: velocity/pose/yaw limits
   **re-derived from Phase-1 introspection** (the pre-bring-up numbers are the starting
   hypothesis, e.g. ‖x‖≤35 cm/s → default ≤20, but confirm against the real driver);
   serial-bus servo pulse range per HTS-30HS. TonyPi: bus 0–1000 / PWM 500–2500, min move
   200 ms.
2. **Whitelist:** only action-group names inventoried in Phase 1 may run; joint commands
   only through vetted skills.
3. **Bounded actions:** every walk/turn is duration- or step-bounded (≤3 s / ≤4 steps per
   call); continuous motion requires re-invocation.
4. **Watchdog:** adapter-side deadman — if the orchestrator stops polling, send stop within
   1 s (ROSPug zero-velocity; TonyPi StopActionGroup; TonyPi `Heartbeat` only while a Func
   is intentionally loaded).
5. **E-stop paths:** Ctrl-C handler, `stop` text command, spoken "stop", and a
   `physicalagent stop` CLI — all call `stop_all()` on both adapters.
6. **Dry-run mode** (`--dry-run`): full loop, commands printed, nothing sent.
7. **Untrusted model output:** schema-validate + range-check every ER/Live JSON reply;
   reject-and-retry on malformed output. (Points are `[y, x]` 0–1000; boxes
   `[ymin, xmin, ymax, xmax]`.)

## 6. Phased plan (revised: software-first, then hardware bring-up)

Robots are not yet on the network, so the phases that don't need live hardware run first.
**Hardware-in-the-loop checkpoint:** stop and ask the operator before each robot first moves,
and before any new motion class.

- **Phase 0 — Scope + skeleton.** *(this doc; decisions locked)* Repo skeleton (`uv`,
  Python ≥3.10; google-genai, roslibpy, opencv-python, numpy, requests, sounddevice,
  python-dotenv, pytest), `config.py` single source of truth, `.env` handling.
  ✅ `python -m physicalagent.doctor` verifies the API key with a trivial ER 1.6 pointing
  call on a saved test image and prints the point.
- **Phase 2a — Adapters + safety (TDD, mock-first).** *(runs before Phase 1 — no hardware
  needed)* `RobotAdapter` ABC; both adapters against recorded/mock fixtures; exhaustive
  unit tests for clamps, whitelist, bounded actions, watchdog, rate limit, `stop_all`,
  TonyPi Heartbeat keepalive. ROSPug adapter is written against the §2.3 *hypothesis* with
  a clean seam so Phase-1 findings slot in.
  ✅ tests green; `--dry-run` loop prints correct commands for both robots.
- **Phase 3 — Perception.** `brain/er.py`: frame → ER 1.6 → points/boxes/trajectories
  parsed, denormalized (`[y,x]`→pixels), drawn on frame, saved to `artifacts/`. Latency +
  token-cost logging. ✅ operator visually confirms ≥8/10 pointing results on saved room images;
  latency table written.
- **Phase 1 — Hardware bring-up.** *(gated on: both robots on the dedicated router in STA
  mode)* **ROSPug:** SSH in; detect ROS distro (`rosversion -d` / `ros2 --version`, Docker
  `docker ps`); `rostopic list` + `rosmsg`/`rostopic type` to dump the **real** topic/
  service/message vocabulary; confirm rosbridge `:9090` + video `:8080` from the PC;
  enumerate serial-bus servo IDs + `/joint_states`; `ls` the on-robot action-group dir;
  probe `/scan`. **TonyPi:** JSON-RPC `echo` on `:9030`, snapshot `:8080`,
  `ls /home/pi/TonyPi/ActionGroups/`. Everything recorded in `docs/hardware_notes.md`; the
  ROSPug adapter's vocab + safety clamps are updated from findings.
  ✅ `doctor` shows both cameras' live frames saved + both control channels answering.
- **Phase 2b — Hardware smoke test (operator present).** ROSPug stand → pose wiggle → 2 s slow
  walk → stand/reset; TonyPi stand → wave → one step. ✅ Ctrl-C and `stop` halt motion
  instantly.
- **Phase 4 — Text-commanded agent loop.** `brain/orchestrator.py` (shipped as `physicalagent/shell.py` + `brain/er.py`): ER 1.6 native function
  calling over the skill library; system prompt encodes each robot's embodiment + limits +
  the observe→act→verify contract; episode logging (frames, calls, verdicts). ✅ 3 scripted
  household tasks per robot succeed ≥2/3 runs; episode logs reviewable.
- **Phase 5 — Voice.** Live session (`gemini-3.1-flash-live-preview`), PC mic 16 kHz PCM in
  / 24 kHz out, session-resumption + GoAway + context compression, push-to-talk, barge-in
  "stop" on a fast path (pending tool calls are dropped on VAD interruption). ✅ spoken
  "have the dog find the ball and go to it" works end-to-end; "stop" halts <500 ms.
- **Phase 6 — Polish + stretch.** README, cost/latency report, failure notes; then optional:
  ROSPug lidar/SLAM `navigate_to` + obstacle-stop, dual-robot cooperative task, MCP wrapper.

## 7. Repository layout

```
physicalagent/
├── physicalagent/
│   ├── config.py            # robot IPs, model IDs, limits (single source of truth)
│   ├── doctor.py            # Phase 0/1 diagnostics
│   ├── brain/er.py          # ER 1.6: point/detect/trajectory/plan/success + overlay render
│   ├── brain/orchestrator.py# function-calling loop, episode logging (shipped as shell.py + brain/er.py)
│   ├── voice/live.py        # Live API session, audio I/O, tool plumbing
│   ├── robots/base.py       # RobotAdapter ABC + ToolSpec
│   ├── robots/rospug.py     # roslibpy adapter (vocab finalized in Phase 1)
│   ├── robots/tonypi.py     # JSON-RPC adapter (+ ssh helper)
│   ├── safety.py            # clamps, whitelist, watchdog, stop_all
│   └── cli.py               # text REPL, --voice, --dry-run, stop
├── tests/                   # mock-first unit tests (Phase 2a)
├── artifacts/               # perception overlays, episode logs (git-ignored contents)
├── docs/
│   ├── hardware_notes.md    # datasheet facts + Phase-1 introspection results
│   ├── hardware/            # ROSPug datasheet PDF (git-ignored)
│   └── design/              # this spec
└── reference/               # PuppyPi-ros1|ros2 (analogy), TonyPi (authoritative) — git-ignored
```

## 8. Open items / ⚠️RECHECK at build time

- ROSPug **ROS distro** unknown (Jetson Nano stock = Ubuntu 18.04/Melodic; HiWonder may
  ship a custom image). Detect in Phase 1; the RViz screenshot in the datasheet looks ROS1.
- ROSPug **exact topic/service/servo/action vocabulary** — Phase 1 introspection is
  authoritative; §2.3 is only the starting hypothesis.
- Model preview IDs (`gemini-robotics-er-1.6-preview`, `gemini-3.1-flash-live-preview`) and
  Live-vs-Interactions-API surface — re-verify the ⚠️RECHECK items before Phase 5.
- Network peer-to-peer reachability on the dedicated router (confirm no client isolation).
```
