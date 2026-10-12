# Alternative approaches — parked options for when hardware returns

Every path we considered and did **not** take, why, and what would make us revisit.
Nothing here is a recommendation to act now; this is a menu for future sessions so
that no option has to be rediscovered. Written 2026-07-17 after the first ROSPug
bring-up.

Status key: **PARKED** (viable, not now) · **REJECTED** (with reason) · **PLANNED** (in the current plan)

---

## 1. Resolving the units/limits blocker

The single blocking unknown: **is `pug_control/Velocity.x` in cm/s or m/s?** It gates
the safety clamps (see `config.py` for why the ambiguity is asymmetric and dangerous).
Options, best first:

### 1a. Copy the on-robot source — **PLANNED**
`/home/hiwonder/pug/src/pug_driver/pug_control/` contains the node that consumes
`Velocity`. Its clamp code settles units, limits, and semantics permanently.
`scripts/grab_everything.sh` does this first thing. **Cost: seconds. Value: total.**
This is the intended path; everything below is a fallback.

### 1b. Ask HiWonder directly — **PLANNED (do in parallel)**
They have the robot right now to finish setup. Ask for: the `pug_control` source or
docs, the action-group name list, and whether the camera fault is known. Zero
engineering cost, may resolve everything. Costs nothing to ask while they hold it.

### 1c. Empirical ramp on hardware — **PARKED (fallback)**
Robot on a stand, wheels/feet clear of the ground. Command `x = 0.1`, then `0.2`, `0.5`,
`1.0`, measuring actual foot/gait speed at each. The response curve reveals the unit.
Viable but consumes a hardware window and needs the robot physically safe.
**Only if 1a and 1b both fail.**

### 1d. Read limits off the joystick node — **PARKED**
`/joystick_control` already maps a gamepad to velocity commands. Whatever maximum it
emits is, by construction, a safe real-world value. Subscribe to
`/pug_control/velocity_move` while a human drives the robot with the bundled wireless
handle, and record the range. Elegant — it extracts the vendor's own idea of "max"
without reading code — but needs hardware plus the handle.

### 1e. Infer from the URDF/Gazebo config — **PARKED**
`pug_description` holds the URDF; Gazebo controller configs often carry velocity
limits. Weaker evidence than the control node itself, but it arrives free with 1a.

---

## 2. Developing without hardware

### 2a. Mock adapters + recorded fixtures — **PLANNED**
Mock at the `RobotAdapter` boundary; replay the real captured messages in
`tests/fixtures/rospug_observed_2026-07-17.json`. Fast, no dependencies, and the
message shapes are real. **This is the current plan.** Limitation: it tests our code,
not our understanding of the robot — a mock built on a wrong assumption passes happily.
That's precisely what the hardware contract tests (§3a, planned) exist to catch.

### 2b. Fake rosbridge server — **PARKED**
Stand up a local websocket server that speaks the rosbridge protocol and replays the
captured graph + a JPEG loop. Higher fidelity than 2a because it exercises the real
`roslibpy` client path (connection handling, reconnects, service-call plumbing) rather
than stubbing it out. Worth doing **if roslibpy integration bugs start eating time** —
otherwise it's a simulator we'd have to maintain. Medium cost, real payoff for
connection-layer bugs.

### 2c. Gazebo simulation of the real stack — **PARKED (strongest, most expensive)**
The ROSPug genuinely ships sim support: `pug_description` (URDF), `pug_control/enable_sim`
(a real service), and Gazebo `ros_control` topics
(`/pug/{lf,lb,rf,rb}_{joint,thigh,calf}_position_controller/command`). Running HiWonder's
actual stack in Gazebo would de-blind *everything* — units, limits, gait behaviour,
action groups — with zero risk to real hardware and no battery drain.

Blockers: (1) we don't have the source (needs 1a first); (2) it's ROS1 **Melodic** on
**Ubuntu 18.04**, and this PC has no ROS and runs a much newer distro → needs Docker
(`osrf/ros:melodic-desktop-full`) + Gazebo + an X socket or VNC; (3) HiWonder's sim may
expect Jetson-specific bits.

**Revisit the moment we have the source tree.** This is the highest-value parked item:
it converts every future "will this work?" into something testable offline, and it's the
only way to test *motion* logic without risking the robot. Estimated cost: a day to
stand up; saves far more if the project runs long.

### 2d. Buy/borrow a second robot — **REJECTED**
Cost and lead time dwarf the benefit for a one-robot project.

---

## 3. Guarding against wrong assumptions

### 3a. Hardware contract tests — **PLANNED**
A fast (<60s) pytest suite marked `@pytest.mark.hardware` that asserts every assumption
baked into `config.py` against a live robot: does `/pug_control/velocity_move` exist? does
its type match? does `run_action_group` accept a name? Read-only, no motion.
Turns each hardware window from exploratory into confirmatory, and catches vendor
firmware/image changes.

### 3b. Schema-validate every model reply — **PLANNED**
ER 1.6 is preview; treat all output as untrusted. Validate + range-check, reject-and-retry
on malformed JSON. Already in the design spec §5.7.

---

## 4. Topology

### 4a. Plain in-process Python loop — **PLANNED (chosen)**
Lowest latency, simplest to debug at the hardware boundary. Chosen for v1.

### 4b. MCP server wrapper — **PARKED (Phase 6)**
An MCP-server wrapper, so other agents can drive PhysicalAgent.
Deferred deliberately: it's a wrapper over a working core, so it costs little to add
later and would slow the first physical demo if done now.

### 4c. `robotmcp/ros-mcp-server` — **PARKED**
An off-the-shelf PC-side MCP server over rosbridge :9090, ROS1-tested, with a Gemini
example. Could shortcut 4b entirely. Worth an evaluation spike when we reach Phase 6
rather than hand-rolling. Risk: generic ROS introspection ≠ our safety-gated skill
vocabulary, so we'd likely still need our own safety layer in front.

### 4d. nasa-jpl/ROSA — **REJECTED**
LangChain ROS agent; needs a rospy environment on the PC (we deliberately have none —
roslibpy exists precisely so the PC needs no ROS). Heavy. Revisit only if generic ROS
introspection becomes a goal in itself.

---

## 5. Networking

### 5a. Wired, PC-as-router (`ipv4.method=shared`) — **PLANNED (chosen, proven)**
Worked first time. PC keeps Wi-Fi internet (Gemini API) while the robot is on ethernet,
and the robot gets NAT'd internet for free (useful for `apt`/`pip` on a fresh image).
Zero robot-side config. Profile `robot-wired` on `enp132s0`, `autoconnect=no` so it never
fights the existing `g1-wired` (Unitree, static 192.168.123.222).
**Caveat: one robot at a time** — fine for single-robot work, insufficient for the
dual-robot demo.

### 5b. Cheap router / unmanaged switch — **PARKED (needed for dual-robot)**
The path to running ROSPug + TonyPi simultaneously. PC on ethernet to the router (or
Wi-Fi), both robots in STA mode. **This becomes necessary for the Phase-6 dual-robot
cooperative task**, and for any single-robot work if the cable is inconvenient.

### 5c. PC as Wi-Fi AP — **PARKED**
A `Hotspot` profile (SSID `<PC_HOTSPOT>`, `ipv4.method=shared`) already exists on this PC and
supports AP mode. Both robots could join it, giving dual-robot without buying a router.
Catch: the PC's single Wi-Fi adapter can't be an AP and a site-network client at once, so the
PC would need **wired internet** for the Gemini API. Good option *if* a wired internet
drop is available.

### 5d. Robot's own AP (`HW-XXXXXXXX` / `<FACTORY_DEFAULT>`, 192.168.149.1) — **PARKED (fallback + the only untethered-walking path until STA is set up)**
CONFIRMED from the ROSPug manual (2026-07-20): on boot it broadcasts `HW-XXXXXXXX`
(device-unique 8 hex chars, e.g. `HW-XXXXXXXX`), password `<AP_PASSWORD>` (HiWonder factory default), robot at
`192.168.149.1`. Zero setup, no extra hardware, and — importantly — it works while the
robot is **walking** (wired can't). But the PC loses internet (single Wi-Fi radio → no
Gemini API) unless the PC has a wired internet drop, and both robots share the same AP IP
so they can't coexist on their own APs. **Also the standard way to reach the robot in order
to configure STA mode (5b).** Fine for untethered single-robot work where perception can
tolerate no cloud, or as the on-ramp to STA.

### 5e. The site Wi-Fi network (`<SITE_SSID>`, a /20) — **REJECTED**
A managed `/20` we don't control; client isolation would likely block PC↔robot silently.
Not worth the debugging time.

---

## 6. Camera

### 6a. `image_raw` direct from gscam — **PLANNED (chosen)**
Skips `image_proc` entirely. Chosen because `image_proc` proved fragile (name-collision
churn during our restart) while `image_raw` published reliably. We don't need
rectification for Gemini pointing — ER works fine on raw wide-angle frames.

### 6b. Rectified `image_rect_color` — **PARKED**
Needed only if lens distortion measurably hurts pointing accuracy, or if we ever convert
image points to metric robot-frame coordinates (where rectification + `camera_info`
matter). Revisit if Phase 3 pointing accuracy disappoints — the IMX219 is wide-angle, so
barrel distortion at the frame edges is a plausible error source.

### 6c. `/compressed_image` — **PARKED**
`image_proc` is launched with `image_rect_color/compressed` remapped to
`/compressed_image` (it's what the WonderPi app consumes). Lower bandwidth. Irrelevant on
a wired link; potentially useful over Wi-Fi.

### 6d. Camera liveness strategy — **PLANNED**
We watched `gscam` publish for ~79s then cleanly self-exit, root cause unknown. The
adapter must detect frame starvation and react. Options, in preference order:
- **Detect + fail loudly** (chosen for v1): frames stop → raise, halt the loop. Never let
  the agent act on a stale frame.
- **Auto-restart gscam over SSH** — parked; effective but hides the fault and needs SSH
  in the hot path.
- **Fix the root cause** — best, but needs hardware + the source. Suspect the Argus mode
  mismatch (mode 5 = 1280x720@120fps selected for a 640x360@30 request).

---

## 7. Voice (Phase 5)

### 7a. Gemini Live API, 2-model split — **PLANNED**
ER 1.6 has no Live support, so voice needs `gemini-3.1-flash-live-preview` with the skill
library as sync tools + a `see()` tool that delegates to ER.

### 7b. Google's GA "Interactions API" — **PARKED ⚠️RECHECK**
Google's docs banner now recommends it over the preview Live API. **Re-verify which is the
right realtime surface before building Phase 5** — this may have changed again by then.

### 7c. Local STT/TTS (Whisper + Piper) — **PARKED**
Removes cloud latency/cost from the voice path and works offline. Loses Live's native
audio↔audio quality and barge-in handling. Only if Live proves too slow or costly.

---

## 8. Rejected outright

- **Streaming ER into a tight servo loop** — 1-3s round trips vs a gait engine needing
  kHz. The two-brain split exists for exactly this reason. Non-negotiable.
- **Trusting the pre-bring-up (PuppyPi) assumptions for the ROSPug** — empirically refuted on 2026-07-17
  (`puppy_control` → `pug_control`). Kept only as analogy for *limits* we haven't verified.
- **Trusting robot-side timestamps** — the ROSPug's clock was 3 years off (no RTC battery).
  Stamp PC-side always.
