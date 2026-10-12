# PhysicalAgent — Multi-Robot Fleet Design

**Date:** 2026-07-21
**Status:** adopted 2026-07-22. Phases 2a–2b (TonyPi) were deliberately left
unplanned until the probe sweep could convert this document's INFERRED rows into
VERIFIED capture data.
**Supersedes:** nothing. Extends `physicalagent-design.md` from one robot to six.

Provenance markers follow `config.py` convention: **VERIFIED** (vendor source or observed),
**INFERRED** (reasoned from a spec), **UNKNOWN** (say so, never guess).

---

## 1. Scope

Make the working ROSPug control stack drive six robots, switchable with one command; then
push the agent further (fleet control, manipulation, persistent memory, lower latency).

### 1.1 The list is six robots, not seven

> "Open-Source Robotic Hand AiHand Powered by micro:bit V2" is **one product** — Hiwonder's
> AiHand, a 6-DOF hand whose controller *is* a micro:bit V2. VERIFIED.

**ROSPug · MiniHexa · MechDog · TonyPi · SO-ARM101 · AiHand**

---

## 2. Capability matrix

| | **ROSPug** | **MiniHexa** | **MechDog** | **TonyPi** | **SO-ARM101** | **AiHand** |
|---|---|---|---|---|---|---|
| Embodiment | quadruped | hexapod | quadruped | humanoid | 6-DOF arm | 6-DOF hand |
| Compute | Jetson Nano, **ROS1 Melodic** | ESP32-WROOM | ESP32-S3 | Raspberry Pi | none (host-driven) | micro:bit V2 |
| Actuators | 12 × serial-bus, 30 kg·cm @12V | 18 × PWM, 1.8 kgf·cm | 8 × coreless | 16–18 × serial-bus | 6 × STS3215 (1:345) | 5 × PWM 1.8 kgf·cm + 1 base |
| Feedback | position, temp, leg IK | none published | UNKNOWN | position, temp | **12-bit mag encoder, pos + load** | **none at all** |
| Transport | rosbridge WS :9090 | USB serial 115200 ASCII | USB-C serial + WiFi AP/STA | JSON-RPC HTTP :9030 | USB serial, Feetech bus | USB-TTL `0x55 0x55` |
| Camera | CSI 640×360 → MJPEG :8080 | optional module, **own AP, no pixels to robot** | vision module, own web page | MJPEG :8080 | none (add USB webcam) | WonderCam — **classifications only, no frames** |
| Lidar | Oradar MS200 360° | none | none | none | n/a | n/a |
| Battery telemetry | mV over ROS topic | **`A\|<mV>&` pushed @1 Hz** | UNKNOWN | UNKNOWN | n/a (mains) | board reports supply V |
| Power | 3S 11.1 V | 2S 7.4 V | 2S 7.4 V | 2S 7.4 V | 5 V/12 V PSU | 7.4 V 3 A |
| Weight | ~2–2.5 kg (INFERRED) | 0.58 kg | UNKNOWN | ~1.5 kg (INFERRED) | UNKNOWN | 0.73 kg (191 g hand only) |
| **Supervision class** | **deadman** (built) | **latching — UNSAFE** | UNKNOWN, assume unsafe | **deadman** (must build) | **tethered** | **tethered** |

**UNKNOWN worth flagging:** vendor publishes no payload figure for ROSPug, MechDog or
SO-ARM101. The commonly-cited SO-101 "~400 g" is INFERRED, not official — bench-verify it.

---

## 3. The existing code is already ~80% multi-robot ready

Measured ROSPug references per module:

| Module | refs | Verdict |
|---|---|---|
| `robots/base.py` | 0 | **Already the universal seam** |
| `robots/transport.py`, `brain/er.py`, `shell.py`, `turn_control.py`, `perception.py` | 0 | Reusable as-is |
| `safety.py` | 1 (+24 module-level constants) | Extract a profile |
| `doctor.py` | 1 | Per-robot checks |
| `cli.py` | 6 | The wiring point |

`RobotAdapter` makes **no transport assumptions** — which is exactly why six wildly different
links (ROS WebSocket, HTTP-RPC, three flavours of serial) fit behind it without violence.
`Brain` and `shell` are already robot-agnostic and need no changes at all.

**Do not over-abstract the transport.** The `Transport` ABC is ROS-shaped (publish/subscribe/
call_service). Forcing JSON-RPC or a Feetech bus into that shape would be a lie. Each adapter
owns its own link type and its own fake for tests. The shared discipline — inject the I/O,
keep the adapter pure — is what ports, not the interface.

---

## 4. Safety architecture (the load-bearing section)

### 4.1 Three supervision classes

The ROSPug deadman does not port universally. TonyPi's `Heartbeat()` guards only a loaded
"Func", not motion (VERIFIED from `RPCServer.py`). ESP32/micro:bit robots may not run our code.

| Class | Contract | Robots |
|---|---|---|
| `deadman` | Robot runs our watchdog, PC heartbeats, robot self-stops on silence. Continuous motion allowed. | ROSPug ✅, TonyPi (buildable) |
| `command_expiry` | Firmware accepts only bounded commands and stops by default. Safe by construction; no continuous motion. | MiniHexa (after firmware change), MechDog, AiHand |
| `tethered` | Non-mobile, cannot run away. Unplug = dead. | SO-ARM101, AiHand |

**A robot with no supervision story gets no motion tools.** Not negotiable.

### 4.2 🔴 MiniHexa is the dangerous one

`remote.ino` issues every movement command with **`step_num = -1` (walk forever)**, and
`Robot::move()` is non-blocking and latching. **There is no firmware watchdog and no command
timeout.** If the PC process dies or USB is pulled mid-walk, the hexapod walks until the
battery is flat. VERIFIED from vendor firmware.

A PC-side watchdog does **not** fix this — it is on the wrong side of the break, exactly the
argument already written into `physicalagent_deadman.py`.

**Fix:** flash Hiwonder's own MicroPython firmware, whose `Robot().go([vx,vy,ω], step_num,
period_ms)` accepts a **finite `step_num`** that the stock Arduino UART parser never exposes.
A small `main.py` reads framed USB commands, calls `.go()` with bounded step counts, and runs
its own silence watchdog. That converts MiniHexa to genuine `command_expiry` using
vendor-supported firmware. (⚠️ `github.com/Hiwonder/MiniHexa` has **no LICENSE file** despite
"open source" marketing — reimplement the wire protocol from spec, don't vendor their C++.)

**Decided 2026-07-22: bench-test before flashing.** Phase 4 must first confirm on the
bench that the MicroPython build drives all 18 servos and that `go(..., step_num=N)` genuinely
terminates — flashing is destructive (wipes stored servo deviations, and Arduino↔MicroPython
firmware families overwrite each other). **Until that is proven, MiniHexa is sensing-only:
no motion tools are exposed to the model.**

### 4.3 "The safe state" is embodiment-dependent

The ROSPug deadman argues, correctly for a quadruped, that *stopping is the safe state*: halt
the gait and four legs hold it statically stable. **That reasoning does not survive porting to
a humanoid.** A bipedal TonyPi frozen mid-stride is a fall.

| Policy | Meaning | Robots |
|---|---|---|
| `freeze` | cut motion, hold servo angles | ROSPug, MiniHexa, MechDog |
| `settle_to_stance` | stop the action group, then command a stable stance | TonyPi |
| `lower_and_release` | move to rest pose, then disable torque | SO-ARM101, AiHand |

Note the tension: `settle_to_stance` commands motion *while unsupervised* — which the ROSPug
deadman deliberately refuses. For a humanoid the tradeoff inverts; the risk of freezing exceeds
the risk of one short scripted settle. TonyPi ships `stand_up_front`/`stand_up_back`, so the
vendor anticipated falls.

### 4.4 Transport-arbitration hazard (MiniHexa, and check MechDog)

MiniHexa latches UART → WiFi → BLE and **silently ignores lower-priority transports**. A phone
connecting over BLE kills your USB control link with no error. Preflight must assert no
`hiwonder` SSID in range and no BLE peer attached. VERIFIED.

---

## 5. Architecture: the RobotProfile registry

```python
@dataclass(frozen=True)
class RobotProfile:
    name: str                     # "rospug" -> ./robot --robot rospug
    embodiment: str               # quadruped|hexapod|humanoid|arm|hand
    supervision: SupervisionClass # deadman | command_expiry | tethered
    safe_state: SafeStatePolicy   # freeze | settle_to_stance | lower_and_release
    safety: SafetyProfile         # ALL limits — replaces the 24 module-level constants
    link: LinkSpec                # host/port | serial device/baud
    adapter_factory: Callable     # (link, guard) -> RobotAdapter
    doctor: Callable              # per-robot preflight
    embodiment_prompt: str        # robot-specific system-prompt fragment
```

`safety.py` changes from reading module globals to taking a `SafetyProfile`. Everything else
is untouched. **This is the whole "single command change".**

```bash
./robot                    # default from .env
./robot --robot tonypi     # <- the switch
./robot probe              # detect what's plugged in, suggest a profile
./robot fleet              # coordinator over all connected robots
```

`VENDOR_MAX_*` moves into each profile but keeps its invariant status — the existing
`TestVendorCeilingsAreNeverExceeded` pattern is replicated per robot.

---

## 6. Canonical tool vocabulary

One vocabulary, capability-gated, so the model's skills transfer when you switch robots.

| Tool | Gate |
|---|---|
| `get_status`, `stop` | universal |
| `look` | has camera |
| `scan_surroundings` | has lidar |
| `walk`, `turn`, `strafe` | mobile |
| `set_pose` | posture-capable |
| `do_action(name)` | has action groups |
| `move_arm_to(x,y,z)`, `set_gripper(pct)`, `home_arm` | arm |
| `set_fingers(...)`, `gesture(name)` | hand |
| `run_skill(name)` | has a trained VLA policy (§7) |

Units stay **cm and degrees** at the boundary; the adapter converts to SI before the guard.
The adapter declares its subset; the CLI builds the tool list from it.

---

## 7. Three control tiers

Extends the existing two-brain split.

| Tier | Rate | Role | Tech |
|---|---|---|---|
| 1 Deliberative | 1–3 s | task planning, spatial reasoning, tool choice | Gemini ER 1.6 / 3.5 Flash |
| 2 Visuomotor | ~30 Hz | learned manipulation skills | **GR00T N1.7-3B** |
| 3 Realtime | kHz | gait, balance, servo loops | vendor firmware |

**Tier 2 choice — GR00T, not Gemini's VLA.** Only Gemini's *reasoning* model is API-accessible;
the Gemini Robotics VLA is partner/on-device only. GR00T N1.7 is **GA, Apache-2.0, on HF, and
integrated into LeRobot**, with first-class SO-100/SO-101 support already present in the local
`Isaac-GR00T` checkout (`examples/SO100/`, `so101_config.py`, `eval_so100.py`). Local RTX 5070 Ti
(12 GB) is adequate for inference; fine-tuning a 3 B model on 12 GB will need LoRA/quantisation
or a rented GPU — **verify before committing.**

**LeRobot vs our RobotAdapter — complementary, not competing.** LeRobot's `Robot` is
`get_observation()`/`send_action()` dicts at 30 Hz in joint space, built for policy learning.
Ours is semantic skills with clamping and refusals, built for an LLM. For the arms, our
adapter's *implementation* wraps `SO101Follower` → free Feetech bus handling, calibration
persistence, normalisation. Pipeline: record demos with LeRobot → fine-tune GR00T → adapter
exposes `run_skill("pick_up_block")` as **one** semantic tool Gemini can call.

---

## 8. Fleet controller

Pattern: **centralized deliberation, decentralized execution, event-driven re-planning.**

- **Coordinator** (Gemini 3.5 Flash — built for agentic/parallel tool use) gets *task-level*
  verbs only: `list_robots`, `assign_task(robot, task)`, `get_fleet_status`,
  `wait_for(robot)`, `stop_robot`, `stop_all`, `send_message(from,to,text)`.
- **Per-robot Brain** — the *existing* `Brain`, one instance per robot, own conversation, own
  heartbeat, own deadman, own thread.
- Robots run **in parallel** — essential when each model call costs 1–3 s.
- Failure isolation: one robot's crash cannot stop another's supervision.

**Rejected: one flat conversation with namespaced tools** (`rospug.walk`, `tonypi.wave`).
9 tools × 6 robots = 54 tools, serialized execution, and — decisively — a single failure mode
takes down supervision for the whole fleet.

**`stop_all()` fans out directly, never through the coordinator model** — mirrors the existing
shell `stop` design.

---

## 9. Physical integration — verdicts

### 9.1 SO-ARM101 + AiHand → ❌ **a downgrade, don't do it**

The SO-101's stock gripper (STS3215, ~19 kg·cm, magnetic encoder, position **and load**
readback on the arm's own bus) beats the AiHand on every axis that matters.

The AiHand is Hiwonder's `uHand` in a new box: 5 × PWM hobby servos (1.8 kgf·cm), **rigid
linkage fingers, fixed thumb opposition, acrylic frame, and no feedback of any kind** — no
position, no force, no tactile, no current. Vendor publishes **no grip force, no max object
weight, no span** for any uHand variant, and leaves reviews on its own store saying *"the
robotic hand cannot really grasp something."*

Costs of swapping: ~150 g net at a **2.7× longer moment arm** (~3.8 kg·cm added static wrist
moment vs a ~6.5 kg·cm continuous rating → sag, thermal derating); electrically incompatible
(Feetech bus @1 Mbaud vs PWM; **12 V destroys an LFD-01M**) so it needs its own USB device and
power rail; and it **breaks LeRobot** — `SO101Follower` expects 6 motors with `gripper` at id 6,
so changing the action space invalidates existing SO-101 datasets and pretrained policies.

**Two defensible narrow uses:** proving the adapter abstraction spans a 1-DOF jaw *and* a 5-DOF
hand; and HRI/expressive gesture work, where it is genuinely good. If a real hand is wanted
later: Inspire RH56DFQ or LEAP Hand (10–20× the price — that gap is real).

### 9.2 SO-ARM101 on the ROSPug's back → ❌ **not for walking**

- MiniHexa (0.58 kg, 1.8 kgf·cm servos): **zero payload.** Absolutely not.
- TonyPi (humanoid): balance margin is the constraint, not torque. No.
- ROSPug: the only candidate, and still poor. An SO-ARM101 plus its power is a large fraction
  of the robot's own mass, mounted **high**, which hurts a walking quadruped more than the mass
  alone — and this robot already scrubs on carpet with unreliable odometry.

**Better answer, and it's native:** the ROSPug's `ros_robot_controller` board exposes an
**unused PWM servo port** (`/ros_robot_controller/pwm_servo/set_state`, with position/offset/
duration and a readback service — VERIFIED in vendor source). HiWonder sells a **2-DOF PWM arm
attachment** for this robot family, and the PuppyPi vendor source ships `puppy_with_arm` +
`ArmMoveIK` for exactly that. So: small PWM arm on the pug (native, no extra compute, rides the
existing rosbridge transport and safety layer), SO-ARM101 stays a fixed station arm.

⚠️ The ROSPug vendor source has **no** arm support (unlike PuppyPi) — the ROS-side integration
would be ours to write, though the PuppyPi code is a direct template.

**Vendor trick worth stealing:** `puppy_with_arm` pitches the *body* down (pitch −17°,
height −10) to extend reach. Arm and body share one workspace — a `reach_toward` tool should
compose body pose with arm IK, not treat them separately.

**If you still want to try it:** measure, don't guess. Add ballast in 100 g steps and watch
gait quality and battery sag — the existing `_drive_for()` sag detector is already the right
instrument.

### 9.3 Connecting robots via the Jetson's USB ports → ✅ **for data, ❌ for power**

The Jetson Nano has spare USB and can absolutely host a Feetech bus adapter or a USB-TTL link.
Data is not the problem. **Power is**: servo buses want amps at 6–12 V that the Jetson cannot
source. Any attached arm needs its own supply. Also note this makes the *pug* the host, so its
compute and battery now carry the arm's workload.

---

## 10. Charging manager — staged, hardware-first

### 10.1 The inversion: mostly, don't use the arm

Two findings collapse this problem:

1. **Magnetic + pogo-pin docks self-align to ~0.15 mm** and forgive angular error. Robotic
   connector insertion (peg-in-hole) needs compliance hardware and still reaches only ~93% in
   research conditions — 1-in-14 failures, unacceptable for LiPo.
2. **The ROSPug can power down its own servos in software** — `bus_servo_enable_torque(id,
   False)`, VERIFIED in the board SDK. The largest draw goes away with no switch flip.

So the task collapses to something with no manipulation in it:

> low battery → navigate to dock via **AprilTag visual servo** → seat on pogo contacts →
> `bus_servo_enable_torque(all, False)` → charge → wake on voltage threshold

AprilTag matters specifically because rf2o translation is unreliable (0.5×–2.5×) — tags give
exact 6-DoF ground truth, and the detector is **already running on the ROSPug**.

### 10.2 Safety — make the failure impossible, not detected

This mirrors the principle this project already proved: *the guarantee must not depend on the
model behaving.*

1. **One dock per robot, each with its own permanently-wired correct charger.** The arm never
   selects a charger, so "wrong charger" stops being a thing the VLM must catch.
2. **Physically unique connectors per voltage class** (3S 11.1 V vs 2S 7.4 V) — wrong mate is
   mechanically impossible.
3. **Relay/smart-plug in series**, cut on voltage-out-of-envelope, over-current, or over-time.
4. **Absolute charge-time ceiling + thermal watch.**
5. **The VLM may only WITHHOLD permission, never grant it alone.**
6. **No unattended overnight charging in v1.**

### 10.3 Staging

- **v1** — passive pogo dock, AprilTag self-docking, software torque-off. **No arm.**
- **v2** — arm flips a physical switch for robots that need one (±2–3 mm — achievable).
- **v3** — arm handles exceptions: re-seat a misaligned robot, press a recessed button.

The SO-ARM101's real value is manipulation research (§7), not being a charging robot.

---

## 11. Smarter and smoother

**Interaction latency**
- **Gemini context caching** — the system prompt and tool declarations repeat every hop;
  ~90% cost reduction and lower time-to-first-token.
- **Streaming function calling** — partial args stream, so we can act sooner.
- **Multimodal function responses** — `look` returns its image *inside* the function response
  instead of a separate user Part.
- Speculative sensing: fetch camera + lidar concurrently before the model asks.
- The `elapsed_s`/`think_s` instrumentation already exists but has never been analysed —
  **do that first**, before optimising anything.

**Motion smoothness**
- Velocity ramping instead of step-command → hard stop.
- Command blending: skip the full stop between consecutive same-direction moves.
- A 10 Hz reactive layer owning obstacle-stop while the model sets intent.

**Smarter**
- **Persistent spatial memory — AprilTag-anchored topological map.** Tags on room landmarks;
  each tag is a named place; objects associate to the nearest tag. Robust precisely because it
  sidesteps the odometry defect. *Alternative:* enable the vendor's `move_base` + gmapping +
  AMCL (present, not running) for a metric map and free `navigate_to(x,y)` — richer, but its
  launch files duplicate the camera and kill running nodes, so it needs the same surgical
  include-extraction already used for rf2o.
- Self-calibration: the 1.22 turn factor was hand-measured — make the robot re-derive it.
- Learned macros: successful sequences saved as new named tools.

---

## 12. Bring-up kit — `./robot probe`

You will plug robots in one at a time. Generalize `grab_everything.sh` into a transport-agnostic
probe that writes a capture folder: USB/serial enumeration by VID:PID (micro:bit DAPLink
`0d28:0204`, CH340 `1a86:7523`, CP2102 `10c4:ea60`, FTDI `0403:6001`), ROS graph dump, HTTP port
scan, BLE scan, firmware/version strings, and a timed telemetry sample.

Embodies this project's own rule: **artifacts cannot be re-derived from insight — copy first,
analyse later.** Output feeds directly into writing that robot's profile.

**Per-robot bring-up order:** probe → write profile with limits marked UNVERIFIED → doctor
passes → sensing-only Gemini run → supervised first motion → unlock.

---

## 13. Phased roadmap

| Phase | Deliverable | Gate |
|---|---|---|
| **0** | Extract `RobotProfile`/`SafetyProfile`; ROSPug becomes profile #1 | 244 tests still green |
| **1** | `./robot probe`; `--robot` switch; profile registry | switching works with one robot |
| **2a** | **TonyPi deadman** — robot-side watchdog daemon + `settle_to_stance` policy | heartbeat cut on a *walking* humanoid leaves it standing, not fallen |
| **2b** | **TonyPi** adapter (JSON-RPC transport, 117 action groups) | Gemini drives TonyPi; instant `stop` works |
| **3** | **SO-ARM101** adapter via LeRobot `SO101Follower` + leader teleop | arm moves under Gemini; torque-off on exit; teleop records a demo |
| **4** | **MiniHexa** — bench-test MicroPython build, then firmware w/ bounded `step_num` + watchdog | robot stops itself when the PC dies |
| **5** | **MechDog**, **AiHand** adapters | — |
| **6** | Fleet coordinator | two robots cooperate on one task |
| **7** | GR00T tier-2 on the SO-ARM101 | one learned skill callable as a tool |
| **8** | Charging dock v1 | robot self-docks and charges unattended-but-supervised |

**Ordering — decided 2026-07-22: TonyPi first.** Rationale for and against, recorded so
it is not relitigated:

*For:* we hold the complete TonyPi vendor source, so it is not a blind bring-up. It exercises a
genuinely different transport (JSON-RPC over HTTP vs rosbridge WebSocket), which is the strongest
possible test that `RobotAdapter` really is transport-agnostic. And it is a second *mobile* robot,
which is where the value is.

*Against, and therefore mitigated:* TonyPi is the higher-risk first port. Unlike the tethered arm
it can run away, it has **no motion deadman today** (`Heartbeat()` guards only a loaded Func), and
as a biped its safe state is not "freeze". So **phase 2a is split out and gated ahead of 2b** —
the deadman and the humanoid safe-state policy land and are proven on hardware *before* the
adapter exposes a single motion tool. First supervised motion happens only after 2a passes.

---

## 14. Decisions and remaining questions

### 14.1 Decided 2026-07-22

| # | Decision | Consequence |
|---|---|---|
| 1 | **Fleet is six robots.** AiHand is one micro:bit-powered product. | Matrix in §2 is final. |
| 2 | **SO-ARM101 is a leader + follower pair.** | Teleoperation is available → hand-guided demo recording → the **GR00T training path in §7 is open**, not blocked. Phase 3 includes a teleop record as its gate. |
| 3 | **MiniHexa: bench-test the MicroPython build before committing.** | Phase 4 gains an explicit verification step: confirm the MicroPython firmware drives all 18 servos and that `go(..., step_num=N)` really terminates, **before** flashing for real. Save servo deviations first — flashing wipes them. Until proven, MiniHexa gets sensing tools only. |
| 4 | **TonyPi first**, ahead of the SO-ARM101. | §13 reordered; deadman work split into its own gated phase 2a. |

### 14.2 Settled by research, recorded so they are not re-asked

- Arm + hand combo is a **downgrade** (§9.1).
- SO-ARM101 **cannot ride the ROSPug while walking**; use the native PWM-arm path (§9.2).
- **GR00T, not Gemini's VLA**, for tier 2 — availability, not quality (§7).
- A flat multi-robot conversation is the **wrong** fleet architecture (§8).
- Charging should **not** go through the arm in v1 (§10).

### 14.3 Still open — needed later, not blocking the plan

1. **MechDog kit tier** (Standard/Advanced/Ultimate) and whether the vision module is present.
2. **AiHand chirality** — left or right? Separate SKUs with mirrored servo deviations.
3. **Fleet size** — how many robots running at once? Drives USB vs WiFi topology.
4. **Charging** — willing to buy/print pogo docks and fit contacts to each robot?
5. **GPU budget** — GR00T N1.7-3B *inference* fits the RTX 5070 Ti (12 GB); **fine-tuning likely
   does not** without LoRA/quantisation. Bigger GPU or cloud budget available?
6. **ROSPug arm** — willing to buy the HiWonder 2-DOF PWM attachment and print adapter plates?
