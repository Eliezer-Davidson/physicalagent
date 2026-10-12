# G1 control tiers — LOCKED / NORMAL / FULL / TWO-KEY

A safety policy for a 35 kg humanoid, written before any of it was built and
frozen into code by checksum. It is the design document behind
`physicalagent/safety.py`, `physicalagent/profiles/g1.py` and
`tests/test_g1_locomotion_plane.py`, which reads the locomotion row of the
table below and fails if the code and the table ever disagree — in either
direction.

Extracted from a longer design document; the tier policy is reproduced
here unchanged apart from removing references to machines and material that are
not part of this repository.

---

## 1. THE TIER TABLE — LOCKED / NORMAL / FULL / TWO-KEY

### 1.0 What a tier is, and what it is not

A tier is a **refusal ceiling**: the widest command the stack will transmit. It
is never evidence of progress — a robot that does nothing violates no limit at
any tier. Every gate artifact therefore pairs its limit log with a
commanded-vs-achieved trace showing the motion actually happened.

The ladder is strictly nested: every envelope at a tier contains the envelope
below it; every rate floor is monotonically relaxed. Raising a tier can only
widen range and speed — never disable a protection, never change a safe-state
policy (ported from `tonypi_servo.py`'s ladder invariant and its
PROTECTION_OVERRIDE seam).

Tier selection is explicit (profile field / env var / CLI flag), defaults to
LOCKED, and an unknown tier name **raises** — a typo must never relax a clamp
(port of `ServoTier.resolve`).

**Tier limits do not bound firmware built-ins.** An FSM id is a behaviour, not a
command envelope — invoking one steps outside every numeric limit in this table
by construction. That is why built-ins live behind a whitelist and a probe
protocol (§1.2 note e), not behind the slew clamps.

**Raw full-joint control is not on this ladder.** See §1.3.

**All rates are specified in rad/s (or units/s) and converted to per-tick values
through one tested constant `TICK_HZ = 50`.** The vendor's low-level example runs
at 500 Hz (`g1_low_level_example.py:79`, `control_dt_ = 0.002`) — ten times the
arm_sdk example's 50 Hz — so a per-tick number silently reused across planes
would multiply real angular velocity by 10. The constant is pinned by test.

### 1.1 Arming preconditions (any tier above LOCKED)

Two lists, deliberately separate. The first is machine-checked — code refuses to
arm. The second is operator-attested — a named human logs the attestation with a
timestamp, and arming refuses if the attestation record is absent. Attestations
are not machine checks and this table does not pretend they are.

**A. Machine-checked by the doctor (any failure refuses arming):**

| Check | Pass condition |
|---|---|
| P1 sign-off artifact | state-decode sign-off file exists with non-empty signer name and date |
| lowstate fresh AND live | messages arriving, age < 200 ms over a 5 s window, **and** changing-field liveness: the tick counter increments and consecutive IMU frames are not bit-identical. Age alone is farmable: a wedged firmware republishing one frozen frame reads "fresh" forever. |
| arbiter ownership | ControlOwner reports exactly one owner == us. `MotionSwitcherClient.CheckMode` (`motion_switcher_client.py:26`) is consulted **fail-closed only**: a mismatch refuses arming; a match is necessary, never sufficient — CheckMode's truthfulness on this firmware is unverified until the P1/P3 cross-validation (observe a real `mode_machine` transition in lowstate across a mode change) is signed. Until then its output is a claim, not a fact. |
| host network posture | DDS bound to the dedicated robot interface; that interface has no default route. This checks OUR host only. The robot's own radios (AP/BLE — the UniPwn surface) are a P1 verification item and a per-session attestation, not something this check can see. |
| software e-stop | latch → refuse → explicit-reset cycle exercised this session (pure refusal logic, testable at LOCKED) |
| session recording | command log actively being written; recording process running (existence — where the camera points is attestation B) |
| motion tokens | the propose-and-confirm token subsystem is active (see table row) |

**B. Operator-attested (name + timestamp logged; absent record refuses arming):**

- named operator physically present
- clearance per tier (2 m NORMAL / 4 m FULL and probes / gantry bay two-key)
- camera actually aimed at the robot
- robot radio state (AP/BLE) as verified at P1
- stand/bench placement, when the capability requires it
- two-key only: gantry bearing the robot; e-stop holder present and distinct
  from the operator; vendor remote present and **exercised this session**

### 1.2 The table

"—" = the capability's writer/client object is **not constructible** at that
tier — the object is never created, not "created but refuses".

| Capability | LOCKED | NORMAL | FULL | TWO-KEY (§1.3) |
|---|---|---|---|---|
| **What holds the robot up** | firmware (we never touch motors) | firmware balance — legs never commanded | firmware; SONIC only after a named human signs the P7/P8 results | **NOTHING — gantry mandatory** |
| Speech / volume / LED — `AudioClient` (`g1_audio_client.py`: `TtsMaker` :28, `SetVolume` :47, `LedControl` :54). An RPC client with no motor authority — which is why it coexists with LOCKED's no-motor-objects invariant. | ✔ | ✔ | ✔ | ✔ |
| `rt/lowstate` subscribe | ✔ | ✔ | ✔ | ✔ |
| Motor-command capable objects | **—** (none constructible; AudioClient is the only command-capable client at LOCKED) | arm_sdk writer + `LocoClient` restricted surface | + velocity surface + probe protocol | + raw `rt/lowcmd` writer |
| Motion execution authority (v1 default, all tiers) | n/a | every motion tool call executes only against a **fresh single-use operator token** (propose-and-confirm). Machine-checked. Revisit only after P4 evidence, as a signed change. | same | same, plus the grant |
| **arm_sdk blend weight** `motor_cmd[29].q` | — | ≤ 1.0; ramp ≤ 0.5/s (full engage ≥ 2 s) | ≤ 1.0; ramp ≤ 1.0/s (≥ 1 s) | n/a — plane released before raw ownership |
| arm_sdk commanded joints | — | arm targets 15–28 only. Waist 12–14: **hold-at-engage** — vendor gains, q frozen at engage, zero slew (the vendor example demonstrates only nonzero-gain rows with explicit q, `arm_joints` :91–103; zero-gain-row blend semantics are UNCHARACTERISED and are not used at any tier). Legs 0–11 never carry commands. | arm targets 15–28 + waist targets 12–14 (vendor-demonstrated; WaistRoll/Pitch refused if the profile says locked-waist variant, per `g1_arm7_sdk_dds_example.py:41-44`) | n/a |
| arm_sdk `q` targets | — | inside vendor joint-limit table minus 0.1 rad margin (table checksummed, provenance line per row) | same | scope-defined; v1 scope is hold-current-pose |
| arm_sdk `q` slew | — | ≤ 0.5 rad/s (= 0.01 rad/tick at TICK_HZ 50) | ≤ 2.0 rad/s (= 0.04 rad/tick) | 0 (hold-pose) |
| arm_sdk `dq`, `tau_ff` | — | both forced 0 | both forced 0 | per signed scope; v1: both 0 |
| arm_sdk gains | — | kp ≤ 60, kd ≤ 1.5 (the vendor example's own values, `g1_arm7_sdk_dds_example.py:74-75`) | kp ≤ 60, kd ≤ 1.5 — FULL widens range and speed, never stiffness | kp ≤ 60, kd ≤ 2.0, per signed scope |
| Locomotion velocity commands (`LocoClient.SetVelocity` :58 / `Move` :107) | — | **surface not constructible** — no stepping at NORMAL | \|vx\| ≤ 0.3 m/s, \|vy\| ≤ 0.2 m/s, \|ω\| ≤ 0.5 rad/s as command ceilings, provisional until the P5b-measured envelope; P5 results may only LOWER them | — (no locomotion under raw, v1) |
| Firmware FSM built-ins | — | whitelisted **statically-stable** ids only (e.g. Damp, Sit — after each is characterised). Whitelist starts EMPTY. | + whitelisted dynamically-stable ids that earned a P5b (gantry) envelope; + the probe protocol (note e) for uncharacterised ids | — |
| Damp — the de-facto stop (`LocoClient.Damp` :75) | **—**. No command client exists at LOCKED; the vendor remote is the only stop. Say so out loud at session start. | software Damp always-allowed once constructed; its tool result states: "damping engaged — Unitree documents that the robot then slowly falls to the ground." Never labelled a no-cost stop. | same | operator-invoked; gantry takes the load; **reachability of Damp after `ReleaseMode` is unverified and is a P9 entry measurement** |
| Inspire hands | — | bench only (attested bench placement + reported body mode == damp): normalized travel 0.30–0.70; min move time 800 ms/command | bench: full travel 0.0–1.0; min move time 300 ms. On-body only after P6 sign-off. | n/a |
| Clearance | n/a | 2 m, no bystanders | 4 m, no bystanders | gantry bay only |
| Humans | — | named operator | named operator | operator + dedicated e-stop holder (distinct person) |

Notes bound to the table:

- **(a) Waist at NORMAL is hold-only** because the 3-DoF waist moves torso mass
  above the support polygon; arms-only is the smallest useful envelope. The
  hold-at-engage form (vendor gains + frozen q) is used instead of kp=0 rows
  because zero-gain-row semantics under blend are uncharacterised — if zero-gain
  meant zero-torque, the smallest tier would un-stiffen the waist. **P4 entry
  criteria (unconditional):** (i) bench/sim characterisation of what firmware
  does with a deliberately stalled arm_sdk stream — hold, drift, or drop to
  damp; (ii) verification that legs are unaffected by `rt/arm_sdk` message
  contents; (iii) blend semantics of held rows confirmed in sim.
- **(b) The safe state on the arm_sdk plane is the handover ramp — PROVISIONAL.**
  `stop` on this plane = freeze q targets, ramp w → 0 at ≤ the tier's fastest
  rate, cease publishing. It is not Damp. This claim is provisional until the
  stalled-stream characterisation in note (a) is signed, because ramp-out with a
  dead link is publishing into the void: "disengage completed" then means local
  publishing finished, not that handover happened. When handover cannot be
  confirmed from lowstate, the stop result is marked UNCONFIRMED and the
  fallback is the operator's damp (remote). An ack is not evidence of action.
- **(c) Hand floors are deliberately NEW values, not the ported aihand ladder**
  (aihand: 800/400/120 ms). NORMAL 800 = the ported LOCKED floor, FULL 300 —
  stricter than aihand's FULL — because the Inspire's mechanical travel and
  protocol convention are both unverified. Revisit at P6 sign-off. Any future
  curl gate lives in [0.10, 0.20] rad (0.05 rad/action scale).
- **(d) Whitelist entries are dated human acts.** Entry schema:
  `{id, artifact_path, promoted_by, date}` — all four validated together. An
  artifact path alone is a flag-shaped check any file satisfies; the
  hardware_validated lesson is that an asserted flag carries no information.
- **(e) Probe protocol (P5), resolving the static/dynamic circularity.**
  P5a — on-stand, statically-supported: characterise FSM semantics (which id
  does what; the `706` Squat2StandUp/StandUp2Squat ambiguity; balance-mode
  integers). P5b — gantry: dynamically-stable ids earn measured envelopes under
  provisional ceilings. Any uncharacterised-id probe requires: tier FULL, the
  4 m clearance, stand or gantry attestation, a fresh single-use operator token
  per invocation, and the vendor remote exercised this session. Decided
  2026-07-30: a crane harness exists, so P5b is reachable from the start. There
  is no spare G1 and no parts budget, so probes prefer vendor-documented or
  community-precedented ids first; genuinely undocumented ids are probed on the
  harness only.
- **(f) Convenient-failure ledger** (what each scalar reads when the robot fails
  the easy way — written down before any gate cites these numbers):
  "no velocity cap exceeded" reads GREEN on a robot that never steps; "blend
  weight in band" reads GREEN at w=0 forever; "0 refusals" reads GREEN on an LLM
  that calls nothing; "lowstate fresh" reads GREEN on a wedged firmware
  republishing one frame (hence the liveness check); "ramp-out completed" reads
  GREEN locally while the robot never received it (hence UNCONFIRMED marking).
  None of these may be a gate's pass criterion alone; every gate names the
  motion that must be SEEN plus the limit that must hold.

### 1.3 The two-key rule — why raw `rt/lowcmd` is not a tier

Every rung of the ladder shares one property: **firmware (or later SONIC) is
balancing the legs the whole time.** Raw `rt/lowcmd` after `ReleaseMode` removes
the balancer. That is not a wider envelope — it is a different regime in which
the default outcome of doing nothing is a 35 kg fall. Risk is not monotone
across that boundary, so the boundary must not be an increment on the same dial:
a single ladder would make "nothing is keeping it upright" reachable by an enum
bump.

Raw joint control requires both keys plus physical preconditions:

1. **Key 1 — tier = FULL**, selected explicitly for the session.
2. **Key 2 — a signed grant file** (`grants/raw_lowcmd_grant.yaml`) validated
   before the raw writer will construct **and before `ReleaseMode` may be
   invoked** — the release call is the act that actually removes the balancer,
   so it is gated by the same grant, and MotionSwitcher operations are excluded
   from the FULL firmware-API whitelist so no whitelist entry can reach it.
   Fields: `robot_serial`, `scope` (v1 permits only `hold_current_pose`),
   `granted_by` (name), `date`, `expires` (≤ 24 h), `gantry_attested_by` (name),
   `estop_holder` (name, must differ from operator). **A grant is single-arming:
   consumed at first raw-writer construction.** Re-arming needs a fresh grant
   plus fresh §1.1-B attestations. A grant is a dated human act — the same shape
   as a verdict, and like a verdict it is never auto-renewed.
3. **Physical**: gantry bearing the robot; the named e-stop holder on the stop;
   vendor remote present and exercised this session.
4. **Protocol**: the arbiter's blocking handover transaction completes —
   CheckMode consulted fail-closed, `ReleaseMode` issued under the grant, and
   the mode change **observed in lowstate** (`mode_machine` transition), not
   inferred from the ack, before the first raw frame. The exit path back to a
   firmware mode (SelectMode re-acquire sequence + remote fallback) is defined
   in writing before the session starts. Every raw `LowCmd_` carries the
   computed CRC (`unitree_sdk2py/utils/crc.py`) — an integrity checksum, not
   authentication (UniPwn means the link itself is not trustworthy against an
   attacker); our stack makes it non-optional by construction. The
   `--disable-crc-check` flag belongs to the SONIC deploy binary
   (`g1_deploy_onnx_ref`), is a sim-only configuration by policy, and has no
   equivalent in this stack's hardware path.
5. **Engage sanity** (hold-current-pose): engagement requires 10 consecutive
   fresh lowstate frames with per-joint spread < 0.05 rad; q freezes to their
   mean; at the first raw frame |q_frozen − q_measured| must be < 0.1 rad or
   engagement refuses. A pose frozen from one garbled frame is a commanded step
   jump in the no-balancer regime.

v1 raw scope is **hold-current-pose and nothing else** (P9): q frozen, dq=0,
tau_ff=0, kp ≤ 60, kd ≤ 2.0. Any wider scope is a new signed grant with a new
named scope.

### 1.4 Sign-off

This table is a safety policy for a machine that can injure someone. It takes
effect only when signed; the signed values are then frozen into code by checksum
(tests A.4–A.5), and the checksum covers the signature block — an unsigned table
cannot go green. Changing any number afterwards is a new signature. A limit that
proves annoying in practice is raised by one signature, not a redesign — the
caution here is in sequencing, not in pace.

Signing authority (decided 2026-07-30): the author of this repository, who is
also the robot's operator.

Signed by the author of this repository, 2026-07-30.
(The checksum tests freeze this section as signed.)

