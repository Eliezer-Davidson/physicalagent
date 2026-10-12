# G1 CONTROL ARCHITECTURE — the three-layer stack, and where a WAM goes

Written 2026-07-30 after a primary-source audit (model-hub APIs, licence texts,
repository source, vendor PDFs and model cards). Companion to `g1-safety-tiers.md`,
the tier table.

The architecture evaluated here: a high-reasoning LLM at the top managing tasks,
a whole-body controller of the SONIC class underneath keeping the robot upright,
and a manipulation world-action model (WAM) in between for the arms and hands.

**Verdict: sound. Three corrections to the model slots, one interface discovery,
and one hard constraint.**

---

## 1. The architecture, with the evidence for each layer

| Tier | Rate | Occupant (v1 → later) | Why this rate |
|---|---|---|---|
| **Semantic** — what to do | 0.2–1 Hz | cloud LLM, tool calls only | network round trip; never emits trajectories |
| **Visuomotor** — how to move the arms/hands | 5–25 Hz | v1: scripted + IK reach → later: a manipulation WAM | the manipulation models surveyed all run here: DreamZero 7 Hz, MotionWAM 4.9 Hz, Cosmos-Policy 25 Hz, GE-Act 30 Hz |
| **Whole-body** — stay upright | 50 Hz | firmware (P4–P5) → SONIC (P7–P8) | SONIC `NUM_MOTORS: 29`, dt 0.02 |
| **Joint** — track the targets | 500 Hz | Unitree low-level API → motor-board PD | vendor `control_dt_ = 0.002` |
| **Hands** — fingers | 100 Hz, separate wire | this project's driver + primitive library | `dfx_inspire_service`, two USB-serial adapters, own DDS topic |

Every shipping legged humanoid keeps a fast whole-body layer under the learned
one; what changed recently is that the fast layer is now learned rather than
hand-written.

Two observations about the field that bear on the design, without naming the
individual results: every system surveyed that puts a learned manipulation model
over a balance controller keeps the hand on a **separate channel** — even the ones
that argue hardest against decoupling still decouple the fingers; and the
strongest real-robot manipulation results on this robot class ship **no weights
and no code**, so they are a real result that cannot be downloaded, not a
starting point.

## 2. Three corrections to the model slots

**(a) DreamDojo is not a controller — it is a dreamer.** 10.81 FPS after
distillation, and its own stated applications are *"live teleoperation, policy
evaluation, and model-based planning."* It belongs in the **evaluation** slot —
score a policy in imagination instead of burning robot time — not in the
manipulation loop. That slot is genuinely valuable to this project precisely
because there is only one G1 and no spares.

**(b) DreamZero cannot run here.** 14B on a Wan2.1-I2V-14B base, ~44.8 GB of
shards, its own documentation states *"tested on GB200, H100, at least 2 GPUs"*,
and the NVFP4 engine is GB200-only. Also a license trap: the code is Apache-2.0
and the AgiBot checkpoint is Apache-2.0, but the flagship **DROID checkpoint is
under a licence that forbids commercial use**. Excellent result (7 Hz closed loop, >2× VLA
generalization, new embodiment from 30 min of play data) — wrong hardware class
for a 12 GB laptop.

**(c) HOMIE is not in the same slot as SONIC/GR00T.** HOMIE is RL for the *lower
body* with upper-body joint angles set **directly** from teleop — an
isomorphic-hardware teleop system, not a whole-body foundation controller. Its
value here is a measurement, not a checkpoint: it runs **263 Hz on the arm
channel and 293 Hz on the hand channel** — independent confirmation that the
hands are a separate bus, at a different rate, on real hardware.

**Of the three named body controllers, SONIC is the only one that is
simultaneously: G1-native, weights-released, commercially licensed (NVIDIA OML),
and shipping a production C++/TensorRT deploy stack.** It is one of exactly three
deployable open G1 whole-body controllers in existence (the others: NVlabs
ProtoMotions `g1-bones-deploy`, sim-to-sim only by NVIDIA's own model card; and
LeCAR ASAP, MIT-licensed but per-motion skills rather than a general controller).
Weights updated 2026-07-27 — it is actively maintained, not abandoned.

## 3. The interface discovery — the seam a WAM plugs into

The obvious worry with this architecture is that SONIC's 64-dim motion tokens are
**checkpoint-private** (NVIDIA's own docs: *"the same token produces different
poses across checkpoints"*), so a manipulation model cannot address them.

**It does not have to.** SONIC also accepts teleop-format signals:
**head pose + both wrist poses + waist height + root linear/angular velocity.**
That is a human-meaningful, checkpoint-stable interface. A manipulation model that
emits wrist poses can drive SONIC without ever touching the latent space — the
same shape as a behavior model emitting hand and foot *poses* into an MPC.

So the seam is:

```
LLM  ──tool call──▶  manipulation model  ──wrist poses + root velocity──▶  SONIC  ──29 joint targets──▶  500 Hz writer
                              │
                              └──grasp primitive──▶  hand driver  ──rt/inspire/cmd @100 Hz──▶  fingers
```

Every arrow in that diagram is a documented interface. The middle box is the only
one this project does not have.

There is also a second, already-built bridge worth knowing: **Isaac-GR00T N1.7 GA
ships a `UNITREE_G1_SONIC` embodiment tag** — the VLA emits the 64-dim latents
SONIC's decoder expands. NVIDIA closed that loop themselves. (Fine-tuning N1.7
needs 40 GB+, so it is not a laptop activity, but the path exists.)

## 4. What can actually occupy the manipulation slot on 12 GB

Checked against the HuggingFace API and license texts on 2026-07-30, not READMEs.

| Model | Size / VRAM | License | Emits actions? | Embodiment evidence |
|---|---|---|---|---|
| **Cosmos-Policy 2B** | 3.91 GB file; **6.0–8.9 GB VRAM published** (10 GB with planning) | **NSCLv1 — NON-commercial** | yes: 16×7-D chunk + predicted frames + value | ALOHA 2 bimanual, 93.6% / 4 tasks, 25 Hz |
| **Cosmos3-Edge-Policy-DROID** (W4A16 community quant) | 6.38 GB disk, **8.87 GB peak on a 4090**, 563 ms p50 | **OpenMDW 1.1 — commercial OK** | yes: 32×8-D DROID chunks + video | sim only (RoboLab), 74% on one task, 34 downloads |
| **VLA-JEPA** (`lerobot`) | 3.08B, 6.16 GB | **Apache-2.0** | yes, 7-D | *"No evaluation results have been provided"* |
| **UnifoLM-WMA-0** (Unitree) | 10.44 GB gated / 16.72 GB dual | **a licence that forbids commercial use — NOT commercial** | world model + action head | **Z1 arm and G1 with dexterous hands** — GIFs, no success rates, no Hz |
| **iVideoGPT** | ~1.01 GB | **MIT** | frames only (action-conditioned) | — |
| DreamZero / GigaWorld / Kairos / FastWAM | 12–45 GB, ≥1 datacenter GPU | mixed | yes | arms |

Read the top two rows together: **the one that fits and is commercially usable has
never touched a real robot; the ones with real-robot evidence are non-commercial.**
Unitree's own UnifoLM-WMA-0 is the closest artifact in existence to what this slot
needs — a world model with an action head, running on a G1 with dexterous hands —
and it is under a licence that forbids commercial use despite reading like a normal open-source
release. No date currently forces commercial cleanliness (decided 2026-07-30), so
it is *usable for research today*; it just cannot be the thing that ships.

**And the constraint that decides the schedule: no WAM anywhere speaks G1+Inspire.**
NVIDIA publishes Cosmos3's supported action embodiments — camera, AV, egocentric
human, Franka, dual Franka, AgiBot, UR, Google robot, WidowX, UMI. Not one legged
robot. Occupying the manipulation slot means fine-tuning a model onto a G1+Inspire
action contract, which needs G1+Inspire demonstration data that does not exist
anywhere, including here.

## 5. What this changes in the build

Structurally: **almost nothing**, which is the point. The gate ladder already
builds the layers this architecture needs, in the order it needs them.

| Layer | Gate that builds it | Changed? |
|---|---|---|
| Safety, arbiter, tiers, state honesty | P0–P3 | no — and this is the layer *nobody in the industry has published*, WAM or otherwise |
| Semantic (LLM) | P2, extended at P4b | no |
| Visuomotor v1 (scripted + IK reach) | P4 | no — it is the WAM slot's placeholder and its A/B baseline |
| Hands | P6 | no — separation confirmed by every source checked |
| Whole-body (SONIC) | P7–P8 | no |
| **Data spine** | NEW, from P1 | **yes — this is the real change** |
| **Manipulation WAM** | NEW P10 | **yes — a research gate, deliberately last** |

**The data spine is now the highest-leverage addition.** A manipulation WAM in
that slot needs G1+Inspire trajectories, and the field's own numbers say the budget
is small: Psi0 adapts on **80 teleop episodes per task**; DreamZero transfers to a
new embodiment from **30 minutes of play data**; GE-Act does cross-embodiment on
~250 demos (~1 h). Those are single-session quantities. But every hardware session
that does not record synchronized video + state + commands in a training-ready
format is a session whose data cannot be recovered — and there is one robot, and
finite robot time. Retrofitting this is expensive; adding it at P1 is nearly free.

**P10 — MANIPULATION-WAM (sketch, not a gate spec).** Entered only after P4 and P6
are signed, because it needs a working reach, a working hand, and a baseline to
A/B against. Fine-tune a fitting checkpoint (Cosmos-Policy 2B for research;
Cosmos3-Edge if commercial cleanliness ever binds) onto the G1+Inspire action
contract using the data spine's episodes; it emits wrist poses + a grasp primitive
into the P4/P6 interfaces; it inherits every tier limit and refusal in the signed
table unchanged, because it commands through the same plane a scripted reach does.
The pass criterion is a straight A/B against the P4 scripted baseline on the same
task — one component changed.

## 6. The counter-argument, kept in view

The strongest objection, from people who build controllers for a living, is that
a visually plausible prediction which ignores contact, mass, friction or physical
feasibility may help representation learning but is not yet a reliable substrate
for robot control — a latent action learned from video is not yet a robot
command. And on the data question: synthetic experience lets a system rehearse,
not learn more about the world; information about the world still has to be
injected from somewhere.

Both are position statements, not refutations — the real-robot manipulation
numbers in the table above are real numbers. But they are the reason the WAM goes
in the manipulation slot *above a proven balance layer*, with a scripted baseline
to A/B against, rather than anywhere near the thing keeping the robot upright.

**Nothing in this document changes the tier table.** A WAM commands through
the same arm_sdk plane, under the same envelopes, with the same refusals — which is
the whole reason the safety layer was built first and separately from the brain.
