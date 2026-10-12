# unitree_sim_isaaclab — local modifications, preserved as a patch

Upstream: https://github.com/unitreerobotics/unitree_sim_isaaclab
Base commit: see `BASE_COMMIT` (`e30c25b1dffdf92ada1d6c8c1fe9a47bdde0fecc`)
Licence: **Apache License 2.0** (`SPDX-License-Identifier: Apache-2.0`) —
verified in-repo, see `00-preexisting-local-mods.patch`, which carries the
upstream header verbatim. Full text: [`../LICENSES/Apache-2.0.txt`](../LICENSES/Apache-2.0.txt).
Checkout location: `$UNITREE_SIM_ROOT`
Interpreter: `$ISAAC_PY` (the Isaac-side conda env)

Patch 00 captures the local edits that predate the numbered series, kept separate
so the numbered patches stay minimal. `BASE_COMMIT` plays the same role as
`PATCH-BASE.txt` in the sibling `third_party/groot-wbc-local/`: the upstream commit
the patches are generated against.

## What the patch contains

`00-preexisting-local-mods.patch` is **not** a plain `git diff` — a plain `git diff`
only covers tracked files, and one of the edits in patch 00 is a top-level import
of a file that was itself untracked. A patch built from `git diff` alone applies
"cleanly" and then fails at runtime with `ModuleNotFoundError` the first time
`sim_main.py` is imported. The patch actually contains, in one file, five diff
hunks:

- **Modifications to 4 already-tracked files** (`git diff -- <paths>`):
  - `sim_main.py` — wraps `gym.make()` in a try/except that prints the full
    traceback on env-creation failure before re-raising.
  - `action_provider/create_action_provider.py` — adds
    `from action_provider.action_provider_policy import ACTPolicyActionProvider`
    and an `elif args.action_source == "policy"` branch.
  - `tasks/common_config/camera_configs.py` — adds two opt-in env-var-gated
    diagnostics (`UNITREE_SIM_CAM_PULLBACK`, `UNITREE_SIM_TILED_CAMERA`) used while
    chasing a camera-readback bug; both off by default.
  - `tasks/common_observations/camera_state.py` — adds one opt-in diagnostic
    (`UNITREE_SIM_DEBUG_CAMDUMP`) that dumps the raw camera annotator buffer; off
    by default.
- **One new, previously-untracked source file**, captured as a `new file mode`
  diff hunk against `/dev/null` (`git diff --no-index -- /dev/null <path>` — this
  reads the working tree only, it never touches the vendor repo's git index):
  - `action_provider/action_provider_policy.py` — the `ACTPolicyActionProvider`
    class that the `create_action_provider.py` edit above imports unconditionally
    at module load. **This file is required for `create_action_provider.py` to
    import at all**, regardless of whether `--action_source policy` is ever
    selected at runtime. Capturing it as patch content (not as a copied `.py`
    file living under our own source tree) is the same mechanism
    `groot-wbc-local/local-sonic-mods.patch` uses for its own new files.

## Known gaps — named explicitly, not silently dropped

Reproducing this patch on top of `BASE_COMMIT` gets you a working checkout, but it
is not the *entire* vendor working tree. The following exist on the real machine
and are deliberately **not** captured here, with the reason for each:

- **`.env-backup-20260731/`** (untracked, ~520 KB: a README, two verification
  scripts, two evidence PNGs, one `pip freeze` snapshot) — audit trail for a
  separate, already-fixed bug: numpy 2.x silently returned wrong-stride RGB from
  the Isaac camera readback and nothing raised, which is why the numpy version is
  pinned. It is
  not imported by any code path `sim_main.py` reaches, it is not needed to
  reproduce a working *setup*, and its own README says as much ("deleting the
  folder breaks no code, only the audit trail"). Checked for secrets
  (`pip-freeze-before.txt`, both scripts) — none found. Excluded because it
  documents an unrelated incident, not this baseline; if it is ever
  needed, capture it as its own numbered patch
  (`git diff --no-index --binary -- /dev/null <path>` per file, same mechanism as
  above) rather than folding it into `00-preexisting-local-mods.patch`.
- **`assets/`** (~1.7 GB) — gitignored by the vendor's own `.gitignore`, not
  tracked at any commit, not part of anyone's local edits. It holds the USD
  robot/scene/model files `sim_main.py` and the task configs load at runtime
  (e.g. `assets/robots/g1-29dof-inspire-base-fix-usd/...usd`,
  `assets/model/policy.onnx`). A fresh clone + this patch will not have it; it
  must be obtained through the vendor's own asset setup (see
  `doc/isaacsim5.1_install.md` in the vendor tree) independently of this
  directory. Out of scope for a "freeze the local edits" patch — nobody edited
  a 1.7 GB binary asset tree by hand.
- **`teleimager/`** — a normal git submodule (`.gitmodules`, pinned at
  `b81de448bca9c696d7ce145f4af71c66146d0b69`), not a local modification. A plain
  `git clone` leaves it empty; run
  `git submodule update --init --recursive` after checking out `BASE_COMMIT` (see
  reproduction steps below). Its own `src/*.egg-info/` build artifact (created by
  a local `pip install -e .`) is gitignored and regenerates on install; not
  captured.
- **3 tracked `tasks/utils/__pycache__/*.pyc` files**, shown as modified by
  `git status` in the real checkout — compiled bytecode, not source, tied to the
  local Python build and already stale relative to the `.py` they cache.
  Deliberately excluded from the patch; they regenerate automatically the first
  time the interpreter imports the corresponding module.

Nothing else was untracked or modified in the real checkout at capture time
(`git status --ignored --short`, checked 2026-08-06 — only the items listed above
plus ordinary `__pycache__`/`.venv`-class ignores).

## Reproducing on a fresh machine

    git clone https://github.com/unitreerobotics/unitree_sim_isaaclab
    cd unitree_sim_isaaclab
    git checkout $(cat /path/to/third_party/unitree-sim-isaaclab-local/BASE_COMMIT)
    git submodule update --init --recursive
    for p in /path/to/third_party/unitree-sim-isaaclab-local/0*.patch; do
        git apply "$p" || { echo "FAILED at $p -- tree is PARTIALLY patched"; break; }
    done
    # then separately: obtain assets/ per the vendor's own install doc

Apply **every numbered patch, in order** — stopping after 00 reproduces only the
pre-existing vendor mods and silently gives you the wrong DDS domain (1, not 0),
the x-first IMU quaternion and no `rt/odostate` publisher: a tree that runs but
diverges from the one every measurement in this repo was taken on. (That partial
recipe is exactly what an earlier revision of this section said; the 2026-08-07
review caught it.)

This reproduces the source tree byte-for-byte against the real checkout (verified
below) — it does not reproduce `.env-backup-20260731/` or `assets/`, for the
reasons stated above. The odostate publisher (03) also imports
`simbridge/odostate_fields.py` from a `physicalagent` checkout at the
path recorded inside the patch; on a machine without it, that import failure is
confined to `rt/odostate` (warn-once, everything else publishes normally).

## Regenerating after further vendor-tree work

If more local edits accumulate before they can be upstreamed, capture them the
same way this one was built — tracked-file diffs plus any new untracked files as
separate `--no-index` hunks, concatenated into the next numbered patch:

    cd "$UNITREE_SIM_ROOT"

    # 1. Diff every tracked file that changed since the previous patch was applied.
    git diff -- <path1> <path2> ... > /tmp/tracked.patch

    # 2. For each NEW untracked file the edits depend on (check with
    #    `git status --short` / `git ls-files --others --exclude-standard` first —
    #    never `git add -N`, it writes to the vendor repo's index):
    git diff --no-index -- /dev/null <new_path> >> /tmp/tracked.patch

    # 3. Concatenate onto the next numbered patch file in this directory, e.g.
    #    01-<description>.patch, and note it under "Patches, in order" below.

    # 4. Verify against a throwaway clone (see Verification below) before trusting it.

## Patches, in order

- `00-preexisting-local-mods.patch` — everything that was already in the working
  tree, uncommitted, on 2026-08-06. Captured before any change of this
  project's so the two never merge into one blob. Contains the 4 tracked-file edits
  and the 1 new untracked source file listed above under "What the patch
  contains".
- `01-dds-domain-and-interface.patch` — DDS domain 1 → 0 on `lo` (detail in its
  own entry at the end of this file).
- `02-imu-quaternion-w-first.patch` — `dds/g1_robot_dds.py`: upstream publishes
  the pelvis IMU quaternion `[x,y,z,w]`; SONIC and GR00T's own MuJoCo bridge
  read **w-first**. Measured on an upright robot: `[0.0039,-0.0224,-0.0415,0.9989]`
  before, `[0.9988,0.0035,-0.0235,-0.0438]` after. Also settles the open
  question at `physicalagent/robots/sonic_telemetry.py` — w-first wins.
- `03-odostate-publisher.patch` — `tasks/common_observations/g1_29dof_state.py`:
  publishes `rt/odostate` (the stack's only source of pelvis height) from
  `root_link_state_w`, all 13 floats gated through
  `simbridge/odostate_fields.py` (import deferred + warn-per-cause, so a
  missing checkout costs odostate only). Conventions matched to GR00T's MuJoCo
  bridge: w-first orientation, root-link (not COM) velocities, BODY-frame
  angular velocity, tick = sim-time ms. Reworked 2026-08-07 after review
  (was: raw COM velocities bypassing the gate, world-frame ang vel, counter
  tick, module-level import that could kill the whole state publisher).

- `04-sidecar-and-wholebody-reset.patch` — `sim_main.py` (NOTE: `00` also
  touches this file; `04` was generated as a before/after `--no-index` diff on
  the post-00 state and applies only after it). Two additions: (a) starts the
  `simbridge/isaac_sidecar.py` ZMQ answering service in a daemon thread that
  owns its socket end to end and NEVER touches the env — the main loop feeds
  it plain-python world snapshots at ~10 Hz instead, because
  `omni.physx.tensors` is not thread-safe and a concurrent tensor read raced
  the physics step into a GPU-context-killing CUDA assert (2026-08-07);
  pose/contact/camera questions on port 5570, missing checkout costs the
  sidecar only; (b) makes
  reset category `'2'` on `rt/reset_pose/cmd` mean "reset all" in EVERY mode —
  upstream mapped both categories to object-reset under wholebody DDS, leaving
  no way to reset the ROBOT over DDS, which the get-up probes need between
  trials (no elastic band on this backend).
- `05-sonic-full-body-routing.patch` — `action_provider/action_provider_wh_dds.py`:
  upstream's `dds_wholebody` mode is an XR-teleop hybrid — legs are driven by
  the vendor's own in-sim `policy.onnx`, the waist is pinned to defaults, and
  the external controller's `rt/lowcmd` reaches ONLY the 14 arm joints, with
  the message's kp/kd parsed and discarded (found 2026-08-12; this is why
  "SONIC balances G1 in Isaac" was false and the get-up A/B was void). With
  `SONIC_WB_FULL_BODY=1` in the sim's environment, all 29 body joints track
  the lowcmd positions (hw order → articulation order, mapped by NAME with
  the order pinned against `policy_parameters.hpp:143-176`), the local policy
  is neither run nor loaded, the RL delay/clip/scale post-processing is
  bypassed (these are absolute targets, not RL actions), and the message's
  per-joint kp/kd are written into the PhysX drives (refused if any value is
  non-positive; re-applied on a slow cadence so an env reset cannot silently
  restore the asset gains). Without the env var the vendor behavior is
  byte-identical, and both modes announce themselves at startup — the
  launcher hard-fails if the full-body banner is missing.
- `06-render-throttle-and-loop-profile.patch` — `action_provider/action_provider_wh_dds.py`:
  two things the real-time-factor work needed. (a) `SONIC_WB_RENDER_EVERY=N`
  renders on one iteration in N. This provider calls `env.sim.render()`
  DIRECTLY, so the vendor's `--no_render` and `--render_interval` (which only
  gate `env.step()`, never called on this path) cannot touch it — MEASURED
  2026-08-12: RTF 0.216 with rendering "disabled" vs 0.216 without the flag.
  With the throttle, render falls from 41.0 ms to 5.0 ms per iteration and RTF
  goes 0.212 → 0.392; adding `--camera_write_interval 100 --camera_exclude
  world_camera,robot_camera` reaches 0.431. Cameras refresh only on rendered
  iterations, which is why the sidecar's camera staleness gate matters.
  (b) a `[wb-prof]` line every 200 iterations giving physics / render / obs
  milliseconds and the implied RTF ceiling — the vendor's own `[Performance]`
  block is printed WITHOUT flush into a redirected, block-buffered stdout and
  discarded at `os._exit`, which is why no such line has ever appeared in any
  of this project's sim logs. This one flushes.
  Default `N=1` is byte-identical vendor behaviour apart from the profile line.
- `07-pin-real-time-factor.patch` — `action_provider/action_provider_wh_dds.py`:
  `SONIC_WB_TARGET_RTF=x` sleeps when the loop runs FASTER than x, pinning the
  real-time factor the way the SONIC MuJoCo reference pins 1.0 with an
  absolute-deadline limiter. This exists because the external controller is
  paced on the WALL clock and must be slowed by 1/RTF to stay in step
  (`tools/sonic_timescale.so`) — and a DRIFTING RTF makes that pairing a coin
  flip. MEASURED 2026-08-18, four runs separating perfectly on whether the
  scale matched the rate: RTF 0.179 with scale 1.0 collapsed, 0.296 with 2.5
  collapsed, **0.291 with 3.4 STOOD**, 0.205 with 3.4 collapsed. Pin the
  target BELOW the worst achievable rate (0.205 observed under load; the
  launcher defaults to 0.20) and a fixed scale of 1/target is always right.
  An overrun forgives its debt rather than catching up — chasing a missed
  deadline makes every later frame sleep-free and the pinning silently stops
  pinning. `0` disables (vendor behaviour). The `[wb-prof]` line reports the
  pinned target and how many iterations ran late.
## Verification (2026-08-12 — full loop, patches 00–05)

Cloned the vendor repo LOCALLY to a scratch dir, checked out `BASE_COMMIT`
(`e30c25b1dffdf92ada1d6c8c1fe9a47bdde0fecc`), initialized the `teleimager`
submodule from the local module store (`git -c protocol.file.allow=always
submodule update --init`), applied **all six patches 00–05 in order** (each
`git apply` exit 0), then diffed the result against the real checkout
(excluding only the named gaps above — `.git`, `__pycache__`,
`.env-backup-20260731/`, `assets/`, and the submodule's own gitignored
`*.egg-info`):

    diff -r --exclude='.git' --exclude='__pycache__' \
      --exclude='.env-backup-20260731' --exclude='assets' --exclude='*.egg-info' \
      <scratch-clone> "$UNITREE_SIM_ROOT"

**Exit code 0 — no differences.** The repro loop above reproduces the live
tree byte-for-byte as of 2026-08-12. (History: the original 2026-08-06 run
verified patch 00 only, against the tree as it stood then — its record lived
in that run's own report. `tests/test_full_body_patch.py`
additionally rebuilds BASE_COMMIT + patches 05–07 and asserts the result is
byte-identical to the live vendor provider; it runs only where
`UNITREE_SIM_ROOT` points at the checkout, so drift fails there before launch.)

- `01-dds-domain-and-interface.patch` — `dds/dds_master.py`: upstream calls
  `ChannelFactoryInitialize(1)` (DDS domain 1, default interface). Our whole
  stack is domain 0 on loopback: the SONIC deploy binary is launched as
  `g1_deploy_onnx_ref lo`, `UnitreeTelemetry` defaults to
  `domain_id=0, interface="lo"`, and `scripts/sonic_check.py` hard-codes
  `ChannelFactoryInitialize(0, "lo")`. MEASURED 2026-08-06: with upstream's
  domain 1, the sim published `rt/lowstate` at ~80 Hz with real joint values
  while our side saw **zero** traffic — a silent, total mismatch. Patched to
  `ChannelFactoryInitialize(0, "lo")`.
