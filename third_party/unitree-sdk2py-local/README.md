# unitree_sdk2_python (Isaac side) — local modifications

Upstream: https://github.com/unitreerobotics/unitree_sdk2_python
Licence: **BSD 3-Clause** (Unitree Robotics), `SPDX-License-Identifier: BSD-3-Clause`.
Full text, with Unitree's copyright line, list of conditions and disclaimer:
[`../LICENSES/BSD-3-Clause-unitree_sdk2_python.txt`](../LICENSES/BSD-3-Clause-unitree_sdk2_python.txt).
The generated `_OdoState_.py` this patch adds has no upstream copyright header of
its own (verified), so that file is the notice for it — carry it if you
redistribute an applied tree.
Checkout location: the Isaac-side `unitree_sdk2_python`, wherever that env keeps it.
Base commit: see `BASE_COMMIT`
Used by: the `unitree_sim_env` conda env (Unitree's Isaac Lab sim imports it).

## Why this exists

There are **two** `unitree_sdk2_python` checkouts in a full install of this
stack and they are not the same:

| checkout | used by | has `OdoState_` |
|---|---|---|
| the Isaac-side checkout | the Isaac conda env | **NO** |
| `$WBC_ROOT/external_dependencies/unitree_sdk2_python` | `.venv_sim` (MuJoCo, this stack) | yes |

`rt/odostate` is the only source of pelvis height in this stack, so the Isaac
side could not even construct the message. Measured 2026-08-06.

The two `_LowState_.py` files differ ONLY in two commented-out lines
(`# position:`, `# linear_velocity:`) — the wire type is identical. GR00T moved
those fields out of `LowState_` into a separate `OdoState_`, which is why the
Isaac-side SDK, tracking upstream, has no `OdoState_` at all.

## Patches, in order

- `01-register-odostate.patch` — registers `OdoState_` in
  `unitree_sdk2py/idl/unitree_hg/msg/dds_/__init__.py` **and carries the message
  definition itself** (`_OdoState_.py`, copied verbatim from the BSD-3-licensed SDK inside GR00T-WholeBodyControl's
  `external_dependencies/`, so
  the two sides agree on the wire type) as a `git diff --no-index -- /dev/null`
  new-file hunk — the same mechanism `../unitree-sim-isaaclab-local` uses for
  its own new files. `git apply` of this one patch is the whole procedure.

  (An earlier revision shipped `_OdoState_.py` as a loose file with a manual
  copy step, justified by "`git diff` cannot carry a new untracked file" — false,
  as the sibling directory's own `--no-index` mechanism demonstrates, and the
  missed-copy failure surfaced only later, at import time, as the very
  deferred-runtime trap this directory's closing note warns about. Folded into
  the patch 2026-08-07 after review. Verified: the patch applies cleanly on the
  pristine `BASE_COMMIT` tree.)

## Orientation convention

`OdoState_.orientation`'s IDL comment says `[x, y, z, w]`, but GR00T's MuJoCo
bridge writes `floating_base_pose[3:7]` into it under the comment
`# quaternion: w, x, y, z`, and `UnitreeTelemetry` reads it w-first. **W-first is
what this stack means.** Our publisher writes w-first to match.
