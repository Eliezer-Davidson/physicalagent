# Local modifications to GR00T-WholeBodyControl — preserved as a patch

Upstream: [`NVlabs/GR00T-WholeBodyControl`](https://github.com/NVlabs/GR00T-WholeBodyControl)
(source code **Apache-2.0**, `SPDX-License-Identifier: Apache-2.0`; model weights
are under the NVIDIA Open Model License and are **not** redistributed here — this
file is a source diff only). Full licence text:
[`../LICENSES/Apache-2.0.txt`](../LICENSES/Apache-2.0.txt).

The local working tree is a clone of that repository with changes on a
`local-sonic-mods` branch. That clone's only remote is NVIDIA upstream, so the
commits exist nowhere else — **this patch is the durable copy**, kept here
instead of forking a 2 GB repository to hold ~2 MB of changes.

## What the patch contains

All of it was written for this repository, expressed as a
diff against the Apache-2.0 upstream:

- **Simulation harness** — configurable fall-reset height (`SONIC_FALL_RESET_M`),
  per-phase loop timing, a scene chosen without editing the source, and the
  viewer-sync ordering fix.
- **Hand configuration** — the inspire-rigid hand config plus both
  whole-body-controller config variants it needs.
- **`InferenceEngine` fix** (`gear_sonic_deploy/src/TRTInference/`).
- **`hand_primitives/`** — eight Inspire closure vectors with rendered montages.
- **`build_sonic_g1_inspire.py`** — builds the G1 + Inspire MJCF that the
  MuJoCo side of this project runs against.

## What was deliberately removed before publication

Hunks carrying retargeted motion data and the rendering scripts that only ever
consumed it belong to a **separate, unpublished motion-retargeting project**
and were filtered out of this patch: eleven blocks of retargeted joint/body
CSVs with their renders, the converter that produced them, and the two
`reference/render_*.py` scripts that only ever displayed them. Nothing that
remains depends on them. Verify with:

    grep -c '/home/' local-sonic-mods.patch     # expects 0

## Environment variables the patched files read

Machine layout is never hard-coded. Set these before applying or running:

| Variable | What it points at |
|---|---|
| `WBC_ROOT` | this GR00T-WholeBodyControl checkout |
| `ISAAC_GROOT_ROOT` | an `Isaac-GR00T` checkout (the `gr00t` editable dependency) |
| `INSPIRE_HAND_ASSETS` | `xr_teleoperate`'s `assets/inspire_hand` directory |

## To reproduce on a fresh machine

    git clone https://github.com/NVlabs/GR00T-WholeBodyControl
    cd GR00T-WholeBodyControl
    git checkout "$(cat <this dir>/PATCH-BASE.txt)"
    git apply --binary <this dir>/local-sonic-mods.patch

## To regenerate after further vendor-tree work

    cd "$WBC_ROOT"
    git diff --binary main local-sonic-mods > <this dir>/local-sonic-mods.patch

Then re-run the filter above — removing the cross-project hunks is not
automatic.
