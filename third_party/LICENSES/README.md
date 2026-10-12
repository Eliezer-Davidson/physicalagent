# Upstream licence texts

The patches under `third_party/` are diffs against other projects' source, so
they reproduce context lines from files that are not this repository's to
license. The full, unmodified text of each upstream licence is kept here so a
recipient of this repository receives it with the material it covers, as those
licences require.

| File | Covers | Upstream | Base commit |
|---|---|---|---|
| [`Apache-2.0.txt`](Apache-2.0.txt) | `../unitree-sim-isaaclab-local/*.patch` (8 patches) — Copyright (c) 2025, Unitree Robotics Co., Ltd. | <https://github.com/unitreerobotics/unitree_sim_isaaclab> | `e30c25b1dffdf92ada1d6c8c1fe9a47bdde0fecc` |
| [`Apache-2.0.txt`](Apache-2.0.txt) | `../groot-wbc-local/local-sonic-mods.patch` — Copyright (c) NVIDIA Corporation & Affiliates (source code only; no model weights are redistributed) | <https://github.com/NVlabs/GR00T-WholeBodyControl> | `0e35637c34cb0296658b744735f63c5fdbd4ca62` |
| [`BSD-3-Clause-unitree_sdk2_python.txt`](BSD-3-Clause-unitree_sdk2_python.txt) | `../unitree-sdk2py-local/01-register-odostate.patch` — Copyright (c) 2016-2024 HangZhou YuShu TECHNOLOGY CO.,LTD. ("Unitree Robotics") | <https://github.com/unitreerobotics/unitree_sdk2_python> | `4f12b0134e87ae33f3401bebf1c49f527dff4abd` |

`Apache-2.0.txt` is the standard, unmodified Apache License 2.0 text
(`md5 3b83ef96387f14655fc854ddc3c6bd57`). `BSD-3-Clause-unitree_sdk2_python.txt`
is the upstream SDK's own `LICENSE` file, copied byte for byte, so the copyright
notice, the list of conditions and the disclaimer all travel with the patch.

**Applying a patch.** These licences follow the code, not this directory. If you
apply one of these patches into your own checkout and then redistribute the
result, carry the corresponding notice with it. That matters most for
`01-register-odostate.patch`: the 32-line `_OdoState_.py` it adds is machine
output from the Eclipse Cyclone DDS `idlc` Python backend and carries **no**
copyright header of its own upstream (verified against the upstream file), so
the only notice for it is the one in this directory.

Neither this directory nor the patches change the terms of this repository's own
[`LICENSE`](../../LICENSE), which covers only this repository's own code.
