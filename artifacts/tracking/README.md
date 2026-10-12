# tracking probe runs — 2026-08-18

Eight runs of `scripts/sonic_tracking_probe.py`. This file exists because the
probe wrote its head-camera frames to fixed names (`stand.jpg`, `kneel.jpg`),
so **each run deleted the previous run's images in place**. Five of the eight
pairs were gone by the end of the day — including the MuJoCo baseline row.
Only three pairs survive, and they survive by accident: two had been copied
aside under another name, one was still on disk when this index was written.

The probe now writes `stand-<stamp>.jpg` / `kneel-<stamp>.jpg`. Nothing can
overwrite anything again.

Every surviving image is byte-verified against the `camera_frame` record its
own run wrote (`bytes=` field), so provenance is from the data, not mtimes.

## The runs

`backend` is quoted from each run's own `backend: <x>` record. `scale` for the
Isaac runs is **reconstructed**, not recorded: SONIC publishes lowcmd at
500 Hz, the LD_PRELOAD shim divides that by `SONIC_TIME_SCALE`, so
`scale ≈ 500 / cmd_hz`. `needed` is `1 / RTF` — the scale at which the
controller stops outrunning the physics. The probe now logs `run_config`
unconditionally so no future run needs this reconstruction.

The `LEGS NOT WIRED` verdict the 10:06 and 10:39 files carry is the closed-loop
probe's false negative — correlation measured inside a diverging loop; the
open-loop routing probe of 10:14 (`artifacts/routing/routing-20260818-101450-fullbody.jsonl`)
shows the same path routing correctly.

| run | backend | takeover z | pelvis min–max | pelvis std | tilt mean/max | leg RMS | RTF | cmd Hz | scale | needed | outcome |
|---|---|---|---|---|---|---|---|---|---|---|---|
| 10:06:10 | isaac | — | — | 0.1659 | — | 7.653 | 0.179 | 476.1 | ~1.0 | 5.6 | collapse |
| 10:39:57 | isaac | — | — | 0.1626 | — | 5.342 | 0.296 | 198.4 | ~2.5 | 3.4 | collapse |
| 10:43:23 | isaac | 0.752 | 0.808–0.998 | 0.0499 | not measured | 4.074 | 0.291 | 146.9 | ~3.4 | 3.4 | **stayed up** |
| 10:45:45 | mujoco | 0.960 | 0.957–0.972 | 0.0037 | not measured | 3.275 | 1.000 | 464.8 | 1.0 | 1.0 | band still attached |
| 10:48:11 | mujoco | 0.965 | 0.000–0.791 | 0.1807 | 69°/172° | 1.564 | −0.026 | 187.4 | 1.0 | — | SONIC never started |
| 10:52:44 | mujoco | 0.969 | 0.765–0.765 | 0.0002 | 0°/0° | 0.008 | 1.000 | 486.1 | 1.0 | 1.0 | **BASELINE** |
| 11:01:20 | isaac | 0.159 | 0.113–0.478 | 0.1211 | 92°/149° | 4.211 | 0.205 | 146.3 | ~3.4 | 4.9 | collapse |
| 11:13:25 | isaac | 0.155 | 0.065–0.539 | 0.1340 | 79°/116° | 7.783 | 0.191 | 99.9 | ~5.0 | 5.2 | collapse |

### Units warning

`tilt_mean_deg` / `tilt_max_deg` hold **radians** in every run before
11:13:25 — the key was named `_deg` while the helper returned radians. The
table above is converted. The stored JSON is not; read those two fields as
radians for runs 10:06 through 11:01. This is the defect behind the units fix in
`scripts/sonic_tracking_probe.py`, and it is why the 10:48 MuJoCo row reads "3.000" in the raw file and 172° here.
Cross-checked against pelvis height in every case: 10:48 reaches pelvis 0.000,
which only agrees with 172°, never with 3°.

### What the table shows

**The scale/RTF pairing decides the Isaac outcome.** Every collapse ran a
controller faster than its physics; the one run that stayed up is the one where
the shim's scale met `1/RTF` (3.4 vs 3.44 needed). That is the finding patch 07
exists to make repeatable.

**It is necessary, not sufficient.** Run 11:13 came within 5% of the needed
scale and still collapsed — because it was already on the floor before SONIC's
first command landed (takeover z 0.155, against 0.752 for the run that stayed
up). That is the startup-window bug, and it is now the dominant failure.

**"Stayed up" is not "stood."** The 10:43 run never fell, but tilt was not
instrumented yet and its `stand` frame shows a markedly rotated horizon. Its
pelvis also wanders 0.808–0.998 with std 0.0499 — 250× the MuJoCo baseline's
0.0002 — and its legs churn at 4.074 rad/s against the baseline's 0.008.
Against MuJoCo it is not a stand; it is a robot that has not fallen yet.

## The images

| file | run | what it shows |
|---|---|---|
| `stand-20260818-100610.jpg`, `kneel-20260818-100610.jpg` | 10:06 isaac | the first collapse (its file carries the probe's false `LEGS NOT WIRED` verdict, see above) |
| `stand-20260818-104323.jpg`, `kneel-20260818-104323.jpg` | 10:43 isaac | the only Isaac run that stayed up |
| `stand-20260818-111325.jpg`, `kneel-20260818-111325.jpg` | 11:13 isaac | the startup-window collapse, horizon ~45° over |

Earlier hand-copies of the 10:43 pair are not published; the stamped files above
are the byte-verified originals. `stand.jpg` / `kneel.jpg`, the last fixed-name
outputs (the 11:13 run), are not published either — their content lives on under
the stamped names above.

**Missing, permanently:** the image pairs from 10:39, 10:45, 10:48, 10:52 and
11:01. The 10:52 loss is the one that hurts — that is the MuJoCo baseline row,
the only genuinely steady stand recorded, and it has no surviving picture.
