# Dead ends — do not retry these

A catalogue of diagnoses that were made, tested, and **disproved or retracted**
while bringing up the HiWonder robots. It is kept because the expensive part of
this work was not the code that shipped; it was ruling things out, and a dead end
that is not written down gets re-walked.

Two of these entries (**D3**, **D12**) are retractions of earlier conclusions
reached here.

---

**D1 — The TonyPi camera endpoint: `?action=snapshot` is correct;
`?action=stream` is the stale one.**
Measured on a healthy robot: snapshot **10/10 distinct frames**, stream
**1/10**. An earlier session measured the opposite *on a wedged robot*, believed
it for a day, and migrated the production path to the stream. That was reverted.
**Do not re-reverse it.** The tested stream parser survives as `stream_frame()`
in `physicalagent/robots/tonypi_link.py`, unused, with the measurement in its
docstring.

**D2 — `GetRunningFunc` is not telemetry.**
The vendor implementation is a hardcoded `return (True, (0,))`. The earlier
finding "the RPC acknowledges a start it does not perform" was reading that stub.
Do not build state tracking on it.

**D3 — `GetBusServosPulse` wedging the RPC server: RETRACTED.**
Diagnosed during a window in which the network was also dropping; the two could
not be separated. The `runbymainth` single-main-thread hazard is real in the
vendor source (one queue, a 2 s busy-wait at `count > 200`) but was never
demonstrated. **Do not code around a bug that was never reproduced.**

**D4 — There is no battery method on the TonyPi RPC.**
26 methods enumerated; none returns voltage. The only voltage source is the
physical display on the robot's back. Stop looking — ask the operator.

**D5 — `tonypi.local` does not resolve.**
The robot boots into AP-direct mode and mDNS does not work over that link. The
default host is therefore the vendor's own AP address, `192.168.149.1`.

**D6 — AI mesh generators cannot produce a structural bracket.**
Every text-to-3D system evaluated emits decorative meshes with no hole tolerances
and no datums. The architectural reason none of them work: **no text-to-CAD
system accepts an existing assembly as constraint input.** The leverage is in
*writing the parametric CAD code*, not in generating geometry.

**D7 — GUI CAD was not needed.**
The recommendation was to sew IGES surfaces by hand in a GUI. A headless script
did it 8/8 in seconds.

**D8 — Two of eight brackets sew with INVERTED NORMALS.**
Two vendor brackets come out of `BRepBuilderAPI_Sewing` as closed shells with
**negative volume**. OCCT hands them over without complaint, and every downstream
boolean then does the opposite of what you meant — a cut adds material, a fuse
removes it, and nothing errors. The importer now measures volume, reverses, and
asserts positive. **Any new geometry importer must do the same check.**

**D9 — Keep the interpreters separate.**
The robot/test virtualenv is `uv`-managed and has no CAD libraries; the CAD
environment has no robot stack. Do not try to merge them.

**D10 — `pkill -f "tmp/foo.py"` kills your own shell**, because the
pattern matches the `pkill` command line itself (exit 144). Use a bracket class:
`pkill -f "[f]oo.py"`.

**D11 — "Lighter AND longer" is a trap.**
The frame is only ~30 % of robot mass; servos are 51 %. Material substitution
saves nothing at equal stiffness, and maximum walking speed at the torque limit
is independent of leg length. Worse: **116 `.d6a` action groups — 2,375
keyframes, ~38,000 hand-tuned joint-space servo values — silently break the
moment a link length changes.** Choose the servo before the CAD. If a longer limb
does happen, write the retargeter with an identity negative control; do not hand
re-record.

**D12 — The licence question is unresolved, and this repository does not resolve
it.** A vendor's CAD being publicly downloadable is **not** permission to sell
derivative printed frames. Nobody has asked them in writing. Do not assume either
way.
