"""SonicLink — the seam between a tool call and NVIDIA's SONIC policy, plus the
MuJoCo camera view. No Gemini here, no tool logic; this is transport only.

WHY A SEAM AT ALL
    SONIC runs as a separate compiled process (`g1_deploy_onnx_ref`) that talks
    DDS on loopback to a MuJoCo simulation. How a *command* reaches that process
    is, as of 2026-07-31, not yet verified on this machine — the candidates are a
    DDS topic it subscribes to, its keyboard handler, or a small patch. So the
    command path is isolated behind this interface exactly the way TonyLink and
    AiHand isolate theirs, and FakeSonicLink lets the adapter and the Gemini loop
    be built and tested with nothing running.

    This mirrors aihand_codec's discipline: the unverified thing is named as
    unverified, and a Fake exists so the layers above it are not blocked on it.

WHAT IS VERIFIED (2026-07-31, this machine)
    * `gear_sonic_deploy/target/release/g1_deploy_onnx_ref` is BUILT (5.4 MB).
    * `deploy.sh sim` selects the loopback interface for MuJoCo.
    * The policy loop is 50 Hz; a command writer runs at 500 Hz; a target-velocity
      planner exists at `policy/planner/target_vel/V2/planner_sonic.onnx`.
    * The G1 MuJoCo models load: 29 actuators, 1 camera, nq=36 (rigid hands) or
      nq=60 (finger joints present but unactuated).

WHAT IS NOT VERIFIED
    * Which input path accepts a programmatic target velocity. Until a probe
      shows a real robot moving in response, VERIFIED_COMMAND_PATH stays None and
      every send is explicitly a candidate. Promoting it is a human act with a
      name and a date.

    CANDIDATE EVIDENCE, awaiting sign-off (2026-07-31):
      scripts/sonic_prove.sh runs a control phase and a Gemini-commanded phase
      against the same policy in one session, with ground-truth pelvis position
      logged by the simulator itself. artifacts/sonic_prove/ (run 4) holds:
          CONTROL  19.9 s, no commands published : net 0.01 m, min height 0.78 m
          GEMINI   21.8 s, prompt-driven         : net 2.74 m (+2.73 x, +0.12 y)
                                                   path 3.00 m, min height 0.75 m
      The model issued two 3 s walks at 0.5 m/s with a status check between, and
      the position trace shows exactly two movement bursts separated by a pause;
      ~3 m of path over 6 s of commanded walking is ~0.5 m/s, as commanded. No
      sim reset, no position discontinuity, no fall.
      artifacts/sonic_prove_run3/ is an independent repeat: CONTROL 0.00 m,
      GEMINI 2.66 m. Run 2 was VOID (a sim stall corrupted attribution) and is
      not kept -- see sonic_analyze.py, which now refuses to score such a run.
      Artifacts per run: pose.csv.<pid>, marks.csv, VERDICT.txt, gemini.log,
      rec/run.mp4, rec/strip8.png.
      To accept, set VERIFIED_COMMAND_PATH = "zmq_manager:planner" with your name
      and the date. It is deliberately not promoted here.

SAFETY POSTURE (sim today, hardware later)
    Nothing here can hurt anyone — it drives a simulation. But the shape is kept
    honest so it ports: velocities are clamped, every command is bounded in time
    rather than latching, and stop() is always available. A latched walk that
    outlives the process is the failure mode this shape exists to prevent.
"""
from __future__ import annotations

import math
import time
from dataclasses import dataclass, field

# --- limits -------------------------------------------------------------------
# The provisional FULL-tier ceilings from the G1 tier table
# (docs/design/g1-safety-tiers.md, "Locomotion velocity commands"), so that what is
# demonstrated here is inside what hardware would allow.
#
# They are NOT a safety mechanism in simulation; they are a rehearsal of one.
#
# Until 2026-08-19 these read 0.6 / 0.3 / 0.8 -- exactly double the table on
# every axis -- under a comment asserting they matched it. Every clamp test
# passed, because each one compared the code to itself. The table is signed and
# P5b results may only LOWER these; tests/test_g1_locomotion_plane.py now pins
# both sides against hand-transcribed literals so neither can move quietly.
MAX_VX_M_S = 0.3
MAX_VY_M_S = 0.2
MAX_WZ_RAD_S = 0.5
# Every motion command expires. The model must re-issue to keep moving, so a
# confused reasoner or a dead loop leaves the robot stopped, not walking away.
MAX_DURATION_S = 5.0
DEFAULT_DURATION_S = 2.0

# Flipped to a path name ONLY when a probe has moved the simulated robot through
# that path. Until then every send is a candidate and the operator is told so.
VERIFIED_COMMAND_PATH: str | None = None


def clamp_velocity(vx: float, vy: float, wz: float) -> tuple[float, float, float]:
    """Clamp a velocity triple to the sim envelope. Returns floats, never raises —
    an out-of-range request becomes a slower move the model can see in the result,
    not an exception that breaks the agent loop."""
    return (
        max(-MAX_VX_M_S, min(MAX_VX_M_S, float(vx))),
        max(-MAX_VY_M_S, min(MAX_VY_M_S, float(vy))),
        max(-MAX_WZ_RAD_S, min(MAX_WZ_RAD_S, float(wz))),
    )


def clamp_duration(seconds: float | None) -> float:
    """Bound a command's lifetime. None means the default, not 'forever'."""
    if seconds is None:
        return DEFAULT_DURATION_S
    return max(0.0, min(MAX_DURATION_S, float(seconds)))


@dataclass
class SonicState:
    """What we can say about the robot right now. Every field is Optional and
    `fresh` is explicit, because a state we could not read must say so rather
    than report a plausible constant — the lying-telemetry failure this project
    has already been bitten by once."""

    fresh: bool = False
    root_height_m: float | None = None
    tilt_rad: float | None = None
    velocity: tuple[float, float, float] | None = None
    mode: str | None = None
    note: str | None = None

    def as_dict(self) -> dict:
        return {
            "state_available": self.fresh,
            "root_height_m": self.root_height_m,
            "tilt_rad": self.tilt_rad,
            "velocity": list(self.velocity) if self.velocity else None,
            "mode": self.mode,
            "note": self.note,
        }


class SonicLink:
    """Interface every SONIC transport implements."""

    def set_velocity(self, vx: float, vy: float, wz: float, duration_s: float) -> dict:
        raise NotImplementedError

    def set_course(self, move_heading: float, facing_heading: float,
                   speed: float, duration_s: float) -> dict:
        """Absolute headings in the policy's frame — see ZmqSonicLink.set_course."""
        raise NotImplementedError

    def stop(self) -> dict:
        raise NotImplementedError

    def emergency_stop(self) -> dict:
        """Damp the motors. This is the fall watchdog's ONLY actuator, and it
        belongs on the interface for exactly that reason: it was implemented on
        ZmqSonicLink alone, so every other transport armed a watchdog that
        could not damp. The failure was invisible — the beat's bare except
        turned it into a printed line, and the offline suite emitted
        'AUTO-DAMP FAILED' ten times with nothing asserting on it."""
        raise NotImplementedError

    def select_mode(self, name: str, *, duration_s: float | None = None,
                    height_m: float | None = None,
                    speed: float | None = None) -> dict:
        raise NotImplementedError

    def set_height(self, height_m: float | None) -> dict:
        raise NotImplementedError

    def read_state(self) -> SonicState:
        raise NotImplementedError


class FakeSonicLink(SonicLink):
    """Deterministic offline stand-in. Integrates the commanded velocity so the
    adapter, the tool surface and the whole Gemini loop can be exercised with no
    simulator, no DDS and no network — the same discipline that got the sibling
    stack to 868 passing tests with the robot switched off."""

    def __init__(self, *, start_height: float = 0.78):
        self.commands: list[dict] = []
        self.mode = "stand"
        self._pos = [0.0, 0.0]
        self._yaw = 0.0
        self._height = start_height
        self._vel = (0.0, 0.0, 0.0)

    def set_velocity(self, vx, vy, wz, duration_s) -> dict:
        vx, vy, wz = clamp_velocity(vx, vy, wz)
        duration_s = clamp_duration(duration_s)
        # integrate so a test can assert the robot actually went somewhere
        self._pos[0] += (vx * math.cos(self._yaw) - vy * math.sin(self._yaw)) * duration_s
        self._pos[1] += (vx * math.sin(self._yaw) + vy * math.cos(self._yaw)) * duration_s
        self._yaw += wz * duration_s
        self._vel = (vx, vy, wz)
        cmd = {"kind": "velocity", "vx": vx, "vy": vy, "wz": wz, "duration_s": duration_s}
        self.commands.append(cmd)
        return cmd

    def set_course(self, move_heading, facing_heading, speed, duration_s) -> dict:
        """Integrate along an ABSOLUTE heading, and snap yaw to the commanded
        facing.

        Deliberately an idealised robot: it goes exactly where it is pointed, at
        exactly the speed asked, immediately. That is what makes it useful for
        testing a control law -- a convergence failure in these tests is the
        law's fault and nothing else's. Whether the real robot tracks a course
        is a separate question that only the simulator can answer.
        """
        speed = max(0.0, min(float(speed), MAX_VX_M_S))
        duration_s = clamp_duration(duration_s)
        self._pos[0] += speed * math.cos(move_heading) * duration_s
        self._pos[1] += speed * math.sin(move_heading) * duration_s
        self._yaw = float(facing_heading)
        self._vel = (speed, 0.0, 0.0)
        cmd = {"kind": "course", "move_heading_rad": float(move_heading),
               "facing_heading_rad": float(facing_heading), "speed": speed,
               "duration_s": duration_s}
        self.commands.append(cmd)
        return cmd

    def stop(self) -> dict:
        self._vel = (0.0, 0.0, 0.0)
        cmd = {"kind": "stop"}
        self.commands.append(cmd)
        return cmd

    def emergency_stop(self) -> dict:
        """Damping is recorded like any other command so a test can assert the
        watchdog's actuator actually fired. Without this the offline suite
        exercised the auto-damp path ten times and every one of them failed
        with AttributeError into a printed line nobody read."""
        self._vel = (0.0, 0.0, 0.0)
        cmd = {"kind": "emergency_stop"}
        self.commands.append(cmd)
        return cmd

    def select_mode(self, name: str, *, duration_s=None, height_m=None,
                    speed=None) -> dict:
        self.mode = name
        cmd = {"kind": "mode", "name": name, "duration_s": duration_s,
               "height_m": height_m, "speed": speed}
        self.commands.append(cmd)
        return cmd

    def set_height(self, height_m) -> dict:
        self._height = 0.788 if height_m is None or height_m < 0 else float(height_m)
        cmd = {"kind": "height", "height_m": height_m}
        self.commands.append(cmd)
        return cmd

    def read_state(self) -> SonicState:
        return SonicState(
            fresh=True,
            root_height_m=self._height,
            tilt_rad=0.0,
            velocity=self._vel,
            mode=self.mode,
            note="FakeSonicLink — integrated commands, no physics",
        )

    # test conveniences
    @property
    def position(self) -> tuple[float, float]:
        return (self._pos[0], self._pos[1])

    @property
    def yaw(self) -> float:
        return self._yaw


@dataclass
class MujocoView:
    """Renders the G1's on-board camera out of a MuJoCo model, as JPEG bytes for
    the vision model. Separate from SonicLink because the camera is a property of
    the simulation, not of the policy — and on hardware it will be a completely
    different device.

    Headless rendering uses EGL; an interactive viewer needs GLX on this Wayland
    box. Getting that backwards looks like a broken simulator, so the caller
    passes it explicitly rather than us guessing.
    """

    model_path: str
    width: int = 640
    height: int = 480
    camera: int | str = 0
    _model: object = field(default=None, repr=False)
    _data: object = field(default=None, repr=False)
    _renderer: object = field(default=None, repr=False)

    def open(self) -> "MujocoView":
        import mujoco  # imported lazily so the module is importable without it

        self._model = mujoco.MjModel.from_xml_path(self.model_path)
        self._data = mujoco.MjData(self._model)
        self._renderer = mujoco.Renderer(self._model, height=self.height, width=self.width)
        return self

    def frame_jpeg(self, quality: int = 85) -> bytes:
        """Latest camera view as JPEG bytes. Raises if the view was never opened —
        a frame we cannot render must fail loudly, never return a placeholder
        image the model would reason about as if it were real."""
        if self._renderer is None:
            raise RuntimeError("MujocoView.open() was never called — no frame to render")
        import io

        import mujoco
        from PIL import Image

        mujoco.mj_forward(self._model, self._data)
        self._renderer.update_scene(self._data, camera=self.camera)
        rgb = self._renderer.render()
        buf = io.BytesIO()
        Image.fromarray(rgb).save(buf, format="JPEG", quality=quality)
        return buf.getvalue()


class ScriptedClock:
    """Injectable clock so bounded commands are testable without sleeping."""

    def __init__(self, start: float = 0.0):
        self.now = start

    def time(self) -> float:
        return self.now

    def sleep(self, seconds: float) -> None:
        self.now += seconds


class RealClock:
    def time(self) -> float:
        return time.monotonic()

    def sleep(self, seconds: float) -> None:
        time.sleep(seconds)


# Upper-body reference support, added 2026-07-31 alongside the planner-topic
# `upper_body_position` field. Defined on the base class and the Fake so the
# whole arm tool surface is testable with no simulator.
def _link_set_upper_body(self, values, *, slew_rad_s=None):
    raise NotImplementedError


def _link_upper_body_travel_time(self):
    """Seconds until a ramped arm reference reaches its target. Zero on links
    that apply the reference instantly, which is every link except the real
    ZMQ one -- the Fake is an idealised robot on purpose."""
    return 0.0


SonicLink.set_upper_body = _link_set_upper_body
SonicLink.upper_body_travel_time = _link_upper_body_travel_time


def _fake_set_upper_body(self, values, *, slew_rad_s=None):
    self.upper_body = None if values is None else list(values)
    cmd = {"kind": "upper_body", "values": self.upper_body,
           "slew_rad_s": slew_rad_s}
    self.commands.append(cmd)
    return cmd


FakeSonicLink.set_upper_body = _fake_set_upper_body
FakeSonicLink.upper_body_travel_time = _link_upper_body_travel_time
FakeSonicLink.upper_body = None
