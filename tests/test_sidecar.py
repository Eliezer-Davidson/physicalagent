"""Sidecar ops, exercised against a REAL MuJoCo model.

Not mocked. The things that break here -- mj_ray's argument order, the
renderer's GL context, whether a ray fired from inside the robot escapes it --
are exactly the things a mock would paper over, and each of them fails silently
rather than loudly: a scan that hits the robot's own thigh returns a perfectly
well-formed 0.15 m in every direction.

Skipped entirely when mujoco is absent, so the offline suite still runs.
"""
from __future__ import annotations

import pathlib
import sys
import threading

import pytest

mujoco = pytest.importorskip("mujoco")
pytest.importorskip("zmq")
pytest.importorskip("cv2")

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from simbridge.sidecar import Sidecar  # noqa: E402

from physicalagent.paths import G1_MODEL_DIR  # noqa: E402

SCENE = G1_MODEL_DIR / "scene_29dof_inspire_actuated.xml"
RIGID = G1_MODEL_DIR / "scene_29dof_inspire_rigid.xml"

pytestmark = pytest.mark.skipif(not SCENE.exists(),
                                reason="actuated scene not generated")

_PORT = [5700]


class FakeEnv:
    """Stands in for DefaultEnv: the sidecar only ever touches mj_model,
    mj_data, elastic_band and sidecar_finger_targets."""

    def __init__(self, scene: pathlib.Path, extra_xml: str = ""):
        if extra_xml:
            # Wrap the scene in a model that also contains an obstacle, so the
            # scan can be tested for a TRUE positive. An empty world only ever
            # proves the scan returns nothing, which it would also do if it were
            # completely broken.
            #
            # Written NEXT TO the scene, not to a temp dir: the model's mesh
            # paths are relative, and from_xml_string has no directory to
            # resolve them against.
            wrapper = scene.parent / "_pytest_scene_with_obstacles.xml"
            wrapper.write_text(f'<mujoco><include file="{scene.name}"/>'
                               f'<worldbody>{extra_xml}</worldbody></mujoco>')
            try:
                self.mj_model = mujoco.MjModel.from_xml_path(str(wrapper))
            finally:
                wrapper.unlink(missing_ok=True)
        else:
            self.mj_model = mujoco.MjModel.from_xml_path(str(scene))
        self.mj_data = mujoco.MjData(self.mj_model)
        mujoco.mj_forward(self.mj_model, self.mj_data)
        self.elastic_band = None
        self.sidecar_finger_targets = {}


# A box 2 m in front of the robot (+x) at chest height, and one 3 m to its left.
# Both carry their geom AT the body origin, so these ranges and bearings mean
# the same thing before and after _op_objects moved to the bounding-box centre.
# That is a property of THIS fixture, not of scenes in general -- the lounge
# furniture's geometry sits up to 1.8 m from its body origin, which is the
# defect tests/test_sidecar_geometry.py covers.
OBSTACLES = """
  <body name="crate" pos="2 0 0.9">
    <geom type="box" size="0.3 0.3 0.3" rgba="0.8 0.4 0.1 1"/>
  </body>
  <body name="pillar" pos="0 3 1.0">
    <geom type="cylinder" size="0.2 1.0" rgba="0.3 0.3 0.9 1"/>
  </body>
"""


@pytest.fixture(scope="module")
def world():
    """The robot plus two named obstacles at known places."""
    return FakeEnv(SCENE, OBSTACLES)


@pytest.fixture(scope="module")
def world_sidecar(world):
    _PORT[0] += 1
    sc = Sidecar(world, port=_PORT[0])
    yield sc
    sc.close()


@pytest.fixture(scope="module")
def env():
    return FakeEnv(SCENE)


@pytest.fixture(scope="module")
def sidecar(env):
    _PORT[0] += 1
    sc = Sidecar(env, port=_PORT[0])
    yield sc
    sc.close()


def call(sidecar, op, **kw):
    """Bypass the socket and call the dispatcher directly. The transport is
    ordinary REQ/REP; what is worth testing is the physics behind it."""
    return sidecar._dispatch({"op": op, **kw})


class TestPing:
    def test_reports_the_model_it_actually_loaded(self, sidecar):
        r = call(sidecar, "ping")
        assert r["ok"] and r["nq"] == 60 and r["nu"] == 53
        assert "head_camera" in r["cameras"]
        assert r["hand_actuators"] == 2       # two sides present


class TestScan:
    def test_finds_a_real_obstacle_at_the_right_range_and_bearing(self, world_sidecar):
        """The true positive. A crate 2 m straight ahead must come back as a
        crate, at 2 m, at bearing 0 -- not merely as 'something'."""
        r = call(world_sidecar, "scan", beams=36)
        assert r["ok"]
        ahead = [(i, rng) for i, rng in enumerate(r["ranges_m"])
                 if rng is not None and r["hit"][i] == "crate"]
        assert ahead, f"the crate was not seen at all: {r['hit']}"
        i, rng = min(ahead, key=lambda t: t[1])
        assert rng == pytest.approx(1.7, abs=0.4), "wrong range to the crate"
        bearing = 360.0 * i / r["beams"]
        assert bearing < 20 or bearing > 340, f"crate reported at {bearing} deg"

    def test_distinguishes_two_obstacles_in_different_directions(self, world_sidecar):
        r = call(world_sidecar, "scan", beams=36)
        seen = {h for h in r["hit"] if h}
        assert {"crate", "pillar"} <= seen, f"only saw {seen}"

    def test_never_reports_the_robots_own_body_as_an_obstacle(self, world_sidecar):
        r = call(world_sidecar, "scan", beams=36)
        for h in r["hit"]:
            if h and h.startswith("("):
                continue                     # the explicit 'unknown' marker
            assert not h or h in ("crate", "pillar"), f"scan returned {h!r}"

    def test_a_ray_that_never_escapes_is_unknown_and_NOT_a_range(self, sidecar):
        """This was a live bug: after the re-cast budget ran out the accumulated
        self-hit distance was returned as a nameless obstacle at 0.12 m, which
        reads as a wall in the robot's face. Unknown must be None with a reason,
        never a number."""
        r = call(sidecar, "scan", beams=24)
        for rng, name in zip(r["ranges_m"], r["hit"]):
            if name and name.startswith("(unknown"):
                assert rng is None
            if rng is not None:
                assert name is not None, (
                    "a range with no name is an unattributable obstacle")

    def test_the_floor_is_not_reported_as_an_obstacle(self, sidecar):
        """Rays are horizontal, so a hit on the ground plane means the geometry
        or the origin height is wrong."""
        r = call(sidecar, "scan", beams=16)
        assert not any(h == "world" for h in r["hit"] if h)

    def test_empty_scene_reports_nothing_rather_than_zero(self, sidecar):
        """An empty world must yield None ranges. Returning 0.0 would read as
        'obstacle touching the robot' — the most dangerous possible wrong answer
        for a tool whose job is collision avoidance."""
        r = call(sidecar, "scan", beams=12, max_range=10.0)
        assert all(x is None or x > 0.0 for x in r["ranges_m"])

    def test_max_range_hides_things_that_are_too_far(self, world_sidecar):
        near = call(world_sidecar, "scan", beams=36, max_range=1.0)
        assert "crate" not in {h for h in near["hit"] if h}
        far = call(world_sidecar, "scan", beams=36, max_range=10.0)
        assert "crate" in {h for h in far["hit"] if h}

    def test_beam_count_is_bounded(self, sidecar):
        assert call(sidecar, "scan", beams=9999)["beams"] <= 180
        assert call(sidecar, "scan", beams=0)["beams"] >= 4

    def test_bearings_are_labelled_relative_to_the_robot(self, sidecar):
        r = call(sidecar, "scan", beams=8)
        assert "forward" in r["bearing_deg_is_relative_to"]
        assert "NOT the G1's Livox" in r["source"], (
            "the scan must never be presented as the real lidar")


class TestObjects:
    def test_the_robots_own_links_are_not_listed_as_objects(self, sidecar):
        r = call(sidecar, "objects")
        names = [o["name"] for o in r["objects"]]
        assert not any("wrist" in n or "elbow" in n or "pelvis" in n for n in names)

    def test_lists_real_objects_with_range_and_bearing(self, world_sidecar):
        r = call(world_sidecar, "objects")
        by_name = {o["name"]: o for o in r["objects"]}
        assert {"crate", "pillar"} <= set(by_name)
        assert by_name["crate"]["range_m"] == pytest.approx(2.0, abs=0.1)
        assert abs(by_name["crate"]["bearing_deg"]) < 5      # dead ahead
        assert by_name["pillar"]["bearing_deg"] == pytest.approx(90.0, abs=5)
        # Measured off the geometry, not the body frame. NOTE: this module has
        # never executed in this venv (no zmq), so treat it as unproven until
        # it does -- the covering test is tests/test_sidecar_geometry.py.
        assert by_name["crate"]["measured_from"] == "bounding_box_centre"
        assert by_name["crate"]["size_m"] == pytest.approx([0.6, 0.6, 0.6], abs=0.01)

    def test_sorted_nearest_first(self, world_sidecar):
        ranges = [o["range_m"] for o in call(world_sidecar, "objects")["objects"]]
        assert ranges == sorted(ranges)

    def test_bearing_is_wrapped_to_a_half_turn(self, sidecar):
        for o in call(sidecar, "objects")["objects"]:
            assert -180.0 <= o["bearing_deg"] <= 180.0


class TestContacts:
    def test_reports_the_ground_contact_and_not_self_contact(self, env, sidecar):
        mujoco.mj_step(env.mj_model, env.mj_data)
        r = call(sidecar, "contacts")
        assert r["ok"]
        for row in r["contacts"]:
            assert row["robot_part"] != row["touching"]


class TestCamera:
    def test_renders_a_real_jpeg_from_the_head_camera(self, sidecar):
        r = call(sidecar, "camera", width=160, height=120)
        if not r.get("ok"):
            pytest.skip(f"no GL context here: {r.get('error')}")
        import base64
        blob = base64.b64decode(r["jpeg_b64"])
        assert blob[:2] == b"\xff\xd8", "not a JPEG"
        assert len(blob) > 500

    def test_depth_is_metric_and_downsampled(self, sidecar):
        r = call(sidecar, "camera", depth=True, width=160, height=120)
        if not r.get("ok"):
            pytest.skip("no GL context here")
        assert r["kind"] == "depth_m"
        assert len(r["grid_rows_top_to_bottom"]) == 3
        assert all(len(row) == 5 for row in r["grid_rows_top_to_bottom"])


class TestHands:
    def test_a_scalar_closure_moves_every_finger_of_one_hand(self, env, sidecar):
        env.sidecar_finger_targets.clear()
        r = call(sidecar, "hands", right=1.0)
        assert r["ok"]
        assert r["targets_commanded"]["right"]["joints_set"] == 12
        assert len(env.sidecar_finger_targets) == 12

    def test_the_reply_is_named_and_worded_as_a_command_not_a_measurement(
            self, env, sidecar):
        """A live run had the model read this echo and report that the angles
        'confirm the fingers have moved'. They confirm only what was sent. The
        field name and the note both have to say so."""
        env.sidecar_finger_targets.clear()
        r = call(sidecar, "hands", right=1.0)
        assert "applied" not in r and "joints" not in r
        assert "targets_commanded" in r
        assert "TARGETS, not measurements" in r["note"]
        assert "hand_state" in r["note"]

    def test_hand_state_reports_measured_angles_and_the_gap_to_target(
            self, env, sidecar):
        env.sidecar_finger_targets.clear()
        call(sidecar, "hands", right=1.0)
        r = call(sidecar, "hand_state")
        assert r["ok"]
        row = r["hands"]["right"]["R_index_proximal_joint"]
        assert "angle_rad" in row and "target_rad" in row and "error_rad" in row
        # Nothing has stepped, so the fingers are still open and short of target
        assert row["angle_rad"] == pytest.approx(0.0, abs=1e-6)
        assert "R_index_proximal_joint" in r["stalled_short_of_target"]

    def test_hand_state_refuses_on_a_rigid_scene(self):
        _PORT[0] += 1
        sc = Sidecar(FakeEnv(RIGID), port=_PORT[0])
        try:
            assert sc._dispatch({"op": "hand_state"})["ok"] is False
        finally:
            sc.close()

    def test_named_joint_targets_are_written_verbatim(self, env, sidecar):
        env.sidecar_finger_targets.clear()
        call(sidecar, "hands", left={"L_index_proximal_joint": 0.9})
        assert list(env.sidecar_finger_targets.values()) == [0.9]

    def test_closure_is_clamped_to_the_unit_interval(self, env, sidecar):
        env.sidecar_finger_targets.clear()
        call(sidecar, "hands", right=99.0)
        m = env.mj_model
        for actuator_id, target in env.sidecar_finger_targets.items():
            joint = int(m.actuator_trnid[actuator_id][0])
            lo, hi = m.jnt_range[joint]
            assert lo - 1e-9 <= target <= hi + 1e-9

    @pytest.mark.skipif(not RIGID.exists(), reason="rigid scene missing")
    def test_a_rigid_hand_scene_REFUSES_rather_than_silently_succeeding(self):
        """The failure mode this guards: a model told to grasp, on a robot whose
        hands are solid plastic, getting back {"ok": true} and reporting a hold
        that is physically impossible."""
        _PORT[0] += 1
        sc = Sidecar(FakeEnv(RIGID), port=_PORT[0])
        try:
            r = sc._dispatch({"op": "hands", "right": 1.0})
            assert r["ok"] is False
            assert "rigid" in r["error"]
        finally:
            sc.close()


class TestFallDetection:
    """A fall is followed by an automatic reset that stands the robot back up.
    Every other channel then reports a healthy robot, so unless the reset itself
    is reported, a failed manoeuvre is indistinguishable from a successful one.
    Observed live before this existed: an untested mode dropped the robot, the
    sim reset it, and the model reported 'remains fully balanced and upright'.
    """

    def _fresh(self):
        _PORT[0] += 1
        return Sidecar(FakeEnv(SCENE), port=_PORT[0])

    def test_clean_run_reports_no_fall(self):
        sc = self._fresh()
        try:
            sc.poll()
            r = sc._dispatch({"op": "episode"})
            assert r["fell_and_was_reset_since_last_check"] is False
            assert r["falls_this_session"] == 0
        finally:
            sc.close()

    def test_sim_time_going_backwards_is_caught_as_a_fall(self):
        sc = self._fresh()
        try:
            sc._env.mj_data.time = 12.0
            sc.poll()
            sc._env.mj_data.time = 0.0          # what mj_resetData does
            sc.poll()
            r = sc._dispatch({"op": "episode"})
            assert r["fell_and_was_reset_since_last_check"] is True
            assert r["falls_this_session"] == 1
            assert "FAILED" in r["note"]
        finally:
            sc.close()

    def test_the_latch_clears_so_one_fall_is_not_reported_forever(self):
        """A sticky flag would poison every later reading and the model would
        stop believing it."""
        sc = self._fresh()
        try:
            sc._env.mj_data.time = 5.0
            sc.poll()
            sc._env.mj_data.time = 0.0
            sc.poll()
            assert sc._dispatch({"op": "episode"})["fell_and_was_reset_since_last_check"]
            second = sc._dispatch({"op": "episode"})
            assert second["fell_and_was_reset_since_last_check"] is False
            assert second["falls_this_session"] == 1   # the total still stands
        finally:
            sc.close()

    def test_a_fall_between_two_checks_is_not_lost(self):
        """The whole point: the model asks before and after a manoeuvre, and the
        fall happens in between, at a moment nobody is looking."""
        sc = self._fresh()
        try:
            sc.poll()
            assert not sc._dispatch({"op": "episode"})["fell_and_was_reset_since_last_check"]
            for t in (1.0, 2.0, 0.0, 0.1, 0.2):    # fall in the middle
                sc._env.mj_data.time = t
                sc.poll()
            assert sc._dispatch({"op": "episode"})["fell_and_was_reset_since_last_check"]
        finally:
            sc.close()


class TestRobustness:
    def test_an_unknown_op_raises_so_poll_can_turn_it_into_a_refusal(self, sidecar):
        """_dispatch raises; poll() catches and replies with the error text. The
        model gets a sentence it can act on instead of a hung socket."""
        with pytest.raises(ValueError) as exc:
            sidecar._dispatch({"op": "self_destruct"})
        assert "self_destruct" in str(exc.value)

    def test_poll_never_raises_into_the_simulation_loop(self, sidecar):
        """A broken sidecar must not be able to stop the robot."""
        sidecar.poll()          # nothing pending
        sidecar.poll()

    def test_every_reply_carries_the_simulation_time(self, sidecar):
        assert "sim_time_s" in call(sidecar, "ping")


class TestThreadSafety:
    def test_binding_twice_on_one_port_is_refused(self, env):
        """Two sidecars on one port would each answer half the requests, and the
        model would see a scene that flickered between two simulations."""
        _PORT[0] += 1
        first = Sidecar(env, port=_PORT[0])
        try:
            with pytest.raises(Exception):
                Sidecar(env, port=_PORT[0])
        finally:
            first.close()


def test_import_is_thread_safe_enough_for_the_sim_loop():
    """The sim loop imports the sidecar lazily on its first step. Nothing in the
    module may block on construction."""
    done = threading.Event()
    threading.Thread(target=lambda: (__import__("simbridge.sidecar"),
                                     done.set()), daemon=True).start()
    assert done.wait(5.0)
