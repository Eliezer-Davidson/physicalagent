"""Where an object IS, versus where its body frame happens to sit.

Runs against a REAL MuJoCo model built in-process -- a handful of primitives,
no meshes, ~10 ms to compile -- because the thing under test is MuJoCo's own
geom bookkeeping (body_geomadr / geom_aabb / geom_xmat / geom_xpos), and a mock
of that would assert only that the mock was written to match the code.

NOT in tests/test_sidecar.py on purpose. That file needs zmq to construct a
Sidecar, zmq is not installed in this venv, and the whole module is therefore
SKIPPED -- its existing TestObjects has never executed here, which is why a
defect this size survived in _op_objects. The geometry lives in module-level
functions that import with numpy alone, and the Sidecar is built with __new__
below, so this file actually runs.

WHAT THIS FILE DOES NOT PROVE. No simulator has served this reply to a model.
The arithmetic is proven here and the lounge magnitudes were measured off the
live scene XML with mj_forward on CPU; the behaviour of the agent given a
correct scene awaits a live run.
"""
from __future__ import annotations

import pathlib
import sys

import numpy as np
import pytest

mujoco = pytest.importorskip("mujoco")

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from simbridge.sidecar import Sidecar, body_extent, footprint_range_m  # noqa: E402
from simbridge import sidecar as sidecar_mod  # noqa: E402

# The bug, reproduced to scale. Every lounge body keeps the SOURCE SCAN origin
# (measured on the live scene: all five bodies within 2 cm of each other at
# z~0) while the object it represents stands up to 2.5 m away and up to 0.71 m
# off the floor. scan_origin_thermos and scan_origin_bench are 2 mm apart and
# carry geometry 1 m apart -- the shipped failure, in six primitives.
#
# pelvis is FIRST in worldbody so its freejoint owns qpos[0:7], which is what
# _op_objects and _heading_rad read as the robot's pose.
XML = """
<mujoco>
  <worldbody>
    <geom name="floor" type="plane" size="0 0 0.05"/>
    <body name="pelvis" pos="0 0 0.8">
      <freejoint/>
      <geom type="capsule" fromto="0 0 0 0 0 0.2" size="0.08"/>
    </body>
    <body name="scan_origin_thermos" pos="0 0 0">
      <freejoint/>
      <geom type="cylinder" pos="0 -2 0.5" size="0.04 0.053"/>
    </body>
    <body name="scan_origin_bench" pos="0.002 -0.001 0">
      <freejoint/>
      <geom type="box" pos="0.6 -3 0.35" size="0.7 0.35 0.35"/>
    </body>
    <body name="ghost" pos="1 1 1">
      <freejoint/>
      <inertial pos="0 0 0" mass="1" diaginertia="0.1 0.1 0.1"/>
    </body>
    <body name="tilted" pos="0 0 0" euler="0 0 45">
      <freejoint/>
      <geom type="box" pos="3 0 0.5" size="0.5 0.1 0.05"/>
    </body>
    <body name="stage" pos="0 5 0">
      <geom type="plane" size="0 0 0.05"/>
      <geom type="box" pos="0 0 0.1" size="0.3 0.3 0.1"/>
    </body>
  </worldbody>
</mujoco>
"""


class FakeEnv:
    """The two attributes _op_objects touches. DefaultEnv is a simulator."""

    def __init__(self, m, d):
        self.mj_model, self.mj_data = m, d


@pytest.fixture(scope="module")
def world():
    m = mujoco.MjModel.from_xml_string(XML)
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)                  # positions only; no stepping
    return m, d


@pytest.fixture(scope="module")
def objects(world):
    """The REAL _op_objects reply, keyed by name.

    Sidecar.__init__ binds a zmq REP socket and zmq is not in this venv -- and
    the socket is not what is under test. __new__ plus the two attributes the
    op actually reads exercises the shipping code path rather than a copy of
    it; _dispatch and _op_objects touch only self._env and
    self._robot_body_ids.
    """
    m, d = world
    sc = Sidecar.__new__(Sidecar)
    sc._env = FakeEnv(m, d)
    sc._robot_body_ids = None                # _robot_bodies() fills this
    reply = sc._dispatch({"op": "objects"})
    assert reply["ok"]
    return {o["name"]: o for o in reply["objects"]}


class TestTheGeomTypeConstantsAreTheRealOnes:
    def test_unbounded_types_are_plane_and_hfield(self):
        """The constant is written as integer literals so sidecar.py imports
        without mujoco. That is only safe while the literals are still right."""
        assert sidecar_mod._UNBOUNDED_GEOM_TYPES == (
            int(mujoco.mjtGeom.mjGEOM_PLANE), int(mujoco.mjtGeom.mjGEOM_HFIELD))


class TestObjectsAreReportedWhereTheirGeometryIs:
    def test_the_robot_is_not_listed_among_the_objects(self, objects):
        assert "pelvis" not in objects

    def test_the_thermos_is_2m_away_and_not_at_the_body_origin(self, objects):
        t = objects["scan_origin_thermos"]
        assert t["range_m"] == pytest.approx(2.0, abs=0.01)
        assert t["range_m"] > 1.0, "reported at the body origin, not the geom"

    def test_the_thermos_reads_at_table_height_not_at_the_feet(self, objects):
        # geom centre z=0.5, pelvis z=0.8 -> -0.30. The BODY origin is at z=0,
        # i.e. -0.80: the robot's ankles. That was the shipped number, for an
        # object standing on a table.
        t = objects["scan_origin_thermos"]
        assert t["height_m"] == pytest.approx(-0.30, abs=0.01)
        assert t["top_m"] == pytest.approx(-0.25, abs=0.01)

    def test_two_bodies_sharing_an_origin_are_two_different_places(self, objects):
        # 2 mm apart as bodies. goto_object() resolved both to one point.
        t, b = objects["scan_origin_thermos"], objects["scan_origin_bench"]
        assert abs(t["range_m"] - b["range_m"]) > 0.5
        assert abs(t["bearing_deg"] - b["bearing_deg"]) > 5.0

    def test_size_is_reported_and_is_the_object_not_the_scene(self, objects):
        assert objects["scan_origin_thermos"]["size_m"] == \
            pytest.approx([0.08, 0.08, 0.11], abs=0.01)
        assert objects["scan_origin_bench"]["size_m"] == \
            pytest.approx([1.4, 0.7, 0.7], abs=0.01)

    def test_a_rotated_object_reports_its_world_envelope(self, objects):
        # A 1.0 x 0.2 box turned 45 deg occupies 0.849 in both x and y. Reading
        # geom_size directly would report 1.0 x 0.2 along the wrong axes -- and
        # for the lounge's meshes geom_size does not describe the shape at all.
        s = objects["tilted"]["size_m"]
        assert s[0] == pytest.approx(0.85, abs=0.01)
        assert s[1] == pytest.approx(0.85, abs=0.01)

    def test_nearest_edge_is_shorter_than_the_range_to_the_centre(self, objects):
        b = objects["scan_origin_bench"]
        assert b["nearest_edge_m"] < b["range_m"]
        assert b["nearest_edge_m"] == pytest.approx(2.65, abs=0.02)

    def test_the_list_is_still_sorted_by_the_range_it_displays(self, world):
        m, d = world
        sc = Sidecar.__new__(Sidecar)
        sc._env = FakeEnv(m, d)
        sc._robot_body_ids = None
        ranges = [o["range_m"] for o in sc._dispatch({"op": "objects"})["objects"]]
        assert ranges == sorted(ranges)

    def test_standing_over_a_footprint_reads_zero_not_a_negative_number(self):
        assert footprint_range_m([-1.0, -1.0, 0.0], [1.0, 1.0, 1.0],
                                 0.2, -0.3) == 0.0


class TestItRefusesToInventGeometryItDoesNotHave:
    def test_a_body_with_no_geoms_has_no_extent(self, world):
        m, d = world
        assert body_extent(m, d, m.body("ghost").id) is None

    def test_a_geomless_body_falls_back_to_the_origin_and_says_so(self, objects):
        # A silent fallback to d.xpos here would be the original bug, hiding
        # inside the fix. It must be visible ON THE OBJECT.
        g = objects["ghost"]
        assert g["measured_from"].startswith("body_origin")
        assert "size_m" not in g and "top_m" not in g and "nearest_edge_m" not in g

    def test_a_measured_object_says_where_the_number_came_from(self, objects):
        assert objects["scan_origin_thermos"]["measured_from"] == \
            "bounding_box_centre"

    def test_an_infinite_plane_does_not_become_the_objects_size(self, objects):
        # MuJoCo gives a plane half-extents of 1e10. One unfiltered plane geom
        # makes every object in the scene the size of the world.
        assert objects["stage"]["size_m"] == pytest.approx([0.6, 0.6, 0.2],
                                                           abs=0.01)
        assert max(objects["stage"]["size_m"]) < 100.0


class TestTheMathMatchesTheObviousLoop:
    def test_the_vectorised_box_equals_a_naive_per_geom_loop(self, world):
        """The einsum is an optimisation of something obvious. If they ever
        disagree the obvious one is right."""
        m, d = world
        for name in ("scan_origin_bench", "tilted", "stage"):
            b = m.body(name).id
            box = body_extent(m, d, b)
            adr, num = int(m.body_geomadr[b]), int(m.body_geomnum[b])
            lo, hi = np.full(3, np.inf), np.full(3, -np.inf)
            for g in range(adr, adr + num):
                if int(m.geom_type[g]) in (0, 1):
                    continue
                rot = np.array(d.geom_xmat[g]).reshape(3, 3)
                a = np.array(m.geom_aabb[g])
                c = np.array(d.geom_xpos[g]) + rot @ a[:3]
                h = np.abs(rot) @ a[3:]
                lo, hi = np.minimum(lo, c - h), np.maximum(hi, c + h)
            assert np.allclose(lo, box["lo"]) and np.allclose(hi, box["hi"]), name
