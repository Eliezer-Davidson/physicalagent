"""One protocol, two backends. Drift between the MuJoCo and Isaac sidecars must
fail here, not on a robot -- this is the same double-drift class that let an
unguarded lowstate() call ship and take the fall watchdog down.

PARSED WITH ast, NOT IMPORTED. sidecar.py needs numpy and isaac_sidecar.py needs
zmq; neither exists in the stdlib-only project venv that runs this suite. Reading
the source keeps the contract enforceable in the same suite as everything else,
on a machine with no simulator attached.
"""
from __future__ import annotations

import ast
import pathlib

import pytest

REPO = pathlib.Path(__file__).resolve().parents[1]
MUJOCO_SRC = REPO / "simbridge" / "sidecar.py"
ISAAC_SRC = REPO / "simbridge" / "isaac_sidecar.py"
# Ops the Isaac backend really implements (2026-08-07: pose/episode from the
# live env, camera from the sim's shm image server, contacts from the scene's
# ContactSensor). Every one of these must also exist on the MuJoCo sidecar --
# one protocol, two backends.
ISAAC_OPS = ("ping", "body_pose", "episode", "camera", "contacts")
# Ops that exist on MuJoCo and are NOT yet implemented on Isaac. Declaring one
# of these on the Isaac sidecar before implementing it must fail here.
NOT_YET_ON_ISAAC = ("scan", "objects", "hands", "hand_state", "band")


def _ops(path):
    """Every _op_* method defined on any class in the file."""
    return {n.name[len("_op_"):]
            for cls in ast.walk(ast.parse(path.read_text()))
            if isinstance(cls, ast.ClassDef)
            for n in cls.body
            if isinstance(n, (ast.FunctionDef, ast.AsyncFunctionDef))
            and n.name.startswith("_op_")}


def _const(path, name):
    for node in ast.parse(path.read_text()).body:
        if isinstance(node, ast.Assign):
            for t in node.targets:
                if isinstance(t, ast.Name) and t.id == name:
                    return ast.literal_eval(node.value)
    raise AssertionError(f"{name} not found in {path.name}")


class TestBothSidecarsShareOneProtocol:
    def test_protocol_versions_match(self):
        assert _const(ISAAC_SRC, "PROTOCOL") == _const(MUJOCO_SRC, "PROTOCOL")

    def test_default_ports_match(self):
        assert (_const(ISAAC_SRC, "DEFAULT_PORT")
                == _const(MUJOCO_SRC, "DEFAULT_PORT") == 5570)

    @pytest.mark.parametrize("op", ISAAC_OPS)
    def test_isaac_implements_the_op(self, op):
        assert op in _ops(ISAAC_SRC), f"Isaac sidecar is missing _op_{op}"

    @pytest.mark.parametrize("op", ISAAC_OPS)
    def test_the_op_exists_on_the_mujoco_sidecar_too(self, op):
        assert op in _ops(MUJOCO_SRC)

    def test_isaac_declares_no_op_it_has_not_implemented(self):
        """A sidecar that advertises a stub is the lie this project spent a day
        removing: _op_ping once reported cameras:['head_camera'] while every
        sensing op was unimplemented."""
        declared = _ops(ISAAC_SRC) & set(NOT_YET_ON_ISAAC)
        assert not declared, (
            f"ops declared before implementation: {sorted(declared)}")


def _dispatch_keys(path):
    """The string keys of the routing table inside _dispatch -- the ops that
    are actually REACHABLE, as opposed to merely defined. Identified precisely
    as the dict literal that .get() is called on ({...}.get(op)), because
    _dispatch also contains error-reply dict literals."""
    for cls in ast.walk(ast.parse(path.read_text())):
        if not isinstance(cls, ast.ClassDef):
            continue
        for fn in cls.body:
            if isinstance(fn, ast.FunctionDef) and fn.name == "_dispatch":
                for node in ast.walk(fn):
                    if (isinstance(node, ast.Call)
                            and isinstance(node.func, ast.Attribute)
                            and node.func.attr == "get"
                            and isinstance(node.func.value, ast.Dict)):
                        return {k.value for k in node.func.value.keys
                                if isinstance(k, ast.Constant)}
    raise AssertionError(f"no dispatch dict found in {path.name}")


class TestShmImageHeaderLayout:
    """The sidecar parses the vendor image server's shared-memory segments
    with its own mirror of SimpleImageHeader. If the mirror drifts (field
    order, a type width), every parsed frame is garbage that may still carry
    a plausible timestamp -- so the exact byte layout is pinned here, in the
    stdlib venv, via ctypes built from the ast-extracted field list."""

    CANON = [("timestamp", "c_uint64"), ("height", "c_uint32"),
             ("width", "c_uint32"), ("channels", "c_uint32"),
             ("image_name", ("c_char", 16)), ("data_size", "c_uint32"),
             ("encoding", "c_uint32"), ("quality", "c_uint32")]

    def _fields_from_source(self):
        for cls in ast.walk(ast.parse(ISAAC_SRC.read_text())):
            if isinstance(cls, ast.ClassDef) and cls.name == "_ShmImageHeader":
                for node in cls.body:
                    if (isinstance(node, ast.Assign)
                            and node.targets[0].id == "_fields_"):
                        out = []
                        for elt in node.value.elts:
                            name = elt.elts[0].value
                            t = elt.elts[1]
                            if isinstance(t, ast.BinOp):        # c_char * 16
                                out.append((name, (t.left.attr, t.right.value)))
                            else:
                                out.append((name, t.attr))
                        return out
        raise AssertionError("_ShmImageHeader._fields_ not found")

    def test_field_list_matches_the_vendor_layout(self):
        assert self._fields_from_source() == self.CANON

    def test_layout_is_48_bytes_with_payload_at_48(self):
        import ctypes
        fields = [(n, getattr(ctypes, t) if isinstance(t, str)
                   else getattr(ctypes, t[0]) * t[1])
                  for n, t in self._fields_from_source()]

        class H(ctypes.LittleEndianStructure):
            _fields_ = fields

        assert ctypes.sizeof(H) == 48
        assert H.data_size.offset == 36 and H.encoding.offset == 40


class TestDeclaredEqualsReachableEqualsAdvertised:
    """Method names, the dispatch table, _IMPLEMENTED_OPS and ping's advertised
    ops are four copies of one fact. Any of them can drift green if only method
    names are checked -- deleting an op from the dispatch dict leaves the
    method defined but unreachable, and every name-based test still passes."""

    def test_isaac_dispatch_table_is_exactly_the_implemented_set(self):
        assert _dispatch_keys(ISAAC_SRC) == set(ISAAC_OPS)

    def test_isaac_implemented_constant_matches_this_test(self):
        assert tuple(_const(ISAAC_SRC, "_IMPLEMENTED_OPS")) == ISAAC_OPS

    def test_ping_advertises_the_constant_not_a_second_list(self):
        assert '"ops": list(_IMPLEMENTED_OPS)' in ISAAC_SRC.read_text()
