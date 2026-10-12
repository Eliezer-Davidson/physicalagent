"""The locomotion plane, pinned to the G1 tier table.

The tier table lives at `docs/design/g1-safety-tiers.md`; this file is
the test it names for the locomotion plane.

Why the numbers are written out as literals here rather than imported:
a test that reads `MAX_VX_M_S` and asserts the code clamps to `MAX_VX_M_S`
is a tautology and cannot catch drift. On 2026-08-18 the code carried
0.6 / 0.3 / 0.8 -- exactly double the table on every axis -- underneath a
comment claiming it matched the table. Every existing clamp test passed.

And the table itself is asserted, in the other direction. The signed
document permits P5b results to LOWER these ceilings and never to raise
them, so a change to either side must fail here and be answered by a
human, not absorbed silently by a symbol.
"""
from __future__ import annotations

import pathlib

import physicalagent.robots.sonic_link as link_mod

# Transcribed by hand from docs/design/g1-safety-tiers.md, the FULL-tier column
# of the "Locomotion velocity commands" row. Do not replace these with imports.
TABLE_MAX_VX_M_S = 0.3
TABLE_MAX_VY_M_S = 0.2
TABLE_MAX_WZ_RAD_S = 0.5

TIER_TABLE = (pathlib.Path(__file__).resolve().parents[1]
              / "docs" / "design" / "g1-safety-tiers.md")


def test_full_tier_clamps_vx_vy_omega_to_table():
    assert link_mod.MAX_VX_M_S == TABLE_MAX_VX_M_S
    assert link_mod.MAX_VY_M_S == TABLE_MAX_VY_M_S
    assert link_mod.MAX_WZ_RAD_S == TABLE_MAX_WZ_RAD_S


def test_signed_table_still_states_the_clamped_ceilings():
    """Guards the other direction: raising the table to meet the code."""
    rows = [
        line
        for line in TIER_TABLE.read_text(encoding="utf-8").splitlines()
        if "Locomotion velocity commands" in line
    ]
    assert len(rows) == 1, f"expected exactly one locomotion row, found {len(rows)}"
    row = rows[0]
    for literal in ("0.3 m/s", "0.2 m/s", "0.5 rad/s"):
        assert literal in row, f"signed table no longer states {literal!r}: {row}"


def test_clamp_velocity_holds_the_table_ceiling_in_both_signs():
    fast = link_mod.clamp_velocity(9.0, 9.0, 9.0)
    assert fast == (TABLE_MAX_VX_M_S, TABLE_MAX_VY_M_S, TABLE_MAX_WZ_RAD_S)
    reverse = link_mod.clamp_velocity(-9.0, -9.0, -9.0)
    assert reverse == (-TABLE_MAX_VX_M_S, -TABLE_MAX_VY_M_S, -TABLE_MAX_WZ_RAD_S)
