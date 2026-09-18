# Copyright (c) 2026, Fraunhofer IPA.
# All rights reserved.
#
# SPDX-License-Identifier: Apache-2.0

"""Scene layout: quaternion helpers, the hammer composition, env spacing."""

from __future__ import annotations

import json
import math

import pytest

from conftest import ASSETS_DIR, joints, placement

P = placement()


def _norm(q: tuple[float, ...]) -> float:
    return math.sqrt(sum(c * c for c in q))


def test_yaw_quaternion_rotates_x_onto_y() -> None:
    q = P.quat_from_rotate_xyz_deg(0.0, 0.0, 90.0)
    assert P.quat_rotate(q, (1.0, 0.0, 0.0)) == pytest.approx((0.0, 1.0, 0.0), abs=1e-12)


def test_rotate_xyz_applies_x_before_z() -> None:
    """USD rotateXYZ is Rz * Ry * Rx: X first. (1,0,0) -X90-> (1,0,0) -Z90-> (0,1,0);
    the opposite order would also give (0,1,0), so use +y instead:
    (0,1,0) -X90-> (0,0,1) -Z90-> (0,0,1)."""
    q = P.quat_from_rotate_xyz_deg(90.0, 0.0, 90.0)
    assert P.quat_rotate(q, (0.0, 1.0, 0.0)) == pytest.approx((0.0, 0.0, 1.0), abs=1e-12)


def test_robot_mount_matches_the_placement_convention() -> None:
    assert joints().MOUNT_ROT == pytest.approx(P.quat_from_rotate_xyz_deg(0.0, 0.0, 90.0))
    # 2 cm above the base plate, as joint_w in the workcell xacro.
    plate = P.BASE_PLATE_POS
    assert joints().MOUNT_POS == pytest.approx((plate[0], plate[1], plate[2] + 0.02))


def test_hammer_is_the_shelf_local_pose_composed() -> None:
    # Shelf yaw 90: local +x -> world +y, local z unchanged.
    assert P.HAMMER_POS == pytest.approx((0.61, 0.27 + 0.04318, 0.94 + 0.43525))
    assert _norm(P.HAMMER_ROT) == pytest.approx(1.0)
    # Above the middle board's top surface (board centre 0.34, 0.04 thick).
    assert P.HAMMER_POS[2] > P.SHELF_POS[2] + 0.36


def test_env_spacing_clears_the_cell() -> None:
    bounds = json.loads((ASSETS_DIR / "cell_bounds.json").read_text())["cell"]
    footprint = max(hi - lo for lo, hi in zip(bounds["min"][:2], bounds["max"][:2], strict=True))
    assert P.ENV_SPACING >= footprint + 0.5
