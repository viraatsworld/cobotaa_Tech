# Copyright (c) 2026, Fraunhofer IPA.
# All rights reserved.
#
# SPDX-License-Identifier: Apache-2.0

"""Where everything in the Techtory cell sits, in the cell (= environment) frame.

Plain Python -- no Isaac, no torch -- so the layout can be tested on its own.
Every number is transcribed from a source that already places it, and the
source is named next to it; nothing here is tuned.

Quaternions are ``(x, y, z, w)``, Isaac Lab 3.x order.
"""

from __future__ import annotations

import math

__all__ = [
    "BASE_PLATE_POS",
    "CELL_POS",
    "ENV_SPACING",
    "HAMMER_POS",
    "HAMMER_ROT",
    "SHELF_POS",
    "SHELF_ROT",
    "quat_from_rotate_xyz_deg",
    "quat_mul",
    "quat_rotate",
]

Vec3 = tuple[float, float, float]
Quat = tuple[float, float, float, float]

##
# Quaternion helpers.
##


def quat_mul(a: Quat, b: Quat) -> Quat:
    """Hamilton product ``a * b`` of two ``(x, y, z, w)`` quaternions."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def quat_rotate(q: Quat, v: Vec3) -> Vec3:
    """Rotate vector ``v`` by the unit quaternion ``q``."""
    x, y, z, w = quat_mul(quat_mul(q, (v[0], v[1], v[2], 0.0)), (-q[0], -q[1], -q[2], q[3]))
    del w
    return (x, y, z)


def _axis_angle(axis: int, degrees: float) -> Quat:
    half = math.radians(degrees) / 2.0
    xyz = [0.0, 0.0, 0.0]
    xyz[axis] = math.sin(half)
    return (xyz[0], xyz[1], xyz[2], math.cos(half))


def quat_from_rotate_xyz_deg(rx: float, ry: float, rz: float) -> Quat:
    """The rotation a USD ``xformOp:rotateXYZ`` of ``(rx, ry, rz)`` degrees makes.

    USD's rotateXYZ applies X first, then Y, then Z about the parent's fixed
    axes, i.e. the matrix ``Rz * Ry * Rx``. That is the op
    ``techtory_cobotta_isaacsim``'s spawners author, so composing the same way
    reproduces its placement exactly.
    """
    return quat_mul(_axis_angle(2, rz), quat_mul(_axis_angle(1, ry), _axis_angle(0, rx)))


def _compose(parent_pos: Vec3, parent_rot: Quat, local_pos: Vec3, local_rot: Quat) -> tuple[Vec3, Quat]:
    offset = quat_rotate(parent_rot, local_pos)
    pos = (parent_pos[0] + offset[0], parent_pos[1] + offset[1], parent_pos[2] + offset[2])
    return pos, quat_mul(parent_rot, local_rot)


##
# Layout.
##

# cell_link is the environment origin (techtory_demo_description.urdf:
# world -> cell_link is identity). Its mesh spans about 2.18 x 2.18 x 2.13 m
# (robot/assets/cell_bounds.json).
CELL_POS: Vec3 = (0.0, 0.0, 0.0)

# cell_link -> robot_base_plate_link (techtory_cell.xacro).
BASE_PLATE_POS: Vec3 = (-0.275, -0.24, 0.94)

# The robot mount is in robot/joints.py (MOUNT_POS / MOUNT_ROT): 2 cm above
# the plate, yawed 90 degrees.

# <xacro:shelf prefix="shelf_"> origin in techtory_cobotta_workcell.urdf.xacro,
# and the same pose add_shelf() authors in techtory_cobotta_isaacsim.
SHELF_POS: Vec3 = (0.61, 0.27, 0.94)
SHELF_ROT: Quat = quat_from_rotate_xyz_deg(0.0, 0.0, 90.0)

# techtory_cobotta_isaacsim spawns the hammer as a CHILD of the shelf prim
# (/World/Shelf/Hammer), so its translate/rotateXYZ are in the shelf frame.
# Composed here rather than hand-typed, so moving the shelf moves the hammer.
_HAMMER_LOCAL_POS: Vec3 = (0.04318, 0.0, 0.43525)
_HAMMER_LOCAL_ROT: Quat = quat_from_rotate_xyz_deg(90.0, 0.0, 180.0)
HAMMER_POS, HAMMER_ROT = _compose(SHELF_POS, SHELF_ROT, _HAMMER_LOCAL_POS, _HAMMER_LOCAL_ROT)

# Distance between neighbouring cells. The cell is ~2.18 m on a side, so 3 m
# leaves ~0.8 m of empty floor between them; tests/test_placement.py checks
# this against the committed mesh bounds.
ENV_SPACING = 3.0
