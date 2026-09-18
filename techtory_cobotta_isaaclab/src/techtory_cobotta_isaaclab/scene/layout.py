# Copyright (c) 2026, Fraunhofer IPA.
# All rights reserved.
#
# SPDX-License-Identifier: Apache-2.0

"""Where everything sits in the cell, as plain numbers.

Every pose here is in the **cell frame**, which is also each environment's
origin: the workcell USD is spawned at the origin, so these are the same
coordinates ``techtory_cobotta_isaacsim/scripts/main.py`` and the workcell URDF
use. Quaternions are ``(x, y, z, w)`` -- Isaac Lab 3.x order.

Pure Python on purpose: no torch, no ``pxr``. The scene configs are imported
before Kit starts, and ``tests/test_layout.py`` checks these numbers against a
USD stage that reproduces the Isaac Sim demo's authoring.

.. note::
    An object's pose here is where its **rigid body** goes, not where its USD
    file's root prim goes. Isaac Lab's ``RigidObject`` writes ``init_state`` onto
    the body prim on every reset, so a body authored with its own offset inside
    the file (the hammer's) must have that offset folded in, or the object turns
    over on the first reset.
"""

from __future__ import annotations

import math
from typing import NamedTuple

__all__ = [
    "HAMMER_BODY",
    "HAMMER_BODY_OFFSET",
    "HAMMER_ON_SHELF",
    "IDENTITY",
    "ROBOT_MOUNT",
    "SHELF",
    "SODA_CAN_BODY",
    "SODA_CAN_ON_SHELF",
    "Pose",
    "compose",
    "quat_from_rotate_xyz_deg",
    "quat_mul",
    "quat_rotate",
    "yaw_deg",
]

Vec3 = tuple[float, float, float]
Quat = tuple[float, float, float, float]

IDENTITY: Quat = (0.0, 0.0, 0.0, 1.0)


class Pose(NamedTuple):
    """A rigid transform: position [m] and orientation ``(x, y, z, w)``."""

    pos: Vec3
    rot: Quat = IDENTITY


def quat_mul(a: Quat, b: Quat) -> Quat:
    """Hamilton product ``a * b`` (apply ``b`` first, then ``a``)."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def quat_rotate(q: Quat, v: Vec3) -> Vec3:
    """Rotate ``v`` by the unit quaternion ``q``."""
    x, y, z, _ = quat_mul(quat_mul(q, (v[0], v[1], v[2], 0.0)), (-q[0], -q[1], -q[2], q[3]))
    return (x, y, z)


def _axis_angle(axis: Vec3, angle_rad: float) -> Quat:
    s = math.sin(angle_rad / 2.0)
    return (axis[0] * s, axis[1] * s, axis[2] * s, math.cos(angle_rad / 2.0))


def yaw_deg(yaw: float) -> Quat:
    """Rotation about +Z by ``yaw`` degrees."""
    return _axis_angle((0.0, 0.0, 1.0), math.radians(yaw))


def quat_from_rotate_xyz_deg(rx: float, ry: float, rz: float) -> Quat:
    """The rotation a USD ``xformOp:rotateXYZ`` of ``(rx, ry, rz)`` degrees authors.

    USD applies X first, then Y, then Z, which is the product ``Rz * Ry * Rx``.
    The Isaac Sim demo places its objects with exactly this op.
    """
    qx = _axis_angle((1.0, 0.0, 0.0), math.radians(rx))
    qy = _axis_angle((0.0, 1.0, 0.0), math.radians(ry))
    qz = _axis_angle((0.0, 0.0, 1.0), math.radians(rz))
    return quat_mul(qz, quat_mul(qy, qx))


def compose(parent: Pose, child: Pose) -> Pose:
    """``child`` expressed in ``parent``'s frame, returned in the frame ``parent`` is expressed in.

    The orientation comes back with ``w >= 0``; ``q`` and ``-q`` are the same rotation.
    """
    offset = quat_rotate(parent.rot, child.pos)
    pos = (parent.pos[0] + offset[0], parent.pos[1] + offset[1], parent.pos[2] + offset[2])
    rot = quat_mul(parent.rot, child.rot)
    if rot[3] < 0.0:
        rot = (-rot[0], -rot[1], -rot[2], -rot[3])
    return Pose(pos, rot)


##
# The cell.
##

ROBOT_MOUNT = Pose((-0.275, -0.24, 0.96), yaw_deg(90.0))
"""Cobotta base in the cell.

The workcell URDF chain: ``cell_link -> robot_base_plate_link`` at
``(-0.275, -0.24, 0.94)``, then ``+0.02`` m and a quarter turn to
``cobotta_pro_base_link``. The plate's top face is at z = 0.956, so 0.96 leaves
4 mm of clearance instead of burying the base in the plate.
"""

SHELF = Pose((0.61, 0.27, 0.94), yaw_deg(90.0))
"""Shelf bottom-centre, standing on the table (``shelf_`` origin in the workcell xacro).

In the shelf's own frame the three boards' top faces are at z = 0.16, 0.36 and
0.80, the boards span x in [-0.5, 0.5] and y in [-0.15, 0.15], and +y faces the
robot.
"""

##
# Objects, placed on the shelf's middle board.
##

HAMMER_ON_SHELF = Pose((0.04318, 0.0, 0.3733), quat_from_rotate_xyz_deg(90.0, 0.0, 180.0))
"""Hammer file root in the shelf frame, lying flat on the middle board.

x, y and the orientation are the Isaac Sim demo's (``spawn_objects.add_hammer``).
z is not: the demo's 0.43525 puts the hammer's lowest point 6.5 cm above the
board, so it drops -- and lands differently -- on every reset. 0.3733 leaves 3 mm.
The handle is about 1.0 m from the robot base horizontally, at the edge of the
arm's reach, exactly as in the demo.
"""

HAMMER_BODY_OFFSET = Pose((0.0, 0.0, 0.0), (0.5, 0.5, 0.5, 0.5))
"""Pose of the hammer's rigid body (``/World/hammer``) inside ``hammer1.usd``.

A quarter turn about Z (``xformOp:orient``) followed by the unit-conversion
``rotateX:unitsResolve`` of 90 deg that brought ``hammer.usd`` from Y-up.
"""

HAMMER_BODY = compose(compose(SHELF, HAMMER_ON_SHELF), HAMMER_BODY_OFFSET)
"""Where the hammer's rigid body starts, in the cell frame."""

SODA_CAN_ON_SHELF = Pose((-0.35, 0.08, 0.436))
"""Soda can standing on the middle board, towards the robot.

The can's body is its mesh, authored upright with its base 0.072 m below the
origin, so z = 0.436 stands it 4 mm above the board. It is about 0.8 m from the
robot base horizontally -- well inside the reach the hammer is at the edge of.
"""

SODA_CAN_BODY = compose(SHELF, SODA_CAN_ON_SHELF)
"""Where the soda can's rigid body starts, in the cell frame."""
