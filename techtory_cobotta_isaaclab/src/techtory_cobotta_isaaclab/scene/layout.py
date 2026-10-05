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
    "BIN_FLOOR_Z",
    "BIN_FOOTPRINT",
    "BIN_INTERIOR",
    "BIN_RELEASE_HEIGHT",
    "BIN_RIM_Z",
    "BIN_TARGET",
    "HAMMER_BODY",
    "HAMMER_BODY_OFFSET",
    "HAMMER_ON_SHELF",
    "IDENTITY",
    "PALLET",
    "PALLET_FOOTPRINT",
    "ROBOT_BASE_PLATE",
    "ROBOT_MOUNT",
    "SHELF",
    "SHELF_FOOTPRINT",
    "SODA_CAN_BODY",
    "SODA_CAN_ON_SHELF",
    "SPAWN_ZONE",
    "TABLE_TOP",
    "TABLE_TOP_Z",
    "Box2D",
    "Pose",
    "compose",
    "quat_conj",
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

##
# The table, the bin and the spawn zone, for the pick-to-bin tasks.
##


def quat_conj(q: Quat) -> Quat:
    """The inverse of the unit quaternion ``q``."""
    return (-q[0], -q[1], -q[2], q[3])


class Box2D(NamedTuple):
    """An axis-aligned rectangle in the cell frame's x/y plane [m]."""

    center: tuple[float, float]
    half: tuple[float, float]

    def shrunk(self, margin: float) -> Box2D:
        return Box2D(self.center, (self.half[0] - margin, self.half[1] - margin))

    def contains(self, x: float, y: float) -> bool:
        return abs(x - self.center[0]) <= self.half[0] and abs(y - self.center[1]) <= self.half[1]


TABLE_TOP_Z = 0.942
"""Top face of the cell's table [m]: the surface the pallet, the shelf and the robot's plate stand on.

Measured by raycasting the workcell's collision mesh: flat at 0.942 over
x in [-0.78, 0.78], y in [-0.76, 0.82]. The workcell URDF mounts the plate,
shelf and pallet at 0.94, 2 mm into it -- harmless between static colliders.
"""

TABLE_TOP = Box2D((0.0, 0.03), (0.78, 0.79))
"""The flat part of the table top, from the same raycast.

It is a slotted plate: grooves 16 mm wide and at least 12 mm deep run along x,
every 52 mm. ``WORKCELL_CFG`` explains what that means for contacts.
"""

ROBOT_BASE_PLATE = Box2D((-0.275, -0.24), (0.15, 0.15))
"""The robot's base plate on the table (its collision mesh spans x [-0.425, -0.125], y [-0.39, -0.09])."""

SHELF_FOOTPRINT = Box2D((0.61, 0.27), (0.15, 0.5))
"""The shelf's boards seen from above: the shelf frame's [-0.5, 0.5] x [-0.15, 0.15], turned 90 deg.

Its 40 mm legs stand inside this footprint (``shelf.urdf``).
"""

PALLET = Pose((-0.16, 0.3, 0.94))
"""The blue pallet ("bin"): footprint centre on the table, its frame unrotated.

From ``techtory_cobotta_workcell.urdf.xacro`` and the Isaac Sim demo's
``spawn_objects.add_pallet``. In the robot base frame this is (0.54, -0.115):
the real system's ``bin_place`` (``techtory_cobotta_system/config/poses_hammer.yaml``).
"""

PALLET_FOOTPRINT = Box2D(PALLET.pos[:2], (0.30, 0.20))
"""The pallet's outer walls seen from above (``assets/urdf/pallet.urdf``: 0.60 x 0.40 m)."""

BIN_INTERIOR = Box2D(PALLET.pos[:2], (0.285, 0.185))
"""Inside the pallet's 15 mm walls: 0.57 x 0.37 m."""

BIN_FLOOR_Z = PALLET.pos[2] + 0.015
"""Top of the pallet's floor plate [m]."""

BIN_RIM_Z = PALLET.pos[2] + 0.075
"""Top of the pallet's walls [m]."""

BIN_FOOTPRINT = BIN_INTERIOR.shrunk(0.03)
"""The interior less a 3 cm margin. The gripper only lets go once every corner
of the object is over it, so nothing is dropped across the rim."""

BIN_RELEASE_HEIGHT = 0.15
"""How far above the rim the object's lowest corner may be when it is released [m]."""

BIN_TARGET: Vec3 = (BIN_FOOTPRINT.center[0], BIN_FOOTPRINT.center[1], BIN_RIM_Z + 0.12)
"""Where the object's box centre is carried to: over the bin, 12 cm above the rim.

Above the rim on purpose. A target on the bin floor would make straight-line
distance shaping pay for dragging the object along the table into the wall.
"""

SPAWN_ZONE = Box2D((0.175, -0.19), (0.267, 0.267))
"""The fixed area of the table top where pick tasks lay their object, cell frame.

The whole object stays inside it, at any yaw: its centre is drawn from the zone
shrunk by the object's footprint radius. On the table the pallet stands on, in
front of and to the right of the robot: robot base frame x in [-0.22, 0.32],
y in [-0.72, -0.18]. It clears the base plate, the pallet and the shelf by at
least 1.5 cm (``tests/test_pick_to_bin_layout.py``). ``SPAWN_ZONE_MARKER_CFG``
draws it.
"""
