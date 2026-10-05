# Copyright (c) 2026, Fraunhofer IPA.
# All rights reserved.
#
# SPDX-License-Identifier: Apache-2.0

"""What a pick task needs to know about the object it picks, one :class:`GraspableObject` each.

The pick-to-bin task is written against this description, not against a
particular object: to give it a new object, measure it into a
:class:`GraspableObject`, add it to :data:`GRASP_OBJECTS` and its
``RigidObjectCfg`` to ``scene_cfg.GRASPABLE_OBJECT_CFGS``, under the same name.

Every vector is in the object's **rigid-body frame** unless it says otherwise,
because that is the pose the simulator reports. Plain Python, like
:mod:`.layout`: safe to import before Kit starts.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from techtory_cobotta_isaaclab.scene.layout import (
    HAMMER_BODY_OFFSET,
    IDENTITY,
    TABLE_TOP_Z,
    Pose,
    compose,
    quat_conj,
    quat_from_rotate_xyz_deg,
    quat_rotate,
)

__all__ = ["GRASP_OBJECTS", "HAMMER", "SODA_CAN", "GraspableObject"]

Vec3 = tuple[float, float, float]
Quat = tuple[float, float, float, float]

_CORNER_SIGNS = [(sx, sy, sz) for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]


@dataclass(frozen=True)
class GraspableObject:
    """An object the RG6 picks from above, as the pick-to-bin task sees it."""

    name: str
    """Key in :data:`GRASP_OBJECTS` and ``scene_cfg.GRASPABLE_OBJECT_CFGS``."""

    body_offset: Pose
    """The rigid body's pose in the USD file's root frame."""

    box_center: Vec3
    """Centre of the object's bounding box [m]."""

    box_half: Vec3
    """Half-extents of the bounding box along the body axes [m]. Its corners decide "inside the bin"."""

    grasp_point: Vec3
    """Where the pads close on it [m]."""

    grasp_axis: Vec3 | None
    """Unit vector the jaw closes *across*: the TCP's x axis lines up with it, modulo a half turn.

    ``None`` for an object that is round seen from above: any yaw grasps it.
    """

    grasp_height: float
    """How far straight above :attr:`grasp_point` the TCP goes to grasp [m].

    The RG6's pad tips drop as the jaw closes -- 22 mm above the TCP fully open,
    17 mm below at half open, 32 mm below at 25 mm -- so this decides whether the
    pads meet the object or the table first.
    """

    jaw_on_object: tuple[float, float]
    """``finger_joint`` range [rad] where the jaw stalls on the object at :attr:`grasp_point`.

    Stalled outside it, the jaw closed on something else (the table, another part).
    """

    resting_rot: Quat
    """Orientation of the USD file's root when the object rests on the table, at yaw 0."""

    rest_height: float
    """Height of the file's root above the table when resting [m], a few mm of clearance included."""

    @property
    def resting_body(self) -> Pose:
        """The body's pose resting on the table, at yaw 0, with the file root at the cell origin's x, y."""
        return compose(Pose((0.0, 0.0, TABLE_TOP_Z + self.rest_height), self.resting_rot), self.body_offset)

    def corners(self, body: Pose) -> list[Vec3]:
        """The bounding box's 8 corners for the body at ``body``, in that pose's frame."""
        return [
            compose(body, Pose(tuple(c + s * h for c, s, h in zip(self.box_center, signs, self.box_half, strict=True))))
            .pos
            for signs in _CORNER_SIGNS
        ]  # fmt: skip

    @property
    def footprint_radius(self) -> float:
        """How far the resting object reaches from its box centre seen from above, at any yaw [m]."""
        body = self.resting_body
        center = compose(body, Pose(self.box_center)).pos
        return max(math.hypot(c[0] - center[0], c[1] - center[1]) for c in self.corners(body))

    @property
    def rest_clearance(self) -> float:
        """The resting bounding box's lowest corner above the table [m]."""
        return min(c[2] for c in self.corners(self.resting_body)) - TABLE_TOP_Z


##
# The hammer (hammer1.usd), measured from its mesh.
##

# In the file's root frame (metres, Z up): the handle runs along +x, the head is
# at the -x end and is 0.10 m tall along z, and the part is 0.02 m thick along y.
_HAMMER_BOX_CENTER_FILE: Vec3 = (0.0464, 0.0, 0.0104)
_HAMMER_BOX_HALF_FILE: Vec3 = (0.1056, 0.0100, 0.0508)
_HAMMER_GRASP_FILE: Vec3 = (0.04, 0.0, -0.0035)
_HAMMER_HANDLE_AXIS_FILE: Vec3 = (1.0, 0.0, 0.0)


def _hammer_file_to_body(v: Vec3) -> Vec3:
    return quat_rotate(quat_conj(HAMMER_BODY_OFFSET.rot), v)


HAMMER = GraspableObject(
    name="hammer",
    body_offset=HAMMER_BODY_OFFSET,
    box_center=_hammer_file_to_body(_HAMMER_BOX_CENTER_FILE),
    # HAMMER_BODY_OFFSET is a 120 deg turn about (1, 1, 1), which only permutes axes,
    # so the box stays axis-aligned in the body frame.
    box_half=tuple(abs(c) for c in _hammer_file_to_body(_HAMMER_BOX_HALF_FILE)),
    # On the handle's centreline, 16 mm from the centre of mass towards the end;
    # the handle is 21-27 mm wide and 15 mm thick there.
    grasp_point=_hammer_file_to_body(_HAMMER_GRASP_FILE),
    grasp_axis=_hammer_file_to_body(_HAMMER_HANDLE_AXIS_FILE),
    # The handle's centreline is ~10 mm over the table: a TCP 25 mm above it
    # (~34 mm over the table, like the real system's `pick` pose at 33 mm) leaves
    # the pad tips ~2 mm off the table when they meet the handle.
    grasp_height=0.025,
    jaw_on_object=(0.38, 0.56),
    # Lying flat: the 20 mm thickness (file y) points up.
    resting_rot=quat_from_rotate_xyz_deg(90.0, 0.0, 0.0),
    # 10 mm half-thickness plus 3.3 mm, as on the shelf.
    rest_height=0.0133,
)
"""The 0.3 kg, 0.21 m hammer, lying flat; gripped across the handle."""

##
# The soda can (soda_can.usd): its body is its mesh, upright, 60 mm across, 150 mm tall.
##

SODA_CAN = GraspableObject(
    name="soda_can",
    body_offset=Pose((0.0, 0.0, 0.0), IDENTITY),
    box_center=(0.0, 0.0, 0.0026),
    box_half=(0.03, 0.03, 0.075),
    # 15 mm under the lid. The RG6's inner knuckles reach down to 19 mm above the
    # TCP, so the lid must stay below that; the pads then hold the top ~45 mm.
    grasp_point=(0.0, 0.0, 0.0626),
    grasp_axis=None,
    grasp_height=0.0,
    # 60 mm between the pads is finger_joint +0.24 (measured); the can stalls the
    # jaw around there (+0.26 in check_ft_payload.py).
    jaw_on_object=(0.15, 0.36),
    resting_rot=IDENTITY,
    # Base 72.4 mm below the body origin, plus 3 mm.
    rest_height=0.0754,
)
"""The 0.35 kg soda can, standing upright; gripped round its top."""

GRASP_OBJECTS: dict[str, GraspableObject] = {obj.name: obj for obj in (HAMMER, SODA_CAN)}
"""Every object the pick tasks know, by name."""
