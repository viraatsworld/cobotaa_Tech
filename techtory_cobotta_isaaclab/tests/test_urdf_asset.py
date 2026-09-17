# Copyright (c) 2026, Fraunhofer IPA.
# All rights reserved.
#
# SPDX-License-Identifier: Apache-2.0

"""The committed assets are self-contained and carry every correction.

Each correction fails silently when it regresses -- the robot still imports
and still looks right -- so each one is asserted here. No Isaac, no GPU.
"""

from __future__ import annotations

import xml.etree.ElementTree as ElementTree

import pytest

from conftest import ASSETS_DIR, builder, joints

ROBOT_URDF = ASSETS_DIR / "cobotta_rg6.urdf"
SHELF_URDF = ASSETS_DIR / "shelf.urdf"


@pytest.fixture(scope="module")
def robot() -> ElementTree.Element:
    return ElementTree.parse(ROBOT_URDF).getroot()


@pytest.fixture(scope="module")
def shelf() -> ElementTree.Element:
    return ElementTree.parse(SHELF_URDF).getroot()


def _joints(root: ElementTree.Element) -> dict[str, ElementTree.Element]:
    return {j.get("name"): j for j in root.findall("joint")}


def _roots(root: ElementTree.Element) -> set[str]:
    children = {j.find("child").get("link") for j in root.findall("joint")}
    return {link.get("name") for link in root.findall("link")} - children


def test_single_root_and_no_orphan_world(robot: ElementTree.Element) -> None:
    names = {link.get("name") for link in robot.findall("link")}
    assert "world" not in names
    assert _roots(robot) == {joints().ROOT_BODY}


def test_no_ros_only_elements_or_mimics(robot: ElementTree.Element) -> None:
    for tag in ("ros2_control", "gazebo", "transmission"):
        assert robot.find(tag) is None, tag
    assert not list(robot.iter("mimic"))


def test_meshes_are_local_and_present(robot: ElementTree.Element) -> None:
    meshes = [m.get("filename") for m in robot.iter("mesh")]
    assert meshes
    for filename in meshes:
        assert not filename.startswith("package://"), filename
        assert (ASSETS_DIR / filename).is_file(), filename


def test_every_link_is_a_body(robot: ElementTree.Element) -> None:
    """Fixed-joint merging is off, so a link without inertia would not import."""
    for link in robot.findall("link"):
        inertial = link.find("inertial")
        assert inertial is not None, link.get("name")
        assert float(inertial.find("mass").get("value")) > 0.0


def test_ft_body_hangs_off_j6_through_a_locked_joint(robot: ElementTree.Element) -> None:
    """The wrist F/T reads the incoming joint of FT_BODY; it must be the flange,
    and a locked revolute (Newton's JointWrenchSensor skips fixed joints)."""
    j = joints()
    joint = _joints(robot)[j.FT_JOINT]
    assert joint.get("type") == "revolute"
    assert joint.find("parent").get("link") == "cobotta_pro_J6"
    assert joint.find("child").get("link") == j.FT_BODY
    limit = joint.find("limit")
    assert float(limit.get("lower")) == pytest.approx(-j.WRIST_LOCK)
    assert float(limit.get("upper")) == pytest.approx(j.WRIST_LOCK)
    assert builder().WRIST_JOINT == j.FT_JOINT
    assert builder().WRIST_LOCK == pytest.approx(j.WRIST_LOCK)


def test_non_fixed_joints_are_arm_gripper_and_wrist_lock(robot: ElementTree.Element) -> None:
    j = joints()
    moving = {name for name, e in _joints(robot).items() if e.get("type") != "fixed"}
    assert moving == set(j.ARM_JOINTS) | set(j.GRIPPER_JOINTS) | {j.FT_JOINT}


def test_gripper_mimic_matches_the_source_urdf() -> None:
    """GRIPPER_MIMIC is transcribed from the <mimic> elements the builder strips."""
    source = ElementTree.parse(builder().DEFAULT_ROBOT_URDF).getroot()
    assert builder().read_mimics(source) == joints().GRIPPER_MIMIC


def test_home_pose_within_limits(robot: ElementTree.Element) -> None:
    by_name = _joints(robot)
    for name, value in joints().HOME_POSE.items():
        limit = by_name[name].find("limit")
        assert float(limit.get("lower")) <= value <= float(limit.get("upper")), name


def test_shelf_is_primitive_boxes_with_bodies(shelf: ElementTree.Element) -> None:
    names = {link.get("name") for link in shelf.findall("link")}
    assert all(n.startswith("shelf_") for n in names)
    assert len(names) == 8  # base anchor, 4 legs, 3 boards
    assert not list(shelf.iter("mesh"))
    for link in shelf.findall("link"):
        assert link.find("inertial") is not None, link.get("name")
    boards = {
        j.get("name"): float(j.find("origin").get("xyz").split()[2])
        for j in shelf.findall("joint")
        if "board" in j.get("name")
    }
    assert boards == {
        "shelf_board_top_joint": 0.78,
        "shelf_board_middle_joint": 0.34,
        "shelf_board_bottom_joint": 0.14,
    }


def test_cell_meshes_present() -> None:
    for key in ("cell_link.STL", "robot_base_link.STL"):
        assert (ASSETS_DIR / "meshes" / "techtory_cell_description" / "urdf" / "mesh" / key).is_file()
