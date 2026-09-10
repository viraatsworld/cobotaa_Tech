"""Compile the real workcell description into MJCF and check the simulator-side invariants.

These are the properties that silently degrade rather than crash: an under-powered arm sags
instead of erroring, a misnamed FT sensor produces no interfaces instead of failing, and a
lost mimic constraint leaves the gripper half-open. Each is asserted directly.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
import mujoco
import numpy as np
import pytest
import xacro

from techtory_cobotta_mujoco.model import (
    ARM_JOINT_PREFIX,
    FINGER_JOINT,
    FT_SENSOR,
    FT_SITE,
    generate_scene,
)


NAMESPACE = "cobotta_pro_"
ARM_JOINTS = [f"{ARM_JOINT_PREFIX}{index}" for index in range(1, 7)]
# techtory_cobotta_isaacsim/scripts/main.py HOME_JOINT_POSITIONS
HOME = [0.0, 0.349066, 1.309, 0.0, 1.48353, 0.0]
# The RG6 drives five mimic joints off finger_joint.
MIMIC_COUNT = 5


@pytest.fixture(scope="module")
def share() -> Path:
    return Path(get_package_share_directory("techtory_cobotta_mujoco"))


@pytest.fixture(scope="module")
def robot_xml(share: Path) -> str:
    return xacro.process_file(
        str(share / "urdf/techtory_cobotta_mujoco.urdf.xacro"),
        mappings={
            "hardware_type": "mujoco",
            "headless": "true",
            "namespace": NAMESPACE,
            "cvrb_prefix": NAMESPACE,
            "initial_positions_file": str(share / "config/initial_positions.yaml"),
        },
    ).toxml()


@pytest.fixture(scope="module")
def model(robot_xml: str, share: Path, tmp_path_factory) -> mujoco.MjModel:
    scene = generate_scene(
        robot_xml,
        str(tmp_path_factory.mktemp("scene")),
        actuator_config=str(share / "config/actuators.yaml"),
    )
    return mujoco.MjModel.from_xml_path(scene)


def _joint_id(model: mujoco.MjModel, name: str) -> int:
    return mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)


def _sensor_id(model: mujoco.MjModel, name: str) -> int:
    return mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, name)


def test_base_is_welded(model: mujoco.MjModel) -> None:
    """The cell is bolted to the floor -- a free joint would let the whole workcell drift."""
    types = [model.jnt_type[i] for i in range(model.njnt)]
    assert mujoco.mjtJoint.mjJNT_FREE not in types


def test_actuator_per_controlled_joint(model: mujoco.MjModel) -> None:
    """Six arm joints plus the single driven finger joint, and nothing else."""
    names = {
        mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i) for i in range(model.nu)
    }
    assert names == {*ARM_JOINTS, FINGER_JOINT}


def test_actuator_names_match_joint_names(model: mujoco.MjModel) -> None:
    """mujoco_ros2_control resolves tendon-transmission actuators to joints by name."""
    for index in range(model.nu):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, index)
        assert _joint_id(model, name) != -1, f"actuator '{name}' has no same-named joint"


def test_mimic_joints_become_equality_constraints(model: mujoco.MjModel) -> None:
    """The RG6's five mimic joints are couplings, not actuators."""
    assert model.neq == MIMIC_COUNT
    for index in range(model.neq):
        assert model.eq_type[index] == mujoco.mjtEq.mjEQ_JOINT


def test_force_torque_sensor_is_present(model: mujoco.MjModel) -> None:
    """A misnamed sensor fails silently in the hardware interface, so pin the names here."""
    force = _sensor_id(model, f"{FT_SENSOR}_force")
    torque = _sensor_id(model, f"{FT_SENSOR}_torque")
    assert force != -1 and torque != -1
    assert model.sensor_type[force] == mujoco.mjtSensor.mjSENS_FORCE
    assert model.sensor_type[torque] == mujoco.mjtSensor.mjSENS_TORQUE
    assert mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, FT_SITE) != -1


def test_force_torque_site_sits_below_the_flange(model: mujoco.MjModel) -> None:
    """The sensor must measure the J6 -> gripper interface, not some other joint.

    MuJoCo reports the wrench between the site's body and its parent, so the site's body has
    to be the gripper base and its parent the last arm link.
    """
    site = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, FT_SITE)
    body = model.site_bodyid[site]
    parent = model.body_parentid[body]
    assert mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, parent) == f"{NAMESPACE}J6"


def test_arm_holds_the_home_pose_under_gravity(model: mujoco.MjModel) -> None:
    """Catches under-powered actuators, which otherwise just sag instead of failing.

    The Denso description carries placeholder inertias and effort="1"; if the gains ever get
    re-derived from the URDF the arm collapses and this is what notices.
    """
    data = mujoco.MjData(model)
    for name, target in zip(ARM_JOINTS, HOME):
        data.qpos[model.jnt_qposadr[_joint_id(model, name)]] = target
        data.ctrl[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)] = target
    mujoco.mj_forward(model, data)
    for _ in range(1000):  # 2 s at the 0.002 s timestep
        mujoco.mj_step(model, data)

    reached = np.array(
        [data.qpos[model.jnt_qposadr[_joint_id(model, name)]] for name in ARM_JOINTS]
    )
    error = np.abs(reached - np.array(HOME))
    assert error.max() < 0.02, f"arm drifted from home by {error} rad"


def test_joint_force_limits_are_not_the_urdf_placeholder(model: mujoco.MjModel) -> None:
    """The URDF's effort="1" must not survive into the model.

    MuJoCo's importer maps <limit effort> onto a per-joint `actuatorfrcrange` that is applied
    on top of the actuator's own `forcerange`, tighter wins. Left at the Denso description's
    placeholder the arm is clamped to 1 Nm and cannot hold itself up -- and because the
    actuator's forcerange still reads correctly, this is invisible unless checked here.
    """
    for name in ARM_JOINTS:
        low, high = model.jnt_actfrcrange[_joint_id(model, name)]
        assert high >= 100.0, (
            f"{name} is clamped to {high} Nm of actuator force; the URDF placeholder "
            "effort limit leaked into the model"
        )
        assert low == -high


def test_wrench_reports_the_tool_load_at_rest(model: mujoco.MjModel) -> None:
    """At rest the wrist sensor must carry the gripper's weight, not zero."""
    data = mujoco.MjData(model)
    for name, target in zip(ARM_JOINTS, HOME):
        data.qpos[model.jnt_qposadr[_joint_id(model, name)]] = target
        data.ctrl[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)] = target
    mujoco.mj_forward(model, data)
    for _ in range(500):
        mujoco.mj_step(model, data)

    force = data.sensordata[
        model.sensor_adr[_sensor_id(model, f"{FT_SENSOR}_force")]:][:3]
    torque = data.sensordata[
        model.sensor_adr[_sensor_id(model, f"{FT_SENSOR}_torque")]:][:3]
    assert np.all(np.isfinite(force)) and np.all(np.isfinite(torque))
    # The RG6 and its fingers weigh well under 10 kg; anything outside this band means the
    # sensor is reading the wrong body or the inertias have changed shape.
    assert 0.5 < np.linalg.norm(force) < 100.0, f"resting |F| = {np.linalg.norm(force)} N"


def test_scene_declares_a_ground_plane(model: mujoco.MjModel) -> None:
    assert mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "ground") != -1
