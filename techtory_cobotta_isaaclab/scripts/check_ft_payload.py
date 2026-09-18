# Copyright (c) 2026, Fraunhofer IPA.
# All rights reserved.
#
# SPDX-License-Identifier: Apache-2.0

"""Check the wrist force/torque sensor against known loads, with and without a payload.

Three phases, one environment:

1. **Rest.** The gripper hangs from the flange. The joint-wrench sensor must read
   the gripper's own weight, and which way it points tells the sensor's sign
   convention (load on the arm, or the arm's support). Then the wrist tilts, and
   the torque must match the moment of the gripper links' weights about the
   flange -- which checks the torque's reference point, not just its sign.
2. **Grip.** The soda can is held between the pads while the jaw closes on it,
   then let go. It must stay in the jaw, and the stalled finger must be quiet
   enough for a gripper controller to declare the stall (below 0.1 rad/s). The
   wrist reading must grow by the can's weight: PhysX's joint wrench has to
   carry the payload (the Isaac Sim demo found it did not on Isaac Sim 6.0.1;
   if that ever returns, this fails). The ``payload_wrench`` estimate must
   agree with the measured change.
3. **Lift.** The shoulder raises the held can. It must stay held; the inertial
   swing in the reading is printed.

It prints the measurements, then PASS or FAIL::

    python scripts/check_ft_payload.py                  # headless
    python scripts/check_ft_payload.py --viz kit        # watch it
"""

import argparse
import sys

import gymnasium as gym
import torch

from isaaclab.app import add_launcher_args, launch_simulation

from isaaclab_tasks.utils import resolve_task_config, setup_preset_cli

import techtory_cobotta_isaaclab.tasks  # noqa: F401

DEFAULT_TASK = "TechtoryCobottaIsaaclab-Base-COBOTTA"
DEFAULT_PHYSICS = "isaacsim_physx"

PRE_GRASP_JAW = 0.15  # rad on finger_joint: ~72 mm between the pads, room for the 60 mm can
STALL_SPEED = 0.1  # rad/s, what the ParallelGripperCommand controller is configured with
WRENCH = slice(21, 27)  # policy observation slice of the wrist wrench


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--task", type=str, default=DEFAULT_TASK, help="Name of the task.")
    add_launcher_args(parser)
    # No visualizer unless --viz asks for one. (A ["none"] default would be taken as a
    # visualizer name on the kit-less path; only an explicit --viz none means "disable".)
    parser.set_defaults(device=None)
    args_cli, hydra_args = setup_preset_cli(parser)
    if not any(arg.startswith(("physics=", "env.sim.physics=")) for arg in hydra_args):
        hydra_args.append(f"physics={DEFAULT_PHYSICS}")
    sys.argv = [sys.argv[0]] + hydra_args
    return args_cli


def _vec(v: torch.Tensor) -> str:
    return "(" + " ".join(f"{x:+7.3f}" for x in v.tolist()) + ")"


def main() -> None:
    args_cli = parse_args()
    env_cfg, _ = resolve_task_config(args_cli.task, "")
    env_cfg.scene.num_envs = 1
    if args_cli.device is not None:
        env_cfg.sim.device = args_cli.device
    args_cli.device = env_cfg.sim.device
    env_cfg.episode_length_s = 60.0

    with launch_simulation(env_cfg, args_cli):
        # Imported here: they pull in Isaac Lab modules that need the app running.
        from isaaclab.utils.math import quat_apply, quat_apply_inverse, quat_mul

        from techtory_cobotta_isaaclab.robot import (
            FT_BODY,
            FT_ROBOT_ENTITY,
            GRIPPER_JOINT,
            GRIPPER_OPEN,
            TCP_OFFSET,
            payload_wrench,
        )
        from techtory_cobotta_isaaclab.robot import observations as ft_obs

        env = gym.make(args_cli.task, cfg=env_cfg)
        u = env.unwrapped
        device = u.device
        robot, can, sensor = u.scene["robot"], u.scene["soda_can"], u.scene["wrist_ft"]
        ft_body = robot.find_bodies(FT_BODY)[0][0]
        ft_row = sensor.find_bodies(FT_BODY)[0][0]
        finger = robot.find_joints(GRIPPER_JOINT)[0][0]
        gripper_ids = [i for i, name in enumerate(robot.body_names) if not name.startswith("cobotta_pro_")]
        payload_cfg = FT_ROBOT_ENTITY.copy()
        payload_cfg.resolve(u.scene)
        gravity = torch.tensor(u.sim.cfg.gravity, device=device)
        env0 = torch.tensor([0], device=device)

        actions = torch.zeros(1, 7, device=device)
        state = {"obs": None}

        def ft_pose() -> tuple[torch.Tensor, torch.Tensor]:
            return robot.data.body_link_pos_w.torch[:, ft_body], robot.data.body_link_quat_w.torch[:, ft_body]

        def hold_can_in_jaw() -> None:
            # Can centred on the TCP, its axis along the F/T x-axis, across the
            # closing direction (y), so the pads close on its side.
            pos, quat = ft_pose()
            tcp = pos + quat_apply(quat, torch.tensor([TCP_OFFSET], device=device))
            y90 = torch.tensor([[0.0, 0.7071068, 0.0, 0.7071068]], device=device)
            can.write_root_pose_to_sim_index(root_pose=torch.cat([tcp, quat_mul(quat, y90)], dim=-1), env_ids=env0)
            can.write_root_velocity_to_sim_index(root_velocity=torch.zeros(1, 6, device=device), env_ids=env0)

        def run(seconds: float, grip: float, arm: tuple[int, float] | None = None, hold_can: bool = False) -> list:
            actions.zero_()
            actions[:, 6] = grip
            if arm is not None:
                actions[:, arm[0]] = arm[1]
            samples = []
            for _ in range(max(1, int(seconds / u.step_dt))):
                if hold_can:
                    hold_can_in_jaw()
                with torch.inference_mode():
                    obs, _, _, _, _ = env.step(actions)
                state["obs"] = obs["policy"][0]
                down = gravity_in_ft() / torch.linalg.norm(gravity)
                load_along_gravity = torch.dot(state["obs"][WRENCH][:3], down).item()
                samples.append((robot.data.joint_vel.torch[0, finger].item(), load_along_gravity))
            return samples

        def raw_load() -> tuple[torch.Tensor, torch.Tensor]:
            return sensor.data.force.torch[0, ft_row].clone(), sensor.data.torque.torch[0, ft_row].clone()

        def gravity_in_ft() -> torch.Tensor:
            return quat_apply_inverse(ft_pose()[1], gravity.unsqueeze(0))[0]

        env.reset()
        failures = []

        # 1. Rest -------------------------------------------------------------------
        run(1.5, grip=0.0)
        force, torque = raw_load()
        m_gripper = robot.data.body_mass.torch[0, gripper_ids].sum().item()
        weight = m_gripper * gravity_in_ft()
        sign = 1.0 if torch.dot(force, weight) > 0 else -1.0
        rest_error = (torch.linalg.norm(sign * force - weight) / torch.linalg.norm(weight)).item()
        print("\n[1] REST")
        print(f"    gripper mass {m_gripper:.3f} kg, its weight in the F/T frame {_vec(weight)} N")
        print(f"    raw joint wrench  F {_vec(force)} N   T {_vec(torque)} N*m")
        print(
            f"    -> PhysX reports the {'load on the arm' if sign > 0 else 'support of the arm'}: "
            f"_RAW_TO_LOAD = {sign:+.0f} (currently {ft_obs._RAW_TO_LOAD:+.0f}), error {100 * rest_error:.2f} %"
        )
        if rest_error > 0.03:
            failures.append(f"rest reading is {100 * rest_error:.1f} % off the gripper weight")
        if sign != ft_obs._RAW_TO_LOAD:
            failures.append("_RAW_TO_LOAD has the wrong sign")

        # 1b. Tilted rest: the torque, about the flange (base_link origin) -----------
        # Tilt the wrist so gravity has a lateral component, then compare with the
        # wrench computed from every gripper link's mass and centre of mass.
        run(2.0, grip=0.0, arm=(4, 0.6))  # j5 +0.3 rad
        pos, quat = ft_pose()
        coms = quat_apply_inverse(
            quat.expand(len(gripper_ids), 4), robot.data.body_com_pos_w.torch[0, gripper_ids] - pos
        )
        weights = robot.data.body_mass.torch[0, gripper_ids].unsqueeze(-1) * gravity_in_ft()
        expected_force, expected_torque = weights.sum(0), torch.linalg.cross(coms, weights, dim=-1).sum(0)
        force, torque = raw_load()
        torque_error = (torch.linalg.norm(sign * torque - expected_torque)).item()
        print("\n[1b] TILTED REST (j5 +0.3 rad)")
        print(f"    expected  F {_vec(expected_force)} N   T {_vec(expected_torque)} N*m (from link masses and CoMs)")
        print(f"    measured  F {_vec(sign * force)} N   T {_vec(sign * torque)} N*m")
        print(f"    torque error {torque_error:.4f} N*m")
        if torque_error > 0.05 * torch.linalg.norm(expected_torque).item() + 0.005:
            failures.append(f"tilted torque is {torque_error:.3f} N*m off the gripper's own moment")
        run(2.0, grip=0.0)  # back home

        rest_raw = sign * raw_load()[0]
        rest_obs = state["obs"][WRENCH].clone()

        # 2. Grip -------------------------------------------------------------------
        run(1.0, grip=PRE_GRASP_JAW / GRIPPER_OPEN)
        run(1.0, grip=-1.0, hold_can=True)  # jaw closes onto a can that cannot fall yet
        samples = run(2.0, grip=-1.0)  # let go: the grip alone holds it now
        speeds = torch.tensor([abs(s[0]) for s in samples[len(samples) // 2 :]])
        p95 = torch.quantile(speeds, 0.95).item()
        pos, quat = ft_pose()
        tcp = pos + quat_apply(quat, torch.tensor([TCP_OFFSET], device=device))
        slip = torch.linalg.norm(can.data.root_link_pos_w.torch - tcp).item()
        jaw = robot.data.joint_pos.torch[0, finger].item()
        m_can = can.data.body_mass.torch[0, 0].item()
        expected = m_can * gravity_in_ft()
        force, torque = raw_load()
        delta_raw = sign * force - rest_raw
        payload = payload_wrench(u, payload_cfg)[0]
        delta_obs = state["obs"][WRENCH] - rest_obs
        carried = torch.dot(delta_raw, expected).item() / torch.dot(expected, expected).item()
        observed_error = (torch.linalg.norm(delta_obs[:3] - expected) / torch.linalg.norm(expected)).item()
        estimate_error = (torch.linalg.norm(payload[:3] - delta_obs[:3]) / torch.linalg.norm(expected)).item()
        print("\n[2] GRIP (soda can)")
        print(f"    jaw stalled at {jaw:+.3f} rad, finger |vel| p95 {p95:.4f} rad/s (controller needs < {STALL_SPEED})")
        print(f"    can {1000 * slip:.1f} mm from the TCP after 2 s on its own")
        print(f"    can weight in F/T frame   {_vec(expected)} N   ({m_can:.3f} kg)")
        print(f"    joint wrench change       {_vec(delta_raw)} N   -> carries {100 * carried:.0f} % of it")
        print(f"    wrist_wrench change       {_vec(delta_obs[:3])} N   ({100 * observed_error:.1f} % off)")
        print(f"    payload_wrench estimate   {_vec(payload[:3])} N   ({100 * estimate_error:.1f} % off the change)")
        if slip > 0.02:
            failures.append(f"the can slipped {1000 * slip:.0f} mm out of the grip")
        if p95 > STALL_SPEED:
            failures.append(f"the stalled finger rings at {p95:.3f} rad/s")
        if observed_error > 0.10:
            failures.append(f"wrist_wrench changed by {100 * observed_error:.0f} % less/more than the can's weight")
        if estimate_error > 0.10:
            failures.append(f"payload_wrench is {100 * estimate_error:.0f} % off the measured payload")

        # 3. Lift -------------------------------------------------------------------
        samples = run(2.0, grip=-1.0, arm=(1, -0.4))  # shoulder back 0.2 rad
        along = torch.tensor([s[1] for s in samples])
        pos, quat = ft_pose()
        tcp = pos + quat_apply(quat, torch.tensor([TCP_OFFSET], device=device))
        slip = torch.linalg.norm(can.data.root_link_pos_w.torch - tcp).item()
        static = (m_gripper + m_can) * torch.linalg.norm(gravity).item()
        print("\n[3] LIFT (shoulder -0.2 rad)")
        print(f"    wrist load along gravity: min {along.min():.3f}  max {along.max():.3f}  end {along[-1]:.3f} N")
        print(f"    (gripper + can) * g = {static:.3f} N; the swing is the inertial load of the move")
        print(f"    can {1000 * slip:.1f} mm from the TCP after the lift")
        if slip > 0.02:
            failures.append(f"the can was dropped during the lift ({1000 * slip:.0f} mm)")
        if abs(along[-1].item() - static) > 0.05 * static:
            failures.append(f"settled after the lift, the wrist reads {along[-1]:.2f} N instead of {static:.2f} N")

        env.close()

        # Report inside the context: leaving it closes Kit, which ends the process.
        # An exception is what makes the launcher exit with a non-zero status.
        if failures:
            raise RuntimeError("FAIL:\n  - " + "\n  - ".join(failures))
        print("\nPASS")


if __name__ == "__main__":
    main()
