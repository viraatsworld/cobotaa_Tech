# Copyright (c) 2026, Fraunhofer IPA.
# All rights reserved.
#
# SPDX-License-Identifier: Apache-2.0

"""Prove the hammer-to-bin task is solvable before spending GPU hours on it.

Three modes, all through the task's own 4-D action ``[dx, dy, dz, dyaw]``:

* default -- a hand-written controller in every environment: above the handle,
  turn to the grasp yaw, descend (the gripper rule closes; on a miss, back off
  and retry), lift, hold still for
  2 s (the TCP must not sag: the IK drift check), carry over the bin (the rule
  lets go), wait. Prints how each environment's first episode ended and what
  each reward term paid, then PASS or FAIL.
* ``--resets`` -- reset, wait 2 s with zero action, and check every hammer lies
  flat in its spawn area, the arm is at home and the jaw is open.
* ``--hold`` -- zero action for 10 s at home: the TCP must not drift.

Every mode also checks no arm joint ever exceeds its velocity limit::

    python scripts/check_hammer_to_bin.py --num_envs 16
    python scripts/check_hammer_to_bin.py --resets --num_envs 1024
    python scripts/check_hammer_to_bin.py --hold
    python scripts/check_hammer_to_bin.py --viz kit --num_envs 4   # watch it
"""

import argparse
import math
import sys

import gymnasium as gym
import torch

from isaaclab.app import add_launcher_args, launch_simulation

from isaaclab_tasks.utils import resolve_task_config, setup_preset_cli

import techtory_cobotta_isaaclab.tasks  # noqa: F401

DEFAULT_TASK = "TechtoryCobottaIsaaclab-HammerToBin-COBOTTA"

MAX_DRIFT = 0.002  # m, TCP drift allowed while holding still
SUCCESS_SHARE = 0.8  # the scripted controller must succeed in this share of environments

# Scripted controller phases.
ABOVE, DESCEND, CLOSE, LIFT, HOLD, CARRY, WAIT = range(7)
PHASE_NAMES = ("above", "descend", "close", "lift", "hold", "carry", "wait")
HOVER = 0.08  # m above the grasp target before descending
HOLD_STEPS = 50  # 2 s at 25 Hz


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--task", type=str, default=DEFAULT_TASK, help="Name of the task.")
    parser.add_argument("--num_envs", type=int, default=16, help="Number of environments.")
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--resets", action="store_true", help="Check reset states only.")
    mode.add_argument("--hold", action="store_true", help="Check the arm holds still at home.")
    add_launcher_args(parser)
    parser.set_defaults(device=None)
    args_cli, hydra_args = setup_preset_cli(parser)
    sys.argv = [sys.argv[0]] + hydra_args
    return args_cli


def main() -> None:
    args_cli = parse_args()
    env_cfg, _ = resolve_task_config(args_cli.task, "")
    env_cfg.scene.num_envs = args_cli.num_envs
    if args_cli.device is not None:
        env_cfg.sim.device = args_cli.device
    args_cli.device = env_cfg.sim.device
    # The scripted run is slower than a policy needs to be: it holds still for 2 s
    # to measure drift, and a retry after a missed grasp costs about 4 s. 30 s
    # keeps the check about whether the task can be done, not how fast.
    env_cfg.episode_length_s = 30.0

    with launch_simulation(env_cfg, args_cli):
        # Imported here: they pull in Isaac Lab modules that need the app running.
        from isaaclab.utils.math import quat_apply, quat_apply_inverse

        from techtory_cobotta_isaaclab.robot import ARM_JOINTS, GRIPPER_OPEN, HOME_POSE
        from techtory_cobotta_isaaclab.scene import layout
        from techtory_cobotta_isaaclab.tasks.hammer_to_bin.mdp import state

        env = gym.make(args_cli.task, cfg=env_cfg)
        base = env.unwrapped
        n, device = base.num_envs, base.device
        arm_term = base.action_manager.get_term("arm")
        robot = state.robot(base)
        arm_ids = robot.find_joints(list(ARM_JOINTS), preserve_order=True)[0]
        vel_limit = robot.data.joint_vel_limits.torch[:, arm_ids]
        worst_speed = torch.zeros(n, device=device)
        failures: list[str] = []

        def step(action: torch.Tensor):
            with torch.inference_mode():
                out = env.step(action)
            speed = (robot.data.joint_vel.torch[:, arm_ids].abs() / vel_limit).amax(dim=1)
            worst_speed.copy_(torch.maximum(worst_speed, speed))
            return out

        def to_base(vector_e: torch.Tensor) -> torch.Tensor:
            return quat_apply_inverse(robot.data.root_link_quat_w.torch, vector_e)

        env.reset()
        zero = torch.zeros(n, 4, device=device)

        if args_cli.resets:
            # 2 s: the jaw starts half open and opens at 0.6 rad/s.
            for _ in range(50):
                step(zero)
            corners = state.hammer_corners(base)
            lowest = corners[..., 2].amin(dim=1) - layout.WORK_SURFACE_Z
            _, quat = state.hammer_pose(base)
            up = quat_apply(quat, torch.tensor([[1.0, 0.0, 0.0]], device=device).expand(n, 3))[:, 2].abs()
            tilt = torch.rad2deg(torch.acos(up.clamp(max=1.0)))
            center = state.hammer_center(base)[:, :2]
            area_c = torch.tensor(layout.HAMMER_SPAWN_AREA.center, device=device)
            area_h = torch.tensor(layout.HAMMER_SPAWN_AREA.half, device=device)
            outside = ((center - area_c).abs() > area_h + 0.01).any(dim=1)
            home = torch.tensor([HOME_POSE[j] for j in ARM_JOINTS], device=device)
            home_err = (robot.data.joint_pos.torch[:, arm_ids] - home).abs().amax(dim=1)
            jaw_err = (state.jaw_position(base) - GRIPPER_OPEN).abs()
            print(f"\n{n} resets, after 2 s with zero action:")
            print(f"  hammer tilt          max {tilt.max():6.2f} deg   (< 5)")
            low, high = lowest.min() * 1000, lowest.max() * 1000
            print(f"  lowest corner        {low:+6.1f} .. {high:+6.1f} mm over the table")
            print(f"  hammer speed         max {state.hammer_speed(base).max():6.3f} m/s")
            print(f"  outside spawn area   {int(outside.sum())}")
            print(f"  arm off home         max {home_err.max():6.4f} rad")
            print(f"  jaw off open         max {jaw_err.max():6.3f} rad")
            if tilt.max() > 5.0:
                failures.append("a hammer is tilted")
            # The bounding box's corners are sharp, the hammer's are not: a corner may dip a few mm.
            if lowest.min() < -0.005 or lowest.max() > 0.01:
                failures.append("a hammer is not resting on the table")
            if outside.any():
                failures.append("a hammer left its spawn area")
            if home_err.max() > 0.01:
                failures.append("the arm is not at home")
            if jaw_err.max() > 0.05:
                failures.append("the jaw did not open")

        elif args_cli.hold:
            for _ in range(5):
                step(zero)
            start = state.tcp_pose(base)[0].clone()
            drift = torch.zeros(n, device=device)
            for _ in range(250):
                step(zero)
                drift = torch.maximum(drift, torch.linalg.norm(state.tcp_pose(base)[0] - start, dim=-1))
            print(f"\nTCP drift over 10 s with zero action: max {drift.max() * 1000:.2f} mm (< {MAX_DRIFT * 1000:.0f})")
            if drift.max() > MAX_DRIFT:
                failures.append("the TCP drifted while holding still")

        else:
            phase = torch.full((n,), ABOVE, dtype=torch.long, device=device)
            phase_steps = torch.zeros(n, dtype=torch.long, device=device)
            lift_z = torch.zeros(n, device=device)
            hold_start = torch.zeros(n, 3, device=device)
            hold_drift = torch.zeros(n, device=device)
            done = torch.zeros(n, dtype=torch.bool, device=device)
            outcome = ["timeout"] * n
            reached = torch.zeros(n, dtype=torch.long, device=device)
            retries = torch.zeros(n, dtype=torch.long, device=device)
            terms = base.reward_manager.active_terms
            paid = torch.zeros(n, len(terms), device=device)
            bin_target = torch.tensor(layout.BIN_TARGET, device=device)
            pos_step, yaw_step = arm_term.cfg.pos_step, arm_term.cfg.yaw_step

            for _ in range(int(env_cfg.episode_length_s / base.step_dt) + 5):
                tcp = state.tcp_pose(base)[0]
                goal = tcp.clone()
                yaw_action = torch.zeros(n, device=device)
                grasp = state.grasp_target(base)
                yaw_err = state.grasp_yaw_error(base)

                above = phase == ABOVE
                goal[above] = grasp[above] + torch.tensor([0.0, 0.0, HOVER], device=device)
                yaw_action[above] = (-yaw_err / yaw_step).clamp(-1, 1)[above]
                arrived = above & (torch.linalg.norm(tcp - goal, dim=-1) < 0.01) & (yaw_err.abs() < 0.03)
                phase[arrived] = DESCEND

                descend = phase == DESCEND
                goal[descend] = grasp[descend]
                yaw_action[descend] = (-yaw_err / yaw_step).clamp(-1, 1)[descend]
                closed = descend & state.gripper_rule(base).closed
                phase[closed] = CLOSE
                phase_steps[closed] = 0

                # A miss: the rule reopened. Back off above the handle (which re-arms it) and retry.
                rule = state.gripper_rule(base)
                retry = ((phase == CLOSE) | (phase == LIFT) | (phase == HOLD)) & ~rule.closed & ~rule.released
                phase[retry] = ABOVE
                retries += retry.long()

                grasped = state.hammer_grasped(base)
                lifting = (phase == CLOSE) & grasped & (phase_steps > 5)
                lift_z[lifting] = (bin_target[2] + tcp[:, 2] - state.hammer_center(base)[:, 2])[lifting]
                phase[lifting] = LIFT

                lift = phase == LIFT
                goal[lift, 2] = lift_z[lift]
                lifted = lift & ((tcp[:, 2] - lift_z).abs() < 0.005)
                phase[lifted] = HOLD
                phase_steps[lifted] = 0
                hold_start[lifted] = tcp[lifted]

                hold = phase == HOLD
                goal[hold] = hold_start[hold]
                hold_drift[hold] = torch.maximum(hold_drift, torch.linalg.norm(tcp - hold_start, dim=-1))[hold]
                held = hold & (phase_steps >= HOLD_STEPS)
                phase[held] = CARRY

                carry = phase == CARRY
                goal[carry] = (tcp + bin_target - state.hammer_center(base))[carry]
                phase[carry & state.gripper_rule(base).released] = WAIT

                action = torch.zeros(n, 4, device=device)
                action[:, :3] = (to_base(goal - tcp) / pos_step).clamp(-1, 1)
                action[:, 3] = yaw_action
                # HOLD commands nothing at all: the drift check is about zero action.
                action[hold] = 0.0
                reached = torch.maximum(reached, phase)
                phase_steps += 1

                _, _, terminated, truncated, _ = step(action)
                step_reward = base.reward_manager._step_reward * base.step_dt
                paid[~done] += step_reward[~done]
                ended = (terminated | truncated) & ~done
                for i in ended.nonzero().flatten().tolist():
                    tm = base.termination_manager
                    outcome[i] = next(
                        (t for t in ("success", "dropped_outside_bin") if bool(tm.get_term(t)[i])), "timeout"
                    )
                done |= ended
                # A reset puts the env back at ABOVE; it is ignored after its first episode.
                phase[ended] = ABOVE
                if done.all():
                    break

            print(f"\nScripted pick-and-place, first episode of {n} environments:")
            header = f"  {'env':>3}  {'outcome':<20} {'furthest phase':<15} {'retries':>7} {'hold drift':>10}  "
            print(header + "  ".join(terms))
            for i in range(n):
                row = "  ".join(f"{v:{max(len(t), 6)}.2f}" for t, v in zip(terms, paid[i].tolist(), strict=True))
                print(
                    f"  {i:3d}  {outcome[i]:<20} {PHASE_NAMES[int(reached[i])]:<15} {int(retries[i]):7d}"
                    f" {hold_drift[i] * 1000:8.2f}mm  {row}"
                )
            successes = sum(o == "success" for o in outcome)
            print(f"\n  success {successes}/{n}, hold drift max {hold_drift.max() * 1000:.2f} mm")
            if successes < math.ceil(SUCCESS_SHARE * n):
                failures.append(f"only {successes}/{n} scripted episodes succeeded")
            if hold_drift.max() > MAX_DRIFT:
                failures.append("the TCP drifted while holding the hammer")

        print(f"  arm joint speed      max {worst_speed.max():.3f} x its velocity limit (<= 1.02)")
        if worst_speed.max() > 1.02:
            failures.append("an arm joint exceeded its velocity limit")

        env.close()
        print("\n" + ("PASS" if not failures else "FAIL: " + "; ".join(failures)))
        if failures:
            sys.exit(1)


if __name__ == "__main__":
    main()
