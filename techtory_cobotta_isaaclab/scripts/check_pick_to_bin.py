# Copyright (c) 2026, Fraunhofer IPA.
# All rights reserved.
#
# SPDX-License-Identifier: Apache-2.0

"""Check a pick-to-bin task is set up right before spending GPU hours on it.

These check the *setup* -- resets, the arm action, the gripper rule, the reward
bookkeeping -- not how well anything is learnt. Three modes, all through the
task's own 4-D action ``[dx, dy, dz, dyaw]``:

* ``--resets`` -- reset, wait 2 s with zero action: every object must rest as
  it should (not tipped, on the table, inside the spawn zone), the arm at home
  and the jaw open.
* ``--hold`` -- zero action for 10 s at home: the TCP must not drift.
* default -- a hand-written controller in every environment, as a probe: above
  the grasp point, turn to the grasp yaw, descend (the gripper rule closes; on a
  miss, back off and retry), lift, hold still (1 s to settle, then 2 s measured), carry over the bin (the
  rule lets go), wait. It prints how each environment's first episode ended and
  what each reward term paid. The task is broken if *no* episode can succeed,
  if a one-off milestone pays twice, or if the arm drifts while holding the
  object; a probe that sometimes fails is not, the policy is there to do better.

Every mode also checks no arm joint ever exceeds its velocity limit::

    python scripts/check_pick_to_bin.py --resets --num_envs 4096
    python scripts/check_pick_to_bin.py --hold
    python scripts/check_pick_to_bin.py --num_envs 64
    python scripts/check_pick_to_bin.py --task TechtoryCobottaIsaaclab-SodaCanToBin-COBOTTA --num_envs 64
    python scripts/check_pick_to_bin.py --viz kit --num_envs 4   # watch it
"""

import argparse
import sys

import gymnasium as gym
import torch

from isaaclab.app import add_launcher_args, launch_simulation

from isaaclab_tasks.utils import resolve_task_config, setup_preset_cli

import techtory_cobotta_isaaclab.tasks  # noqa: F401

DEFAULT_TASK = "TechtoryCobottaIsaaclab-HammerToBin-COBOTTA"

MAX_DRIFT = 0.002  # m, TCP drift allowed while holding still
ONE_OFF_TERMS = ("reached_object", "picked_object", "near_bin", "placed_in_bin", "success")

# Scripted controller phases.
ABOVE, DESCEND, CLOSE, LIFT, HOLD, CARRY, WAIT = range(7)
PHASE_NAMES = ("above", "descend", "close", "lift", "hold", "carry", "wait")
HOVER = 0.08  # m above the grasp target before descending
SETTLE_STEPS = 25  # 1 s at 25 Hz for the arm to coast to a stop before drift is measured
HOLD_STEPS = 75  # then 2 s of measuring


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
    # The probe is slower than a policy needs to be: it holds still for 2 s to
    # measure drift, and a retry after a missed grasp costs about 4 s.
    env_cfg.episode_length_s = 30.0

    with launch_simulation(env_cfg, args_cli):
        # Imported here: they pull in Isaac Lab modules that need the app running.
        from isaaclab.utils.math import quat_apply, quat_apply_inverse

        from techtory_cobotta_isaaclab.robot import ARM_JOINTS, GRIPPER_OPEN, HOME_POSE
        from techtory_cobotta_isaaclab.scene import layout
        from techtory_cobotta_isaaclab.scene.layout import quat_conj, quat_rotate
        from techtory_cobotta_isaaclab.tasks.pick_to_bin.mdp import state

        env = gym.make(args_cli.task, cfg=env_cfg)
        base = env.unwrapped
        n, device = base.num_envs, base.device
        obj = state.spec(base)
        arm_term = base.action_manager.get_term("arm")
        robot = state.robot(base)
        arm_ids = robot.find_joints(list(ARM_JOINTS), preserve_order=True)[0]
        vel_limit = robot.data.joint_vel_limits.torch[:, arm_ids]
        worst_speed = torch.zeros(n, device=device)
        accel_p99: list[float] = []
        command_accel_p99: list[float] = []
        failures: list[str] = []
        print(f"\n{args_cli.task}: object '{obj.name}', {n} environments")

        def step(action: torch.Tensor):
            with torch.inference_mode():
                out = env.step(action)
            speed = (robot.data.joint_vel.torch[:, arm_ids].abs() / vel_limit).amax(dim=1)
            worst_speed.copy_(torch.maximum(worst_speed, speed))
            accel_p99.append(torch.quantile(robot.data.joint_acc.torch[:, arm_ids].abs().flatten(), 0.99).item())
            command_rms = arm_term.joint_target_acceleration_sq.sqrt()
            command_accel_p99.append(torch.quantile(command_rms, 0.99).item())
            return out

        def to_base(vector_e: torch.Tensor) -> torch.Tensor:
            return quat_apply_inverse(robot.data.root_link_quat_w.torch, vector_e)

        env.reset()
        zero = torch.zeros(n, 4, device=device)

        if args_cli.resets:
            # 2 s: the jaw starts half open and opens at 0.6 rad/s.
            for _ in range(50):
                step(zero)
            corners = state.object_corners(base)
            lowest = corners[..., 2].amin(dim=1) - (layout.TABLE_TOP_Z + obj.rest_clearance)
            # Tilt: the angle between where "up" is on the object now and where it is at rest.
            up_b = torch.tensor(quat_rotate(quat_conj(obj.resting_body.rot), (0.0, 0.0, 1.0)), device=device)
            up = quat_apply(state.object_pose(base)[1], up_b.expand(n, 3))[:, 2]
            tilt = torch.rad2deg(torch.acos(up.clamp(-1.0, 1.0)))
            zone_c = torch.tensor(layout.SPAWN_ZONE.center, device=device)
            zone_h = torch.tensor(layout.SPAWN_ZONE.half, device=device)
            outside = ((corners[..., :2] - zone_c).abs() > zone_h + 0.01).any(dim=-1).any(dim=-1)
            home = torch.tensor([HOME_POSE[j] for j in ARM_JOINTS], device=device)
            home_err = (robot.data.joint_pos.torch[:, arm_ids] - home).abs().amax(dim=1)
            jaw_err = (state.jaw_position(base) - GRIPPER_OPEN).abs()
            print(f"\n{n} resets, after 2 s with zero action:")
            print(f"  tilt from resting    max {tilt.max():6.2f} deg   (< 5)")
            low, high = lowest.min() * 1000, lowest.max() * 1000
            print(f"  lowest corner        {low:+6.1f} .. {high:+6.1f} mm from where it was laid")
            print(f"  object speed         max {state.object_speed(base).max():6.3f} m/s")
            print(f"  outside spawn zone   {int(outside.sum())}")
            print(f"  arm off home         max {home_err.max():6.4f} rad")
            print(f"  jaw off open         max {jaw_err.max():6.3f} rad")
            if tilt.max() > 5.0:
                failures.append("an object tipped over")
            # The bounding box's corners are sharp, the object's are not, and it settles a few mm.
            if lowest.min() < -0.01 or lowest.max() > 0.01:
                failures.append("an object is not resting on the table")
            if outside.any():
                failures.append("an object left the spawn zone")
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

            # The arm ramps its commanded TCP velocity at a limited acceleration, so the probe
            # steers the commanded target (TCP + lead) and brakes on the same profile:
            # v = min(v_max, sqrt(2 a |error|)), which stops on the goal instead of overshooting.
            v_max = torch.tensor([pos_step] * 3 + [yaw_step], device=device) / base.step_dt
            accel = torch.tensor([arm_term.cfg.tcp_acceleration] * 3 + [arm_term.cfg.yaw_acceleration], device=device)

            for _ in range(int(env_cfg.episode_length_s / base.step_dt) + 5):
                tcp = state.tcp_pose(base)[0]
                goal = tcp.clone()
                yaw_goal = torch.zeros(n, device=device)  # yaw still to turn; 0 holds the yaw
                grasp = state.grasp_target(base)
                yaw_err = state.grasp_yaw_error(base)

                above = phase == ABOVE
                goal[above] = grasp[above] + torch.tensor([0.0, 0.0, HOVER], device=device)
                yaw_goal[above] = -yaw_err[above]
                arrived = above & (torch.linalg.norm(tcp - goal, dim=-1) < 0.01) & (yaw_err.abs() < 0.03)
                phase[arrived] = DESCEND

                descend = phase == DESCEND
                goal[descend] = grasp[descend]
                yaw_goal[descend] = -yaw_err[descend]
                closed = descend & state.gripper_rule(base).closed
                phase[closed] = CLOSE
                phase_steps[closed] = 0

                # A miss: the rule reopened. Back off above the grasp point (which re-arms it) and retry.
                rule = state.gripper_rule(base)
                retry = ((phase == CLOSE) | (phase == LIFT) | (phase == HOLD)) & ~rule.closed & ~rule.released
                phase[retry] = ABOVE
                retries += retry.long()

                grasped = state.object_grasped(base)
                lifting = (phase == CLOSE) & grasped & (phase_steps > 5)
                lift_z[lifting] = (bin_target[2] + tcp[:, 2] - state.object_center(base)[:, 2])[lifting]
                phase[lifting] = LIFT

                lift = phase == LIFT
                goal[lift, 2] = lift_z[lift]
                lifted = lift & ((tcp[:, 2] - lift_z).abs() < 0.005)
                phase[lifted] = HOLD
                phase_steps[lifted] = 0

                # HOLD: zero action. The arm coasts to a stop under its acceleration limit
                # first; drift is measured from where it settled, SETTLE_STEPS later.
                hold = phase == HOLD
                settled = hold & (phase_steps == SETTLE_STEPS)
                hold_start[settled] = tcp[settled]
                measuring = hold & (phase_steps > SETTLE_STEPS)
                drift = torch.linalg.norm(tcp - hold_start, dim=-1)
                hold_drift[measuring] = torch.maximum(hold_drift, drift)[measuring]
                held = hold & (phase_steps >= HOLD_STEPS)
                phase[held] = CARRY

                carry = phase == CARRY
                goal[carry] = (tcp + bin_target - state.object_center(base))[carry]
                phase[carry & state.gripper_rule(base).released] = WAIT

                lead = arm_term.target_lead
                error = torch.cat([to_base(goal - tcp), yaw_goal.unsqueeze(-1)], dim=-1) - lead
                speed = torch.minimum(v_max, torch.sqrt(2.0 * accel * error.abs()))
                action = (torch.sign(error) * speed / v_max).clamp(-1, 1)
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

            print(f"\nScripted probe, first episode of {n} environments:")
            header = f"  {'env':>3}  {'outcome':<20} {'furthest phase':<15} {'retries':>7} {'hold drift':>10}  "
            print(header + "  ".join(terms))
            for i in range(n):
                row = "  ".join(f"{v:{max(len(t), 6)}.2f}" for t, v in zip(terms, paid[i].tolist(), strict=True))
                print(
                    f"  {i:3d}  {outcome[i]:<20} {PHASE_NAMES[int(reached[i])]:<15} {int(retries[i]):7d}"
                    f" {hold_drift[i] * 1000:8.2f}mm  {row}"
                )
            successes = sum(o == "success" for o in outcome)
            print(f"\n  success {successes}/{n} (information only), hold drift max {hold_drift.max() * 1000:.2f} mm")
            if successes == 0:
                failures.append("no episode could succeed")
            for name in ONE_OFF_TERMS:
                weight = base.reward_manager.get_term_cfg(name).weight
                if (paid[:, terms.index(name)] > weight * 1.001).any():
                    failures.append(f"'{name}' paid more than once in an episode")
            if hold_drift.max() > MAX_DRIFT:
                failures.append("the TCP drifted while holding the object")

        # 99th percentile: the value 99% of the samples (every environment, every step) stay below;
        # unlike the maximum, one freak sample does not decide it.
        print(f"  commanded joint accel 99th pct {max(command_accel_p99):.2f} rad/s^2 (rms over 6 joints; information)")
        print(f"  measured joint accel 99th pct {max(accel_p99):.2f} rad/s^2 (incl. servo vibration; information)")
        print(f"  arm joint speed      max {worst_speed.max():.3f} x its velocity limit (<= 1.02)")
        if worst_speed.max() > 1.02:
            failures.append("an arm joint exceeded its velocity limit")

        env.close()
        print("\n" + ("PASS" if not failures else "FAIL: " + "; ".join(failures)))
        if failures:
            sys.exit(1)


if __name__ == "__main__":
    main()
