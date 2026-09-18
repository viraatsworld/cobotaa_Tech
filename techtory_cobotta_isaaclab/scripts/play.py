# Copyright (c) 2026, Fraunhofer IPA.
# All rights reserved.
#
# SPDX-License-Identifier: Apache-2.0

"""Drive the Cobotta through each joint in turn, then open and close the gripper.

The fastest way to tell whether the robot and its cell work. One thing is
commanded at a time, and as each phase ends the script prints environment 0's
arm joints, jaw angle, TCP position, wrist wrench and object positions -- so a
joint that does not move, a gripper that does not close, an object that falls
through the shelf or a wrist sensor that reads nothing shows up as a number.

Runs on Newton (MuJoCo-Warp) by default; ``physics=isaacsim_physx`` selects
Isaac Sim PhysX, the only backend with a working wrist F/T and grasp::

    python scripts/play.py                     # Isaac Sim viewport
    python scripts/play.py --viz none          # printed readout only, no window
    python scripts/play.py physics=isaacsim_physx
    python scripts/play.py --num_envs 4 --phase_seconds 3.0

Expect each joint to settle near its commanded offset (the arm moves at its
MoveIt velocity limits, 0.33-0.60 rad/s, so give phases 2 s or more), the
jaw to reach about -0.62 rad open and +0.62 rad closed, both objects to stay
put on the shelf, and -- on Isaac Sim PhysX -- the wrist to read the gripper's
weight (~9.8 N) at rest. Newton reads zero there.
"""

import argparse
import sys

import gymnasium as gym
import torch

from isaaclab.app import add_launcher_args, launch_simulation

from isaaclab_tasks.utils import resolve_task_config, setup_preset_cli

import techtory_cobotta_isaaclab.tasks  # noqa: F401

DEFAULT_TASK = "TechtoryCobottaIsaaclab-Base-COBOTTA"
DEFAULT_PHYSICS = "newton_mjwarp"
ARM_OFFSET = 0.3  # rad; small enough that no single joint swings the arm into the cell

# (label, joint index to offset or None, offset in action units, gripper command)
# Arm actions are offsets from the home pose at 0.5 rad per unit.
# fmt: off
PHASES: list[tuple[str, int | None, float, float]] = [
    ("settle",        None,  0.0,  0.0),
    *[(f"joint {j + 1} +", j, ARM_OFFSET / 0.5, 0.0) for j in range(6)],
    ("home",          None,  0.0,  0.0),
    ("gripper open",  None,  0.0,  1.0),
    ("gripper close", None,  0.0, -1.0),
    ("gripper half",  None,  0.0,  0.0),
]
# fmt: on

# Observation slices, per ObservationsCfg.PolicyCfg declaration order.
ARM_POS = slice(0, 6)
GRIPPER_POS = slice(12, 13)
TCP_POS = slice(14, 17)
WRENCH = slice(21, 27)
HAMMER_POS = slice(27, 30)
SODA_CAN_POS = slice(34, 37)


def parse_args() -> argparse.Namespace:
    """Parse the script's flags and hand the remainder (preset selectors, Hydra overrides) to Hydra."""
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--task", type=str, default=DEFAULT_TASK, help="Name of the task.")
    parser.add_argument("--num_envs", type=int, default=1, help="Number of environments to simulate.")
    parser.add_argument("--phase_seconds", type=float, default=2.5, help="How long to hold each command.")
    add_launcher_args(parser)
    # Let the task config pick the device; open the Isaac Sim viewport unless told otherwise.
    parser.set_defaults(device=None, visualizer=["kit"])
    args_cli, hydra_args = setup_preset_cli(parser)

    if not any(arg.startswith(("physics=", "env.sim.physics=")) for arg in hydra_args):
        hydra_args.append(f"physics={DEFAULT_PHYSICS}")
    sys.argv = [sys.argv[0]] + hydra_args
    return args_cli


def _fmt(values: torch.Tensor, width: int = 7, digits: int = 3) -> str:
    return " ".join(f"{v:+{width}.{digits}f}" for v in values.tolist())


def report(label: str, obs: torch.Tensor) -> None:
    """Print environment 0's joints, jaw, TCP, wrist wrench and object positions."""
    row = obs[0]
    print(f"  {label:<14} q=[{_fmt(row[ARM_POS], 6, 3)}] jaw={row[GRIPPER_POS].item():+.3f}")
    print(f"  {'':<14} tcp=[{_fmt(row[TCP_POS])}] F=[{_fmt(row[WRENCH][:3], 6, 2)}] T=[{_fmt(row[WRENCH][3:], 6, 3)}]")
    print(f"  {'':<14} hammer=[{_fmt(row[HAMMER_POS])}] can=[{_fmt(row[SODA_CAN_POS])}]")


def main() -> None:
    args_cli = parse_args()
    torch.manual_seed(42)

    env_cfg, _ = resolve_task_config(args_cli.task, "")
    env_cfg.scene.num_envs = args_cli.num_envs
    if args_cli.device is not None:
        env_cfg.sim.device = args_cli.device
    args_cli.device = env_cfg.sim.device
    # One episode for the whole sweep: a timeout would reset the robot mid-sweep.
    env_cfg.episode_length_s = args_cli.phase_seconds * (len(PHASES) + 1)
    try:
        env_cfg.validate()
    except (TypeError, ValueError) as exc:
        raise SystemExit(f"Invalid environment configuration: {exc}") from None

    with launch_simulation(env_cfg, args_cli):
        env = gym.make(args_cli.task, cfg=env_cfg)
        print(f"[INFO]: physics:           {type(env_cfg.sim.physics).__name__}")
        print(f"[INFO]: observation space: {env.observation_space}")
        print(f"[INFO]: action space:      {env.action_space}")

        obs, _ = env.reset()
        unwrapped = env.unwrapped
        sim = unwrapped.sim
        steps = max(1, int(args_cli.phase_seconds / unwrapped.step_dt))

        print(f"\nHolding each command for {args_cli.phase_seconds:.1f} s (policy observations, env 0):\n")
        actions = torch.zeros(env.action_space.shape, device=unwrapped.device)

        for label, joint, offset, grip in PHASES:
            actions.zero_()
            if joint is not None:
                actions[:, joint] = offset
            actions[:, 6] = grip

            for _ in range(steps):
                # Closing the viewport window ends the sweep.
                if not sim.is_headless_or_exist_active_visualizer():
                    break
                with torch.inference_mode():
                    obs, _, _, _, _ = env.step(actions)
            else:
                report(label, obs["policy"] if isinstance(obs, dict) else obs)
                continue
            print("[INFO]: visualizer closed, stopping.")
            break

        env.close()


if __name__ == "__main__":
    main()
