# Copyright (c) 2026, Fraunhofer IPA.
# All rights reserved.
#
# SPDX-License-Identifier: Apache-2.0

"""Drive the Cobotta Pro + RG6 through each joint in turn, reading the wrist F/T.

This is the fastest way to tell whether the robot actually works. It commands
one thing at a time -- each arm joint out and back, then the gripper open and
closed -- and prints the arm, the tool position and the wrist wrench as each
phase ends, so a joint that does not track or a sensor that reads nothing
shows up as a number rather than as a policy that will not learn.

    python scripts/play.py                  # Kit viewport (default visualizer)
    python scripts/play.py --viz none       # printed readings only, no window
    python scripts/play.py --num_envs 4 --phase_seconds 1.5
    python scripts/play.py --physics newton_mjwarp --viz newton_gl   # kitless Newton

What to expect: every ``q`` column tracks its commanded offset from the home
pose; ``grip`` reaches about -0.628 open and +0.628 closed; and at rest the
wrist force is about 9.81 N -- the 1.0 kg RG6's weight -- with near-zero
torque. The direction that 9.81 N points in depends on how the wrist is
oriented, which is why ``|F|`` is printed too.
"""

import argparse
import contextlib
import sys

import gymnasium as gym
import torch

from isaaclab.app import add_launcher_args, launch_simulation

import isaaclab_tasks  # noqa: F401

with contextlib.suppress(ImportError):
    import isaaclab_tasks_experimental  # noqa: F401
from isaaclab_tasks.utils import (
    resolve_task_config,
    setup_preset_cli,
)

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--task", type=str, default="Techtory-Cobotta-Base-v0", help="Name of the task.")
parser.add_argument("--num_envs", type=int, default=1, help="Number of environments to simulate.")
parser.add_argument("--phase_seconds", type=float, default=2.0, help="How long to hold each command.")
# NOT forwarded to the launcher as `physics`: the launcher's own override swaps
# every physics config for a bare, untuned one (make_physics_cfg), and does not
# touch the scene. Selecting the Hydra preset instead picks this env's tuned
# `newton_mjwarp` solver AND every scene preset of that name (e.g. the cell's
# Newton variant). `--physics X` is exactly `physics=X`.
parser.add_argument(
    "--physics",
    dest="physics_preset",
    choices=["physx", "isaacsim_physx", "ovphysx", "newton_mjwarp"],
    default=None,
    help="Physics backend preset (same as the Hydra token physics=NAME).",
)
add_launcher_args(parser)
parser.set_defaults(visualizer=["kit"])
args_cli, hydra_args = setup_preset_cli(parser)
if args_cli.physics_preset is not None:
    hydra_args.append(f"physics={args_cli.physics_preset}")
    # Kit's viewport cannot run beside a kitless backend; default to Newton's GL window.
    if args_cli.physics_preset == "newton_mjwarp" and args_cli.visualizer == ["kit"]:
        args_cli.visualizer = ["newton_gl"]
sys.argv = [sys.argv[0]] + hydra_args

import techtory_cobotta_isaaclab.tasks  # noqa: F401

# (label, arm joint index or None, arm offset in action units, gripper).
# Arm actions are offsets from the home pose at scale 0.5 rad, so 0.6 is
# 0.3 rad. Gripper: +1 open, -1 closed, 0 half-open.
PHASES: list[tuple[str, int | None, float, float]] = [
    ("settle", None, 0.0, 0.0),
    *[(f"joint {i + 1} +", i, 0.6, 0.0) for i in range(6)],
    *[(f"joint {i + 1} -", i, -0.6, 0.0) for i in range(6)],
    ("home", None, 0.0, 0.0),
    ("gripper open", None, 0.0, 1.0),
    ("gripper close", None, 0.0, -1.0),
    ("gripper half", None, 0.0, 0.0),
]

# Observation slices, per ObservationsCfg.PolicyCfg declaration order.
ARM_POS = slice(0, 6)
GRIPPER_POS = slice(12, 13)
EE_POS = slice(14, 17)
WRENCH = slice(21, 27)


def report(label: str, obs: torch.Tensor) -> None:
    """Print environment 0's arm, gripper, tool position and wrist wrench."""
    row = obs[0]
    q = " ".join(f"{v:+.2f}" for v in row[ARM_POS].tolist())
    grip = row[GRIPPER_POS].item()
    ee = " ".join(f"{v:+.3f}" for v in row[EE_POS].tolist())
    wrench = row[WRENCH]
    f, t = wrench[:3], wrench[3:]
    print(
        f"  {label:<14} q=[{q}] grip={grip:+.3f} ee=[{ee}] | "
        f"F=({f[0]:+7.2f} {f[1]:+7.2f} {f[2]:+7.2f}) |F|={f.norm():6.2f} N  "
        f"|T|={t.norm():.3f} N.m"
    )


def main() -> None:
    torch.manual_seed(42)

    env_cfg, _ = resolve_task_config(args_cli.task, "")

    with launch_simulation(env_cfg, args_cli):
        env_cfg.scene.num_envs = args_cli.num_envs
        if args_cli.device is not None:
            env_cfg.sim.device = args_cli.device
        # One phase per episode segment; do not let the clock reset mid-sweep.
        env_cfg.episode_length_s = args_cli.phase_seconds * (len(PHASES) + 1)

        env = gym.make(args_cli.task, cfg=env_cfg)
        print(f"[INFO]: observation space: {env.observation_space}")
        print(f"[INFO]: action space:      {env.action_space}")

        obs, _ = env.reset()
        device = env.unwrapped.device
        steps = max(1, int(args_cli.phase_seconds / env.unwrapped.step_dt))

        print(f"\nDriving each joint for {args_cli.phase_seconds:.1f} s:\n")
        actions = torch.zeros(env.action_space.shape, device=device)

        for label, joint, offset, grip in PHASES:
            actions.zero_()
            if joint is not None:
                actions[:, joint] = offset
            actions[:, 6] = grip

            for _ in range(steps):
                with torch.inference_mode():
                    obs, _, _, _, _ = env.step(actions)

            report(label, obs["policy"] if isinstance(obs, dict) else obs)

        env.close()


if __name__ == "__main__":
    main()
