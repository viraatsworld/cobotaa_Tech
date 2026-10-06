# Copyright (c) 2026, Fraunhofer IPA.
# All rights reserved.
#
# SPDX-License-Identifier: Apache-2.0

"""The task registrations and the Isaac Lab CLI entry point."""

import importlib.metadata

import gymnasium as gym
import pytest

import techtory_cobotta_isaaclab.tasks  # noqa: F401

pytestmark = pytest.mark.unit

TASK = "TechtoryCobottaIsaaclab-Base-COBOTTA"


def test_task_registration() -> None:
    spec = gym.spec(TASK)
    assert spec.entry_point == "isaaclab.envs:ManagerBasedRLEnv"
    assert (
        spec.kwargs["env_cfg_entry_point"] == "techtory_cobotta_isaaclab.tasks.base.config.cobotta.env_cfg:BaseEnvCfg"
    )
    for library in ("rsl_rl", "rl_games", "skrl", "sb3"):
        assert f"{library}_cfg_entry_point" in spec.kwargs


@pytest.mark.parametrize(
    ("task", "cfg", "agent"),
    [
        ("TechtoryCobottaIsaaclab-HammerToBin-COBOTTA", "HammerToBinEnvCfg", "PPORunnerCfg"),
        ("TechtoryCobottaIsaaclab-HammerToBin-COBOTTA-Play", "HammerToBinEnvCfg_PLAY", "PPORunnerCfg"),
        ("TechtoryCobottaIsaaclab-SodaCanToBin-COBOTTA", "SodaCanToBinEnvCfg", "SodaCanPPORunnerCfg"),
        ("TechtoryCobottaIsaaclab-SodaCanToBin-COBOTTA-Play", "SodaCanToBinEnvCfg_PLAY", "SodaCanPPORunnerCfg"),
    ],
)
def test_pick_to_bin_registration(task: str, cfg: str, agent: str) -> None:
    spec = gym.spec(task)
    assert spec.entry_point == "isaaclab.envs:ManagerBasedRLEnv"
    module = "techtory_cobotta_isaaclab.tasks.pick_to_bin.config.cobotta"
    assert spec.kwargs["env_cfg_entry_point"] == f"{module}.env_cfg:{cfg}"
    assert spec.kwargs["rsl_rl_cfg_entry_point"] == f"{module}.agents.rsl_rl_ppo_cfg:{agent}"


def test_isaaclab_cli_discovers_the_package() -> None:
    """`isaaclab train/play/zero_agent` import every package in this entry-point group."""
    names = {ep.value for ep in importlib.metadata.entry_points(group="isaaclab.tasks")}
    assert "techtory_cobotta_isaaclab.tasks" in names
