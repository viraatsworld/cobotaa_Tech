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


def test_isaaclab_cli_discovers_the_package() -> None:
    """`isaaclab train/play/zero_agent` import every package in this entry-point group."""
    names = {ep.value for ep in importlib.metadata.entry_points(group="isaaclab.tasks")}
    assert "techtory_cobotta_isaaclab.tasks" in names
