# Copyright (c) 2026, Fraunhofer IPA.
# All rights reserved.
#
# SPDX-License-Identifier: Apache-2.0

import gymnasium as gym

from . import agents

##
# Register Gym environments: one pick-to-bin task per object, each with a Play variant.
##

_VARIANTS = {
    # task id: (env config, PPO config)
    "TechtoryCobottaIsaaclab-HammerToBin-COBOTTA": ("HammerToBinEnvCfg", "PPORunnerCfg"),
    "TechtoryCobottaIsaaclab-HammerToBin-COBOTTA-Play": ("HammerToBinEnvCfg_PLAY", "PPORunnerCfg"),
    "TechtoryCobottaIsaaclab-SodaCanToBin-COBOTTA": ("SodaCanToBinEnvCfg", "SodaCanPPORunnerCfg"),
    "TechtoryCobottaIsaaclab-SodaCanToBin-COBOTTA-Play": ("SodaCanToBinEnvCfg_PLAY", "SodaCanPPORunnerCfg"),
}

for _task, (_env_cfg, _agent_cfg) in _VARIANTS.items():
    gym.register(
        id=_task,
        entry_point="isaaclab.envs:ManagerBasedRLEnv",
        disable_env_checker=True,
        kwargs={
            "env_cfg_entry_point": f"{__name__}.env_cfg:{_env_cfg}",
            "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:{_agent_cfg}",
        },
    )
