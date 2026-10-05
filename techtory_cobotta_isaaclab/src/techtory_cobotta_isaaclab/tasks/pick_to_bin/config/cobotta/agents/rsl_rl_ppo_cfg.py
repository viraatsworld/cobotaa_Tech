# Copyright (c) 2026, Fraunhofer IPA.
# All rights reserved.
#
# SPDX-License-Identifier: Apache-2.0

from isaaclab.utils import configclass

from isaaclab_rl.rsl_rl import RslRlMLPModelCfg, RslRlOnPolicyRunnerCfg, RslRlPpoAlgorithmCfg


@configclass
class PPORunnerCfg(RslRlOnPolicyRunnerCfg):
    """PPO for pick-to-bin (the hammer), from the base task's config.

    Asymmetric: the actor sees only what the real robot can provide (the
    ``policy`` group); the critic also gets the simulator's ground truth
    (``critic``). ``gamma`` is raised for the long horizon -- 500 policy steps,
    with the big rewards at the end.

    Batch: 4096 environments x 32 steps = 131k transitions per iteration, in 4
    mini-batches of 32k -- small next to the 20 GB of GPU memory, as the networks
    are small. 3000 iterations are ~390M steps.
    """

    num_steps_per_env = 32
    max_iterations = 3000
    save_interval = 100
    experiment_name = "techtory_cobotta_hammer_to_bin"
    obs_groups = {"actor": ["policy"], "critic": ["policy", "critic"]}
    actor = RslRlMLPModelCfg(
        hidden_dims=[256, 128, 64],
        activation="elu",
        obs_normalization=True,
        distribution_cfg=RslRlMLPModelCfg.GaussianDistributionCfg(init_std=1.0),
    )
    critic = RslRlMLPModelCfg(
        hidden_dims=[256, 128, 64],
        activation="elu",
        obs_normalization=True,
    )
    algorithm = RslRlPpoAlgorithmCfg(
        value_loss_coef=1.0,
        use_clipped_value_loss=True,
        clip_param=0.2,
        entropy_coef=0.005,
        num_learning_epochs=5,
        num_mini_batches=4,
        learning_rate=1.0e-3,
        schedule="adaptive",
        gamma=0.995,
        lam=0.95,
        desired_kl=0.01,
        max_grad_norm=1.0,
    )


@configclass
class SodaCanPPORunnerCfg(PPORunnerCfg):
    """The same PPO for the soda can, logged apart."""

    experiment_name = "techtory_cobotta_soda_can_to_bin"
