# Copyright (c) 2022-2025, The Isaac Lab Project Developers
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

from isaaclab.utils import configclass

from isaaclab_rl.rsl_rl import (
    RslRlOnPolicyRunnerCfg,
    RslRlPpoActorCriticCfg,
    RslRlPpoAlgorithmCfg,
)


@configclass
class LiftCubePPORunnerCfg(RslRlOnPolicyRunnerCfg):
    # -------------------------------
    # Rollout / Training Horizon
    # -------------------------------
    # 한 env 당 수집하는 step 수 (control step 기준).
    # episode_length_s = 10s, sim.dt=0.01, decimation=4 → 약 250 control step/episode
    num_steps_per_env = 32          # 24 → 32로 약간 증가 (조금 더 안정적인 업데이트)
    max_iterations = 2500           # 1500 → 2500 (조금 더 충분히 학습)
    save_interval = 50
    experiment_name = "e0509_lift_delta_joint"

    # Observation/action groups
    obs_groups = {
        "policy": ["policy"],  # Actor용 obs
        "critic": ["policy"],  # Critic도 동일한 obs 사용
    }

    # -------------------------------
    # Actor–Critic Network 설정
    # -------------------------------
    policy = RslRlPpoActorCriticCfg(
        # 액션은 "default_joint_pos + action * 0.03" 형태의 delta joint 이므로
        # 초기에 너무 큰 exploration을 쓰면 이상한 포즈로 튀기 쉬워서 std를 작게
        init_noise_std=0.2,            # 0.5 → 0.2 로 축소

        # 관측을 정규화해서 joint_pos_rel / joint_vel_rel / target_pose / last_action 스케일을 정리
        actor_obs_normalization=True,   # False → True
        critic_obs_normalization=True,  # False → True

        actor_hidden_dims=[256, 128, 64],
        critic_hidden_dims=[256, 128, 64],
        activation="elu",
    )

    # -------------------------------
    # PPO Hyper-parameters
    # -------------------------------
    algorithm = RslRlPpoAlgorithmCfg(
        value_loss_coef=1.0,
        use_clipped_value_loss=True,
        clip_param=0.2,

        # exploration 정도 (엔트로피 보너스)
        entropy_coef=0.006,     # 그대로 유지 (너무 크면 계속 흔들릴 수 있으니 과하게 올리진 않음)

        num_learning_epochs=5,
        num_mini_batches=4,

        # 학습률은 약간 키우거나, 그대로 1e-4를 써도 됨.
        # 여기서는 조금 더 빠르게 수렴하도록 2.5e-4 제안
        learning_rate=2.5e-4,   # 1.0e-4 → 2.5e-4
        schedule="adaptive",

        # 리턴 감쇠 계수
        gamma=0.99,             # 0.98 → 0.99 (10초 horizon에서 조금 더 긴 시야)
        lam=0.95,

        desired_kl=0.01,
        max_grad_norm=1.0,
    )
