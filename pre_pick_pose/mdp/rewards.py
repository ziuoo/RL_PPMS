# Copyright (c) 2022-2025, The Isaac Lab Project Developers
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

from __future__ import annotations

from typing import TYPE_CHECKING

import torch

from isaaclab.assets import RigidObject
from isaaclab.managers import SceneEntityCfg
from isaaclab.sensors import FrameTransformer
from isaaclab.utils.math import quat_error_magnitude

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv


def position_command_error_tanh(
    env: ManagerBasedRLEnv,
    std: float,
    command_name: str,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("ee_frame"),
) -> torch.Tensor:
    """(선택) tanh 커널 기반 위치 보상.

    Args:
        env: The environment.
        std: Standard deviation for tanh kernel.
        command_name: Name of the command (pose command).
        asset_cfg: Frame transformer configuration for end-effector.

    Returns:
        Reward tensor based on position error.
    """
    asset: FrameTransformer = env.scene[asset_cfg.name]
    command = env.command_manager.get_command(command_name)

    # EE 현재 위치 (base frame 기준)
    curr_pos_b = asset.data.target_pos_source[:, 0, :]
    des_pos_b = command[:, :3]

    pos_error = torch.norm(des_pos_b - curr_pos_b, dim=1)
    return 1.0 - torch.tanh(pos_error / std)


def position_command_error_exp(
    env: ManagerBasedRLEnv,
    alpha: float,
    command_name: str,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("ee_frame"),
) -> torch.Tensor:
    """목표 위치 도달 보상 (exponential kernel).

    R = exp(-alpha * ||P_target - P_ee||^2)

    - delta joint 액션을 쓰더라도, 목표는 여전히 EE pose이므로 그대로 사용 가능.
    - alpha는 5.0~20.0 정도에서 조정.

    Returns:
        (num_envs,) 0~1 사이 보상.
    """
    asset: FrameTransformer = env.scene[asset_cfg.name]
    command = env.command_manager.get_command(command_name)

    curr_pos_b = asset.data.target_pos_source[:, 0, :]
    des_pos_b = command[:, :3]

    pos_error_sq = torch.sum((des_pos_b - curr_pos_b) ** 2, dim=1)
    return torch.exp(-alpha * pos_error_sq)


def orientation_command_error(
    env: ManagerBasedRLEnv,
    command_name: str,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("ee_frame"),
) -> torch.Tensor:
    """목표 자세(쿼터니언) 도달 보상.

    Returns:
        (num_envs,) orientation error 기반 보상.
    """
    asset: FrameTransformer = env.scene[asset_cfg.name]
    command = env.command_manager.get_command(command_name)

    curr_quat_b = asset.data.target_quat_source[:, 0, :]  # (qw, qx, qy, qz)
    des_quat_b = command[:, 3:]

    quat_err = quat_error_magnitude(curr_quat_b, des_quat_b)
    return 1.0 - torch.tanh(quat_err / 0.25)


def z_axis_alignment_reward(
    env: ManagerBasedRLEnv,
    command_name: str,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("ee_frame"),
) -> torch.Tensor:
    """EE의 z축이 목표 z축과 얼마나 잘 align 되는지 보상.

    - 1.0  : 완전히 같은 방향
    - 0.0  : 직교
    - -1.0 : 반대 방향
    """
    asset: FrameTransformer = env.scene[asset_cfg.name]
    command = env.command_manager.get_command(command_name)

    curr_quat_b = asset.data.target_quat_source[:, 0, :]
    des_quat_b = command[:, 3:]

    # 현재 z축
    curr_w, curr_x, curr_y, curr_z = curr_quat_b[:, 0], curr_quat_b[:, 1], curr_quat_b[:, 2], curr_quat_b[:, 3]
    curr_z_axis = torch.stack(
        [
            2 * (curr_x * curr_z + curr_w * curr_y),
            2 * (curr_y * curr_z - curr_w * curr_x),
            1 - 2 * (curr_x**2 + curr_y**2),
        ],
        dim=1,
    )

    # 목표 z축
    des_w, des_x, des_y, des_z = des_quat_b[:, 0], des_quat_b[:, 1], des_quat_b[:, 2], des_quat_b[:, 3]
    des_z_axis = torch.stack(
        [
            2 * (des_x * des_z + des_w * des_y),
            2 * (des_y * des_z - des_w * des_x),
            1 - 2 * (des_x**2 + des_y**2),
        ],
        dim=1,
    )

    curr_z_axis_norm = curr_z_axis / (torch.norm(curr_z_axis, dim=1, keepdim=True) + 1e-8)
    des_z_axis_norm = des_z_axis / (torch.norm(des_z_axis, dim=1, keepdim=True) + 1e-8)

    dot_product = torch.sum(curr_z_axis_norm * des_z_axis_norm, dim=1)
    return dot_product


def multi_axis_alignment_reward(
    env: ManagerBasedRLEnv,
    command_name: str,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("ee_frame"),
) -> torch.Tensor:
    """x, y, z 세 축 정렬 정도를 평균 내는 자세 보상."""
    asset: FrameTransformer = env.scene[asset_cfg.name]
    command = env.command_manager.get_command(command_name)

    curr_quat_b = asset.data.target_quat_source[:, 0, :]
    des_quat_b = command[:, 3:]

    curr_w, curr_x, curr_y, curr_z = (
        curr_quat_b[:, 0],
        curr_quat_b[:, 1],
        curr_quat_b[:, 2],
        curr_quat_b[:, 3],
    )
    des_w, des_x, des_y, des_z = (
        des_quat_b[:, 0],
        des_quat_b[:, 1],
        des_quat_b[:, 2],
        des_quat_b[:, 3],
    )

    # x-axis
    curr_x_axis = torch.stack(
        [
            1 - 2 * (curr_y**2 + curr_z**2),
            2 * (curr_x * curr_y + curr_w * curr_z),
            2 * (curr_x * curr_z - curr_w * curr_y),
        ],
        dim=1,
    )
    des_x_axis = torch.stack(
        [
            1 - 2 * (des_y**2 + des_z**2),
            2 * (des_x * des_y + des_w * des_z),
            2 * (des_x * des_z - des_w * des_y),
        ],
        dim=1,
    )

    # y-axis
    curr_y_axis = torch.stack(
        [
            2 * (curr_x * curr_y - curr_w * curr_z),
            1 - 2 * (curr_x**2 + curr_z**2),
            2 * (curr_y * curr_z + curr_w * curr_x),
        ],
        dim=1,
    )
    des_y_axis = torch.stack(
        [
            2 * (des_x * des_y - des_w * des_z),
            1 - 2 * (des_x**2 + des_z**2),
            2 * (des_y * des_z + des_w * des_x),
        ],
        dim=1,
    )

    # z-axis
    curr_z_axis = torch.stack(
        [
            2 * (curr_x * curr_z + curr_w * curr_y),
            2 * (curr_y * curr_z - curr_w * curr_x),
            1 - 2 * (curr_x**2 + curr_y**2),
        ],
        dim=1,
    )
    des_z_axis = torch.stack(
        [
            2 * (des_x * des_z + des_w * des_y),
            2 * (des_y * des_z - des_w * des_x),
            1 - 2 * (des_x**2 + des_y**2),
        ],
        dim=1,
    )

    # 정규화
    def _norm(v):
        return v / (torch.norm(v, dim=1, keepdim=True) + 1e-8)

    curr_x_axis_norm, des_x_axis_norm = _norm(curr_x_axis), _norm(des_x_axis)
    curr_y_axis_norm, des_y_axis_norm = _norm(curr_y_axis), _norm(des_y_axis)
    curr_z_axis_norm, des_z_axis_norm = _norm(curr_z_axis), _norm(des_z_axis)

    x_alignment = torch.sum(curr_x_axis_norm * des_x_axis_norm, dim=1)
    y_alignment = torch.sum(curr_y_axis_norm * des_y_axis_norm, dim=1)
    z_alignment = torch.sum(curr_z_axis_norm * des_z_axis_norm, dim=1)

    return (x_alignment + y_alignment + z_alignment) / 3.0


def medicine_cabinet_distance_penalty(
    env: ManagerBasedRLEnv,
    threshold: float = 0.05,
    ee_frame_cfg: SceneEntityCfg = SceneEntityCfg("ee_frame"),
    cabinet_cfg: SceneEntityCfg = SceneEntityCfg("medicine_cabinet"),
) -> torch.Tensor:
    """End-effector가 cabinet에 너무 가까이 가면 패널티.

    (실제 프로젝트에서 필요 없으면 안 써도 됨)
    """
    ee_frame: FrameTransformer = env.scene[ee_frame_cfg.name]
    cabinet: RigidObject = env.scene[cabinet_cfg.name]

    ee_pos_w = ee_frame.data.target_pos_w[:, 0, :]  # (num_envs, 3)
    cabinet_pos_w = cabinet.data.root_pos_w  # (num_envs, 3)

    distance = torch.norm(ee_pos_w - cabinet_pos_w, dim=1)
    return torch.where(distance < threshold, -1.0, 0.0)


# ======================================================================
# ⭐ Delta joint action 용 부드러움 페널티 (새로 추가)
# ======================================================================

def delta_joint_action_penalty(
    env: ManagerBasedRLEnv,
    scale: float = 0.1,
) -> torch.Tensor:
    """Delta joint 액션 크기를 줄이기 위한 부드러움 패널티.

    전제:
        - action_space가 "delta joint" 방식일 때 사용.
        - env.action_manager.action: (num_envs, num_dofs), 각 step에서 Δq(rad).

    보상:
        R = -scale * ||Δq||^2

    - scale을 키우면: 액션이 더 작아지도록 강하게 압박 (너무 크면 학습이 느려질 수 있음)
    - 0.05 ~ 0.2 정도 범위에서 튜닝 추천.
    """
    actions = env.action_manager.action  # (num_envs, num_dofs)
    # 각 env별로 Δq L2 제곱을 구해서 음수로 반환 (패널티)
    penalty = -scale * torch.sum(actions * actions, dim=1)
    return penalty
