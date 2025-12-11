# Copyright (c) 2022-2025, The Isaac Lab Project Developers
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Common functions that can be used to activate certain terminations for the lift / reaching task.

The functions can be passed to the :class:`isaaclab.managers.TerminationTermCfg` object to enable
the termination introduced by the function.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

import torch

from isaaclab.assets import RigidObject
from isaaclab.managers import SceneEntityCfg
from isaaclab.sensors import FrameTransformer
from isaaclab.utils.math import combine_frame_transforms

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv


# =============================================================================
# 1) EE가 "명령된 EE pose"에 도달했을 때 종료 (delta joint 액션용 핵심)
# =============================================================================

def ee_reached_command_pose(
    env: ManagerBasedRLEnv,
    command_name: str = "ee_pose_command",
    pos_threshold: float = 0.01,
    rot_threshold: float = 0.15,
    ee_frame_cfg: SceneEntityCfg = SceneEntityCfg("ee_frame"),
) -> torch.Tensor:
    """Terminate when end-effector reaches the commanded pose (position + orientation).

    Args:
        env: The environment.
        command_name: Name of the pose command (x, y, z, qw, qx, qy, qz) in base frame.
        pos_threshold: Position tolerance [m].
        rot_threshold: Orientation tolerance [rad] (approx).
        ee_frame_cfg: Frame transformer configuration for end-effector.

    Returns:
        Boolean tensor (num_envs,) indicating which envs should terminate.
    """
    # EE frame (source = robot base, target = EE)
    ee_frame: FrameTransformer = env.scene[ee_frame_cfg.name]

    # Command: (x, y, z, qw, qx, qy, qz) in base frame
    command = env.command_manager.get_command(command_name)
    des_pos_b = command[:, :3]
    des_quat_b = command[:, 3:]  # (qw, qx, qy, qz)

    # 현재 EE pose (base frame 기준) : target_*_source 사용
    curr_pos_b = ee_frame.data.target_pos_source[:, 0, :]   # (num_envs, 3)
    curr_quat_b = ee_frame.data.target_quat_source[:, 0, :] # (num_envs, 4)

    # 위치 오차
    pos_error = torch.norm(des_pos_b - curr_pos_b, dim=1)

    # 쿼터니언 거리 = 2*arccos(|dot(q1, q2)|) 근사 대신, (1 - cos θ) 기반 거리 사용
    # dot = cos(theta/2) 이라서, 1 - dot^2 ~ 회전 크기와 비례
    dot = torch.sum(des_quat_b * curr_quat_b, dim=1).abs()
    dot = torch.clamp(dot, 0.0, 1.0)
    # 회전 거리 근사 (rad)
    # theta ≈ 2 * arccos(dot)
    rot_error = 2.0 * torch.arccos(dot)

    reached = (pos_error < pos_threshold) & (rot_error < rot_threshold)
    return reached


# =============================================================================
# 2) (기존) object 기반 termination - lift task 등에 사용 가능
# =============================================================================

def object_reached_goal(
    env: ManagerBasedRLEnv,
    command_name: str = "object_pose",
    threshold: float = 0.02,
    robot_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
    object_cfg: SceneEntityCfg = SceneEntityCfg("object"),
) -> torch.Tensor:
    """Termination condition for the object reaching the goal position.

    Args:
        env: The environment.
        command_name: The name of the command that is used to control the object.
        threshold: The threshold for the object to reach the goal position. Defaults to 0.02.
        robot_cfg: The robot configuration. Defaults to SceneEntityCfg("robot").
        object_cfg: The object configuration. Defaults to SceneEntityCfg("object").
    """
    robot: RigidObject = env.scene[robot_cfg.name]
    obj: RigidObject = env.scene[object_cfg.name]
    command = env.command_manager.get_command(command_name)

    # desired position in world frame
    des_pos_b = command[:, :3]
    des_pos_w, _ = combine_frame_transforms(robot.data.root_pos_w, robot.data.root_quat_w, des_pos_b)

    # distance object ↔ desired position
    distance = torch.norm(des_pos_w - obj.data.root_pos_w[:, :3], dim=1)
    return distance < threshold


# =============================================================================
# 3) EE가 object에 너무 가까이 가면 종료 (pre-grasp 등에서 사용 가능)
# =============================================================================

def ee_object_contact(
    env: ManagerBasedRLEnv,
    threshold: float = 0.01,
    ee_frame_cfg: SceneEntityCfg = SceneEntityCfg("ee_frame"),
    object_cfg: SceneEntityCfg = SceneEntityCfg("object"),
) -> torch.Tensor:
    """Terminate episode when end-effector contacts (gets too close to) the object.

    Useful for pre-grasp pose learning where we want the robot to approach
    the object but not touch it.
    """
    ee_frame: FrameTransformer = env.scene[ee_frame_cfg.name]
    obj: RigidObject = env.scene[object_cfg.name]

    ee_pos_w = ee_frame.data.target_pos_w[:, 0, :]  # (num_envs, 3)
    obj_pos_w = obj.data.root_pos_w                 # (num_envs, 3)

    distance = torch.norm(ee_pos_w - obj_pos_w, dim=1)
    return distance < threshold


# =============================================================================
# 4) EE가 medicine cabinet에 너무 가까우면 종료 (너가 만든 cabinet 환경용)
# =============================================================================

def robot_medicine_cabinet_contact(
    env: ManagerBasedRLEnv,
    threshold: float = 0.02,
    ee_frame_cfg: SceneEntityCfg = SceneEntityCfg("ee_frame"),
    cabinet_cfg: SceneEntityCfg = SceneEntityCfg("medicine_cabinet"),
) -> torch.Tensor:
    """Terminate episode when robot end-effector gets too close to medicine cabinet.

    Uses distance-based detection between end-effector and cabinet.
    """
    ee_frame: FrameTransformer = env.scene[ee_frame_cfg.name]
    cabinet: RigidObject = env.scene[cabinet_cfg.name]

    ee_pos_w = ee_frame.data.target_pos_w[:, 0, :]  # (num_envs, 3)
    cabinet_pos_w = cabinet.data.root_pos_w         # (num_envs, 3)

    distance = torch.norm(ee_pos_w - cabinet_pos_w, dim=1)
    contact = distance < threshold
    return contact
