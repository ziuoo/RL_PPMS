# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

from __future__ import annotations

import torch
from typing import TYPE_CHECKING

from isaaclab.assets import RigidObject
from isaaclab.managers import SceneEntityCfg
from isaaclab.sensors import ContactSensor, FrameTransformer
from isaaclab.utils.math import combine_frame_transforms, quat_error_magnitude, quat_mul

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv


def position_command_error_tanh(
    env: ManagerBasedRLEnv,
    std: float,
    command_name: str,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("ee_frame"),
) -> torch.Tensor:
    """Reward for reaching the commanded position using tanh kernel.
    
    Args:
        env: The environment.
        std: Standard deviation for tanh kernel.
        command_name: Name of the command (should be pose command).
        asset_cfg: Frame transformer configuration for end-effector.
    
    Returns:
        Reward tensor based on position error.
    """
    # Get end-effector frame
    asset: FrameTransformer = env.scene[asset_cfg.name]
    # Get command (pose command: x, y, z, qw, qx, qy, qz)
    command = env.command_manager.get_command(command_name)
    
    # Get current EE position in base frame
    curr_pos_b = asset.data.target_pos_source[:, 0, :]  # Position in base frame
    
    # Get desired position from command (already in base frame)
    des_pos_b = command[:, :3]
    
    # Calculate position error
    pos_error = torch.norm(des_pos_b - curr_pos_b, dim=1)
    
    # Return tanh-based reward (closer to goal = higher reward)
    return 1.0 - torch.tanh(pos_error / std)


def position_command_error_exp(
    env: ManagerBasedRLEnv,
    alpha: float,
    command_name: str,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("ee_frame"),
) -> torch.Tensor:
    """Reward for reaching the commanded position using exponential kernel.
    
    Uses exponential decay to provide stronger gradients near the goal,
    preventing jittering and encouraging precise positioning.
    
    Reward formula: R = exp(-alpha * ||P_target - P_ee||^2)
    
    Args:
        env: The environment.
        alpha: Exponential decay rate. Higher values = steeper gradient near goal.
               Recommended range: 5.0 ~ 20.0
               - alpha=5.0: Gentle guidance, wider convergence area
               - alpha=10.0: Balanced (recommended starting point)
               - alpha=20.0: Sharp gradient, strict positioning
        command_name: Name of the command (should be pose command).
        asset_cfg: Frame transformer configuration for end-effector.
    
    Returns:
        Reward tensor based on position error (range: 0~1).
        - R ≈ 1.0 when very close to target
        - R ≈ 0.0 when far from target
    """
    # Get end-effector frame
    asset: FrameTransformer = env.scene[asset_cfg.name]
    # Get command (pose command: x, y, z, qw, qx, qy, qz)
    command = env.command_manager.get_command(command_name)
    
    # Get current EE position in base frame
    curr_pos_b = asset.data.target_pos_source[:, 0, :]  # Position in base frame
    
    # Get desired position from command (already in base frame)
    des_pos_b = command[:, :3]
    
    # Calculate squared L2 distance
    pos_error_squared = torch.sum((des_pos_b - curr_pos_b) ** 2, dim=1)
    
    # Return exponential reward: R = exp(-alpha * distance^2)
    # This creates sharp gradient near goal, preventing jittering
    return torch.exp(-alpha * pos_error_squared)


def orientation_command_error(
    env: ManagerBasedRLEnv,
    command_name: str,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("ee_frame"),
) -> torch.Tensor:
    """Reward for reaching the commanded orientation.
    
    Args:
        env: The environment.
        command_name: Name of the command (should be pose command).
        asset_cfg: Frame transformer configuration for end-effector.
    
    Returns:
        Reward tensor based on orientation error.
    """
    # Get end-effector frame
    asset: FrameTransformer = env.scene[asset_cfg.name]
    # Get command (pose command: x, y, z, qw, qx, qy, qz)
    command = env.command_manager.get_command(command_name)
    
    # Get current EE orientation in base frame
    curr_quat_b = asset.data.target_quat_source[:, 0, :]  # Quaternion in base frame (qw, qx, qy, qz)
    
    # Get desired orientation from command (already in base frame)
    des_quat_b = command[:, 3:]
    
    # Calculate orientation error magnitude
    quat_error = quat_error_magnitude(curr_quat_b, des_quat_b)
    
    # Return reward (smaller error = higher reward)
    return 1.0 - torch.tanh(quat_error / 0.25)  # std=0.25 for orientation


def z_axis_alignment_reward(
    env: ManagerBasedRLEnv,
    command_name: str,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("ee_frame"),
) -> torch.Tensor:
    """Reward for aligning the EE's z-axis with the target's z-axis.
    
    This encourages the robot to orient its end-effector correctly by maximizing
    the dot product between the current and target z-axes.
    
    Args:
        env: The environment.
        command_name: Name of the command (should be pose command).
        asset_cfg: Frame transformer configuration for end-effector.
    
    Returns:
        Reward tensor based on z-axis alignment (range: -1 to 1).
        - R = 1.0 when z-axes are perfectly aligned (same direction)
        - R = 0.0 when z-axes are perpendicular
        - R = -1.0 when z-axes point in opposite directions
    """
    # Get end-effector frame
    asset: FrameTransformer = env.scene[asset_cfg.name]
    # Get command (pose command: x, y, z, qw, qx, qy, qz)
    command = env.command_manager.get_command(command_name)
    
    # Get current EE orientation in base frame
    curr_quat_b = asset.data.target_quat_source[:, 0, :]  # (qw, qx, qy, qz)
    
    # Get desired orientation from command
    des_quat_b = command[:, 3:]  # (qw, qx, qy, qz)
    
    # Convert quaternions to rotation matrices to extract z-axis
    # For a quaternion (w, x, y, z), the z-axis of the rotated frame is:
    # z = [2(xz + wy), 2(yz - wx), 1 - 2(x^2 + y^2)]
    
    # Current z-axis
    curr_w, curr_x, curr_y, curr_z = curr_quat_b[:, 0], curr_quat_b[:, 1], curr_quat_b[:, 2], curr_quat_b[:, 3]
    curr_z_axis = torch.stack([
        2 * (curr_x * curr_z + curr_w * curr_y),
        2 * (curr_y * curr_z - curr_w * curr_x),
        1 - 2 * (curr_x**2 + curr_y**2)
    ], dim=1)  # Shape: (num_envs, 3)
    
    # Target z-axis
    des_w, des_x, des_y, des_z = des_quat_b[:, 0], des_quat_b[:, 1], des_quat_b[:, 2], des_quat_b[:, 3]
    des_z_axis = torch.stack([
        2 * (des_x * des_z + des_w * des_y),
        2 * (des_y * des_z - des_w * des_x),
        1 - 2 * (des_x**2 + des_y**2)
    ], dim=1)  # Shape: (num_envs, 3)
    
    # Calculate dot product (cosine of angle between z-axes)
    # Normalize to ensure numerical stability
    curr_z_axis_norm = curr_z_axis / (torch.norm(curr_z_axis, dim=1, keepdim=True) + 1e-8)
    des_z_axis_norm = des_z_axis / (torch.norm(des_z_axis, dim=1, keepdim=True) + 1e-8)
    
    dot_product = torch.sum(curr_z_axis_norm * des_z_axis_norm, dim=1)
    
    # Return dot product as reward
    # dot_product = 1.0 means perfect alignment
    # dot_product = -1.0 means opposite directions
    return dot_product


def multi_axis_alignment_reward(
    env: ManagerBasedRLEnv,
    command_name: str,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("ee_frame"),
) -> torch.Tensor:
    """Reward for aligning all three axes (x, y, z) separately.
    
    This provides individual alignment rewards for each axis, making it easier
    for the robot to learn which joints control which rotations.
    
    Args:
        env: The environment.
        command_name: Name of the command (should be pose command).
        asset_cfg: Frame transformer configuration for end-effector.
    
    Returns:
        Reward tensor based on average alignment of all three axes (range: -1 to 1).
    """
    # Get end-effector frame
    asset: FrameTransformer = env.scene[asset_cfg.name]
    # Get command (pose command: x, y, z, qw, qx, qy, qz)
    command = env.command_manager.get_command(command_name)
    
    # Get current EE orientation in base frame
    curr_quat_b = asset.data.target_quat_source[:, 0, :]  # (qw, qx, qy, qz)
    
    # Get desired orientation from command
    des_quat_b = command[:, 3:]  # (qw, qx, qy, qz)
    
    curr_w, curr_x, curr_y, curr_z = curr_quat_b[:, 0], curr_quat_b[:, 1], curr_quat_b[:, 2], curr_quat_b[:, 3]
    des_w, des_x, des_y, des_z = des_quat_b[:, 0], des_quat_b[:, 1], des_quat_b[:, 2], des_quat_b[:, 3]
    
    # Extract x-axis: [1 - 2(y^2 + z^2), 2(xy + wz), 2(xz - wy)]
    curr_x_axis = torch.stack([
        1 - 2 * (curr_y**2 + curr_z**2),
        2 * (curr_x * curr_y + curr_w * curr_z),
        2 * (curr_x * curr_z - curr_w * curr_y)
    ], dim=1)
    des_x_axis = torch.stack([
        1 - 2 * (des_y**2 + des_z**2),
        2 * (des_x * des_y + des_w * des_z),
        2 * (des_x * des_z - des_w * des_y)
    ], dim=1)
    
    # Extract y-axis: [2(xy - wz), 1 - 2(x^2 + z^2), 2(yz + wx)]
    curr_y_axis = torch.stack([
        2 * (curr_x * curr_y - curr_w * curr_z),
        1 - 2 * (curr_x**2 + curr_z**2),
        2 * (curr_y * curr_z + curr_w * curr_x)
    ], dim=1)
    des_y_axis = torch.stack([
        2 * (des_x * des_y - des_w * des_z),
        1 - 2 * (des_x**2 + des_z**2),
        2 * (des_y * des_z + des_w * des_x)
    ], dim=1)
    
    # Extract z-axis: [2(xz + wy), 2(yz - wx), 1 - 2(x^2 + y^2)]
    curr_z_axis = torch.stack([
        2 * (curr_x * curr_z + curr_w * curr_y),
        2 * (curr_y * curr_z - curr_w * curr_x),
        1 - 2 * (curr_x**2 + curr_y**2)
    ], dim=1)
    des_z_axis = torch.stack([
        2 * (des_x * des_z + des_w * des_y),
        2 * (des_y * des_z - des_w * des_x),
        1 - 2 * (des_x**2 + des_y**2)
    ], dim=1)
    
    # Normalize all axes
    curr_x_axis_norm = curr_x_axis / (torch.norm(curr_x_axis, dim=1, keepdim=True) + 1e-8)
    des_x_axis_norm = des_x_axis / (torch.norm(des_x_axis, dim=1, keepdim=True) + 1e-8)
    
    curr_y_axis_norm = curr_y_axis / (torch.norm(curr_y_axis, dim=1, keepdim=True) + 1e-8)
    des_y_axis_norm = des_y_axis / (torch.norm(des_y_axis, dim=1, keepdim=True) + 1e-8)
    
    curr_z_axis_norm = curr_z_axis / (torch.norm(curr_z_axis, dim=1, keepdim=True) + 1e-8)
    des_z_axis_norm = des_z_axis / (torch.norm(des_z_axis, dim=1, keepdim=True) + 1e-8)
    
    # Calculate dot products for each axis
    x_alignment = torch.sum(curr_x_axis_norm * des_x_axis_norm, dim=1)
    y_alignment = torch.sum(curr_y_axis_norm * des_y_axis_norm, dim=1)
    z_alignment = torch.sum(curr_z_axis_norm * des_z_axis_norm, dim=1)
    
    # Average alignment of all three axes
    # This encourages the robot to align all axes, not just one
    return (x_alignment + y_alignment + z_alignment) / 3.0


# def medicine_cabinet_contact_penalty(
#     env: ManagerBasedRLEnv,
#     threshold: float = 1.0,
#     contact_sensor_cfg: SceneEntityCfg = SceneEntityCfg("contact_forces"),
# ) -> torch.Tensor:
#     """Penalize gripper for making physical contact with medicine cabinet.
    
#     Uses ContactSensor to detect actual collision forces. Returns penalty proportional to contact force.
    
#     Args:
#         env: The environment.
#         threshold: Force threshold for penalty (in Newtons). Defaults to 1.0.
#         contact_sensor_cfg: Contact sensor configuration.
        
#     Returns:
#         Penalty value (negative reward) when contact force detected.
#     """
#     # Get contact sensor
#     contact_sensor: ContactSensor = env.scene[contact_sensor_cfg.name]
    
#     # Get net contact forces - shape: (num_envs, num_bodies, 3)
#     net_contact_forces = contact_sensor.data.net_forces_w
    
#     # Calculate magnitude of contact forces for each body
#     # Sum across all bodies in the gripper
#     contact_force_magnitude = torch.norm(net_contact_forces, dim=-1).sum(dim=-1)  # (num_envs,)
    
#     # Return penalty proportional to contact force (normalized by threshold)
#     # Force > threshold gives penalty of -1.0 or more
#     penalty = -contact_force_magnitude / threshold
    
#     return penalty



def medicine_cabinet_distance_penalty(
    env: ManagerBasedRLEnv,
    threshold: float = 0.05,
    ee_frame_cfg: SceneEntityCfg = SceneEntityCfg("ee_frame"),
    cabinet_cfg: SceneEntityCfg = SceneEntityCfg("medicine_cabinet"),
) -> torch.Tensor:
    """Penalize robot end-effector for getting too close to medicine cabinet.
    
    Uses distance-based detection between end-effector center and cabinet.
    Less accurate than ContactSensor as it doesn't detect finger proximity.
    
    Args:
        env: The environment.
        threshold: Distance threshold for contact detection (in meters). Defaults to 0.05 (5cm).
        ee_frame_cfg: End-effector frame configuration.
        cabinet_cfg: Medicine cabinet configuration.
        
    Returns:
        Penalty value (negative reward) when end-effector is too close to cabinet.
    """
    # Get end-effector and medicine cabinet
    ee_frame: FrameTransformer = env.scene[ee_frame_cfg.name]
    cabinet: RigidObject = env.scene[cabinet_cfg.name]
    
    # Get positions
    ee_pos_w = ee_frame.data.target_pos_w[:, 0, :]  # (num_envs, 3)
    cabinet_pos_w = cabinet.data.root_pos_w  # (num_envs, 3)
    
    # Calculate distance
    distance = torch.norm(ee_pos_w - cabinet_pos_w, dim=1)
    
    # Return penalty if too close, otherwise 0
    return torch.where(distance < threshold, -1.0, 0.0)


