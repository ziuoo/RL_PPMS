# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Common functions that can be used to activate certain terminations for the lift task.

The functions can be passed to the :class:`isaaclab.managers.TerminationTermCfg` object to enable
the termination introduced by the function.
"""

from __future__ import annotations

import torch
from typing import TYPE_CHECKING

from isaaclab.assets import RigidObject
from isaaclab.managers import SceneEntityCfg
from isaaclab.sensors import ContactSensor, FrameTransformer
from isaaclab.utils.math import combine_frame_transforms

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv


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
    # extract the used quantities (to enable type-hinting)
    robot: RigidObject = env.scene[robot_cfg.name]
    object: RigidObject = env.scene[object_cfg.name]
    command = env.command_manager.get_command(command_name)
    # compute the desired position in the world frame
    des_pos_b = command[:, :3]
    des_pos_w, _ = combine_frame_transforms(robot.data.root_pos_w, robot.data.root_quat_w, des_pos_b)
    # distance of the end-effector to the object: (num_envs,)
    distance = torch.norm(des_pos_w - object.data.root_pos_w[:, :3], dim=1)

    # rewarded if the object is lifted above the threshold
    return distance < threshold


def ee_object_contact(
    env: ManagerBasedRLEnv,
    threshold: float = 0.01,
    ee_frame_cfg: SceneEntityCfg = SceneEntityCfg("ee_frame"),
    object_cfg: SceneEntityCfg = SceneEntityCfg("object"),
) -> torch.Tensor:
    """Terminate episode when end-effector contacts (gets too close to) the object.
    
    This is useful for pre-grasp pose learning where we want the robot to approach
    the object but not touch it.
    
    Args:
        env: The environment.
        threshold: Distance threshold for contact detection (in meters). Defaults to 0.01 (1cm).
        ee_frame_cfg: End-effector frame configuration.
        object_cfg: Object configuration.
        
    Returns:
        Boolean tensor indicating which environments should terminate.
    """
    # Get end-effector and object
    ee_frame: FrameTransformer = env.scene[ee_frame_cfg.name]
    object: RigidObject = env.scene[object_cfg.name]
    
    # Get positions
    ee_pos_w = ee_frame.data.target_pos_w[:, 0, :]  # (num_envs, 3)
    object_pos_w = object.data.root_pos_w  # (num_envs, 3)
    
    # Calculate distance
    distance = torch.norm(ee_pos_w - object_pos_w, dim=1)
    
    # Terminate if distance is below threshold (contact detected)
    return distance < threshold


def robot_medicine_cabinet_contact(
    env: ManagerBasedRLEnv,
    threshold: float = 0.02,
    ee_frame_cfg: SceneEntityCfg = SceneEntityCfg("ee_frame"),
    cabinet_cfg: SceneEntityCfg = SceneEntityCfg("medicine_cabinet"),
) -> torch.Tensor:
    """Terminate episode when robot end-effector gets too close to medicine cabinet.
    
    Uses distance-based detection between end-effector and cabinet.
    Simple and reliable approach.
    
    Args:
        env: The environment.
        threshold: Distance threshold for contact detection (in meters). Defaults to 0.02 (2cm).
        ee_frame_cfg: End-effector frame configuration.
        cabinet_cfg: Medicine cabinet configuration.
        
    Returns:
        Boolean tensor indicating which environments should terminate.
    """
    # Get end-effector and medicine cabinet
    ee_frame: FrameTransformer = env.scene[ee_frame_cfg.name]
    cabinet: RigidObject = env.scene[cabinet_cfg.name]
    
    # Get end-effector and cabinet positions
    ee_pos_w = ee_frame.data.target_pos_w[:, 0, :]  # (num_envs, 3)
    cabinet_pos_w = cabinet.data.root_pos_w  # (num_envs, 3)
    
    # Calculate distance between end-effector and cabinet
    distance = torch.norm(ee_pos_w - cabinet_pos_w, dim=1)
    
    # Terminate if distance is below threshold (contact detected)
    contact = distance < threshold
    
    # Debug: Print when contact detected
    # if torch.any(contact):
    #     print(f"[DEBUG] ⚠️ CABINET COLLISION! Envs: {contact.sum().item()}, "
    #           f"Distance: {distance[contact].mean().item():.4f}m, Threshold: {threshold}m")
    
    return contact


# ContactSensor-based approach (currently not working properly)
# def robot_medicine_cabinet_contact(
#     env: ManagerBasedRLEnv,
#     threshold: float = 0.02,
#     contact_sensor_cfg: SceneEntityCfg = SceneEntityCfg("contact_forces"),
# ) -> torch.Tensor:
#     """Terminate episode when gripper makes physical contact with medicine cabinet or table.
#     
#     Uses ContactSensor to detect actual collision forces between gripper and objects.
#     Filters out expected contacts (like when gripper is near table surface during reaching).
#     
#     Args:
#         env: The environment.
#         threshold: Force threshold for contact detection (in Newtons). Defaults to 0.02.
#         contact_sensor_cfg: Contact sensor configuration.
#         
#     Returns:
#         Boolean tensor indicating which environments should terminate.
#     """
#     # Get contact sensor
#     contact_sensor: ContactSensor = env.scene[contact_sensor_cfg.name]
#     
#     # Get contact forces
#     net_contact_forces = contact_sensor.data.net_forces_w
#     contact_force_magnitude = torch.norm(net_contact_forces, dim=-1).sum(dim=-1)  # (num_envs,)
#     
#     # Get cabinet and end-effector positions for debug info
#     cabinet: RigidObject = env.scene["medicine_cabinet"]
#     ee_frame: FrameTransformer = env.scene["ee_frame"]
#     
#     ee_pos_w = ee_frame.data.target_pos_w[:, 0, :]  # (num_envs, 3)
#     cabinet_pos_w = cabinet.data.root_pos_w  # (num_envs, 3)
#     distance_to_cabinet = torch.norm(ee_pos_w - cabinet_pos_w, dim=1)
#     
#     # Terminate if contact force exceeds threshold
#     has_contact = contact_force_magnitude > threshold
#     
#     # Debug: Print when contact detected
#     if torch.any(has_contact):
#         is_near_cabinet = distance_to_cabinet < 0.15
#         cabinet_contacts = has_contact & is_near_cabinet
#         table_contacts = has_contact & ~is_near_cabinet
#         
#         if torch.any(cabinet_contacts):
#             print(f"[DEBUG] ⚠️ Cabinet contact! Envs: {cabinet_contacts.sum().item()}, "
#                   f"Force: {contact_force_magnitude[cabinet_contacts].mean().item():.2f}N, "
#                   f"Terminating: {cabinet_contacts.sum().item()}")
#         if torch.any(table_contacts):
#             print(f"[DEBUG] Table contact! Envs: {table_contacts.sum().item()}, "
#                   f"Force: {contact_force_magnitude[table_contacts].mean().item():.2f}N, "
#                   f"Terminating: {table_contacts.sum().item()}")
#     
#     # Additional debug: print max force even when threshold not exceeded
#     if contact_force_magnitude.max() > 0.01:
#         print(f"[DEBUG] Contact forces: max={contact_force_magnitude.max().item():.4f}N, "
#               f"threshold={threshold}N, terminating={has_contact.sum().item()} envs")
#     
#     return has_contact



# def robot_medicine_cabinet_contact(
#     env: ManagerBasedRLEnv,
#     threshold: float = 0.02,
#     ee_frame_cfg: SceneEntityCfg = SceneEntityCfg("ee_frame"),
#     cabinet_cfg: SceneEntityCfg = SceneEntityCfg("medicine_cabinet"),
# ) -> torch.Tensor:
#     """Terminate episode when robot end-effector gets too close to medicine cabinet.
    
#     Uses distance-based detection between end-effector center and cabinet.
#     Less accurate than ContactSensor as it doesn't detect finger collisions.
    
#     Args:
#         env: The environment.
#         threshold: Distance threshold for contact detection (in meters). Defaults to 0.02 (2cm).
#         ee_frame_cfg: End-effector frame configuration.
#         cabinet_cfg: Medicine cabinet configuration.
        
#     Returns:
#         Boolean tensor indicating which environments should terminate.
#     """
#     # Get end-effector and medicine cabinet
#     ee_frame: FrameTransformer = env.scene[ee_frame_cfg.name]
#     cabinet: RigidObject = env.scene[cabinet_cfg.name]
    
#     # Get end-effector and cabinet positions
#     ee_pos_w = ee_frame.data.target_pos_w[:, 0, :]  # (num_envs, 3)
#     cabinet_pos_w = cabinet.data.root_pos_w  # (num_envs, 3)
    
#     # Calculate distance between end-effector and cabinet
#     distance = torch.norm(ee_pos_w - cabinet_pos_w, dim=1)
    
#     # Terminate if distance is below threshold (contact detected)
#     return distance < threshold


