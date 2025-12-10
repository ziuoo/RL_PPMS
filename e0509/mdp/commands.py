# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Custom command generators for E0509 manipulation tasks."""

from __future__ import annotations

import torch
from collections.abc import Sequence
from typing import TYPE_CHECKING

from isaaclab.assets import Articulation, RigidObject
from isaaclab.managers import CommandTerm
from isaaclab.markers import VisualizationMarkers
from isaaclab.utils.math import combine_frame_transforms, compute_pose_error, quat_unique

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedEnv

    from .commands_cfg import DiscretePoseCommandCfg, HybridPoseCommandCfg


class DiscretePoseCommand(CommandTerm):
    """Command generator that randomly selects from a predefined list of poses.

    Unlike UniformPoseCommand which samples from continuous ranges, this command generator
    selects one of the predefined discrete poses at each resampling interval.
    
    The poses are defined in the base frame of the robot.
    """

    cfg: DiscretePoseCommandCfg
    """Configuration for the command generator."""

    def __init__(self, cfg: DiscretePoseCommandCfg, env: ManagerBasedEnv):
        """Initialize the command generator class.

        Args:
            cfg: The configuration parameters for the command generator.
            env: The environment object.
        """
        # initialize the base class
        super().__init__(cfg, env)

        # extract the robot and body index for which the command is generated
        self.robot: Articulation = env.scene[cfg.asset_name]
        self.body_idx = self.robot.find_bodies(cfg.body_name)[0][0]

        # convert predefined poses to tensor
        # poses should be list of (x, y, z, qw, qx, qy, qz)
        self.predefined_poses = torch.tensor(cfg.predefined_poses, device=self.device, dtype=torch.float32)
        self.num_poses = len(self.predefined_poses)

        # create buffers
        # -- commands: (x, y, z, qw, qx, qy, qz) in root frame
        self.pose_command_b = torch.zeros(self.num_envs, 7, device=self.device)
        self.pose_command_b[:, 3] = 1.0
        self.pose_command_w = torch.zeros_like(self.pose_command_b)
        # -- metrics
        self.metrics["position_error"] = torch.zeros(self.num_envs, device=self.device)
        self.metrics["orientation_error"] = torch.zeros(self.num_envs, device=self.device)

    def __str__(self) -> str:
        msg = "DiscretePoseCommand:\n"
        msg += f"\tCommand dimension: {tuple(self.command.shape[1:])}\n"
        msg += f"\tNumber of predefined poses: {self.num_poses}\n"
        msg += f"\tResampling time range: {self.cfg.resampling_time_range}\n"
        return msg

    """
    Properties
    """

    @property
    def command(self) -> torch.Tensor:
        """The desired pose command. Shape is (num_envs, 7).

        The first three elements correspond to the position, followed by the quaternion orientation in (w, x, y, z).
        """
        return self.pose_command_b

    """
    Implementation specific functions.
    """

    def _update_metrics(self):
        # transform command from base frame to simulation world frame
        self.pose_command_w[:, :3], self.pose_command_w[:, 3:] = combine_frame_transforms(
            self.robot.data.root_pos_w,
            self.robot.data.root_quat_w,
            self.pose_command_b[:, :3],
            self.pose_command_b[:, 3:],
        )
        # compute the error
        pos_error, rot_error = compute_pose_error(
            self.pose_command_w[:, :3],
            self.pose_command_w[:, 3:],
            self.robot.data.body_pos_w[:, self.body_idx],
            self.robot.data.body_quat_w[:, self.body_idx],
        )
        self.metrics["position_error"] = torch.norm(pos_error, dim=-1)
        self.metrics["orientation_error"] = torch.norm(rot_error, dim=-1)

    def _resample_command(self, env_ids: Sequence[int]):
        # randomly select one of the predefined poses for each environment
        random_indices = torch.randint(0, self.num_poses, (len(env_ids),), device=self.device)
        
        # assign the selected poses to the environments
        self.pose_command_b[env_ids] = self.predefined_poses[random_indices]
        
        # make sure quaternions have real part as positive if configured
        if self.cfg.make_quat_unique:
            self.pose_command_b[env_ids, 3:] = quat_unique(self.pose_command_b[env_ids, 3:])

    def _update_command(self):
        pass

    def _update_medicine_cabinet_position(self, env_ids: Sequence[int]):
        """Update medicine cabinet position to be 10cm away from goal pose in -y direction (goal frame).
        Medicine cabinet is always kept upright (identity quaternion).
        
        Args:
            env_ids: Environment indices to update.
        """
        # transform command from base frame to world frame
        goal_pos_w, goal_quat_w = combine_frame_transforms(
            self.robot.data.root_pos_w[env_ids],
            self.robot.data.root_quat_w[env_ids],
            self.pose_command_b[env_ids, :3],
            self.pose_command_b[env_ids, 3:],
        )
        
        # offset by 10cm in goal's local -y direction
        # Create offset vector in goal's local frame: [0, -0.1, 0]
        offset_local = torch.zeros(len(env_ids), 3, device=self.device)
        offset_local[:, 1] = -0.1  # -10cm in local y
        
        # Transform offset from goal's local frame to world frame
        cabinet_pos_w, _ = combine_frame_transforms(
            goal_pos_w,
            goal_quat_w,
            offset_local,
            torch.zeros(len(env_ids), 4, device=self.device),  # dummy quaternion
        )
        
        # keep medicine cabinet upright (identity quaternion: w=1, x=0, y=0, z=0)
        cabinet_quat_w = torch.zeros(len(env_ids), 4, device=self.device)
        cabinet_quat_w[:, 0] = 1.0  # w=1 for identity quaternion
        
        # combine position and orientation into single pose tensor (pos + quat)
        cabinet_pose_w = torch.cat([cabinet_pos_w, cabinet_quat_w], dim=-1)
        
        # set medicine cabinet position and orientation (upright)
        self.medicine_cabinet.write_root_pose_to_sim(cabinet_pose_w, env_ids)

    def _set_debug_vis_impl(self, debug_vis: bool):
        # create markers if necessary for the first time
        if debug_vis:
            if not hasattr(self, "goal_pose_visualizer"):
                # -- goal pose only
                self.goal_pose_visualizer = VisualizationMarkers(self.cfg.goal_pose_visualizer_cfg)
            # set their visibility to true
            self.goal_pose_visualizer.set_visibility(True)
        else:
            if hasattr(self, "goal_pose_visualizer"):
                self.goal_pose_visualizer.set_visibility(False)

    def _debug_vis_callback(self, event):
        # check if robot is initialized
        # note: this is needed in-case the robot is de-initialized. we can't access the data
        if not self.robot.is_initialized:
            return
        # update the markers
        # -- goal pose only (current pose is shown by ee_frame debug_vis)
        self.goal_pose_visualizer.visualize(self.pose_command_w[:, :3], self.pose_command_w[:, 3:])


class HybridPoseCommand(CommandTerm):
    """Command generator that samples position from ranges and selects orientation from predefined list.

    This combines the flexibility of continuous position sampling with discrete orientation targets.
    Useful when you want the robot to reach various positions but with specific, controlled orientations.
    
    The poses are defined in the base frame of the robot.
    """

    cfg: HybridPoseCommandCfg
    """Configuration for the command generator."""

    def __init__(self, cfg: HybridPoseCommandCfg, env: ManagerBasedEnv):
        """Initialize the command generator class.

        Args:
            cfg: The configuration parameters for the command generator.
            env: The environment object.
        """
        # initialize the base class
        super().__init__(cfg, env)

        # extract the robot and body index for which the command is generated
        self.robot: Articulation = env.scene[cfg.asset_name]
        self.body_idx = self.robot.find_bodies(cfg.body_name)[0][0]

        # extract medicine cabinet if configured
        if cfg.medicine_cabinet_name is not None:
            self.medicine_cabinet: RigidObject = env.scene[cfg.medicine_cabinet_name]
        else:
            self.medicine_cabinet = None

        # convert predefined orientations to tensor
        # orientations should be list of (qw, qx, qy, qz)
        self.predefined_orientations = torch.tensor(cfg.predefined_orientations, device=self.device, dtype=torch.float32)
        self.num_orientations = len(self.predefined_orientations)

        # create buffers
        # -- commands: (x, y, z, qw, qx, qy, qz) in root frame
        self.pose_command_b = torch.zeros(self.num_envs, 7, device=self.device)
        self.pose_command_b[:, 3] = 1.0
        self.pose_command_w = torch.zeros_like(self.pose_command_b)
        # -- metrics
        self.metrics["position_error"] = torch.zeros(self.num_envs, device=self.device)
        self.metrics["orientation_error"] = torch.zeros(self.num_envs, device=self.device)

    def __str__(self) -> str:
        msg = "HybridPoseCommand:\n"
        msg += f"\tCommand dimension: {tuple(self.command.shape[1:])}\n"
        msg += f"\tNumber of predefined orientations: {self.num_orientations}\n"
        msg += f"\tPosition ranges: x={self.cfg.ranges.pos_x}, y={self.cfg.ranges.pos_y}, z={self.cfg.ranges.pos_z}\n"
        msg += f"\tResampling time range: {self.cfg.resampling_time_range}\n"
        return msg

    """
    Properties
    """

    @property
    def command(self) -> torch.Tensor:
        """The desired pose command. Shape is (num_envs, 7).

        The first three elements correspond to the position, followed by the quaternion orientation in (w, x, y, z).
        """
        return self.pose_command_b

    """
    Implementation specific functions.
    """

    def _update_metrics(self):
        # transform command from base frame to simulation world frame
        self.pose_command_w[:, :3], self.pose_command_w[:, 3:] = combine_frame_transforms(
            self.robot.data.root_pos_w,
            self.robot.data.root_quat_w,
            self.pose_command_b[:, :3],
            self.pose_command_b[:, 3:],
        )
        # compute the error
        pos_error, rot_error = compute_pose_error(
            self.pose_command_w[:, :3],
            self.pose_command_w[:, 3:],
            self.robot.data.body_pos_w[:, self.body_idx],
            self.robot.data.body_quat_w[:, self.body_idx],
        )
        self.metrics["position_error"] = torch.norm(pos_error, dim=-1)
        self.metrics["orientation_error"] = torch.norm(rot_error, dim=-1)

    def _resample_command(self, env_ids: Sequence[int]):
        # sample new pose targets
        # -- position: sample uniformly from ranges
        r = torch.empty(len(env_ids), device=self.device)
        self.pose_command_b[env_ids, 0] = r.uniform_(*self.cfg.ranges.pos_x)
        self.pose_command_b[env_ids, 1] = r.uniform_(*self.cfg.ranges.pos_y)
        self.pose_command_b[env_ids, 2] = r.uniform_(*self.cfg.ranges.pos_z)
        
        # -- orientation: randomly select from predefined orientations
        random_indices = torch.randint(0, self.num_orientations, (len(env_ids),), device=self.device)
        self.pose_command_b[env_ids, 3:] = self.predefined_orientations[random_indices]
        
        # make sure quaternions have real part as positive if configured
        if self.cfg.make_quat_unique:
            self.pose_command_b[env_ids, 3:] = quat_unique(self.pose_command_b[env_ids, 3:])
        
        # update medicine cabinet position if configured
        if self.medicine_cabinet is not None:
            self._update_medicine_cabinet_position(env_ids)

    def _update_command(self):
        pass

    def _update_medicine_cabinet_position(self, env_ids: Sequence[int]):
        """Update medicine cabinet position to be 10cm away from goal pose in -y direction (goal frame).
        Medicine cabinet is always kept upright (identity quaternion).
        
        Args:
            env_ids: Environment indices to update.
        """
        # transform command from base frame to world frame
        goal_pos_w, goal_quat_w = combine_frame_transforms(
            self.robot.data.root_pos_w[env_ids],
            self.robot.data.root_quat_w[env_ids],
            self.pose_command_b[env_ids, :3],
            self.pose_command_b[env_ids, 3:],
        )
        
        # offset by 10cm in goal's local -y direction
        # Create offset vector in goal's local frame: [0, -0.1, 0]
        offset_local = torch.zeros(len(env_ids), 3, device=self.device)
        offset_local[:, 1] = -0.1  # -10cm in local y
        
        # Transform offset from goal's local frame to world frame
        cabinet_pos_w, _ = combine_frame_transforms(
            goal_pos_w,
            goal_quat_w,
            offset_local,
            torch.zeros(len(env_ids), 4, device=self.device),  # dummy quaternion
        )
        
        # keep medicine cabinet upright (identity quaternion: w=1, x=0, y=0, z=0)
        cabinet_quat_w = torch.zeros(len(env_ids), 4, device=self.device)
        cabinet_quat_w[:, 0] = 1.0  # w=1 for identity quaternion
        
        # combine position and orientation into single pose tensor (pos + quat)
        cabinet_pose_w = torch.cat([cabinet_pos_w, cabinet_quat_w], dim=-1)
        
        # set medicine cabinet position and orientation (upright)
        self.medicine_cabinet.write_root_pose_to_sim(cabinet_pose_w, env_ids)

    def _set_debug_vis_impl(self, debug_vis: bool):
        # create markers if necessary for the first time
        if debug_vis:
            if not hasattr(self, "goal_pose_visualizer"):
                # -- goal pose only
                self.goal_pose_visualizer = VisualizationMarkers(self.cfg.goal_pose_visualizer_cfg)
            # set their visibility to true
            self.goal_pose_visualizer.set_visibility(True)
        else:
            if hasattr(self, "goal_pose_visualizer"):
                self.goal_pose_visualizer.set_visibility(False)

    def _debug_vis_callback(self, event):
        # check if robot is initialized
        # note: this is needed in-case the robot is de-initialized. we can't access the data
        if not self.robot.is_initialized:
            return
        # update the markers
        # -- goal pose only (current pose is shown by ee_frame debug_vis)
        self.goal_pose_visualizer.visualize(self.pose_command_w[:, :3], self.pose_command_w[:, 3:])

