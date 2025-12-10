# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Configuration for custom command generators."""

from __future__ import annotations

from dataclasses import MISSING

import isaaclab.sim as sim_utils
from isaaclab.managers import CommandTermCfg
from isaaclab.markers import VisualizationMarkersCfg
from isaaclab.markers.config import FRAME_MARKER_CFG
from isaaclab.utils import configclass
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR

from . import commands


@configclass
class DiscretePoseCommandCfg(CommandTermCfg):
    """Configuration for discrete pose command generator.
    
    This command generator randomly selects from a list of predefined poses
    instead of sampling from continuous ranges.
    """

    class_type: type = commands.DiscretePoseCommand

    asset_name: str = MISSING
    """Name of the asset in the environment for which the commands are generated."""

    body_name: str = MISSING
    """Name of the body in the asset for which the commands are generated."""

    predefined_poses: list[tuple[float, float, float, float, float, float, float]] = MISSING
    """List of predefined poses in robot base frame.
    
    Each pose is a tuple of 7 values: (x, y, z, qw, qx, qy, qz)
    where (x, y, z) is the position and (qw, qx, qy, qz) is the quaternion orientation.
    
    Example:
        predefined_poses = [
            (0.5, 0.0, 0.1, 1.0, 0.0, 0.0, 0.0),  # pose 1
            (0.4, 0.2, 0.15, 0.707, 0.0, 0.707, 0.0),  # pose 2
            # ... more poses
        ]
    """

    make_quat_unique: bool = True
    """Whether to make the quaternion unique by ensuring the real part is positive. Defaults to True."""

    goal_pose_visualizer_cfg: VisualizationMarkersCfg = VisualizationMarkersCfg(
        prim_path="/Visuals/Command/goal_pose",
        markers={
            "frame": sim_utils.UsdFileCfg(
                usd_path=f"{ISAAC_NUCLEUS_DIR}/Props/UIElements/frame_prim.usd",
                scale=(0.3, 0.3, 0.3),  # 마커 크기
            ),
        }
    )
    """The configuration for the goal pose visualization marker."""

    current_pose_visualizer_cfg: VisualizationMarkersCfg = VisualizationMarkersCfg(
        prim_path="/Visuals/Command/body_pose",
        markers={
            "frame": sim_utils.UsdFileCfg(
                usd_path=f"{ISAAC_NUCLEUS_DIR}/Props/UIElements/frame_prim.usd",
                scale=(0.3, 0.3, 0.3),
            ),
        }
    )
    """The configuration for the current pose visualization marker."""


@configclass
class HybridPoseCommandCfg(CommandTermCfg):
    """Configuration for hybrid pose command generator.
    
    This command generator samples position from continuous ranges and
    selects orientation from a list of predefined orientations.
    """

    class_type: type = commands.HybridPoseCommand

    asset_name: str = MISSING
    """Name of the asset in the environment for which the commands are generated."""

    body_name: str = MISSING
    """Name of the body in the asset for which the commands are generated."""

    medicine_cabinet_name: str | None = None
    """Name of the medicine cabinet object to position relative to goal pose. If None, no object is spawned."""

    @configclass
    class Ranges:
        """Ranges for uniform sampling of position commands."""

        pos_x: tuple[float, float] = MISSING
        """Range for x position (in m)."""
        pos_y: tuple[float, float] = MISSING
        """Range for y position (in m)."""
        pos_z: tuple[float, float] = MISSING
        """Range for z position (in m)."""

    ranges: Ranges = MISSING
    """Ranges for position sampling."""

    predefined_orientations: list[tuple[float, float, float, float]] = MISSING
    """List of predefined orientations as quaternions.
    
    Each orientation is a tuple of 4 values: (qw, qx, qy, qz)
    
    Example:
        predefined_orientations = [
            (1.0, 0.0, 0.0, 0.0),  # no rotation
            (0.707, 0.0, 0.707, 0.0),  # 90 deg pitch
            # ... more orientations
        ]
    """

    make_quat_unique: bool = True
    """Whether to make the quaternion unique by ensuring the real part is positive. Defaults to True."""

    goal_pose_visualizer_cfg: VisualizationMarkersCfg = VisualizationMarkersCfg(
        prim_path="/Visuals/Command/goal_pose",
        markers={
            "frame": sim_utils.UsdFileCfg(
                usd_path=f"{ISAAC_NUCLEUS_DIR}/Props/UIElements/frame_prim.usd",
                scale=(0.1, 0.1, 0.1),  # 마커 크기
            ),
        }
    )
    """The configuration for the goal pose visualization marker."""

    current_pose_visualizer_cfg: VisualizationMarkersCfg = VisualizationMarkersCfg(
        prim_path="/Visuals/Command/body_pose",
        markers={
            "frame": sim_utils.UsdFileCfg(
                usd_path=f"{ISAAC_NUCLEUS_DIR}/Props/UIElements/frame_prim.usd",
                scale=(0.1, 0.1, 0.1),
            ),
        }
    )
    """The configuration for the current pose visualization marker."""
