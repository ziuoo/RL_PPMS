# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import os
import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import ArticulationCfg, RigidObjectCfg
from isaaclab.sensors import ContactSensorCfg, FrameTransformerCfg
from isaaclab.sensors.frame_transformer.frame_transformer_cfg import OffsetCfg
from isaaclab.sim.schemas.schemas_cfg import RigidBodyPropertiesCfg
from isaaclab.sim.spawners.from_files.from_files_cfg import UsdFileCfg
from isaaclab.utils import configclass
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR

from isaaclab_tasks.manager_based.manipulation.e0509 import mdp
from isaaclab_tasks.manager_based.manipulation.e0509.lift_env_cfg import LiftEnvCfg

##
# Pre-defined configs
##
from isaaclab.markers.config import FRAME_MARKER_CFG  # isort: skip
from isaaclab_tasks.manager_based.manipulation.e0509.e0509 import E0509_CFG  # isort: skip


@configclass
class e0509CubeLiftEnvCfg(LiftEnvCfg):
    def __post_init__(self):
        # post init of parent
        super().__post_init__()

        # Set e0509 as robot
        self.scene.robot = E0509_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")

        # Set actions for the specific robot type (e0509)
        # Add safety margin to joint limits for training (2 degrees)
        import math
        safety_margin_deg = 5.0
        safety_margin_rad = math.radians(safety_margin_deg)
        
        self.actions.arm_action = mdp.JointPositionActionCfg(
            asset_name="robot", 
            joint_names=["joint_[1-6]"], 
            scale=0.05,  # 0.1~0.3으로 낮추면 더 느리게 움직임 (기본: 0.5)
            use_default_offset=True,
            # Clip actions to respect joint limits with safety margin
            # Joint 3 has restricted range: ±155° - 2° = ±153°
            # Others: ±360° - 2° = ±358°
            clip={
                "joint_1": (-6.2832 + safety_margin_rad, 6.2832 - safety_margin_rad),
                "joint_2": (-6.2832 + safety_margin_rad, 6.2832 - safety_margin_rad),
                "joint_3": (-2.7053 + safety_margin_rad, 2.7053 - safety_margin_rad),  # ±153°
                "joint_4": (-6.2832 + safety_margin_rad, 6.2832 - safety_margin_rad),
                "joint_5": (-6.2832 + safety_margin_rad, 6.2832 - safety_margin_rad),
                "joint_6": (-6.2832 + safety_margin_rad, 6.2832 - safety_margin_rad),
            }
        )
        # Gripper action removed - not used in this task
        # Gripper joints will maintain their initial positions (0.0)
        # Set the body name for the end effector
        self.commands.object_pose.body_name = "end"  # gripper end effector (z-axis points to fingers)
        self.commands.object_pose.medicine_cabinet_name = "medicine_cabinet"  # Enable medicine cabinet spawning

        # Medicine cabinet object
        self.scene.medicine_cabinet = RigidObjectCfg(
            prim_path="{ENV_REGEX_NS}/MedicineCabinet",
            spawn=UsdFileCfg(
                usd_path=os.path.join(os.path.dirname(__file__), "model", "medicine_cabinet.usda"),
                rigid_props=RigidBodyPropertiesCfg(
                    kinematic_enabled=False,  # Kinematic mode: follows goal_pose without physics
                    disable_gravity=False,
                ),
            ),
            init_state=RigidObjectCfg.InitialStateCfg(pos=(0.5, 0.0, 0.2)),  # Will be overridden by command
        )

        # Listens to the required transforms
        marker_cfg = FRAME_MARKER_CFG.copy()
        marker_cfg.markers["frame"].scale = (0.1, 0.1, 0.1)
        marker_cfg.prim_path = "/Visuals/FrameTransformer"
        self.scene.ee_frame = FrameTransformerCfg(
            prim_path="{ENV_REGEX_NS}/Robot/e0509/base_link",  # USD local frame 기준(z-up)
            debug_vis=True,  # Enable to see EE frame axes (Red=X, Green=Y, Blue=Z)
            visualizer_cfg=marker_cfg,
            target_frames=[
                FrameTransformerCfg.FrameCfg(
                    prim_path="{ENV_REGEX_NS}/Robot/e0509/gripper/gripper/end",
                    name="end_effector",
                    offset=OffsetCfg(
                        pos=(0.0, 0.0, 0.0),
                    ),
                ),
            ],
        )

        # Contact sensor for gripper - detects collision with any object
        self.scene.contact_forces = ContactSensorCfg(
            prim_path="{ENV_REGEX_NS}/Robot/e0509/link_6",
            update_period=0.0,  # Update every step
            history_length=3,
            debug_vis=False,
            filter_prim_paths_expr=["{ENV_REGEX_NS}/MedicineCabinet"],  # Only detect medicine cabinet collisions
        )