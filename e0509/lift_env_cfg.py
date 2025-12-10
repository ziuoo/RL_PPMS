# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

from dataclasses import MISSING
from isaaclab.utils.math import quat_from_euler_xyz
import torch

import isaaclab.sim as sim_utils
from isaaclab.assets import ArticulationCfg, AssetBaseCfg, DeformableObjectCfg, RigidObjectCfg
from isaaclab.envs import ManagerBasedRLEnvCfg
from isaaclab.managers import CurriculumTermCfg as CurrTerm
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import RewardTermCfg as RewTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.managers import TerminationTermCfg as DoneTerm
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sensors.frame_transformer.frame_transformer_cfg import FrameTransformerCfg
from isaaclab.sim.spawners.from_files.from_files_cfg import GroundPlaneCfg, UsdFileCfg
from isaaclab.utils import configclass
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR
from isaaclab.markers import VisualizationMarkersCfg

from . import mdp

##
# Scene definition
##


@configclass
class ObjectTableSceneCfg(InteractiveSceneCfg):
    """Configuration for the lift scene with a robot and a object.
    This is the abstract base implementation, the exact scene is defined in the derived classes
    which need to set the target object, robot and end-effector frames
    """

    # robots: will be populated by agent env cfg
    robot: ArticulationCfg = MISSING
    # end-effector sensor: will be populated by agent env cfg
    ee_frame: FrameTransformerCfg = MISSING
    # medicine cabinet: will be populated by agent env cfg
    medicine_cabinet: RigidObjectCfg = MISSING

    # Table
    table = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/Table",
        init_state=AssetBaseCfg.InitialStateCfg(pos=[0.5, 0, 0], rot=[0.707, 0, 0, 0.707]),
        spawn=UsdFileCfg(usd_path=f"{ISAAC_NUCLEUS_DIR}/Props/Mounts/SeattleLabTable/table_instanceable.usd"),
    )

    # plane
    plane = AssetBaseCfg(
        prim_path="/World/GroundPlane",
        init_state=AssetBaseCfg.InitialStateCfg(pos=[0, 0, -1.05]),
        spawn=GroundPlaneCfg(),
    )

    # lights
    light = AssetBaseCfg(
        prim_path="/World/light",
        spawn=sim_utils.DomeLightCfg(color=(0.75, 0.75, 0.75), intensity=3000.0),
    )


##
# MDP settings
##


@configclass
class CommandsCfg:
    """Command terms for the MDP."""
    # Option 3: Use HybridPoseCommandCfg for position range + predefined orientations
    object_pose = mdp.HybridPoseCommandCfg(
        asset_name="robot",
        body_name=MISSING,  # will be set by agent env cfg
        resampling_time_range=(5.0, 5.0),
        debug_vis=True,
        ranges=mdp.HybridPoseCommandCfg.Ranges(
            pos_x=(0.5, 0.6),
            pos_y=(-0.15, 0.15),
            pos_z=(0.05, 0.15),
        ),
        predefined_orientations=[
            ## Top-down grasp
            quat_from_euler_xyz(
                roll=torch.tensor([1.5708]),
                pitch=torch.tensor([0.0]),
                yaw=torch.tensor([1.5708])
            ).squeeze().tolist(),
            # ## Right-side grasp
            quat_from_euler_xyz(
                roll=torch.tensor([0.0]),
                pitch=torch.tensor([0.0]),
                yaw=torch.tensor([-3.14]) 
            ).squeeze().tolist(),
            ## Left-side grasp
            # quat_from_euler_xyz(
            #     roll=torch.tensor([0.0]),
            #     pitch=torch.tensor([0.0]),
            #     yaw=torch.tensor([0.0]) 
            # ).squeeze().tolist(),
        ],
    )


@configclass
class ActionsCfg:
    """Action specifications for the MDP."""

    # will be set by agent env cfg
    arm_action: mdp.JointPositionActionCfg | mdp.DifferentialInverseKinematicsActionCfg = MISSING
    # gripper_action removed - not used in this task


@configclass
class ObservationsCfg:
    """Observation specifications for the MDP."""

    @configclass
    class PolicyCfg(ObsGroup):
        """Observations for policy group."""

        # Robot state - arm only (no gripper)
        joint_pos = ObsTerm(func=mdp.joint_pos_rel, params={"asset_cfg": SceneEntityCfg("robot", joint_names=["joint_[1-6]"])})
        joint_vel = ObsTerm(func=mdp.joint_vel_rel, params={"asset_cfg": SceneEntityCfg("robot", joint_names=["joint_[1-6]"])})
        
        # Target pose command (position: 3 values + orientation: 4 values = 7 total)
        # This includes both position (x,y,z) and orientation (qw,qx,qy,qz)
        target_pose = ObsTerm(func=mdp.generated_commands, params={"command_name": "object_pose"})
        
        # Previous action - arm only
        actions = ObsTerm(func=mdp.last_action)

        def __post_init__(self):
            self.enable_corruption = True
            self.concatenate_terms = True

    # observation groups
    policy: PolicyCfg = PolicyCfg()


@configclass
class EventCfg:
    """Configuration for events."""

    reset_all = EventTerm(func=mdp.reset_scene_to_default, mode="reset")


@configclass
class RewardsCfg:
    """Reward terms for the MDP."""

    # reaching goal pose - using exponential reward to prevent jittering
    reaching_goal_position = RewTerm(
        func=mdp.position_command_error_exp,
        params={"alpha": 10.0, "asset_cfg": SceneEntityCfg("ee_frame"), "command_name": "object_pose"},
        weight=15.0,
    )

    # Multi-axis alignment - encourages all axes (x, y, z) to align with target
    # This provides clearer gradient than single quaternion error
    multi_axis_alignment = RewTerm(
        func=mdp.multi_axis_alignment_reward,
        params={"asset_cfg": SceneEntityCfg("ee_frame"), "command_name": "object_pose"},
        weight=13.0,
    )

    # action penalty
    action_rate = RewTerm(func=mdp.action_rate_l2, weight=-1e-4)

    joint_vel = RewTerm(
        func=mdp.joint_vel_l2,
        weight=-1e-4,
        params={"asset_cfg": SceneEntityCfg("robot")},
    )

    # Medicine cabinet contact penalty - penalize robot for hitting the cabinet
    # medicine_cabinet_contact = RewTerm(
    #     func=mdp.medicine_cabinet_contact_penalty,
    #     params={
    #         "threshold": 1.0,  # Force threshold in Newtons
    #         "contact_sensor_cfg": SceneEntityCfg("contact_forces"),
    #     },
    #     weight=-10.0,  # Large penalty for contact
    # )

    medicine_cabinet_contact = RewTerm(
    func=mdp.medicine_cabinet_distance_penalty,
    params={"threshold": 0.08},
    weight=-20.0,  # 큰 패널티
)


@configclass
class TerminationsCfg:
    """Termination terms for the MDP."""

    time_out = DoneTerm(func=mdp.time_out, time_out=True)

    # Terminate episode when robot contacts medicine cabinet
    medicine_cabinet_contact = DoneTerm(
        func=mdp.robot_medicine_cabinet_contact,
        params={
            "threshold": 0.05,  # 5cm distance threshold
            "ee_frame_cfg": SceneEntityCfg("ee_frame"),
            "cabinet_cfg": SceneEntityCfg("medicine_cabinet"),
        },
    )


@configclass
class CurriculumCfg:
    """Curriculum terms for the MDP."""

    action_rate = CurrTerm(
        func=mdp.modify_reward_weight, params={"term_name": "action_rate", "weight": -1e-1, "num_steps": 10000}
    )

    joint_vel = CurrTerm(
        func=mdp.modify_reward_weight, params={"term_name": "joint_vel", "weight": -1e-1, "num_steps": 10000}
    )


##
# Environment configuration
##


@configclass
class LiftEnvCfg(ManagerBasedRLEnvCfg):
    """Configuration for the lifting environment."""

    # Scene settings
    scene: ObjectTableSceneCfg = ObjectTableSceneCfg(num_envs=4096, env_spacing=2.5)
    # Basic settings
    observations: ObservationsCfg = ObservationsCfg()
    actions: ActionsCfg = ActionsCfg()
    commands: CommandsCfg = CommandsCfg()
    # MDP settings
    rewards: RewardsCfg = RewardsCfg()
    terminations: TerminationsCfg = TerminationsCfg()
    events: EventCfg = EventCfg()
    curriculum: CurriculumCfg = CurriculumCfg()

    def __post_init__(self):
        """Post initialization."""
        # general settings
        self.decimation = 4
        self.episode_length_s = 10.0
        # simulation settings
        self.sim.dt = 0.01  # 100Hz
        self.sim.render_interval = self.decimation

        self.sim.physx.bounce_threshold_velocity = 0.2
        # self.sim.physx.bounce_threshold_velocity = 0.01
        self.sim.physx.gpu_found_lost_aggregate_pairs_capacity = 1024 * 1024 * 4
        self.sim.physx.gpu_total_aggregate_pairs_capacity = 16 * 1024
        self.sim.physx.friction_correlation_distance = 0.00625
