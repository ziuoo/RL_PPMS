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

from isaaclab_tasks.manager_based.manipulation.pre_pick_pose import mdp
from isaaclab_tasks.manager_based.manipulation.pre_pick_pose.lift_env_cfg import LiftEnvCfg

##
# Pre-defined configs
##
from isaaclab.markers.config import FRAME_MARKER_CFG  # isort: skip
from isaaclab_tasks.manager_based.manipulation.pre_pick_pose.e0509 import E0509_CFG  # isort: skip


@configclass
class e0509CubeLiftEnvCfg(LiftEnvCfg):
    def __post_init__(self):
        # post init of parent
        super().__post_init__()

        # ------------------------------------------------------------------
        # 1) 로봇 설정
        # ------------------------------------------------------------------
        # Set e0509 as robot
        self.scene.robot = E0509_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")

        # ------------------------------------------------------------------
        # 2) 액션 설정 (joint delta 스타일: default_joint_pos + action * scale)
        #    -> ROS PPO 컨트롤러의 action_scale(0.03)과 정확히 맞춤
        # ------------------------------------------------------------------
        import math

        safety_margin_deg = 5.0
        safety_margin_rad = math.radians(safety_margin_deg)

        # Isaac Lab의 JointPositionActionCfg는 기본적으로:
        #   target_q = default_joint_pos + action * scale
        # 형태로 동작하므로,
        # 여기서 scale=0.03으로 두면 ROS 쪽 PPOController의
        #   processed = default_joint_pos + raw_action * 0.03
        # 와 정확히 동일한 의미가 됨.
        self.actions.arm_action = mdp.JointPositionActionCfg(
            asset_name="robot",
            joint_names=["joint_[1-6]"],
            scale=0.03,          # ✅ ROS 하드웨어 컨트롤러와 동일한 scale
            use_default_offset=True,
            # ✅ joint limit 에 안전 마진을 둔 clip (rad 단위)
            clip={
                # ≈ ±360° - 5°
                "joint_1": (-6.2832 + safety_margin_rad, 6.2832 - safety_margin_rad),
                "joint_2": (-6.2832 + safety_margin_rad, 6.2832 - safety_margin_rad),
                # joint_3: ≈ ±155° - 5° = ±150°
                "joint_3": (-math.radians(155) + safety_margin_rad, math.radians(155) - safety_margin_rad),
                "joint_4": (-6.2832 + safety_margin_rad, 6.2832 - safety_margin_rad),
                "joint_5": (-6.2832 + safety_margin_rad, 6.2832 - safety_margin_rad),
                "joint_6": (-6.2832 + safety_margin_rad, 6.2832 - safety_margin_rad),
            },
        )

        # self.actions.arm_action = mdp.RelativeJointPositionActionCfg(
        #     asset_name="robot",
        #     joint_names=["joint_[1-6]"],
        #     scale=0.03,          # Δq 크기 (rad)
        #     use_zero_offset=True,
        #     # ⚠️ 여기 clip은 이제 "절대 joint 값"이 아니라 "Δq 범위"에 대한 clip 임
        #     # 예: 한 스텝에 최대 ±5도씩만 움직이게 하고 싶다면
        #     clip={
        #         "joint_1": (-math.radians(5), math.radians(5)),
        #         "joint_2": (-math.radians(5), math.radians(5)),
        #         "joint_3": (-math.radians(5), math.radians(5)),
        #         "joint_4": (-math.radians(5), math.radians(5)),
        #         "joint_5": (-math.radians(5), math.radians(5)),
        #         "joint_6": (-math.radians(5), math.radians(5)),
        #     },
        # )

        # Gripper action removed - not used in this task
        # Gripper joints will maintain their initial positions (0.0)

        # ------------------------------------------------------------------
        # 3) 커맨드 & 메디슨 캐비닛 설정
        # ------------------------------------------------------------------
        # end-effector body 이름 (USD에서 gripper end-effector 링크)
        self.commands.object_pose.body_name = "end"
        # medicine cabinet를 command 기반으로 스폰/이동
        self.commands.object_pose.medicine_cabinet_name = "medicine_cabinet"

        # Medicine cabinet object
        self.scene.medicine_cabinet = RigidObjectCfg(
            prim_path="{ENV_REGEX_NS}/MedicineCabinet",
            spawn=UsdFileCfg(
                usd_path=os.path.join(os.path.dirname(__file__), "model", "medicine_cabinet.usda"),
                rigid_props=RigidBodyPropertiesCfg(
                    # cabinet도 물리에 반응하게 둘지, 완전 kinematic으로 둘지 선택 가능
                    # 여기서는 약간의 물리 반응을 허용 (원래 코드 유지)
                    kinematic_enabled=False,
                    disable_gravity=False,
                ),
            ),
            # 실제 위치는 command에서 덮어쓰기되므로 대략값
            init_state=RigidObjectCfg.InitialStateCfg(pos=(0.5, 0.0, 0.2)),
        )

        # ------------------------------------------------------------------
        # 4) EE frame (FrameTransformer + axis marker)
        # ------------------------------------------------------------------
        marker_cfg = FRAME_MARKER_CFG.copy()
        marker_cfg.markers["frame"].scale = (0.1, 0.1, 0.1)
        marker_cfg.prim_path = "/Visuals/FrameTransformer"

        # base_link를 기준 프레임으로 잡고, gripper end_effector를 target frame으로 둠
        self.scene.ee_frame = FrameTransformerCfg(
            prim_path="{ENV_REGEX_NS}/Robot/e0509/base_link",
            debug_vis=True,   # RVIZ에서 보는 axis처럼 시각화 (Red=X, Green=Y, Blue=Z)
            visualizer_cfg=marker_cfg,
            target_frames=[
                FrameTransformerCfg.FrameCfg(
                    prim_path="{ENV_REGEX_NS}/Robot/e0509/link_6",  # /Robot/e0509/gripper/gripper/end
                    name="link_6",
                    offset=OffsetCfg(
                        pos=(0.0, 0.0, 0.0),
                    ),
                ),
            ],
        )

        # ------------------------------------------------------------------
        # 5) Contact sensor (medicine cabinet 충돌 감지용)
        # ------------------------------------------------------------------
        self.scene.contact_forces = ContactSensorCfg(
            prim_path="{ENV_REGEX_NS}/Robot/e0509/link_6",
            update_period=0.0,  # 매 스텝 업데이트
            history_length=3,
            debug_vis=False,
            filter_prim_paths_expr=[
                "{ENV_REGEX_NS}/MedicineCabinet"
            ],  # medicine_cabinet과의 충돌만 감지
        )
