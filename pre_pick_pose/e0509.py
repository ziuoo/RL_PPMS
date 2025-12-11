# Copyright (c) 2022-2025, The Isaac Lab Project Developers
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Configuration for the E0509 robot.

The following configurations are available:

* :obj:`E0509_CFG`: E0509 robot with gripper

"""

import os
import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets.articulation import ArticulationCfg

##
# 공통으로 쓸 기본 자세 정의
#  - Isaac Lab env, PPO policy, ROS 쪽 컨트롤러에서 모두 이 값을 기준으로 Δq 를 정의
##

E0509_DEFAULT_JOINT_DICT = {
    "joint_1": 0.0,
    "joint_2": 0.0,
    "joint_3": 1.5708,  # 90 deg
    "joint_4": 0.0,
    "joint_5": 1.5708,  # 90 deg
    "joint_6": 0.0,
}

# 필요하면 env 쪽에서 리스트로도 쉽게 쓸 수 있게 정의
E0509_DEFAULT_JOINT_LIST = [
    E0509_DEFAULT_JOINT_DICT["joint_1"],
    E0509_DEFAULT_JOINT_DICT["joint_2"],
    E0509_DEFAULT_JOINT_DICT["joint_3"],
    E0509_DEFAULT_JOINT_DICT["joint_4"],
    E0509_DEFAULT_JOINT_DICT["joint_5"],
    E0509_DEFAULT_JOINT_DICT["joint_6"],
]


##
# Articulation configuration
##

E0509_CFG = ArticulationCfg(
    prim_path="{ENV_REGEX_NS}/Robot",
    spawn=sim_utils.UsdFileCfg(
        # Use e0509_model.usda in same directory as this config file
        usd_path=os.path.join(os.path.dirname(__file__), "model", "e0509_model.usda"),
        activate_contact_sensors=True,  # Enable contact sensors for collision detection
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.0),
        rot=(1.0, 0.0, 0.0, 0.0),
        joint_pos=E0509_DEFAULT_JOINT_DICT,
    ),
    actuators={
        "arm": ImplicitActuatorCfg(
            joint_names_expr=["joint_[1-6]"],
            # 최대 토크
            effort_limit=200.0,
            # 최대 속도 (rad/s)
            #  - 너무 크면 Isaac Lab에서 모션이 과도하게 빨라지고 진동/오버슈트가 심해질 수 있음
            #  - 1.57 rad/s ≈ 90 deg/s 정도로 꽤 보수적인 세팅
            velocity_limit=1.57,
            # stiffness / damping
            #  - stiffness 낮추면 더 부드럽고, high-frequency 진동 감소
            #  - damping 높이면 속도 기반 감쇠 증가 → 마찬가지로 진동 감소
            stiffness=800.0,  # (예: 500~1000 정도가 안정적인 편)
            damping=60.0,     # (예: 40~80 정도 구간 튜닝)
        )
    },
)
"""Configuration of E0509 robot."""
