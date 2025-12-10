# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Configuration for the E0509 robot.

The following configurations are available:

* :obj:`E0509_CFG`: E0509 robot with gripper
* :obj:`FRANKA_ROBOTIQ_GRIPPER_CFG`: Franka robot with Robotiq_2f_85 gripper

"""

import os
import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets.articulation import ArticulationCfg

##
# Configuration
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
                joint_pos={
                    "joint_1": 0.0,
                    "joint_2": 0.0,
                    "joint_3": 1.5708,  # 90 degrees in radians
                    "joint_4": 0.0,
                    "joint_5": 1.5708,  # 90(1.5708) degrees in radians
                    "joint_6": 0.0,
                },
            ),
            actuators={
                "arm": ImplicitActuatorCfg(
                    joint_names_expr=["joint_[1-6]"],
                    effort_limit=200.0,
                    velocity_limit=1.57,  # 낮추면 더 느리게 (기본: 3.14 rad/s = 180°/s, 현재: 1.57 = 90°/s)
                    stiffness=800.0,  # 낮추면 부드럽게 (500~1000 권장)
                    damping=60.0,  # 높이면 진동 감소 (40~80 권장)
                )
            },
        )
"""Configuration of E0509 robot."""