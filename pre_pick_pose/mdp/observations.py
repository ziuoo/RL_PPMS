# Copyright (c) 2022-2025, The Isaac Lab Project Developers
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

from __future__ import annotations

from typing import TYPE_CHECKING

import torch

from isaaclab.assets import RigidObject
from isaaclab.managers import SceneEntityCfg
from isaaclab.utils.math import subtract_frame_transforms

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv


def object_position_in_robot_root_frame(
    env: ManagerBasedRLEnv,
    robot_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
    object_cfg: SceneEntityCfg = SceneEntityCfg("object"),
) -> torch.Tensor:
    """A안: 물체의 위치만 로봇 root frame 기준으로 관측.

    반환값:
        (num_envs, 3) 텐서
        - 각 행: [x, y, z] (robot root frame 기준)
    """
    # 로봇 / 오브젝트 핸들 가져오기
    robot: RigidObject = env.scene[robot_cfg.name]
    object: RigidObject = env.scene[object_cfg.name]

    # 월드 기준 물체 위치
    object_pos_w = object.data.root_pos_w[:, :3]

    # 월드 → 로봇 root frame으로 변환
    # subtract_frame_transforms(
    #   parent_pos_w, parent_quat_w,
    #   child_pos_w, child_quat_w=None
    # )
    object_pos_b, _ = subtract_frame_transforms(
        robot.data.root_pos_w,
        robot.data.root_quat_w,
        object_pos_w,
    )
    return object_pos_b
