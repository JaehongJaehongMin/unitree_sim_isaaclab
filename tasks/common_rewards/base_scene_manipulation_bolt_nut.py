# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0
from __future__ import annotations

import torch
from typing import TYPE_CHECKING

from isaaclab.assets import RigidObject
from isaaclab.managers import SceneEntityCfg

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv


def compute_reward(
    env: ManagerBasedRLEnv,
    th: float = 0.01,
    std: float = 0.1,
    bolt_cfg: SceneEntityCfg = SceneEntityCfg("bolt"),
    nut_cfg: SceneEntityCfg = SceneEntityCfg("nut"),
) -> torch.Tensor:
    bolt: RigidObject = env.scene[bolt_cfg.name]
    nut: RigidObject = env.scene[nut_cfg.name]

    bolt_pos_w = bolt.data.root_com_pos_w
    nut_pos_w = nut.data.root_com_pos_w

    dist = torch.norm(bolt_pos_w - nut_pos_w, dim=1).squeeze()

    rewards = 1 - torch.tanh(dist / std)

    return rewards
