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
    bolt_cfg: SceneEntityCfg = SceneEntityCfg("bottle"),
    nut_cfg: SceneEntityCfg = SceneEntityCfg("cap"),
) -> torch.Tensor:
    bottle: RigidObject = env.scene[bolt_cfg.name]
    cap: RigidObject = env.scene[nut_cfg.name]

    bottle_pos_w = bottle.data.root_com_pos_w
    cap_pos_w = cap.data.root_com_pos_w

    dist = torch.norm(bottle_pos_w - cap_pos_w, dim=1).squeeze()

    rewards = torch.tanh(dist / std)

    return rewards
