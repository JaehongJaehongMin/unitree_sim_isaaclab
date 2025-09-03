# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0
from __future__ import annotations

import torch
from typing import TYPE_CHECKING

from isaaclab.assets import Articulation, DeformableObject, RigidObject
from isaaclab.managers import SceneEntityCfg
import isaaclab.utils.math as math_utils

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv
    from isaaclab.envs import ManagerBasedEnv


def task_done(
    env: ManagerBasedRLEnv,
    th: float = 10.0,
    bolt_cfg: SceneEntityCfg = SceneEntityCfg("bottle"),
    nut_cfg: SceneEntityCfg = SceneEntityCfg("cap"),
) -> torch.Tensor:
    bottle: RigidObject = env.scene[bolt_cfg.name]
    cap: RigidObject = env.scene[nut_cfg.name]

    bolt_pos_w = bottle.data.root_com_pos_w
    nut_pos_w = cap.data.root_com_pos_w

    dist = torch.norm(bolt_pos_w - nut_pos_w, dim=1).squeeze()

    done = dist >= th

    return done


def reset_root_state_uniform_double(
    env: ManagerBasedEnv,
    env_ids: torch.Tensor,
    pose_range: dict[str, tuple[float, float]],
    velocity_range: dict[str, tuple[float, float]],
    asset_0_cfg: SceneEntityCfg = SceneEntityCfg("object_0"),
    asset_1_cfg: SceneEntityCfg = SceneEntityCfg("object_1"),
):
    """Reset the asset root state to a random position and velocity uniformly within the given ranges.

    This function randomizes the root position and velocity of the asset.

    * It samples the root position from the given ranges and adds them to the default root position, before setting
      them into the physics simulation.
    * It samples the root orientation from the given ranges and sets them into the physics simulation.
    * It samples the root velocity from the given ranges and sets them into the physics simulation.

    The function takes a dictionary of pose and velocity ranges for each axis and rotation. The keys of the
    dictionary are ``x``, ``y``, ``z``, ``roll``, ``pitch``, and ``yaw``. The values are tuples of the form
    ``(min, max)``. If the dictionary does not contain a key, the position or velocity is set to zero for that axis.
    """
    # extract the used quantities (to enable type-hinting)
    asset_0: RigidObject | Articulation = env.scene[asset_0_cfg.name]
    asset_1: RigidObject | Articulation = env.scene[asset_1_cfg.name]
    # get default root state
    root_states = asset_0.data.default_root_state[env_ids].clone()
    root_states_1 = asset_1.data.default_root_state[env_ids].clone()

    # poses
    range_list = [
        pose_range.get(key, (0.0, 0.0))
        for key in ["x", "y", "z", "roll", "pitch", "yaw"]
    ]
    ranges = torch.tensor(range_list, device=asset_0.device)
    rand_samples = math_utils.sample_uniform(
        ranges[:, 0], ranges[:, 1], (len(env_ids), 6), device=asset_0.device
    )

    positions = (
        root_states[:, 0:3] + env.scene.env_origins[env_ids] + rand_samples[:, 0:3]
    )
    positions_1 = (
        root_states_1[:, 0:3] + env.scene.env_origins[env_ids] + rand_samples[:, 0:3]
    )
    orientations_delta = math_utils.quat_from_euler_xyz(
        rand_samples[:, 3], rand_samples[:, 4], rand_samples[:, 5]
    )
    orientations = math_utils.quat_mul(root_states[:, 3:7], orientations_delta)
    orientations_1 = math_utils.quat_mul(root_states_1[:, 3:7], orientations_delta)
    # velocities
    range_list = [
        velocity_range.get(key, (0.0, 0.0))
        for key in ["x", "y", "z", "roll", "pitch", "yaw"]
    ]
    ranges = torch.tensor(range_list, device=asset_0.device)
    rand_samples = math_utils.sample_uniform(
        ranges[:, 0], ranges[:, 1], (len(env_ids), 6), device=asset_0.device
    )

    velocities = root_states[:, 7:13] + rand_samples
    velocities_1 = root_states_1[:, 7:13] + rand_samples

    # set into the physics simulation
    asset_0.write_root_pose_to_sim(
        torch.cat([positions, orientations], dim=-1), env_ids=env_ids
    )
    asset_1.write_root_pose_to_sim(
        torch.cat([positions_1, orientations_1], dim=-1), env_ids=env_ids
    )
    asset_0.write_root_velocity_to_sim(velocities, env_ids=env_ids)
    asset_1.write_root_velocity_to_sim(velocities_1, env_ids=env_ids)
