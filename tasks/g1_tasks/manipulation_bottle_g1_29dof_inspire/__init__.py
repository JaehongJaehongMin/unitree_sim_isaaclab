
# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0  

import gymnasium as gym

from . import manipulation_bottle_g1_29dof_inspire_env_cfg


gym.register(
    id="Isaac-Manipulation-Bottle-G129-Inspire-Joint",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    kwargs={
        "env_cfg_entry_point": manipulation_bottle_g1_29dof_inspire_env_cfg.ManipulationG129InspireBaseFixEnvCfg,
    },
    disable_env_checker=True,
)

