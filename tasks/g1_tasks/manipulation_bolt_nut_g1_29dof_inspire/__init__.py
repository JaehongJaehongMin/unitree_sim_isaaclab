
# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0  

import gymnasium as gym

from . import manipulation_bolt_nut_g1_29dof_inspire_env_cfg


gym.register(
    id="Isaac-Manipulation-BoltNut-G129-Inspire-Joint",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    kwargs={
        "env_cfg_entry_point": manipulation_bolt_nut_g1_29dof_inspire_env_cfg.ManipulationG129InspireBaseFixEnvCfg,
    },
    disable_env_checker=True,
)

