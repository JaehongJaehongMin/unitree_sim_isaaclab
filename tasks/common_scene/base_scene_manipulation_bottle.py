# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0
"""
public base scene configuration module
provides reusable scene element configurations, such as tables, objects, ground, lights, etc.
"""
import isaaclab.sim as sim_utils
from isaaclab.assets import AssetBaseCfg, RigidObjectCfg, ArticulationCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sim.spawners.from_files.from_files_cfg import GroundPlaneCfg, UsdFileCfg
from isaaclab.utils import configclass
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR, ISAACLAB_NUCLEUS_DIR
from tasks.common_config import CameraBaseCfg  # isort: skip
import os

ASSET_DIR = f"{ISAACLAB_NUCLEUS_DIR}/Factory"
project_root = os.environ.get("PROJECT_ROOT")


@configclass
class TableBottleSceneCfg(
    InteractiveSceneCfg
):  # inherit from the interactive scene configuration class
    """object table scene configuration class
    defines a complete scene containing robot, object, table, etc.
    """

    # 1. room wall configuration - simplified configuration to avoid rigid body property conflicts
    room_walls = AssetBaseCfg(
        prim_path="/World/envs/env_.*/Room",
        init_state=AssetBaseCfg.InitialStateCfg(
            pos=[0.0, 0.0, 0], rot=[1.0, 0.0, 0.0, 0.0]  # 房间中心点
        ),
        spawn=UsdFileCfg(
            usd_path=f"{ISAAC_NUCLEUS_DIR}/Environments/Simple_Warehouse/warehouse.usd",  # use simple room model
        ),
    )
    # print(f"ISAAC_NUCLEUS_DIR: {ISAAC_NUCLEUS_DIR}")
    # ISAAC_NUCLEUS_DIR: http://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/4.5/Isaac
    # 1. table configuration
    packing_table = AssetBaseCfg(
        prim_path="/World/envs/env_.*/PackingTable",  # table in the scene
        init_state=AssetBaseCfg.InitialStateCfg(
            pos=[0.0, 0.55, -0.2],  # initial position [x, y, z]
            rot=[1.0, 0.0, 0.0, 0.0],
        ),  # initial rotation [x, y, z, w]
        spawn=UsdFileCfg(
            # usd_path=f"{ISAAC_NUCLEUS_DIR}/Props/PackingTable/packing_table.usd",    # table model file
            usd_path=f"{project_root}/assets/objects/PackingTable/PackingTable.usd",  # table model file
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                kinematic_enabled=True
            ),  # set to kinematic object
        ),
    )

    packing_table_2 = AssetBaseCfg(
        prim_path="/World/envs/env_.*/PackingTable_2",
        init_state=AssetBaseCfg.InitialStateCfg(
            pos=[-3.5, 0.55, -0.2], rot=[1.0, 0.0, 0.0, 0.0]
        ),
        spawn=UsdFileCfg(
            usd_path=f"{project_root}/assets/objects/PackingTable/PackingTable.usd",  # table model file
            rigid_props=sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=True),
        ),
    )
    packing_table_3 = AssetBaseCfg(
        prim_path="/World/envs/env_.*/PackingTable_3",
        init_state=AssetBaseCfg.InitialStateCfg(
            pos=[3.5, 0.55, -0.2], rot=[1.0, 0.0, 0.0, 0.0]
        ),
        spawn=UsdFileCfg(
            usd_path=f"{project_root}/assets/objects/PackingTable/PackingTable.usd",  # table model file
            rigid_props=sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=True),
        ),
    )
    packing_table_4 = AssetBaseCfg(
        prim_path="/World/envs/env_.*/PackingTable_4",
        init_state=AssetBaseCfg.InitialStateCfg(
            pos=[3.5, -5, -0.2], rot=[1.0, 0.0, 0.0, 0.0]
        ),
        spawn=UsdFileCfg(
            usd_path=f"{project_root}/assets/objects/PackingTable/PackingTable.usd",  # table model file
            rigid_props=sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=True),
        ),
    )
    packing_table_5 = AssetBaseCfg(
        prim_path="/World/envs/env_.*/PackingTable_5",
        init_state=AssetBaseCfg.InitialStateCfg(
            pos=[-3.5, -5, -0.2], rot=[1.0, 0.0, 0.0, 0.0]
        ),
        spawn=UsdFileCfg(
            usd_path=f"{project_root}/assets/objects/PackingTable/PackingTable.usd",  # table model file
            rigid_props=sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=True),
        ),
    )
    packing_table_6 = AssetBaseCfg(
        prim_path="/World/envs/env_.*/PackingTable_6",
        init_state=AssetBaseCfg.InitialStateCfg(
            pos=[0.0, -5, -0.2], rot=[1.0, 0.0, 0.0, 0.0]
        ),
        spawn=UsdFileCfg(
            usd_path=f"{project_root}/assets/objects/PackingTable/PackingTable.usd",  # table model file
            rigid_props=sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=True),
        ),
    )
    # Object
    # 2. object configuration
    bottle = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/Bottle",
        spawn=sim_utils.UsdFileCfg(
            usd_path=f"{project_root}/assets/objects/bottle0/output/bottle.usd",
            scale=(0.1, 0.1, 0.1),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=False),
        ),
        init_state=RigidObjectCfg.InitialStateCfg(
            pos=(-0.15, 0.50, 0.80),
        ),
    )
    cap = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/BottleCap",
        spawn=sim_utils.UsdFileCfg(
            usd_path=f"{project_root}/assets/objects/bottle0/output/bottle_cap.usd",
            scale=(1.0, 1.0, 1.0),
        ),
        init_state=RigidObjectCfg.InitialStateCfg(
            pos=(-0.15, 0.50, 0.80 + 0.104), rot=(0.34202, 0.0, 0.0, -0.93969)
            # pos=(-0.15, 0.50, 0.80 + 0.109),
            # rot=(0.34202, 0.0, 0.0, -0.93969),
        ),
    )

    # Ground plane
    # 3. ground configuration
    # ground = AssetBaseCfg(
    #     prim_path="/World/GroundPlane",    # ground in the scene
    #     spawn=GroundPlaneCfg( ),    # ground configuration
    # )

    # Lights
    # 4. light configuration
    light = AssetBaseCfg(
        prim_path="/World/light",  # light in the scene
        spawn=sim_utils.DomeLightCfg(
            color=(0.75, 0.75, 0.75), intensity=3000.0  # light color (white)
        ),  # light intensity
    )

    world_camera = CameraBaseCfg.get_camera_config(
        prim_path="/World/PerspectiveCamera",
        pos_offset=(-0.1, 3.6, 1.6),
        rot_offset=(-0.00617, 0.00617, 0.70708, -0.70708),
        focal_length=16.5,
    )
