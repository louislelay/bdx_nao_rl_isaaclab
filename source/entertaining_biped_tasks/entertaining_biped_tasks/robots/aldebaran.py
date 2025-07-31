# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Configuration for the Disney Research robots.

The following configuration parameters are available:

* :obj:`BDX_CFG`: The BD-X robot with implicit Actuator model

Reference:

* https://github.com/rimim/AWD/tree/main/awd/data/assets/go_bdx

"""
from pathlib import Path

TEMPLATE_ASSETS_DATA_DIR = Path(__file__).resolve().parent.parent.parent / "data"

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets.articulation import ArticulationCfg
from isaaclab.utils.assets import ISAACLAB_NUCLEUS_DIR

##
# Configuration
##


# | Joint         | L (rad)     | R (rad)     |
# | ------------- | ----------- | ----------- |
# | HeadYaw       | 0.0000      | –           |
# | HeadPitch     | 0.0000      | –           |
# | ShoulderPitch | **1.3963**  | **1.3963**  |
# | ShoulderRoll  | **0.3491**  | **-0.3491** |
# | ElbowYaw      | **-1.3963** | **1.3963**  |
# | ElbowRoll     | **-1.0472** | **1.0472**  |
# | WristYaw      | 0.0000      | 0.0000      |
# | Hand          | 0.0000      | 0.0000      |
# | HipYawPitch   | 0.0000      | –           |
# | HipRoll       | 0.0000      | 0.0000      |
# | HipPitch      | **-0.3491** | **-0.3491** |
# | KneePitch     | **0.6981**  | **0.6981**  |
# | AnklePitch    | **-0.3491** | **-0.3491** |
# | AnkleRoll     | 0.0000      | 0.0000      |



NAO_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=f"{TEMPLATE_ASSETS_DATA_DIR}/Robots/Aldebaran/Nao/nao.usd",
        activate_contact_sensors=True,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            retain_accelerations=False,
            linear_damping=0.0,
            angular_damping=0.0,
            max_linear_velocity=1000.0,
            max_angular_velocity=1000.0,
            max_depenetration_velocity=1.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=True, solver_position_iteration_count=4, solver_velocity_iteration_count=0
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.35),
        joint_pos={
            "HeadPitch": 0.0,
            "HeadYaw": 0.0,

            ".*ShoulderPitch": 1.3963,
            "LShoulderRoll": 0.3491,
            "RShoulderRoll": -0.3491,

            "LElbowYaw": -1.3963,
            "LElbowRoll": -1.0472,
            "RElbowYaw": 1.3963,
            "RElbowRoll": 1.0472,

            ".*WristYaw": 0.0,
            ".*Hand": 0.0,

            ".*HipYawPitch": 0.0,
            ".*HipRoll": 0.0,

            ".*HipPitch": -0.3491,
            ".*KneePitch": 0.6981,

            ".*AnklePitch": -0.3491,
            ".*AnkleRoll": 0.0,
        },
	# - 'LShoulderRoll': -0.563 not in [-0.314, 1.326]
	# - 'RHipRoll': 1.822 not in [-0.790, 0.379]
	# - 'LKneePitch': -0.379 not in [-0.092, 2.113]
	# - 'RElbowRoll': -0.379 not in [0.035, 1.545]
	# - 'LAnklePitch': 1.809 not in [-1.189, 0.923]
	# - 'RHand': -0.560 not in [0.000, 1.000]

    ),
    actuators={
        "legs": ImplicitActuatorCfg(
            joint_names_expr=[".*Hip.*", ".*Ankle.*", ".*Knee.*"],
            stiffness={
                ".*Hip.*": 200.0,
                ".*Ankle.*": 80.0,
                ".*Knee.*": 120.0,
            },
            damping={
                ".*Hip.*": 20.0,
                ".*Ankle.*": 8.0,
                ".*Knee.*": 12.0,
            },
            effort_limit_sim={
                ".*Hip.*": 8.0,
                ".*Ankle.*": 5.0,
                ".*Knee.*": 5.0,
            },
            velocity_limit_sim={
                ".*Hip.*": 12.0,
                ".*Ankle.*": 8.0,
                ".*Knee.*": 8.0,
            },
        ),
        "arms": ImplicitActuatorCfg(
            joint_names_expr=[".*Elbow.*", ".*Shoulder.*"],
            stiffness={
                ".*": 100.0,
            },
            damping={
                ".*": 10.0,
            },
            effort_limit_sim=5.0,
            velocity_limit_sim=7.0,
        ),
        "head": ImplicitActuatorCfg(
            joint_names_expr=["Head.*"],
            stiffness={
                ".*": 200.0,
            },
            damping={
                ".*": 20.0,
            },
            effort_limit_sim=8.0,
            velocity_limit_sim=12.0,
        ),
        "hands": ImplicitActuatorCfg(
            joint_names_expr=[".*Finger.*", ".*Thumb.*", ".*Wrist.*", ".*Hand.*"],
            stiffness={
                ".*": 200.0,
            },
            damping={
                ".*": 20.0,
            },
        ),
    },
    soft_joint_pos_limit_factor=0.95,
)
"""Configuration for the Disney BD-X robot with implicit actuator model."""

# torso
#  ├─ HeadYaw --> Neck
#  │   └─ HeadPitch --> Head
#  │
#  ├─ LHipYawPitch --> LPelvis
#  │   └─ LHipRoll --> LHip
#  │       └─ LHipPitch --> LThigh
#  │           └─ LKneePitch --> LTibia
#  │               └─ LAnklePitch --> LAnklePitch
#  │                   └─ LAnkleRoll --> l_ankle
#  │
#  ├─ RHipYawPitch --> RPelvis
#  │   └─ RHipRoll --> RHip
#  │       └─ RHipPitch --> RThigh
#  │           └─ RKneePitch --> RTibia
#  │               └─ RAnklePitch --> RAnklePitch
#  │                   └─ RAnkleRoll --> r_ankle
#  │
#  ├─ LShoulderPitch --> LShoulder
#  │   └─ LShoulderRoll --> LBicep
#  │       └─ LElbowYaw --> LElbow
#  │           └─ LElbowRoll --> LForeArm
#  │               └─ LWristYaw --> l_wrist
#  │                   └─ LHand --> l_gripper
#  │                       ├─ LFinger11 --> LFinger11_link
#  │                       │   └─ LFinger12 --> LFinger12_link
#  │                       │       └─ LFinger13 --> LFinger13_link
#  │                       ├─ LFinger21 --> LFinger21_link
#  │                       │   └─ LFinger22 --> LFinger22_link
#  │                       │       └─ LFinger23 --> LFinger23_link
#  │                       └─ LThumb1 --> LThumb1_link
#  │                           └─ LThumb2 --> LThumb2_link
#  │
#  └─ RShoulderPitch --> RShoulder
#      └─ RShoulderRoll --> RBicep
#          └─ RElbowYaw --> RElbow
#              └─ RElbowRoll --> RForeArm
#                  └─ RWristYaw --> r_wrist
#                      └─ RHand --> r_gripper
#                          ├─ RFinger11 --> RFinger11_link
#                          │   └─ RFinger12 --> RFinger12_link
#                          │       └─ RFinger13 --> RFinger13_link
#                          ├─ RFinger21 --> RFinger21_link
#                          │   └─ RFinger22 --> RFinger22_link
#                          │       └─ RFinger23 --> RFinger23_link
#                          └─ RThumb1 --> RThumb1_link
#                              └─ RThumb2 --> RThumb2_link
