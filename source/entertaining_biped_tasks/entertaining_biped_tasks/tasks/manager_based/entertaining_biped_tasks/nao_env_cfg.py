# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

from isaaclab.managers import RewardTermCfg as RewTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.utils import configclass

import isaaclab_tasks.manager_based.locomotion.velocity.mdp as mdp
from isaaclab_tasks.manager_based.locomotion.velocity.velocity_env_cfg import LocomotionVelocityRoughEnvCfg, RewardsCfg

# from . import mdp

##
# Pre-defined configs
##

from entertaining_biped_tasks.robots.aldebaran import NAO_CFG  # isort:skip

##
# Scene definition
##

##
# MDP settings
##


@configclass
class NAORewards(RewardsCfg):
    """Reward terms for the MDP."""

    termination_penalty = RewTerm(func=mdp.is_terminated, weight=-200.0)
    feet_air_time = RewTerm(
        func=mdp.feet_air_time_positive_biped,
        weight=2.50,
        params={
            "sensor_cfg": SceneEntityCfg("contact_forces", body_names=".*_ankle"),
            "command_name": "base_velocity",
            "threshold": 0.3,
        },
    )
    # penalize ankle joint limits
    dof_pos_limits = RewTerm(
        func=mdp.joint_pos_limits,
        weight=-1.0,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=".*Ankle.*")},
    )
    # penalize deviation from default of the joints that are not essential for locomotion
    joint_deviation_hip = RewTerm(
        func=mdp.joint_deviation_l1,
        weight=-0.2,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=[".*HipYawPitch", ".*HipRoll"])},
    )
    joint_deviation_arm = RewTerm(
        func=mdp.joint_deviation_l1,
        weight=-0.3,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=[".*Elbow.*", ".*Shoulder.*",])},
    )


##
# Environment configuration
##


@configclass
class NAOFlatEnvCfg(LocomotionVelocityRoughEnvCfg):
    """NAO flat environment configuration."""

    rewards: NAORewards = NAORewards()

    def __post_init__(self):
        super().__post_init__()
        # scene
        self.scene.robot = NAO_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")
        self.scene.height_scanner.prim_path = "{ENV_REGEX_NS}/Robot/torso"

        # actions
        self.actions.joint_pos.scale = 0.5
        self.actions.joint_pos.joint_names = [
            ".*Hip.*",
            ".*Ankle.*",
            ".*Knee.*",
            ".*Elbow.*",
            ".*Shoulder.*",
            # "Head.*",
            # ".*Finger.*",
            # ".*Thumb.*",
            # ".*Wrist.*",
            # ".*Hand.*",
        ]
        # events
        self.events.push_robot = None
        self.events.add_base_mass.params["asset_cfg"].body_names = ["torso"]
        self.events.add_base_mass.params["mass_distribution_params"] = (0.0, 0.0)
        self.events.reset_robot_joints.params["position_range"] = (1.0, 1.0)
        self.events.base_external_force_torque.params["asset_cfg"].body_names = ["torso"]
        self.events.reset_base.params = {
            "pose_range": {"x": (-0.5, 0.5), "y": (-0.5, 0.5), "yaw": (-3.14, 3.14)},
            "velocity_range": {
                "x": (0.0, 0.0),
                "y": (0.0, 0.0),
                "z": (0.0, 0.0),
                "roll": (0.0, 0.0),
                "pitch": (0.0, 0.0),
                "yaw": (0.0, 0.0),
            },
        }
        self.events.base_com = None

        # terminations
        self.terminations.base_contact.params["sensor_cfg"].body_names = [
            "torso",
            ".*ForeArm",
        ]

        # rewards
        self.rewards.undesired_contacts = None
        self.rewards.dof_torques_l2.weight = -5.0e-6
        self.rewards.track_lin_vel_xy_exp.weight = 2.0
        self.rewards.track_ang_vel_z_exp.weight = 1.0
        self.rewards.action_rate_l2.weight *= 1.5
        self.rewards.dof_acc_l2.weight *= 1.5
        self.rewards.flat_orientation_l2.weight = -2.5
        self.rewards.feet_air_time.weight = 5.0

        # commands
        self.commands.base_velocity.ranges.lin_vel_x = (0.5, 1.0)
        self.commands.base_velocity.ranges.lin_vel_y = (0.0, 0.0)
        self.commands.base_velocity.ranges.ang_vel_z = (-1.0, 1.0)

        # change terrain
        self.scene.terrain.terrain_type = "plane"
        self.scene.terrain.terrain_generator = None

        # no height scan
        self.scene.height_scanner = None
        self.observations.policy.height_scan = None
        # no terrain curriculum
        self.curriculum.terrain_levels = None