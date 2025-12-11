# Copyright 2024 Avular B.V.
# All Rights Reserved
# You may use this code under the terms of the Avular
# Software End-User License Agreement.
#
# You should have received a copy of the Avular
# Software End-User License Agreement license with
# this file, or download it from: avular.com/eula

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetLaunchConfiguration,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare world argument
    world_arg = DeclareLaunchArgument(
        "world",
        default_value="TY_test_area.world",
        description="World file name"
    )
    
    robot_pose_x = SetLaunchConfiguration("robot_pose_x", "0.0")
    robot_pose_y = SetLaunchConfiguration("robot_pose_y", "15.0")
    robot_pose_yaw = SetLaunchConfiguration("robot_pose_yaw", "1.57")

    drive_configuration = DeclareLaunchArgument(
        "drive_configuration", default_value="skid_steer_drive")

    launch_origin_common = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("origin_one_gazebo"),
                        "launch",
                        "origin_sim_common.launch.py",
                    ]
                )
            ]
        ),
        launch_arguments={
            "world": LaunchConfiguration("world"),
            "drive_configuration": LaunchConfiguration("drive_configuration"),
            "robot_pose_x": LaunchConfiguration("robot_pose_x"),
            "robot_pose_y": LaunchConfiguration("robot_pose_y"),
            "robot_pose_yaw": LaunchConfiguration("robot_pose_yaw"),
        }.items(),
    )

    return LaunchDescription(
        [
            world_arg,
            robot_pose_x,
            robot_pose_y,
            robot_pose_yaw,
            drive_configuration,
            launch_origin_common,
        ]
    )
