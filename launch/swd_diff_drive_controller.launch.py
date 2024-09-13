################################################
## Copyright (C) 2022 ez-Wheel S.A.S.         ##
##                                            ##
## @file swd_diff_drive_controller.launch.py  ##
################################################

import sys, os
import os.path

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    # Load specific dbus user session if exists
    session = "/tmp/SYSTEMCTL_dbus.id"
    if os.path.isfile(session):
        print("SYSTEMCTL_dbus.id detected")

        with open(session) as f:
            lines = f.readlines()
        env = dict(line.strip().split("=", 1) for line in lines)
        os.environ.update(env)

    # Launch description
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false", description="Use simulation (Gazebo) clock if true"),
            DeclareLaunchArgument("baseline_m", default_value="0.485"), # Center of left wheel to center of right wheel
            DeclareLaunchArgument("motor_max_speed_rpm", default_value="1050"),                 # 0.49 [m/s] - ratio=14 diameter=125mm
            DeclareLaunchArgument("motor_max_safety_limited_speed_1_rpm", default_value="560"), # 0.26 [m/s] - ratio=14 diameter=125mm
            DeclareLaunchArgument("motor_max_safety_limited_speed_2_rpm", default_value="680"), # 0.32 [m/s] - ratio=14 diameter=125mm
            DeclareLaunchArgument("motor_max_delta_speed_rpm", default_value="525"),  # motor_max_speed_rpm / 2
            DeclareLaunchArgument("accurate_odometry", default_value="false", description="Use a more precise odometry of the SWD product"),
            DeclareLaunchArgument("publish_wheel_odometry", default_value="false", description="Publish odometry of the both wheels"),
            Node(
                package="swd_ros2_controllers",
                executable="swd_diff_drive_controller",
                name="swd_diff_drive_controller",
                output="screen",
                parameters=[
                    {"use_sim_time": LaunchConfiguration("use_sim_time")},
                    {"baseline_m": LaunchConfiguration("baseline_m")},
                    {"left_swd_config_file": "/opt/ezw/usr/etc/ezw-smc-core/swd_left_config.ini"},
                    {"right_swd_config_file": "/opt/ezw/usr/etc/ezw-smc-core/swd_right_config.ini"},
                    {"pub_freq_hz": 20},
                    {"watchdog_receive_ms": 500},
                    {"base_frame": "base_link"},
                    {"odom_frame": "odom"},
                    {"publish_odom": True},
                    {"publish_wheel_odometry": LaunchConfiguration("publish_wheel_odometry")},
                    {"publish_tf": True},
                    {"publish_safety_functions": True},
                    {"motor_max_speed_rpm": LaunchConfiguration("motor_max_speed_rpm")},
                    {"motor_max_safety_limited_speed_1_rpm": LaunchConfiguration("motor_max_safety_limited_speed_1_rpm")},
                    {"motor_max_safety_limited_speed_2_rpm": LaunchConfiguration("motor_max_safety_limited_speed_2_rpm")},
                    {"motor_max_delta_speed_rpm": LaunchConfiguration("motor_max_delta_speed_rpm")},
                    {"have_backward_sls": False},
                    {"left_encoder_relative_error": 0.2},
                    {"right_encoder_relative_error": 0.2},
                    {"accurate_odometry": LaunchConfiguration("accurate_odometry")},
                ],
            ),
        ]
    )
