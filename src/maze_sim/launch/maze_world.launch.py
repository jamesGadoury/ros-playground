#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")

    world = os.path.join(
        get_package_share_directory('maze_sim'),
        'worlds',
        'maze_world_ign.sdf'
    )

    ld = LaunchDescription()

    # Add the commands to the launch description
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": world}.items()
    ))

    ld.add_action(Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist"]
    ))

    ld.add_action(Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/lidar@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan"]
    ))

    return ld
