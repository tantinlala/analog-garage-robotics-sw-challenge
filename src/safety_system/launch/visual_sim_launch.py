import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    config = os.path.join(
        get_package_share_directory('safety_system'),
        'config',
        'visual_sim_params.yaml'
        )

    return LaunchDescription([
        Node(
            package='estop_monitor',
            executable='node_estop_monitor',
            parameters=[config]
        ),
        Node(
            package='proximity_sensor',
            executable='node_proximity_sensor',
            parameters=[config]
        ),
        Node(
            package='speed_limiter',
            executable='node_speed_limiter',
            parameters=[config]
        )
    ])