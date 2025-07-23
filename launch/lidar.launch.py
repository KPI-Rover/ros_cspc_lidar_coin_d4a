
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import LogInfo

import lifecycle_msgs.msg
import os


def generate_launch_description():
    share_dir = get_package_share_directory('cspc_lidar')
    parameter_file = LaunchConfiguration('params_file')
    node_name = 'cspc_lidar'

    log_level = LaunchConfiguration('log_level', default='info')

    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Logging level (e.g., debug, info, warn, error, fatal)'
    )

    params_declare = DeclareLaunchArgument('params_file',
                                           default_value=os.path.join(
                                               share_dir, 'params', 'cspc_lidar.yaml'),
                                           description='FPath to the ROS2 parameters file to use.')

    driver_node = LifecycleNode(package='cspc_lidar',
                                executable='cspc_lidar',
                                name='cspc_lidar',
                                output='screen',
                                emulate_tty=True,
                                parameters=[parameter_file],
                                arguments=['--ros-args', '--log-level', log_level],
                                namespace='/',
                                )


    return LaunchDescription([
        log_level_arg,
        params_declare,
        driver_node,
    ])
