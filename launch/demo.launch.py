from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('cspc_lidar'),
                'launch',
                'lidar.launch.py'
            ])
        )
    )

    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('cspc_lidar'),
                'launch',
                'rviz.launch.py'
            ])
        )
    )

    return LaunchDescription([
        lidar_launch,
        rviz_launch,
    ])
