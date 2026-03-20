import os
import xacro
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('tstar_description')
    xacro_file = os.path.join(pkg_share, 'urdf', 'tartanstar.urdf.xacro')

    robot_description = xacro.process_file(xacro_file).toxml()

    rviz_config = os.path.join(pkg_share, 'launch', 'rviz.rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'model',
            default_value=xacro_file,
            description='Path to URDF file',
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value='false',
            description='Launch RViz',
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),
    ])
