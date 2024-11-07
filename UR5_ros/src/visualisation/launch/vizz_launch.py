from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

import os
import xacro

def generate_launch_description():
    # Declare path to the URDF file
    package_name = 'visualisation'
    xacro_path = 'urdf/ur5_with_gripper.xacro'
    rviz_path = 'rviz/display.rviz'


    xacro_file = os.path.join(get_package_share_directory(package_name), xacro_path)
    xacro_raw_description = xacro.process_file(xacro_file).toxml()

    # rviz_file = os.path.join(get_package_share_directory(package_name), rviz_path)
    moveit_config_path = FindPackageShare('ur_moveit_config').find('ur_moveit_config')
    rviz_config_path = os.path.join(moveit_config_path, 'config', 'moveit.rviz')

    return LaunchDescription([
        # Robot description parameter
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': xacro_raw_description,}]
        ),

        # Joint State Publisher (using GUI for easy control)
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen'
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(moveit_config_path, 'launch', 'ur5e_moveit.launch.py')
            )
        ),

        # RViz for visualization w/ moveit config
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_path],
            output='screen'
        )

    ])
