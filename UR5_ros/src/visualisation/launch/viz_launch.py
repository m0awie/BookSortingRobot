# gripper_launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    urdf_file = os.path.join(
        get_package_share_directory('visualisation'),
        'urdf',
        'gripper.urdf.xacro',
        'ur5_with_gripper.urdf.xacro'
    )
    rviz_config = os.path.join(
        get_package_share_directory('visualisation'),
        'rviz',
        'view_gripper.rviz'
    )
    
    return LaunchDescription([
        # Gripper control node
        Node(
            package='visualisation',
            executable='visualisation',
            name='gripper_viz',
            output='screen'
        ),
        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            arguments=[urdf_file],
            output='screen'
        ),
        # RViz node
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen'
        ),
    ])
