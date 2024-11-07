from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os
import xacro

def generate_launch_description():
    # Declare path to the URDF file
    package_name = 'visualisation'
    xacro_path = 'urdf/ur5_with_gripper.xacro'
    rviz_path = 'rviz/display.rviz'


    xacro_file = os.path.join(get_package_share_directory(package_name), xacro_path)
    xacro_raw_description = xacro.process_file(xacro_file).toxml()

    rviz_file = os.path.join(get_package_share_directory(package_name), rviz_path)

    return LaunchDescription([
        # Robot description parameter
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': xacro_raw_description, 'source_list':['main_joint']}]
        ),

        # Joint State Publisher (using GUI for easy control)
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher',
            output='screen'
        ),

        # RViz for visualization
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_file],
            output='screen'
        )
    ])
