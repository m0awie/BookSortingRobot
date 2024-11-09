import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction
from launch.actions import IncludeLaunchDescription
# import xacro
from launch.substitutions import PathJoinSubstitution, Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    use_fake = True # Fake or Real?
    ip_address = 'yyy.yyy.yyy.yyy'
    fake_str = 'true'

    
    # realsense_launch_file = os.path.join(
    #     get_package_share_directory('realsense2_camera'),
    #     'launch',
    #     'rs_launch.py'
    # )

    xacro_path = 'urdf/ur5_with_gripper.xacro'

    if not use_fake:
        ip_address = '192.168.0.100'
        fake_str = 'false'

    ur_control_launch_args = {
        'ur_type': 'ur5e',
        'robot_ip': ip_address,
        'use_fake_hardware': fake_str,
        'launch_rviz': 'false',
        'description_file': os.path.join(
            get_package_share_directory('visualisation'), # package with the xacro
            xacro_path, # the xacro that initiate the end_effector and ur5e
        )
    }

    moveit_launch_args = {
        'ur_type': 'ur5e',
        'launch_rviz': 'true',
    }

    if use_fake:
        ur_control_launch_args['initial_joint_controller'] = 'joint_trajectory_controller'
        moveit_launch_args['use_fake_hardware'] = fake_str


    ur_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ur_robot_driver'), 'launch', 'ur_control.launch.py'
            ])
        ),
        launch_arguments=ur_control_launch_args.items(),
    )

    # Define the MoveIt server launch with a delay
    moveit_launch = TimerAction(
        period=10.0,  # Delay to allow the UR control to start first
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([
                        FindPackageShare('ur_moveit_config'), 'launch', 'ur_moveit.launch.py'
                    ])
                ),
                launch_arguments=moveit_launch_args.items(),
            ),
        ]
    )
    
    # camera_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(realsense_launch_file),
    #     launch_arguments={
    #         'align_depth': 'true',
    #         'enable_color': 'true',
    #         'enable_depth': 'true',
    #         'pointcloud.enable': 'true'
    #     }.items()
    # )

    launch_description = [
        ur_control_launch,
        moveit_launch,
    ]

    # if not use_fake:
    #     launch_description.append(camera_launch)

    return LaunchDescription(launch_description)