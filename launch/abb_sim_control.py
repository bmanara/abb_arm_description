import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node


def generate_launch_description():
    abb_arm_description_dir = get_package_share_directory('abb_arm_description')
    abb_arm_urdf = os.path.join(abb_arm_description_dir, 'abb_arm', 'robot.urdf')

    robot_description = Command(['xacro ', abb_arm_urdf])
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True    
        }],
        emulate_tty=True,
    )

    rviz2_config = os.path.join(abb_arm_description_dir, 'rviz', 'abb_arm.rviz')
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz2_config],
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        argument=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
    )

    return LaunchDescription([
        robot_state_publisher_node,
        rviz2_node,
        joint_state_broadcaster_spawner,
    ])