import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command

from launch_ros.actions import Node


def generate_launch_description():
    abb_arm_description_dir = get_package_share_directory('abb_arm_description')
    abb_arm_urdf = os.path.join(abb_arm_description_dir, 'urdf', 'robot.urdf.xacro')

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

    world = os.path.join(abb_arm_description_dir, 'worlds', 'empty.world')
    gazebo_params_path = os.path.join(abb_arm_description_dir, 'config', 'gazebo_params.yaml')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            )
        ]),
        launch_arguments={'gz_args': ['--render-engine ', 'ogre2 ', '-r ', world], 
                          'extra_gazebo_args': ' --ros-args --params-file ' + gazebo_params_path}.items()
    )

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-world', 'empty_world', '-topic', '/robot_description', '-name', 'abb_arm'],
        output='screen'
    )

    joint_state_broadcaster_spawner = TimerAction(
        period=2.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["joint_state_broadcaster"]
            )
        ]
    )

    abb_arm_controller_spawner = TimerAction(
        period=2.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["abb_arm_controller"]
            )
        ]
    )

    return LaunchDescription([
        robot_state_publisher_node,
        rviz2_node,
        gazebo,
        spawn_entity,
        joint_state_broadcaster_spawner,
        abb_arm_controller_spawner
    ])