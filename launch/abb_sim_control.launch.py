import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
    

def generate_launch_description():
    abb_arm_description_dir = get_package_share_directory('abb_arm_description')
    abb_arm_urdf = os.path.join(abb_arm_description_dir, 'urdf', 'robot.urdf.xacro')
    launch_rviz= LaunchConfiguration('launch_rviz')
    launch_rviz_arg = DeclareLaunchArgument(
        'launch_rviz',
        default_value='true',
        description='Whether to start RViz'
    )

    mapping_mode = LaunchConfiguration('mapping_mode')
    mapping_mode_arg = DeclareLaunchArgument(
        'mapping_mode',
        default_value='true',
        description='Whether to start in mapping mode (true) or localization/navigation mode with a map(false).'
    )

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
        parameters=[{'use_sim_time': True}],
        arguments=['-d', rviz2_config],
        condition=IfCondition(launch_rviz)
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
        arguments=['-world', 'empty_world', 
                   '-topic', '/robot_description', 
                   '-name', 'abb_arm',
                   '-z', '0.6'],
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

    diff_drive_controller_spawner = TimerAction(
        period=2.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["diff_drive_controller"]
            )
        ]
    )

    twist_mux = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        output='screen',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('abb_arm_description'),
                'config',
                'twist_mux.yaml'
            ]),
            {'use_sim_time': True},
        ],
        remappings=[('/cmd_vel_out', 'diff_drive_controller/cmd_vel_unstamped')]
    )

    bridge_params = os.path.join(get_package_share_directory('abb_arm_description'), 'config', 'gz_bridge.yaml')
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['--ros-args', '-p', f'config_file:={bridge_params}']
    )

    slam_params_file = PathJoinSubstitution([
        FindPackageShare('abb_arm_description'),
        'config',
        'mapper_params_online_async.yaml'
    ])

    slam_launch = TimerAction(
        period=4.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare('slam_toolbox'),
                        'launch',
                        'online_async_launch.py'
                    ])
                ]),
                launch_arguments={
                    'slam_params_file': slam_params_file,
                    'use_sim_time': 'true'
                }.items(),
                condition=IfCondition(mapping_mode)
            )
        ]
    )

    nav2_params_file = PathJoinSubstitution([
        FindPackageShare('abb_arm_description'),
        'config',
        'nav2_params.yaml'
    ])
    map_yaml_file = "/home/brian_2025/Github/moveit_ws/src/abb_arm_description/maps/my_map_save.yaml"  # DO NOT USE PACKAGE FILE PATH (IDK WHY IT DOESNT WORK)
    
    nav2_launch = TimerAction(
        period=4.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare('nav2_bringup'),
                        'launch',
                        'bringup_launch.py'
                    ])
                ]),
                launch_arguments={
                    'use_sim_time': 'true',
                    'params_file': nav2_params_file,
                    'map': map_yaml_file,
                }.items(),
                condition=UnlessCondition(mapping_mode)
            )
        ]
    )

    arguments = [
        launch_rviz_arg,
        mapping_mode_arg
    ]

    nodes_to_launch = [
        robot_state_publisher_node,
        rviz2_node,
        gazebo,
        spawn_entity,
        joint_state_broadcaster_spawner,
        abb_arm_controller_spawner,
        diff_drive_controller_spawner,
        twist_mux,
        ros_gz_bridge,
        slam_launch,
        nav2_launch
    ]   

    return LaunchDescription(arguments + nodes_to_launch)