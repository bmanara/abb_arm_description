from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    mapping_mode = LaunchConfiguration('mapping_mode')
    mapping_mode_arg = DeclareLaunchArgument(
        'mapping_mode',
        default_value='true',
        description='Whether to start in mapping mode (true) or localization/navigation mode with a map (false).'
    )

    control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("abb_arm_description"), "/launch", "/abb_sim_control.launch.py"]
        ),
        launch_arguments={
            'launch_rviz': 'false',
            'mapping_mode': mapping_mode,
        }.items(),
    )

    abb_moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("abb_arm_moveit_config"), "/launch", "/abb_moveit.launch.py"]
        ),
    )

    return LaunchDescription([
        mapping_mode_arg,
        control_launch,
        abb_moveit_launch,
    ])