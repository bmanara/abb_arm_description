from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("abb_arm_description"), "/launch", "/abb_sim_control.launch.py"]
        ),
        launch_arguments={'launch_rviz': 'false'}.items(),
    )

    abb_moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("abb_arm_moveit_config"), "/launch", "/abb_moveit.launch.py"]
        ),
    )

    return LaunchDescription([
        control_launch,
        abb_moveit_launch,
    ])