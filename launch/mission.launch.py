from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_name = 'drone_vision_py'

    mission_file_arg = DeclareLaunchArgument(
        'mission_file',
        default_value='mission_params.yaml',
        description='Mission YAML file located in the config folder'
    )

    mission_params_file = PathJoinSubstitution([
        FindPackageShare(package_name),
        'config',
        LaunchConfiguration('mission_file')
    ])

    offboard_params_file = PathJoinSubstitution([
        FindPackageShare(package_name),
        'config',
        'offboard_params.yaml'
    ])

    offboard_control_node = Node(
        package=package_name,
        executable='offboard_control',
        name='offboard_control_node',
        output='screen',
        parameters=[offboard_params_file]
    )

    mission_planner_node = Node(
        package=package_name,
        executable='mission_planner',
        name='mission_planner_node',
        output='screen',
        parameters=[mission_params_file]
    )

    return LaunchDescription([
        mission_file_arg,
        offboard_control_node,
        mission_planner_node,
    ])