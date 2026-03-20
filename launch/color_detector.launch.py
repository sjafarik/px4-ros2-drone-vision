from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_name = 'drone_vision_py'

    image_topic = '/world/default/model/x500_mono_cam_down_0/link/camera_link/sensor/imager/image'
    camera_info_topic = '/world/default/model/x500_mono_cam_down_0/link/camera_link/sensor/imager/camera_info'

    detector_params_file = PathJoinSubstitution([
        FindPackageShare(package_name),
        'config',
        'color_detector_params.yaml'
    ])

    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='camera_bridge',
        output='screen',
        arguments=[
            f'{image_topic}@sensor_msgs/msg/Image@gz.msgs.Image',
            f'{camera_info_topic}@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
        ]
    )

    color_detector_node = Node(
        package=package_name,
        executable='color_detector',
        name='color_detector_node',
        output='screen',
        parameters=[detector_params_file]
    )

    return LaunchDescription([
        bridge_node,
        color_detector_node,
    ])