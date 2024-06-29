from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from px4_challenge.bridge import image, camera_info
import os

def generate_launch_description():
    bridges = [image(), camera_info()]
    
    aruco_params = os.path.join(
        get_package_share_directory('px4_challenge'),
        'config',
        'aruco_parameters.yaml'
        )
    
    return LaunchDescription([
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            output='screen',
            arguments=[bridge.argument() for bridge in bridges],
            remappings=[bridge.remapping() for bridge in bridges],
        ),
        Node(
            package='ros2_aruco',
            executable='aruco_node',
            parameters=[aruco_params]
        ),
        Node(
            package='px4_challenge',
            namespace='px4_challenge',
            executable='visualizer',
            name='visualizer'
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments = ['0', '0', '0', '1.5707963', '3.14159', '0.0', 'vehicle', 'x500_mono_cam_0/mono_cam/base_link/imager']
        ),
    ])
