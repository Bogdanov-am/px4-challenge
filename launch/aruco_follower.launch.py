from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from px4_challenge.bridge import image, camera_info
import os

def generate_launch_description():
    bridges = [image(), camera_info()]
    package_dir = get_package_share_directory('px4_challenge')
    aruco_params = os.path.join(
        package_dir,
        'config',
        'aruco_parameters.yaml'
        )
    rviz_config = os.path.join(package_dir, 'visualize.rviz')
    
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
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments = ['0', '0', '0', '1.5707963', '3.14159', '0.0', 'vehicle', 'x500_mono_cam_0/mono_cam/base_link/imager']
        ),
        Node(
            package='px4_challenge',
            namespace='',
            executable='pose_publisher',
            name='pose_publisher'
        ),
        Node(
            package='rviz2',
            namespace='',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', [rviz_config]]
        )
    ])
