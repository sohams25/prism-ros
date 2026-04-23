import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    pkg_dir = get_package_share_directory('prism_image_proc')
    params_file = os.path.join(pkg_dir, 'config', 'demo_params.yaml')

    container = ComposableNodeContainer(
        name='prism_image_proc_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='prism_image_proc',
                plugin='prism::ImageProcNode',
                name='resize_node',
                parameters=[params_file],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
        ],
        output='screen',
    )

    return LaunchDescription([container])
