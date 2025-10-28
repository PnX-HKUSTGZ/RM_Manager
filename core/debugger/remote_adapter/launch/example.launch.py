from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('remote_adapter')
    params_file = os.path.join(pkg_share, 'params', 'example.yaml')

    remote_adapter_node = Node(
        package='remote_adapter',
        executable='remote_adapter_node',
        name='remote_adapter',
        output='screen',
        parameters=[params_file]
    )

    rm_manager = Node(
        package="rm_manager",
        executable="rm_manager",
        parameters=[
            # {"referee_port": "None"}
        ]
    )

    return LaunchDescription([
        remote_adapter_node,
        rm_manager,
    ])
