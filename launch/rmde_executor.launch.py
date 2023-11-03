import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = LaunchDescription()

    package_name: str = 'rmde'
    package_shared_directory: str = get_package_share_directory(package_name)
    executable_name: str = 'rmde_executor'

    rmde_executor_node = Node(
        package=package_name,
        executable=executable_name,
        name=executable_name,
        output='screen',
        parameters=[]
    )

    ld.add_action(rmde_executor_node)

    return ld
