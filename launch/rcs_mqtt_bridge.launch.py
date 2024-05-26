import os;
from launch import LaunchDescription;
from launch_ros.actions import Node;
from ament_index_python.packages import get_package_share_directory;


def generate_launch_description() -> LaunchDescription:
    ld: LaunchDescription = LaunchDescription();

    package_name: str = "rcs_mqtt_bridge";
    package_shared_directory: str = get_package_share_directory(package_name);
    parameters: list[str] = [os.path.join(package_shared_directory, "config", "mqtt_config.yaml")];
    executable_name: str = package_name;

    rmde_executor_node: Node = Node(
        package=package_name,
        executable=executable_name,
        name=executable_name,
        arguments=["--ros-args", "--enclave", "/rcs_mqtt_bridge"],
        output="screen",
        parameters=parameters
    )

    ld.add_action(rmde_executor_node);

    return ld;

