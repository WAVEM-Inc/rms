import rclpy

from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.exceptions import ROSInterruptException
from .ros.node import KTPInterface
from .tcp.presentation.Hello import iot_sample_run


def main(args=None) -> None:
    rclpy.init(args=args);

    try:
        node: Node = KTPInterface();
        node_name: str = node.get_name();
        multi_threaded_executor: MultiThreadedExecutor = MultiThreadedExecutor();
        multi_threaded_executor.add_node(node=node);
        multi_threaded_executor.spin();
    except ROSInterruptException as rie:
        node.get_logger().warn(f"===== {node_name} terminated with Ctrl-C {rie} =====");

    node.destroy_node();
    rclpy.shutdown();


if __name__ == '__main__':
    main();
    

__all__ = ['main']