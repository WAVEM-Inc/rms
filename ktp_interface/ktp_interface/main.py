import rclpy;
from rclpy.node import Node;
from rclpy.executors import MultiThreadedExecutor;
from rclpy.exceptions import ROSInterruptException;
from ktp_interface.ros.node import KTPInterface;
from ktp_interface.tcp.application.service import tcp_release;


def main(args=None) -> None:
    rclpy.init(args=args);

    try:
        ktp_interface_node: Node = KTPInterface();
        ktp_interface_node_name: str = ktp_interface_node.get_name();
        ktp_interface_node.get_logger().info(f"{ktp_interface_node_name} created");
        
        multi_threaded_executor: MultiThreadedExecutor = MultiThreadedExecutor();
        multi_threaded_executor.add_node(node=ktp_interface_node);
        multi_threaded_executor.spin();
    except ROSInterruptException as rie:
        ktp_interface_node.get_logger().warn(f"===== {ktp_interface_node_name} terminated with Ctrl-C {rie} =====");
        tcp_release();

    ktp_interface_node.destroy_node();
    rclpy.shutdown();


if __name__ == "__main__":
    main();


__all__ : list[str]= ["main"];