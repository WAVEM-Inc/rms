import rclpy;
import threading;
from rclpy.node import Node;
from rclpy.executors import MultiThreadedExecutor;
from rclpy.exceptions import ROSInterruptException;

from ktp_interface.ros.node import KTPInterface;
from ktp_interface.tcp.application.service import tcp_release;
# from ktp_interface.tcp.application.service import create_tcp_node;


def main(args=None) -> None:
    rclpy.init(args=args);

    try:
        ktp_interface_node: Node = KTPInterface();
        ktp_interface_node_name: str = ktp_interface_node.get_name();
        ktp_interface_node.get_logger().info(f"{ktp_interface_node_name} created");
        
        # ktp_interface_tcp_node: Node = create_tcp_node();
        # ktp_interface_tcp_node_name: str = ktp_interface_tcp_node.get_name();
        # ktp_interface_tcp_node.get_logger().info(f"{ktp_interface_tcp_node_name} created");
        
        multi_threaded_executor: MultiThreadedExecutor = MultiThreadedExecutor();
        multi_threaded_executor.add_node(node=ktp_interface_node);
        # executor_thread = threading.Thread(target=multi_threaded_executor.spin, daemon=True)
        # executor_thread.start();
        
        # multi_threaded_executor2: MultiThreadedExecutor = MultiThreadedExecutor();
        # multi_threaded_executor2.add_node(node=ktp_interface_tcp_node);
        # executor_thread2 = threading.Thread(target=multi_threaded_executor2.spin, daemon=True)
        # executor_thread2.start();
        
        # rate = ktp_interface_node.create_rate(frequency=2);
        # try:
        #     while rclpy.ok():
        #         rate.sleep();
        # except KeyboardInterrupt:
        #     pass;
        
        # rclpy.shutdown();
        # executor_thread.join();
        # executor_thread2.join();
        multi_threaded_executor.spin();
    except ROSInterruptException as rie:
        ktp_interface_node.get_logger().warn(f"===== {ktp_interface_node_name} terminated with Ctrl-C {rie} =====");
        tcp_release();

    # ktp_interface_tcp_node.destroy_node();
    ktp_interface_node.destroy_node();
    rclpy.shutdown();


if __name__ == "__main__":
    main();


__all__ : list[str]= ["main"];