import rclpy

from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.exceptions import ROSInterruptException
from .node.rmde_node import RMDENode
 

def main(args=None) -> None:
    rclpy.init(args=args)

    try:
        __rclpy_node: Node = RMDENode()
        __rclpy_multi_threaded_executor: MultiThreadedExecutor = MultiThreadedExecutor()
        __rclpy_multi_threaded_executor.add_node(node = __rclpy_node)
        __rclpy_multi_threaded_executor.spin()
    except ROSInterruptException:
        __rclpy_node.get_logger().warn('===== {} [{}] terminated with Ctrl-C ====='.format(__rclpy_node.__rclpy_flag, __rclpy_node.__node_name))
        __rclpy_node.mqtt_client.disconnect()
        __rclpy_node.mqtt_client.loop_stop()
    
    __rclpy_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()