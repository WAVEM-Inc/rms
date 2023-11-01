import rclpy

from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.exceptions import ROSInterruptException
from .node.rmde_node import RMDENode
 

def main(args=None) -> None:
    rclpy.init(args=args)

    try:
        rclpy_node: Node = RMDENode()
        rclpy_multi_threaded_executor: MultiThreadedExecutor = MultiThreadedExecutor()
        rclpy_multi_threaded_executor.add_node(node = rclpy_node)
        rclpy_multi_threaded_executor.spin()
    except ROSInterruptException:
        rclpy_node.get_logger().warn('===== {} [{}] terminated with Ctrl-C ====='.format(rclpy_node.rclpy_flag, rclpy_node.node_name))
        rclpy_node.mqtt_client.client.disconnect()
        rclpy_node.mqtt_client.client.loop_stop()
    
    rclpy_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()