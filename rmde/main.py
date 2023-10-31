import rclpy

from rclpy.node import Node
from rclpy.exceptions import ROSInterruptException
from .node.rmde_node import RMDENode
 

def main(args=None) -> None:
    rclpy.init(args=args)

    try:
        node: Node = RMDENode()
        rclpy.spin(node)
    except ROSInterruptException:
        node.get_logger().warn('===== {} [{}] terminated with Ctrl-C ====='.format(node.rclpy_flag, node.node_name))
        node.mqtt_client.client.disconnect()
        node.mqtt_client.client.loop_stop()
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()