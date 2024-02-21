import rclpy

from rclpy.node import Node

from ..tcp.application.service import TCPService

NODE_NAME: str = "ktp_interface"

class KTPInterface(Node):
    
    def __init__(self) -> None:
        super().__init__(NODE_NAME);
        
        self.get_logger().info(f"{NODE_NAME} created");
        
        tcp_service: TCPService = TCPService(node=self);
        tcp_service.initialize();
        

__all__ = ["ktp_interface.ros.node"];