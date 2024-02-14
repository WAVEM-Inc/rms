from rclpy.node import Node

from ..mqtt.client import Client

NODE_NAME: str = "rcs_mqtt_bridge"

class RcsMQTTBridge(Node):
    
    def __init__(self) -> None:
        super().__init__(NODE_NAME);
        
        self.get_logger().info(f"{NODE_NAME} created");
        
        self.__mqtt_client: Client = Client(self);
        
        