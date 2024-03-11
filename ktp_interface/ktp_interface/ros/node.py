import rclpy

from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rosbridge_library.internal import message_conversion

from std_msgs.msg import String

from ..tcp.application.service import TCPService

NODE_NAME: str = "ktp_interface"

class KTPInterface(Node):
    
    def __init__(self) -> None:
        super().__init__(NODE_NAME);
        
        self.get_logger().info(f"{NODE_NAME} created");
        
        self.tcp_service: TCPService = TCPService(node=self);
        self.tcp_service.initialize();
        
        self.create_subscription(
            msg_type=String,
            topic="/chatter",
            qos_profile=qos_profile_system_default,
            callback_group=MutuallyExclusiveCallbackGroup(),
            callback=self.test_cb
        );
        
    def test_cb(self, cb: String) -> None:
        self.tcp_service.send_resource(resource_id="rbt_service_status", properties="hhhiqwer");

__all__ = ["ktp_interface.ros.node"];