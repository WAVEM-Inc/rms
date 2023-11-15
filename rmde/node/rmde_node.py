import rclpy
import paho.mqtt.client as mqtt

from rclpy.node import Node
from ..mqtt.mqtt_client import Client

from ..rms.request.config.handler import ConfigRequestHandler
from ..rms.request.control.handler import ControlRequestHandler
from ..rms.request.path.handler import PathRequestHandler

from ..rms.response.event.handler import EventResponseHandler
from ..rms.response.location.handler import LocationResponseHandler


class RMDENode(Node):
    node_name: str = 'rmde'
    rclpy_flag: str = 'RCLPY'
    mqtt_flag: str = 'MQTT'
    mqtt_client: Client = Client()
    
    def __init__(self) -> None:
        super().__init__(self.node_name)
        self.get_logger().info('===== {} [{}] created ====='.format(self.rclpy_flag, self.node_name))
        
        self.event_response_handler: EventResponseHandler = EventResponseHandler(self, self.mqtt_client)
        self.location_response_handler: LocationResponseHandler = LocationResponseHandler(self, self.mqtt_client)
        
        self.evn_config_request_handler: ConfigRequestHandler = ConfigRequestHandler(self, self.mqtt_client)
        self.control_request_handler: ControlRequestHandler = ControlRequestHandler(self, self.mqtt_client)
        self.path_request_handler: PathRequestHandler = PathRequestHandler(self, self.mqtt_client)
        
        rclpy_timer_loop: float = 1.0
        self.create_timer(rclpy_timer_loop, self.__from_uvc_to_rms__)
        
        self.__from_rms_to_uvc__()
    
    
    def __from_uvc_to_rms__(self) -> None:
        self.location_response_handler.response_to_uvc()
        self.event_response_handler.response_to_uvc()
        
    
    def __from_rms_to_uvc__(self) -> None:
        self.path_request_handler.request_to_uvc()
        self.control_request_handler.request_to_uvc()
        self.evn_config_request_handler.request_to_uvc()
        


__all__ = ['rmde_node']