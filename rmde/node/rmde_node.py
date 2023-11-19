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
    
    def __init__(self) -> None:
        self.__node_name: str = 'rmde'
        self.__rclpy_flag: str = 'RCLPY'
        self.__mqtt_client: Client = Client()
        
        super().__init__(self.__node_name)
        self.get_logger().info('===== {} [{}] created ====='.format(self.__rclpy_flag, self.__node_name))
        
        self.__event_response_handler: EventResponseHandler = EventResponseHandler(self, self.__mqtt_client)
        self.__location_response_handler: LocationResponseHandler = LocationResponseHandler(self, self.__mqtt_client)
        
        self.__config_request_handler: ConfigRequestHandler = ConfigRequestHandler(self, self.__mqtt_client)
        self.__control_request_handler: ControlRequestHandler = ControlRequestHandler(self, self.__mqtt_client)
        self.__path_request_handler: PathRequestHandler = PathRequestHandler(self, self.__mqtt_client)
        
        __rclpy_timer_loop: float = 1.0
        self.create_timer(__rclpy_timer_loop, self.__from_uvc_to_rms)
        
        self.__from_rms_to_uvc()
    
    
    def __from_uvc_to_rms(self) -> None:
        self.__location_response_handler.response_to_uvc()
        self.__event_response_handler.response_to_uvc()
        
    
    def __from_rms_to_uvc(self) -> None:
        self.__path_request_handler.request_to_uvc()
        self.__control_request_handler.request_to_uvc()
        self.__config_request_handler.request_to_uvc()
        


__all__ = ['rmde_node']