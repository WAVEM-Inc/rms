import rclpy
import paho.mqtt.client as mqtt

from rclpy.node import Node
from mqtt import mqtt_client

from rms.response.location.handler.location_response_handler import LocationResponseHandler
from rms.response.event.handler.event_response_handler import EventResponseHandler

from rms.request.path.handler.path_request_handler import PathRequestHandler
from rms.request.control.handler.control_request_handler import ControlRequestHandler
from rms.request.env_config.handler.env_config_request_handler import EnvConfigRequestHandler


class rmde_node(Node):
    node_name: str = 'rmde'
    rclpy_flag: str = 'RCLPY'
    mqtt_flag: str = 'MQTT'
    mqtt_broker: mqtt_client.Client = mqtt_client.Client()
    
    
    def __init__(self) -> None:
        super().__init__(self.node_name)
        self.get_logger().info('===== {} [{}] created ====='.format(self.rclpy_flag, self.node_name))
        
        self.location_response_handler: LocationResponseHandler = LocationResponseHandler(self, self.mqtt_broker)
        self.event_response_handler: EventResponseHandler = EventResponseHandler(self, self.mqtt_broker)
        
        self.path_request_handler: PathRequestHandler = PathRequestHandler(self, self.mqtt_broker)
        self.control_request_handler: ControlRequestHandler = ControlRequestHandler(self, self.mqtt_broker)
        self.evn_config_request_handler: EnvConfigRequestHandler = EnvConfigRequestHandler(self, self.mqtt_broker)
        
        rclpy_timer_loop: float = 1.0
        self.create_timer(rclpy_timer_loop, self.__from_uvc_to_rms__)
        
        self.__from_rms_to_uvc__()
    
    
    def __from_uvc_to_rms__(self) -> None:
        self.location_response_handler.response_to_uvc()
        
    
    def __from_rms_to_uvc__(self) -> None:
        self.path_request_handler.request_to_uvc()
        self.control_request_handler.request_to_uvc()
        self.evn_config_request_handler.request_to_uvc()
        


__all__ = ["rmde_node"]