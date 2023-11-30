import time
import threading

from rclpy.node import Node
from rclpy.timer import Timer
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from ..mqtt.mqtt_client import Client

from ..rms.request.config.handler import ConfigRequestHandler
from ..rms.request.control.handler import ControlRequestHandler
from ..rms.request.path.handler import PathRequestHandler

from ..rms.response.task_event.handler import TaskEventResponseHandler
from ..rms.response.control_event.handler import ControlEventHandler
from ..rms.response.location.handler import LocationResponseHandler

from ..rms.common.service import RobotTypeService


MQTT_THREAD_INTERVAL: int = 60
MQTT_RETRY_INTERVAL: int = 1
RCLPY_NODE_NAME: str = 'rmde'
RCLPY_FLAG: str = 'RCLPY'


class RMDENode(Node):
    
    def __init__(self) -> None:
        super().__init__(RCLPY_NODE_NAME)
        self.mqtt_client: Client = Client()
        self.is_broker_opened: bool = self.mqtt_client.check_broker_opened()
        
        if self.is_broker_opened:
            self.get_logger().info(f'MQTT Broker is opened with [{self.mqtt_client.broker_address}:{str(self.mqtt_client.broker_port)}]')
            self.mqtt_client.connect()
            self.mqtt_client.run()
        else:
            retries: int = 0
            while not self.is_broker_opened:
                self.get_logger().error(f'MQTT Broker is not opened yet.. retrying [{str(retries)}]')
                time.sleep(MQTT_RETRY_INTERVAL)
                retries += 1
                
        self.__start_mqtt_thread()
        
        self.get_logger().info(f'{RCLPY_FLAG} [{RCLPY_NODE_NAME}] node has been created')
        
        self.__location_response_handler: LocationResponseHandler = LocationResponseHandler(rclpy_node = self, mqtt_client = self.mqtt_client)
        self.__task_event_response_handler: TaskEventResponseHandler = TaskEventResponseHandler(rclpy_node = self, mqtt_client = self.mqtt_client)
        self.__control_event_repsonse_handler: ControlEventHandler = ControlEventHandler(rclpy_node = self, mqtt_client = self.mqtt_client)
        
        self.__robot_type_service: RobotTypeService = RobotTypeService(rclpy_node = self)
        
        self.__config_request_handler: ConfigRequestHandler = ConfigRequestHandler(rclpy_node = self, mqtt_client = self.mqtt_client)
        self.__control_request_handler: ControlRequestHandler = ControlRequestHandler(rclpy_node = self, mqtt_client = self.mqtt_client)
        self.__path_request_handler: PathRequestHandler = PathRequestHandler(rclpy_node = self, mqtt_client = self.mqtt_client)
        
        __rclpy_main_timer_period_sec: float = 1.0
        __rclpy_main_timer_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup()
        __rclpy_main_timer: Timer = self.create_timer(
            timer_period_sec = __rclpy_main_timer_period_sec,
            callback_group = __rclpy_main_timer_cb_group,
            callback = self.__from_uvc_to_rms
        )
        
        self.__from_rms_to_uvc()
    
    
    def __check_mqtt_and_reconnect(self) -> None:
        retries: int = 0
        while True:
            if not self.is_broker_opened:
                self.get_logger().error(f'MQTT Broker is not opened yet.. retrying [{str(retries)}]')
                self.mqtt_client.connect()
                self.mqtt_client.run()
                if self.mqtt_client.is_connected:
                    break
            retries += 1
            time.sleep(MQTT_RETRY_INTERVAL)
    
    
    def __start_mqtt_thread(self) -> None:
        mqtt_thread: threading.Thread = threading.Thread(target = self.__check_mqtt_and_reconnect)
        mqtt_thread.daemon = True
        mqtt_thread.start()
            
    
    def __from_uvc_to_rms(self) -> None:
        if (self.mqtt_client.is_connected):
            self.__location_response_handler.response_to_uvc()
            self.__control_event_repsonse_handler.response_to_uvc()
            self.__robot_type_service.select_current_robot_type()
        else:
            return
        
    
    def __from_rms_to_uvc(self) -> None:
        if (self.mqtt_client.is_connected):
            self.__path_request_handler.request_to_uvc()
            self.__control_request_handler.request_to_uvc()
            self.__config_request_handler.request_to_uvc()
        else:
            return


__all__ = ['rmde_node']