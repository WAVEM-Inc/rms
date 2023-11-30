import os
import json
import paho.mqtt.client as mqtt

from configparser import ConfigParser
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.qos import qos_profile_system_default
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from gts_navigation_msgs.msg import NavigationControl

from ....mqtt.mqtt_client import Client
from ...common.service import ConfigService

from .domain import Control
from .domain import ControlCmd

from typing import Any
from typing import Dict


RCLPY_FLAG: str = 'RCLPY'
MQTT_FLAG: str = 'MQTT'


class ControlRequestHandler():
    
    def __init__(self, rclpy_node: Node, mqtt_client: Client) -> None:
        self.__rclpy_node: Node = rclpy_node
        self.__mqtt_client: Client = mqtt_client

        self.__script_directory: str = os.path.dirname(os.path.abspath(__file__))
        self.__mqtt_config_file_path: str = '../../../mqtt/mqtt.ini'
        self.__mqtt_config_service: ConfigService = ConfigService(self.__script_directory, self.__mqtt_config_file_path)
        self.__mqtt_config_parser: ConfigParser = self.__mqtt_config_service.read()
        
        self.__mqtt_control_subscription_topic: str = self.__mqtt_config_parser.get('topics', 'control')
        self.__mqtt_control_subscription_qos: int = int(self.__mqtt_config_parser.get('qos', 'control'))
        
        self.__rclpy_gts_navigation_control_publisher_topic: str = '/gts_navigation/control'
        self.__rclpy_gts_navigation_control_publisher_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup()
        self.__rclpy_gts_navigation_control_publisher: Publisher = self.__rclpy_node.create_publisher(
            msg_type = NavigationControl,
            topic = self.__rclpy_gts_navigation_control_publisher_topic,
            qos_profile = qos_profile_system_default,
            callback_group = self.__rclpy_gts_navigation_control_publisher_cb_group
        )
        
        self.__control: Control = Control()
        self.__control_cmd: ControlCmd = ControlCmd()
        
    
    def request_to_uvc(self) -> None:
        def __mqtt_control_subscription_cb(mqtt_client: mqtt.Client, mqtt_user_data: Dict, mqtt_message: mqtt.MQTTMessage) -> None:
            try:
                mqtt_topic: str = mqtt_message.topic
                mqtt_decoded_payload: str = mqtt_message.payload.decode()
                mqtt_json: Any = json.loads(mqtt_message.payload)
                
                self.__rclpy_node.get_logger().info(f'{MQTT_FLAG} subscription cb payload [{mqtt_decoded_payload}] from [{mqtt_topic}]')
                self.__rclpy_node.get_logger().info(f'{MQTT_FLAG} subscription cb json [{mqtt_decoded_payload}] from [{mqtt_topic}]')

                header_dict: dict = mqtt_json['header']
                self.__control.header = header_dict

                control_cmd_dict: dict = mqtt_json['controlCmd']
                self.__control.controlCmd = control_cmd_dict
                
                ready: bool = self.__control.controlCmd['ready']
                self.__control_cmd.ready = ready

                move: bool = self.__control.controlCmd['move']
                self.__control_cmd.move = move

                stop: bool = self.__control.controlCmd['stop']
                self.__control_cmd.stop = stop
                
                self.__judge_control_cmd()

            except KeyError as ke:
                self.__rclpy_node.get_logger().error(f'Invalid JSON Key in MQTT {self.__mqtt_control_subscription_topic} subscription callback: {ke}')

            except json.JSONDecodeError as jde:
                self.__rclpy_node.get_logger().error(f'Invalid JSON format in MQTT {self.__mqtt_control_subscription_topic} subscription callback: {jde.msg}')

            except Exception as e:
                self.__rclpy_node.get_logger().error(f'Exception in MQTT {self.__mqtt_control_subscription_topic} subscription callback: {e}')
                raise
            
        self.__mqtt_client.subscribe(topic = self.__mqtt_control_subscription_topic, qos = self.__mqtt_control_subscription_qos)
        self.__mqtt_client.client.message_callback_add(sub = self.__mqtt_control_subscription_topic, callback = __mqtt_control_subscription_cb)
        

    def __judge_control_cmd(self) -> None:
        is_cmd_ready: bool = (self.__control_cmd.ready == True)
        is_cmd_move: bool = (self.__control_cmd.move == True)
        is_cmd_stop: bool = (self.__control_cmd.stop == True)
            
        if (is_cmd_ready and is_cmd_move and is_cmd_stop):
            return
        elif (is_cmd_ready and is_cmd_move):
            return
        elif (is_cmd_move and is_cmd_stop):
            return
        elif (is_cmd_ready and is_cmd_stop):
            return
        elif (is_cmd_stop and not is_cmd_ready and not is_cmd_move):
            self.__rclpy_node.get_logger().info('judge gts_navigation/control command is stop')
            
            rclpy_gts_navigation_control: NavigationControl = NavigationControl()
            rclpy_gts_navigation_control.cancel_navigation = True
            rclpy_gts_navigation_control.resume_navigation = False
            
            self.__rclpy_gts_navigation_control_publisher.publish(rclpy_gts_navigation_control)
        elif (is_cmd_move and not is_cmd_stop and not is_cmd_ready):
            self.__rclpy_node.get_logger().info('judge gts_navigation/control command is move')
            
            rclpy_gts_navigation_control: NavigationControl = NavigationControl()
            rclpy_gts_navigation_control.cancel_navigation = False
            rclpy_gts_navigation_control.resume_navigation = True
            
            self.__rclpy_gts_navigation_control_publisher.publish(rclpy_gts_navigation_control)
        else: return
            

__all__ = ['rms_request_control_handler']