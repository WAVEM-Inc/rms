import os
import json
import paho.mqtt.client as mqtt

from rclpy.node import Node
from configparser import ConfigParser

from ....mqtt.mqtt_client import Client
from ...common.service import ConfigService

from .domain import Config
from .domain import SetInfo

from typing import Any
from typing import Dict


RCLPY_FLAG: str = 'RCLPY'
MQTT_FLAG: str = 'MQTT'


class ConfigRequestHandler():
    def __init__(self, rclpy_node: Node, mqtt_client: Client) -> None:
        self.__rclpy_node: Node = rclpy_node
        self.__mqtt_client: Client = mqtt_client
        
        self.__script_directory: str = os.path.dirname(os.path.abspath(__file__))
        self.__mqtt_config_file_path: str = '../../../mqtt/mqtt.ini'
        self.__mqtt_config_service: ConfigService = ConfigService(self.__script_directory, self.__mqtt_config_file_path)
        self.__mqtt_config_parser: ConfigParser = self.__mqtt_config_service.read()
        self.__mqtt_config_subscription_topic: str = self.__mqtt_config_parser.get('topics', 'config')
        self.__mqtt_config_subscription_qos: int = int(self.__mqtt_config_parser.get('qos', 'config'))

        self.__common_config_file_path: str = '../../common/config.ini'
        self.__common_config_service: ConfigService = ConfigService(self.__script_directory, self.__common_config_file_path)
        self.__common_config_parser: ConfigParser = self.__common_config_service.read()
        
        self.__config: Config = Config()
        self.__set_info: SetInfo = SetInfo()
        
    
    def request_to_uvc(self) -> None:
        def __mqtt_config_subscription_cb(mqtt_client: mqtt.Client, mqtt_user_data: Dict, mqtt_message: mqtt.MQTTMessage) -> None:
            try:
                __mqtt_topic: str = mqtt_message.topic
                __mqtt_decoded_payload: str = mqtt_message.payload.decode()
                __mqtt_json: Any = json.loads(mqtt_message.payload)
                
                self.__rclpy_node.get_logger().info('{} subscription cb payload [{}] from [{}]'.format(MQTT_FLAG, __mqtt_decoded_payload, __mqtt_topic))
                self.__rclpy_node.get_logger().info('{} subscription cb json [{}] from [{}]'.format(MQTT_FLAG, __mqtt_json, __mqtt_topic))
                
                __header_dict: dict = __mqtt_json['header']
                self.__config.header = __header_dict

                __setInfo_dict: dict = __mqtt_json['setInfo']            
                self.__config.setInfo = __setInfo_dict

                __robotType: str = self.__config.setInfo['robotType']
                self.__set_info.robotType = __robotType

                __mqttIP: str = self.__config.setInfo['mqttIP']
                self.__set_info.mqttIP = __mqttIP

                __mqttPort: str = self.__config.setInfo['mqttPort']
                self.__set_info.mqttPort = __mqttPort

                __robotCorpId: str = self.__config.setInfo['robotCorpId']
                self.__set_info.robotCorpId = __robotCorpId

                __robotId: str = self.__config.setInfo['robotId']
                self.__set_info.robotId = __robotId

                __workCorpId: str = self.__config.setInfo['workCorpId']
                self.__set_info.workCorpId = __workCorpId

                __workSiteId: str = self.__config.setInfo['workSiteId']
                self.__set_info.workSiteId = __workSiteId

                __batteryEvent: str = self.__config.setInfo['batteryEvent']
                self.__set_info.batteryEvent = __batteryEvent

                self.__common_config_parser.set('header', 'robotType', self.__set_info.robotType)
                self.__common_config_parser.set('header', 'robotCorpId', self.__set_info.robotCorpId)
                self.__common_config_parser.set('header', 'robotId', self.__set_info.robotId)
                self.__common_config_parser.set('header', 'workCorpId', self.__set_info.workCorpId)
                self.__common_config_parser.set('header', 'workSiteId', self.__set_info.workSiteId)
                
                self.__mqtt_config_parser.set('broker', 'host', self.__set_info.mqttIP)
                self.__mqtt_config_parser.set('broker', 'port', self.__set_info.mqttPort)

                __mqtt_topic_to_rms_format: str = 'hubilon/atcplus/ros'
                __modified_location_topic: str = f'{__mqtt_topic_to_rms_format}/{self.__set_info.robotCorpId}/{self.__set_info.robotId}/location'
                __modified_event_topic: str = f'{__mqtt_topic_to_rms_format}/{self.__set_info.robotCorpId}/{self.__set_info.robotId}/event'

                self.__mqtt_config_parser.set('topics', 'location', __modified_location_topic)
                self.__mqtt_config_parser.set('topics', 'event', __modified_event_topic)

                __mqtt_topic_to_ros_format: str = 'hubilon/atcplus/rms'
                __modified_path_topic: str = f'{__mqtt_topic_to_ros_format}/{self.__set_info.robotCorpId}/{self.__set_info.robotId}/path'
                __modified_control_topic: str = f'{__mqtt_topic_to_ros_format}/{self.__set_info.robotCorpId}/{self.__set_info.robotId}/control'
                __modified_config_topic: str = f'{__mqtt_topic_to_ros_format}/{self.__set_info.robotCorpId}/{self.__set_info.robotId}/config'

                self.__mqtt_config_parser.set('topics', 'path', __modified_path_topic)
                self.__mqtt_config_parser.set('topics', 'control', __modified_control_topic)
                self.__mqtt_config_parser.set('topics', 'config', __modified_config_topic)

                with open(self.__common_config_service.config_file_path, 'w') as common_config_file:
                    self.__common_config_parser.write(common_config_file)
                    self.__rclpy_node.get_logger().info('===== Common Configuration has been changed =====')
                
                with open(self.__mqtt_config_service.config_file_path, 'w') as mqtt_config_file:
                    self.__mqtt_config_parser.write(mqtt_config_file)
                    self.__rclpy_node.get_logger().info('===== MQTT Configuration has been changed with IP [%s] reboot required =====' % self.__set_info.mqttIP)
                    self.__rclpy_node.destroy_node()

            except KeyError as ke:
                self.__rclpy_node.get_logger().error(f'Invalid JSON Key in MQTT {self.__mqtt_config_subscription_topic} subscription callback: {ke}')

            except json.JSONDecodeError as jde:
                self.__rclpy_node.get_logger().error(f'Invalid JSON format in MQTT {self.__mqtt_config_subscription_topic} subscription callback: {jde.msg}')

            except Exception as e:
                self.__rclpy_node.get_logger().error(f'Exception in MQTT {self.__mqtt_config_subscription_topic} subscription callback: {e}')
                raise
                
            
        self.__mqtt_client.subscribe(topic = self.__mqtt_config_subscription_topic, qos = self.__mqtt_config_subscription_qos)
        self.__mqtt_client.client.message_callback_add(self.__mqtt_config_subscription_topic, __mqtt_config_subscription_cb)


__all__ = ['rms_request_config_handler']