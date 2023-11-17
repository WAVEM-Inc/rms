import os
import json
import paho.mqtt.client as mqtt

from rclpy.node import Node
from configparser import ConfigParser

from ....mqtt import mqtt_client
from ...common.service import ConfigService
from ...common.domain import Header

from .domain import Config
from .domain import SetInfo

from typing import Any
from typing import Dict


class ConfigRequestHandler():
    rclpy_flag: str = 'RCLPY'
    mqtt_flag: str = 'MQTT'
    
        
    def __init__(self, rclpy_node: Node, mqtt_broker: mqtt_client.Client) -> None:
        self.rclpy_node: Node = rclpy_node
        self.mqtt_broker: mqtt_client.Client = mqtt_broker
        
        self.script_directory: str = os.path.dirname(os.path.abspath(__file__))
        self.mqtt_config_file_path: str = '../../../mqtt/mqtt.ini'
        self.mqtt_config_service: ConfigService = ConfigService(self.script_directory, self.mqtt_config_file_path)
        self.mqtt_config_parser: ConfigParser = self.mqtt_config_service.read()
        self.mqtt_config_subscription_topic: str = self.mqtt_config_parser.get('topics', 'config')
        self.mqtt_config_subscription_qos: int = int(self.mqtt_config_parser.get('qos', 'config'))

        self.common_config_file_path: str = '../../common/config.ini'
        self.common_config_service: ConfigService = ConfigService(self.script_directory, self.common_config_file_path)
        self.common_config_parser: ConfigParser = self.common_config_service.read()
        
        self.env_config: Config = Config()
        self.header: Header = Header()
        self.set_info: SetInfo = SetInfo()
        
    
    def request_to_uvc(self) -> None:
        def mqtt_config_subscription_cb(mqtt_client: mqtt.Client, mqtt_user_data: Dict, mqtt_message: mqtt.MQTTMessage) -> None:
            mqtt_topic: str = mqtt_message.topic
            mqtt_decoded_payload: str = mqtt_message.payload.decode()
            mqtt_json: Any = json.loads(mqtt_message.payload)
            
            self.rclpy_node.get_logger().info('{} subscription cb payload [{}] from [{}]'.format(self.mqtt_flag, mqtt_decoded_payload, mqtt_topic))
            self.rclpy_node.get_logger().info('{} subscription cb json [{}] from [{}]'.format(self.mqtt_flag, mqtt_json, mqtt_topic))
            
            self.env_config = Config(
                header = mqtt_json['header'],
                setInfo = mqtt_json['setInfo']
            )
            
            self.set_info = SetInfo(
                robotType = self.env_config.setInfo['robotType'],
                mqttIP = self.env_config.setInfo['mqttIP'],
                mqttPort = self.env_config.setInfo['mqttPort'],
                robotCorpId = self.env_config.setInfo['robotCorpId'],
                robotId = self.env_config.setInfo['robotId'],
                workCorpId = self.env_config.setInfo['workCorpId'],
                workSiteId = self.env_config.setInfo['workSiteId'],
                batteryEvent = self.env_config.setInfo['batteryEvent']
            )

            self.common_config_parser.set('header', 'robotType', self.set_info.robotType)
            self.common_config_parser.set('header', 'robotCorpId', self.set_info.robotCorpId)
            self.common_config_parser.set('header', 'robotId', self.set_info.robotId)
            self.common_config_parser.set('header', 'workCorpId', self.set_info.workCorpId)
            self.common_config_parser.set('header', 'workSiteId', self.set_info.workSiteId)
            
            self.mqtt_config_parser.set('broker', 'host', self.set_info.mqttIP)
            self.mqtt_config_parser.set('broker', 'port', str(self.set_info.mqttPort))

            mqtt_topic_to_rms_format: str = 'hubilon/atcplus/ros'
            modified_location_topic: str = f'{mqtt_topic_to_rms_format}/{self.set_info.robotCorpId}/{self.set_info.robotId}/location'
            modified_event_topic: str = f'{mqtt_topic_to_rms_format}/{self.set_info.robotCorpId}/{self.set_info.robotId}/event'

            self.mqtt_config_parser.set('topics', 'location', modified_location_topic)
            self.mqtt_config_parser.set('topics', 'event', modified_event_topic)

            mqtt_topic_to_ros_format: str = 'hubilon/atcplus/rms'
            modified_path_topic: str = f'{mqtt_topic_to_ros_format}/{self.set_info.robotCorpId}/{self.set_info.robotId}/path'
            modified_control_topic: str = f'{mqtt_topic_to_ros_format}/{self.set_info.robotCorpId}/{self.set_info.robotId}/control'
            modified_config_topic: str = f'{mqtt_topic_to_ros_format}/{self.set_info.robotCorpId}/{self.set_info.robotId}/config'

            self.mqtt_config_parser.set('topics', 'path', modified_path_topic)
            self.mqtt_config_parser.set('topics', 'control', modified_control_topic)
            self.mqtt_config_parser.set('topics', 'config', modified_config_topic)

            with open(self.common_config_service.__config_file_path__, 'w') as common_config_file:
                self.common_config_parser.write(common_config_file)
                self.rclpy_node.get_logger().info('===== Common Configuration has been changed =====')
            
            with open(self.mqtt_config_service.__config_file_path__, 'w') as mqtt_config_file:
                self.mqtt_config_parser.write(mqtt_config_file)
                self.rclpy_node.get_logger().info('===== MQTT Configuration has been changed with IP [%s] reboot required =====' % self.set_info.mqttIP)
                self.rclpy_node.destroy_node()
                exit(0)
            
        self.mqtt_broker.subscribe(topic = self.mqtt_config_subscription_topic, qos = self.mqtt_config_subscription_qos)
        self.mqtt_broker.client.message_callback_add(self.mqtt_config_subscription_topic, mqtt_config_subscription_cb)


__all__ = ['config_request_handler']