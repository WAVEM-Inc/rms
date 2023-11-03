import os
import json
import configparser
import paho.mqtt.client as mqtt

from rclpy.node import Node

from ....mqtt import mqtt_client

from .domain import Config
from .domain import SetInfo

from typing import Any
from typing import Dict


class ConfigRequestHandler():
    rclpy_flag: str = 'RCLPY'
    mqtt_flag: str = 'MQTT'
    
    mqtt_path_subscription_topic: str = 'hubilon/atcplus/rms/rco0000000/rbt00000000/config'
    
    
    def __init__(self, rclpy_node: Node, mqtt_broker: mqtt_client.Client) -> None:
        self.rclpy_node: Node = rclpy_node
        self.mqtt_broker: mqtt_client.Client = mqtt_broker
        self.env_config: Config = Config()
        self.set_info: SetInfo = SetInfo()
        
        self.config_parser: configparser.ConfigParser = configparser.ConfigParser()
        self.script_directory: str = os.path.dirname(os.path.abspath(__file__))
        self.config_file_path: str = os.path.join(self.script_directory, '../../../mqtt/mqtt.ini')
        self.config_parser.read(self.config_file_path)
        
    
    def request_to_uvc(self) -> None:
        def mqtt_path_subscription_cb(mqtt_client: mqtt.Client, mqtt_user_data: Dict, mqtt_message: mqtt.MQTTMessage) -> None:
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
                workCorpId = self.env_config.setInfo['workCorpId'],
                workSiteId = self.env_config.setInfo['workSiteId'],
                batteryEvent = self.env_config.setInfo['batteryEvent']
            )
            
            self.config_parser.set('broker', 'host', self.set_info.mqttIP)
            self.config_parser.set('broker', 'port', str(self.set_info.mqttPort))
            
            with open(self.config_file_path, 'w') as mqtt_config_file:
                self.config_parser.write(mqtt_config_file)
                self.rclpy_node.get_logger().info('===== MQTT Broker has been changed with [%s] reboot required =====' % self.set_info.mqttIP)
                self.rclpy_node.destroy_node()
                exit(0)
            
        self.mqtt_broker.subscribe(topic = self.mqtt_path_subscription_topic, qos = 0)
        self.mqtt_broker.client.message_callback_add(self.mqtt_path_subscription_topic, mqtt_path_subscription_cb)




__all__ = ['config_request_handler']