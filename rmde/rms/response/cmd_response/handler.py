import os
import json
import paho.mqtt.client as mqtt

from configparser import ConfigParser
from rclpy.node import Node

from ....mqtt.mqtt_client import Client
from ...common.service import ConfigService

from .domain import CmdResponse


class CmdRepsonseHandler():

    def __init__(self, rclpy_node: Node, mqtt_client: Client) -> None:
        self.__rclpy_node: Node = rclpy_node
        self.__mqtt_client: Client = mqtt_client

        self.__script_directory: str = os.path.dirname(
            os.path.abspath(__file__))
        self.__mqtt_config_file_path: str = '../../../mqtt/mqtt.ini'
        self.__mqtt_config_service: ConfigService = ConfigService(
            self.__script_directory, self.__mqtt_config_file_path)
        self.__mqtt_config_parser: ConfigParser = self.__mqtt_config_service.read()

        self.__mqtt_cmd_response_publisher_topic: str = self.__mqtt_config_parser.get(
            'topics', 'cmd_response')
        self.__mqtt_cmd_repsonse_publisher_qos: int = int(
            self.__mqtt_config_parser.get('qos', 'cmd_response'))

        if self.__mqtt_client.is_connected:
            self.__rclpy_node.get_logger().info(
                f'MQTT granted publisher\n\ttopic : {self.__mqtt_cmd_response_publisher_topic}\n\tqos : {self.__mqtt_cmd_repsonse_publisher_qos}')
        else:
            self.__rclpy_node.get_logger().error(
                f'MQTT failed to grant publisher\n\ttopic : {self.__mqtt_cmd_response_publisher_topic}\n\tqos : {self.__mqtt_cmd_response_publisher_topic}')

    def response_to_rms(self, cmd_response: CmdResponse) -> None:
        built_cmd_response: CmdResponse = cmd_response
        self.__mqtt_client.publish(topic=self.__mqtt_cmd_response_publisher_topic, payload=json.dumps(
            built_cmd_response.__dict__), qos=self.__mqtt_cmd_repsonse_publisher_qos)


__all__ = ['rms_response_cmd_repsonse_handler']
