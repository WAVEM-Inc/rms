import os
import json
import paho.mqtt.client as mqtt

from configparser import ConfigParser
from rclpy.node import Node

from ....mqtt.mqtt_client import Client

from ...common.domain import Header
from ...common.service import ConfigService
from ...common.service import UUIDService

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
        self.__common_config_file_path: str = '../../common/config.ini'
        self.__common_config_service: ConfigService = ConfigService(self.__script_directory, self.__common_config_file_path)
        self.__common_config_parser: ConfigParser = self.__common_config_service.read()
        
        self.__uuid_service: UUIDService = UUIDService()
        
        self.__header: Header = Header()
            
    def __build_header(self) -> None:
        robot_corp_id: str = self.__common_config_parser.get(
            'header', 'robotCorpId')
        self.__header.robotCorpId = robot_corp_id

        work_corp_id: str = self.__common_config_parser.get(
            'header', 'workCorpId')
        self.__header.workCorpId = work_corp_id

        work_site_id: str = self.__common_config_parser.get(
            'header', 'workSiteId')
        self.__header.workSiteId = work_site_id

        robot_id: str = self.__common_config_parser.get('header', 'robotId')
        self.__header.robotId = robot_id

        robot_type: str = self.__common_config_parser.get(
            'header', 'robotType')
        self.__header.robotType = robot_type
        
        self.__header.topicUid = self.__uuid_service.generate_uuid()
    
    def response_to_rms(self, cmd_response: CmdResponse) -> None:
        built_cmd_response: CmdResponse = cmd_response
        self.__build_header()
        built_cmd_response.header = self.__header.__dict__
        self.__mqtt_client.publish(topic=self.__mqtt_cmd_response_publisher_topic, payload=json.dumps(
            built_cmd_response.__dict__), qos=self.__mqtt_cmd_repsonse_publisher_qos)


__all__ = ['rms_response_cmd_repsonse_handler']
