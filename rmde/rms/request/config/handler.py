import os
import json
import paho.mqtt.client as mqtt

from rclpy.node import Node
from configparser import ConfigParser

from ....mqtt.mqtt_client import Client
from ...common.service import ConfigService
from ...common.service import TimeService

from .domain import Config
from .domain import SetInfo

from ...response.cmd_response.handler import CmdRepsonseHandler
from ...response.cmd_response.domain import CmdResponse
from ...response.cmd_response.domain import CmdResult
from ...response.cmd_response.domain import CmdResultTopicKindType

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

        self.__time_service: TimeService = TimeService()
        
        self.__config: Config = Config()
        self.__set_info: SetInfo = SetInfo()
        self.__cmd_repsonse_handler: CmdRepsonseHandler = CmdRepsonseHandler(rclpy_node = self.__rclpy_node, mqtt_client = self.__mqtt_client)
        
    
    def request_to_uvc(self) -> None:
        def __mqtt_config_subscription_cb(mqtt_client: mqtt.Client, mqtt_user_data: Dict, mqtt_message: mqtt.MQTTMessage) -> None:
            try:
                mqtt_topic: str = mqtt_message.topic
                mqtt_decoded_payload: str = mqtt_message.payload.decode()
                mqtt_json: Any = json.loads(mqtt_message.payload)
                
                self.__rclpy_node.get_logger().info(f'{MQTT_FLAG} subscription cb payload [{mqtt_decoded_payload}] from [{mqtt_topic}]')
                self.__rclpy_node.get_logger().info(f'{MQTT_FLAG} subscription cb json [{mqtt_decoded_payload}] from [{mqtt_topic}]')
                
                header_dict: dict = mqtt_json['header']
                self.__config.header = header_dict

                cmd_response: CmdResponse = CmdResponse()
                cmd_response.header = header_dict
                
                cmd_result: CmdResult = CmdResult()
                cmd_result.status = 'success'
                cmd_result.startTime = self.__time_service.get_current_datetime()
                cmd_result.topicKind = CmdResultTopicKindType.CONFIG.value

                cmd_response.cmdResult = cmd_result.__dict__

                self.__cmd_repsonse_handler.response_to_rms(cmd_response = cmd_response)

                set_info_dict: dict = mqtt_json['setInfo']            
                self.__config.setInfo = set_info_dict

                robot_type: str = self.__config.setInfo['robotType']
                self.__set_info.robotType = robot_type

                mqtt_ip: str = self.__config.setInfo['mqttIP']
                self.__set_info.mqttIP = mqtt_ip

                mqtt_port: str = self.__config.setInfo['mqttPort']
                self.__set_info.mqttPort = mqtt_port

                robot_corp_id: str = self.__config.setInfo['robotCorpId']
                self.__set_info.robotCorpId = robot_corp_id

                robot_id: str = self.__config.setInfo['robotId']
                self.__set_info.robotId = robot_id

                work_corp_id: str = self.__config.setInfo['workCorpId']
                self.__set_info.workCorpId = work_corp_id

                work_site_id: str = self.__config.setInfo['workSiteId']
                self.__set_info.workSiteId = work_site_id

                battery_event: str = self.__config.setInfo['batteryEvent']
                self.__set_info.batteryEvent = battery_event

                self.__common_config_parser.set('header', 'robotType', self.__set_info.robotType)
                self.__common_config_parser.set('header', 'robotCorpId', self.__set_info.robotCorpId)
                self.__common_config_parser.set('header', 'robotId', self.__set_info.robotId)
                self.__common_config_parser.set('header', 'workCorpId', self.__set_info.workCorpId)
                self.__common_config_parser.set('header', 'workSiteId', self.__set_info.workSiteId)
                
                self.__mqtt_config_parser.set('broker', 'host', self.__set_info.mqttIP)
                self.__mqtt_config_parser.set('broker', 'port', self.__set_info.mqttPort)

                mqtt_topic_to_rms_format: str = 'hubilon/atcplus/ros'
                modified_location_topic: str = f'{mqtt_topic_to_rms_format}/{self.__set_info.robotCorpId}/{self.__set_info.robotId}/location'
                modified_task_event_topic: str = f'{mqtt_topic_to_rms_format}/{self.__set_info.robotCorpId}/{self.__set_info.robotId}/task_event'
                modified_control_event_topic: str = f'{mqtt_topic_to_rms_format}/{self.__set_info.robotCorpId}/{self.__set_info.robotId}/control_event'

                self.__mqtt_config_parser.set('topics', 'location', modified_location_topic)
                self.__mqtt_config_parser.set('topics', 'task_event', modified_task_event_topic)
                self.__mqtt_config_parser.set('topics', 'control_event', modified_control_event_topic)

                mqtt_topic_to_ros_format: str = 'hubilon/atcplus/rms'
                modified_path_topic: str = f'{mqtt_topic_to_ros_format}/{self.__set_info.robotCorpId}/{self.__set_info.robotId}/path'
                modified_control_topic: str = f'{mqtt_topic_to_ros_format}/{self.__set_info.robotCorpId}/{self.__set_info.robotId}/control'
                modified_config_topic: str = f'{mqtt_topic_to_ros_format}/{self.__set_info.robotCorpId}/{self.__set_info.robotId}/config'

                self.__mqtt_config_parser.set('topics', 'path', modified_path_topic)
                self.__mqtt_config_parser.set('topics', 'control', modified_control_topic)
                self.__mqtt_config_parser.set('topics', 'config', modified_config_topic)

                with open(self.__common_config_service.config_file_path, 'w') as common_config_file:
                    self.__common_config_parser.write(common_config_file)
                    self.__rclpy_node.get_logger().info('===== Config Common Configuration has been changed =====')
                
                with open(self.__mqtt_config_service.config_file_path, 'w') as mqtt_config_file:
                    self.__mqtt_config_parser.write(mqtt_config_file)
                    self.__rclpy_node.get_logger().info(f'===== Config MQTT Configuration has been changed with IP [{self.__set_info.mqttIP}] reboot required =====')
                    self.__rclpy_node.destroy_node()

            except KeyError as ke:
                self.__rclpy_node.get_logger().error(f'Invalid JSON Key in MQTT {self.__mqtt_config_subscription_topic} subscription callback: {ke}')

            except json.JSONDecodeError as jde:
                self.__rclpy_node.get_logger().error(f'Invalid JSON format in MQTT {self.__mqtt_config_subscription_topic} subscription callback: {jde.msg}')

            except Exception as e:
                self.__rclpy_node.get_logger().error(f'Exception in MQTT {self.__mqtt_config_subscription_topic} subscription callback: {e}')
                raise
                
            
        self.__mqtt_client.subscribe(topic = self.__mqtt_config_subscription_topic, qos = self.__mqtt_config_subscription_qos)
        self.__mqtt_client.client.message_callback_add(sub = self.__mqtt_config_subscription_topic, callback = __mqtt_config_subscription_cb)


__all__ = ['rms_request_config_handler']