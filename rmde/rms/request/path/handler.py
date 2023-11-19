import os
import json
import importlib
import paho.mqtt.client as mqtt

from configparser import ConfigParser
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.qos import qos_profile_system_default
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rosbridge_library.internal import message_conversion

from gts_navigation_msgs.msg import GoalWaypoints

from ....mqtt.mqtt_client import Client
from ...common.service import ConfigService

from ...common.domain import Header
from .domain import JobInfo
from .domain import JobKindType
from .domain import JobPath
from .domain import Path

from typing import Any
from typing import Dict


RCLPY_FLAG: str = 'RCLPY'
MQTT_FLAG: str = 'MQTT'

class PathRequestHandler():
    def __init__(self, rclpy_node: Node, mqtt_client: Client) -> None:
        self.__rclpy_node: Node = rclpy_node
        self.__mqtt_client: Client = mqtt_client

        self.__script_directory: str = os.path.dirname(os.path.abspath(__file__))
        self.__mqtt_config_file_path: str = '../../../mqtt/mqtt.ini'
        self.__mqtt_config_service: ConfigService = ConfigService(self.__script_directory, self.__mqtt_config_file_path)
        self.__mqtt_config_parser: ConfigParser = self.__mqtt_config_service.read()
        self.__mqtt_path_subscription_topic: str = self.__mqtt_config_parser.get('topics', 'path')
        self.__mqtt_path_subscription_qos: int = int(self.__mqtt_config_parser.get('qos', 'path'))
        
        self.__rclpy_goal_waypoints_publisher_topic: str = '/gts_navigation/waypoints'
        self.__rclpy_goal_waypoints_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup()
        self.__rclpy_goal_waypoints_publisher: Publisher = self.__rclpy_node.create_publisher(
            msg_type = GoalWaypoints,
            topic = self.__rclpy_goal_waypoints_publisher_topic,
            qos_profile = qos_profile_system_default,
            callback_group = self.__rclpy_goal_waypoints_cb_group    
        )
        
        self.__path: Path = Path()
        self.__jobInfo: JobInfo = JobInfo()
        self.__jobPath: JobPath = JobPath()
        
    
    def request_to_uvc(self) -> None:
        def __mqtt_path_subscription_cb(mqtt_client: mqtt.Client, mqtt_user_data: Dict, mqtt_message: mqtt.MQTTMessage) -> None:
            __mqtt_topic: str = mqtt_message.topic
            __mqtt_decoded_payload: str = mqtt_message.payload.decode()
            __mqtt_json: Any = json.loads(mqtt_message.payload)
            
            self.__rclpy_node.get_logger().info('{} subscription cb payload [{}] from [{}]'.format(MQTT_FLAG, __mqtt_decoded_payload, __mqtt_topic))
            self.__rclpy_node.get_logger().info('{} subscription cb json [{}] from [{}]'.format(MQTT_FLAG, __mqtt_json, __mqtt_topic))
            
            __header_dict: dict = __mqtt_json['header']
            self.__path.header = __header_dict

            __jobInfo_dict: dict = __mqtt_json['jobInfo']
            self.__path.jobInfo = __jobInfo_dict

            __jobPath_dict: dict = __mqtt_json['jobPath']
            self.__path.jobPath = __jobPath_dict
            
            __jobPlanId: str = self.__path.jobInfo['jobPlanId']
            self.__jobInfo.jobPlanId = __jobPlanId

            __jobGroupId: str  = self.__path.jobInfo['jobGroupId']
            self.__jobInfo.jobGroupId = __jobGroupId

            __jobOrderId: str  = self.__path.jobInfo['jobOrderId']
            self.__jobInfo.jobOrderId = __jobOrderId

            __jobGroup: str  = self.__path.jobInfo['jobGroup']
            self.__jobInfo.jobGroup = __jobGroup

            __jobKind: str = self.__path.jobInfo['jobKind']
            self.__jobInfo.jobKind = __jobKind

            __areaClsf: str = self.__path.jobPath['areaClsf']
            self.__jobPath.areaClsf = __areaClsf

            __locationList_list: list = self.__path.jobPath['locationList']
            self.__jobPath.locationList = __locationList_list

            __jobKindType_dict: dict = self.__path.jobPath['jobKindType']
            self.__jobPath.jobKindType = __jobKindType_dict
            
            self.__rclpy_node.get_logger().info('JobPath LocationList {}'.format(self.__jobPath.locationList))
            self.__rclpy_publish_goal_waypoints_list__(self.__jobPath.locationList)
            
        self.__mqtt_client.subscribe(topic = self.__mqtt_path_subscription_topic, qos = self.__mqtt_path_subscription_qos)
        self.__mqtt_client.client.message_callback_add(self.__mqtt_path_subscription_topic, __mqtt_path_subscription_cb)

    
    def __lookup_object__(self, module_name: str, module_class_name: str) -> Any:        
        self.__rclpy_node.get_logger().info('{} lookup object path : {}'.format(RCLPY_FLAG, module_name))
        self.__rclpy_node.get_logger().info('{} lookup object module_name : {}'.format(RCLPY_FLAG, module_name))

        __module = importlib.import_module(module_name, self.__rclpy_node.get_name())
        __obj: Any = getattr(__module, module_class_name)

        return __obj
    
    
    def __rclpy_publish_goal_waypoints_list__(self, location_list: list) -> None:
        __rclpy_goal_waypoints_list: list = []
        
        for location in location_list:
            __rclpy_nav_sat_fix_obj: Any = self.__lookup_object__('sensor_msgs.msg', 'NavSatFix')

            __rclpy_location_to_nav_sat_fix_dict: dict = {
                'latitude': location['xpos'],
                'longitude': location['ypos'],
                'altitude': 0.0
            }

            __rclpy_nav_sat_fix: Any = message_conversion.populate_instance(__rclpy_location_to_nav_sat_fix_dict, __rclpy_nav_sat_fix_obj())
            __rclpy_goal_waypoints_list.append(__rclpy_nav_sat_fix)
        
        self.__rclpy_node.get_logger().info('GoalWaypoints List : {}'.format(__rclpy_goal_waypoints_list))
        
        __rclpy_goal_waypoints: GoalWaypoints = GoalWaypoints()
        __rclpy_goal_waypoints.goal_waypoints_list = __rclpy_goal_waypoints_list

        self.__rclpy_node.get_logger().info('GoalWaypoints : {}'.format(__rclpy_goal_waypoints.goal_waypoints_list))
        self.__rclpy_goal_waypoints_publisher.publish(__rclpy_goal_waypoints)
        

__all__ = ['path_request_handler']