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

from ....mqtt import mqtt_client
from ...common.service import ConfigService

from ...common.domain import Header
from .domain import JobInfo
from .domain import JobKindType
from .domain import JobPath
from .domain import Path

from typing import Any
from typing import Dict


class PathRequestHandler():
    rclpy_flag: str = 'RCLPY'
    mqtt_flag: str = 'MQTT'
    
    
    def __init__(self, rclpy_node: Node, mqtt_broker: mqtt_client.Client) -> None:
        self.script_directory: str = os.path.dirname(os.path.abspath(__file__))
        self.mqtt_config_file_path: str = '../../../mqtt/mqtt.ini'
        self.mqtt_config_service: ConfigService = ConfigService(self.script_directory, self.mqtt_config_file_path)
        self.mqtt_config_parser: ConfigParser = self.mqtt_config_service.read()
        self.mqtt_path_subscription_topic: str = self.mqtt_config_parser.get('topics', 'path')
        self.mqtt_path_subscription_qos: int = int(self.mqtt_config_parser.get('qos', 'path'))
        
        self.rclpy_node: Node = rclpy_node
        self.rclpy_goal_waypoints_publisher_topic: str = '/gts_navigation/waypoints'
        
        self.goal_waypoints_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup()
        self.goal_waypoints_publisher: Publisher = self.rclpy_node.create_publisher(
            msg_type = GoalWaypoints,
            topic = self.rclpy_goal_waypoints_publisher_topic,
            qos_profile = qos_profile_system_default,
            callback_group = self.goal_waypoints_cb_group    
        )
        
        self.mqtt_broker: mqtt_client.Client = mqtt_broker
        self.path: Path = Path()
        self.header: Header = Header()
        self.jobInfo: JobInfo = JobInfo()
        self.jobPath: JobPath = JobPath()
        
    
    def request_to_uvc(self) -> None:
        def mqtt_path_subscription_cb(mqtt_client: mqtt.Client, mqtt_user_data: Dict, mqtt_message: mqtt.MQTTMessage) -> None:
            mqtt_topic: str = mqtt_message.topic
            mqtt_decoded_payload: str = mqtt_message.payload.decode()
            mqtt_json: Any = json.loads(mqtt_message.payload)
            
            self.rclpy_node.get_logger().info('{} subscription cb payload [{}] from [{}]'.format(self.mqtt_flag, mqtt_decoded_payload, mqtt_topic))
            self.rclpy_node.get_logger().info('{} subscription cb json [{}] from [{}]'.format(self.mqtt_flag, mqtt_json, mqtt_topic))
            
            self.path = Path(
                header = mqtt_json['header'],
                jobInfo = mqtt_json['jobInfo'],
                jobPath = mqtt_json['jobPath']
            )
            
            self.jobInfo = JobInfo(
                jobPlanId = self.path.jobInfo['jobPlanId'],
                jobGroupId = self.path.jobInfo['jobGroupId'],
                jobOrderId = self.path.jobInfo['jobOrderId'],
                jobGroup = self.path.jobInfo['jobGroup'],
                jobKind = self.path.jobInfo['jobKind']
            )
            
            self.jobPath = JobPath(
                areaClsf = self.path.jobPath['areaClsf'],
                locationList = self.path.jobPath['locationList'],
                jobKindType = self.path.jobPath['jobKindType']
            )
            
            self.rclpy_node.get_logger().info('JobPath LocationList {}'.format(self.jobPath.locationList))
            self.__publish_goal_waypoints_list__(self.jobPath.locationList)
            
        self.mqtt_broker.subscribe(topic = self.mqtt_path_subscription_topic, qos = self.mqtt_path_subscription_qos)
        self.mqtt_broker.client.message_callback_add(self.mqtt_path_subscription_topic, mqtt_path_subscription_cb)

    
    def __lookup_object__(self, module_name: str, module_class_name: str) -> Any:        
        self.rclpy_node.get_logger().info("{} lookup object path : {}".format(self.rclpy_flag, module_name))
        self.rclpy_node.get_logger().info("{} lookup object module_name : {}".format(self.rclpy_flag, module_name))

        module = importlib.import_module(module_name, self.rclpy_node.get_name())
        obj: Any = getattr(module, module_class_name)

        return obj
    
    
    def __publish_goal_waypoints_list__(self, location_list: list) -> None:
        goal_waypoints_list: list = []
        
        for location in location_list:
            rclpy_msg_type: Any = self.__lookup_object__('sensor_msgs.msg', 'NavSatFix')
            location_to_point: dict = {
                'latitude': location['xpos'],
                'longitude': location['ypos'],
                'altitude': 0.0
            }
            point: Any = message_conversion.populate_instance(location_to_point, rclpy_msg_type())
            goal_waypoints_list.append(point)
        
        self.rclpy_node.get_logger().info('GoalWaypoints List : {}'.format(goal_waypoints_list))
        
        goal_waypoints: GoalWaypoints = GoalWaypoints()
        goal_waypoints.goal_waypoints_list = goal_waypoints_list
        self.rclpy_node.get_logger().info('GoalWaypoints : {}'.format(goal_waypoints.goal_waypoints_list))
        self.goal_waypoints_publisher.publish(goal_waypoints)
        

__all__ = ['path_request_handler']