import rclpy
import paho.mqtt.client as mqtt
import json
import importlib

from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.qos import qos_profile_system_default
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rosbridge_library.internal import message_conversion

from geometry_msgs.msg import Point
from gts_navigation_msgs.msg import GoalWaypoints

from mqtt import broker

from rms.common.domain.header import Header
from rms.request.path.domain.path import Path
from rms.request.path.domain.jobInfo.job_info import JobInfo
from rms.request.path.domain.jobPath.job_path import JobPath

from typing import Any
from typing import List
from typing import Dict


class PathRequestHandler():
    rclpy_flag: str = 'RCLPY'
    mqtt_flag: str = 'MQTT'
    
    rclpy_goal_waypoints_publisher_topic: str = '/gts_navigation/waypoints'
    mqtt_path_subscription_topic: str = 'hubilon/atcplus/rms/path'
    
    
    def __init__(self, rclpy_node: Node, mqtt_broker: broker.mqtt_broker) -> None:
        self.rclpy_node: Node = rclpy_node
        
        self.goal_waypoints_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup()
        self.goal_waypoints_publisher: Publisher = self.rclpy_node.create_publisher(
            msg_type = GoalWaypoints,
            topic = self.rclpy_goal_waypoints_publisher_topic,
            qos_profile = qos_profile_system_default,
            callback_group = self.goal_waypoints_cb_group    
        )
        
        self.mqtt_broker: broker.mqtt_broker = mqtt_broker
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
            
            
        self.mqtt_broker.client.subscribe(self.mqtt_path_subscription_topic)
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
            rclpy_msg_type: Any = self.__lookup_object__('geometry_msgs.msg', 'Point')
            location_to_point: dict = {
                'x': location['xpos'],
                'y': location['ypos'],
                'z': 45.0
            }
            point: Any = message_conversion.populate_instance(location_to_point, rclpy_msg_type())
            goal_waypoints_list.append(point)
        
        self.rclpy_node.get_logger().info('GoalWaypoints List : {}'.format(goal_waypoints_list))
        
        goal_waypoints: GoalWaypoints = GoalWaypoints()
        goal_waypoints.goal_waypoints_list = goal_waypoints_list
        self.rclpy_node.get_logger().info('GoalWaypoints : {}'.format(goal_waypoints.goal_waypoints_list))
        self.goal_waypoints_publisher.publish(goal_waypoints)
        

__all__ = ['path_request_handler']