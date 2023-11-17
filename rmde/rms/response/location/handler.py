import os
import json

from configparser import ConfigParser
from rclpy.node import Node
from rclpy.subscription import Subscription
from rclpy.qos import qos_profile_sensor_data
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import BatteryState
from robot_status_msgs.msg import VelocityStatus

from ....mqtt import mqtt_client

from ...common.service import UUIDService
from ...common.service import TimeService
from ...common.service import ConfigService

from ...common.enum_types import RobotType
from ...common.enum_types import AreaCLSFType
from ...common.enum_types import JobGroupType
from ...common.enum_types import JobKindType
from ...common.enum_types import TaskStatusType

from ...common.domain import Header
from .domain import Location
from .domain import JobInfo
from .domain import TaskInfo
from .domain import LastInfo
from .domain import LastInfoLocation

from typing import Any


class LocationResponseHandler():
    
    
    def __init__(self, rclpy_node: Node, mqtt_broker: mqtt_client.Client) -> None:
        self.rclpy_node: Node = rclpy_node

        self.script_directory: str = os.path.dirname(os.path.abspath(__file__))
        self.mqtt_config_file_path: str = '../../../mqtt/mqtt.ini'
        self.mqtt_config_service: ConfigService = ConfigService(self.script_directory, self.mqtt_config_file_path)
        self.mqtt_config_parser: ConfigParser = self.mqtt_config_service.read()
        self.mqtt_location_publisher_topic: str = self.mqtt_config_parser.get('topics', 'location')
        self.mqtt_location_publisher_qos: int = int(self.mqtt_config_parser.get('qos', 'location'))

        self.rclpy_node.get_logger().info('MQTT granted publisher\n\ttopic : {%s}\n\tqos : {%d}' % (self.mqtt_location_publisher_topic, self.mqtt_location_publisher_qos))

        self.common_config_file_path: str = '../../common/config.ini'
        self.common_config_service: ConfigService = ConfigService(self.script_directory, self.common_config_file_path)
        self.common_config_parser: ConfigParser = self.common_config_service.read()
        
        self.rclpy_gps_subscription_topic: str = '/slam_to_gps'
        self.rclpy_battery_state_subscription_topic: str = '/battery/state'
        self.rclpy_velocity_state_subscription_topic: str = '/velocity/state'
        
        self.gps_subscription_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup()
        self.gps_subscription: Subscription = self.rclpy_node.create_subscription(
            msg_type = NavSatFix,
            topic = self.rclpy_gps_subscription_topic,
            callback = self.rclpy_gps_subscription_cb,
            qos_profile = qos_profile_sensor_data,
            callback_group = self.gps_subscription_cb_group
        )
        
        self.battery_state_subscription_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup()
        self.battery_state_subscription: Subscription = self.rclpy_node.create_subscription(
            msg_type = BatteryState,
            topic = self.rclpy_battery_state_subscription_topic,
            callback = self.rclpy_battery_state_subscription_cb,
            qos_profile = qos_profile_sensor_data,
            callback_group = self.battery_state_subscription_cb_group
        )
        
        self.velocity_state_subscription_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup()
        self.velocity_state_subscription: Subscription = self.rclpy_node.create_subscription(
            msg_type = VelocityStatus,
            topic = self.rclpy_velocity_state_subscription_topic,
            callback = self.rclpy_velocity_state_subscription_cb,
            qos_profile = qos_profile_sensor_data,
            callback_group = self.velocity_state_subscription_cb_group
        )
        
        self.mqtt_broker: mqtt_client.Client = mqtt_broker
        self.uuid_service: UUIDService = UUIDService()
        self.time_service: TimeService = TimeService()
        
        self.location_xpos: float = 0.0
        self.location_ypos: float = 0.0
        self.heading: float = 45.0
        self.battery_level: float = 0.0
        self.velocity: float = 0.0
        self.total_dist: int = 0
        
        
    def rclpy_gps_subscription_cb(self, gps_cb: NavSatFix) -> None:
        self.location_xpos = gps_cb.latitude
        self.location_ypos = gps_cb.longitude
        self.heading = gps_cb.altitude
    
    
    def rclpy_battery_state_subscription_cb(self, battery_state_cb: BatteryState) -> None:
        self.battery_level = battery_state_cb.percentage
    
    
    def rclpy_velocity_state_subscription_cb(self, velocity_state_cb: VelocityStatus) -> None:
        current_velocity: float = velocity_state_cb.current_velocity
        distance: float = velocity_state_cb.distance
        
        self.velocity = current_velocity
        self.total_dist = distance
    
    
    def build_location(self) -> Location:
        header: Header = self.__build_header__()
        job_info: JobInfo = self.__build_job_info__()
        last_info: LastInfo = self.__build_last_info__()
        
        location: Location = Location(
            header = header.__dict__,
            jobInfo = job_info.__dict__,
            lastInfo = last_info.__dict__
        )
        
        return location
        
        
    def __build_header__(self) -> Header:
        robotCoprId: str = self.common_config_parser.get('header', 'robotCorpId')
        workCorpId: str = self.common_config_parser.get('header', 'workCorpId')
        workSiteId: str = self.common_config_parser.get('header', 'workSiteId')
        robotId: str = self.common_config_parser.get('header', 'robotId')
        robotType: str = self.common_config_parser.get('header', 'robotType')

        header: Header = Header(
            robotCorpId = robotCoprId,
            workCorpId = workCorpId,
            workSiteId = workSiteId,
            robotId = robotId,
            robotType = robotType
        )

        return header
    

    def __build_job_info__(self) -> JobInfo:
        job_plan_id: str = self.uuid_service.generate_uuid()
        job_group_id: str = self.uuid_service.generate_uuid()
        job_order_id: str = self.uuid_service.generate_uuid()
        
        task_info: TaskInfo = TaskInfo(
            jobGroup = JobGroupType.SUPPLY.value,
            jobKind = JobKindType.MOVE.value,
            taskStatus = TaskStatusType.ASSIGNED.value
        )
        
        job_info: JobInfo = JobInfo(
            jobPlanId = job_plan_id,
            jobGroupId = job_group_id,
            jobOrderId = job_order_id,
            taskInfo = task_info.__dict__
        )
        
        return job_info
    
    
    def __build_last_info__(self) -> LastInfo:
        last_info_location: LastInfoLocation = LastInfoLocation(
            xpos = self.location_xpos,
            ypos = self.location_ypos,
            heading = self.heading
        )
        
        last_info: LastInfo = LastInfo(
            location = last_info_location.__dict__,
            areaClsf = AreaCLSFType.INDOOR.value,
            floor = '1F',
            batteryLevel = self.battery_level,
            velocity = abs(self.velocity),
            totalDist = self.total_dist
        )
        
        return last_info
    
    
    def response_to_uvc(self) -> None:
        built_location: Location = self.build_location()
        self.mqtt_broker.publish(topic = self.mqtt_location_publisher_topic, payload = json.dumps(built_location.__dict__), qos = self.mqtt_location_publisher_qos)
        

__all__ = ['location_response_handler']