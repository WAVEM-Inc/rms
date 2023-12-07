import os
import json

from configparser import ConfigParser
from rclpy.node import Node
from rclpy.subscription import Subscription
from rclpy.client import Client
from rclpy.qos import qos_profile_system_default
from rclpy.qos import qos_profile_sensor_data
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.task import Future

from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import BatteryState
from geometry_msgs.msg import PoseStamped
from robot_status_msgs.msg import VelocityStatus
from robot_status_msgs.msg import NavigationStatus
from gps_iao_door_msgs.msg import InOutDoor
from robot_status_msgs.msg import TaskStatus

from ....mqtt.mqtt_client import Client

from ...common.service import ConfigService

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
from .domain import LastInfoSubLocation


GTS_NAVIGATION_STARTED_CODE: int = 2
GTS_NAVIGATION_COMPLETED_CODE: int = 4


class LocationResponseHandler():

    def __init__(self, rclpy_node: Node, mqtt_client: Client) -> None:
        self.__rclpy_node: Node = rclpy_node
        self.__mqtt_client: Client = mqtt_client

        self.__script_directory: str = os.path.dirname(
            os.path.abspath(__file__))
        self.__mqtt_config_file_path: str = '../../../mqtt/mqtt.ini'
        self.__mqtt_config_service: ConfigService = ConfigService(
            self.__script_directory, self.__mqtt_config_file_path)
        self.__mqtt_config_parser: ConfigParser = self.__mqtt_config_service.read()

        self.__mqtt_location_publisher_topic: str = self.__mqtt_config_parser.get(
            'topics', 'location')
        self.__mqtt_location_publisher_qos: int = int(
            self.__mqtt_config_parser.get('qos', 'location'))

        if self.__mqtt_client.is_connected:
            self.__rclpy_node.get_logger().info(
                f'MQTT granted publisher\n\ttopic : {self.__mqtt_location_publisher_topic}\n\tqos : {self.__mqtt_location_publisher_qos}')
        else:
            self.__rclpy_node.get_logger().error(
                f'MQTT failed to grant publisher\n\ttopic : {self.__mqtt_location_publisher_topic}\n\tqos : {self.__mqtt_location_publisher_qos}')

        self.__common_config_file_path: str = '../../common/config.ini'
        self.__common_config_service: ConfigService = ConfigService(
            self.__script_directory, self.__common_config_file_path)
        self.__common_config_parser: ConfigParser = self.__common_config_service.read()

        self.__rclpy_slam_to_gps_subscription_topic: str = '/slam_to_gps'
        self.__rclpy_slam_to_gps_subscription_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup()
        self.__rclpy_slam_to_gps_subscription: Subscription = self.__rclpy_node.create_subscription(
            msg_type=NavSatFix,
            topic=self.__rclpy_slam_to_gps_subscription_topic,
            callback=self.__rclpy_slam_to_gps_subscription_cb,
            qos_profile=qos_profile_sensor_data,
            callback_group=self.__rclpy_slam_to_gps_subscription_cb_group
        )

        self.__rclpy_ublox_gps_subscription_topic: str = '/ublox/fix'
        self.__rclpy_ublox_gps_subscription_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup()
        self.__rclpy_ublox_gps_subscription: Subscription = self.__rclpy_node.create_subscription(
            msg_type=NavSatFix,
            topic=self.__rclpy_ublox_gps_subscription_topic,
            callback=self.__rclpy_ublox_gps_subscription_cb,
            qos_profile=qos_profile_sensor_data,
            callback_group=self.__rclpy_ublox_gps_subscription_cb_group
        )

        self.__rclpy_rtt_odom_subscription_topic: str = '/rtt_odom'
        self.__rclpy_rtt_odom_subscription_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup()
        self.__rclpy_rtt_odom_subscription: Subscription = self.__rclpy_node.create_subscription(
            msg_type=PoseStamped,
            topic=self.__rclpy_rtt_odom_subscription_topic,
            callback=self.__rclpy_rtt_odom_subscription_cb,
            qos_profile=qos_profile_sensor_data,
            callback_group=self.__rclpy_rtt_odom_subscription_cb_group
        )

        self.__rclpy_battery_state_subscription_topic: str = '/battery/state'
        self.__rclpy_battery_state_subscription_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup()
        self.__rclpy_battery_state_subscription: Subscription = self.__rclpy_node.create_subscription(
            msg_type=BatteryState,
            topic=self.__rclpy_battery_state_subscription_topic,
            callback=self.__rclpy_battery_state_subscription_cb,
            qos_profile=qos_profile_sensor_data,
            callback_group=self.__rclpy_battery_state_subscription_cb_group
        )

        self.__rclpy_velocity_state_subscription_topic: str = '/velocity/state'
        self.__rclpy_velocity_state_subscription_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup()
        self.__rclpy_velocity_state_subscription: Subscription = self.__rclpy_node.create_subscription(
            msg_type=VelocityStatus,
            topic=self.__rclpy_velocity_state_subscription_topic,
            callback=self.__rclpy_velocity_state_subscription_cb,
            qos_profile=qos_profile_sensor_data,
            callback_group=self.__rclpy_velocity_state_subscription_cb_group
        )

        self.__rclpy_in_out_door_subscription_topic: str = '/in_out_door'
        self.__rclpy_in_out_door_subscription_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup()
        self.__rclpy_in_out_door_subscription: Subscription = self.__rclpy_node.create_subscription(
            msg_type=InOutDoor,
            topic=self.__rclpy_in_out_door_subscription_topic,
            callback=self.__rclpy_in_out_door_subscription_cb,
            qos_profile=qos_profile_sensor_data,
            callback_group=self.__rclpy_in_out_door_subscription_cb_group
        )

        self.__rclpy_task_status_subscription_topic: str = '/robot_task/status'
        self.__rclpy_task_status_subscription_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup()
        self.__rclpy_task_status_subscription: Subscription = self.__rclpy_node.create_subscription(
            msg_type=TaskStatus,
            topic=self.__rclpy_task_status_subscription_topic,
            qos_profile=qos_profile_system_default,
            callback_group=self.__rclpy_task_status_subscription_cb_group,
            callback=self.__rclpy_task_status_subscription_cb
        )

        self.__header: Header = Header()
        self.__task_info: TaskInfo = TaskInfo()
        self.__job_info: JobInfo = JobInfo()
        self.__last_info: LastInfo = LastInfo()
        self.__last_info_location: LastInfoLocation = LastInfoLocation()
        self.__sub_last_info_location: LastInfoSubLocation = LastInfoSubLocation()

        self.__job_group: str = ''
        self.__job_kind: str = ''
        self.__job_plan_id: str = ''
        self.__job_group_id: str = ''
        self.__job_order_id: str = ''

    def __rclpy_slam_to_gps_subscription_cb(self, slam_to_gps_cb: NavSatFix) -> None:
        self.__last_info_location.xpos = slam_to_gps_cb.longitude
        self.__last_info_location.ypos = slam_to_gps_cb.latitude

    def __rclpy_ublox_gps_subscription_cb(self, ublox_gps_cb: NavSatFix) -> None:
        self.__sub_last_info_location.xpos = ublox_gps_cb.longitude
        self.__sub_last_info_location.ypos = ublox_gps_cb.latitude

    def __rclpy_rtt_odom_subscription_cb(self, rtt_odom_cb: PoseStamped) -> None:
        heading: float = rtt_odom_cb.pose.orientation.y

        self.__last_info_location.heading = heading
        self.__sub_last_info_location.heading = heading

    def __rclpy_battery_state_subscription_cb(self, battery_state_cb: BatteryState) -> None:
        self.__last_info.batteryLevel = battery_state_cb.percentage

    def __rclpy_velocity_state_subscription_cb(self, velocity_state_cb: VelocityStatus) -> None:
        current_velocity: float = velocity_state_cb.current_velocity
        self.__last_info.velocity = current_velocity

        distance: float = velocity_state_cb.distance
        self.__last_info.totalDist = distance

    def __rclpy_in_out_door_subscription_cb(self, in_out_door_cb: InOutDoor) -> None:
        is_out_door: bool = (in_out_door_cb.determination == True)

        if is_out_door:
            self.__last_info.areaClsf = AreaCLSFType.OUTDOOR.value
        else:
            self.__last_info.areaClsf = AreaCLSFType.INDOOR.value
        return

    def __rclpy_task_status_subscription_cb(self, task_status_cb: TaskStatus) -> None:
        self.__job_group = task_status_cb.job_group
        self.__task_info.jobGroup = self.__job_group

        self.__job_kind = task_status_cb.job_kind
        self.__task_info.jobKind = self.__job_kind
        
        if (self.__task_info.jobGroup != '' and self.__task_info.jobKind != ''):
            self.__task_info.taskStatus = TaskStatusType.ASSIGNED.value
        else:
            self.__task_info.taskStatus = TaskStatusType.UNASSIGNED.value

        self.__job_info.jobPlanId = task_status_cb.job_plan_id
        self.__job_info.jobGroupId = task_status_cb.job_group_id
        self.__job_info.jobOrderId = task_status_cb.job_order_id

    def build_location(self) -> Location:
        __location: Location = Location()

        self.__build_header()
        __location.header = self.__header.__dict__

        self.__build_job_info()
        __location.jobInfo = self.__job_info.__dict__

        self.__build_last_info()
        __location.lastInfo = self.__last_info.__dict__

        return __location

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

    def __build_job_info(self) -> None:
        self.__job_info.taskInfo = self.__task_info.__dict__

    def __build_last_info(self) -> None:
        self.__last_info.location = self.__last_info_location.__dict__
        self.__last_info.subLocation = self.__sub_last_info_location.__dict__

        floor: str = '1F'
        self.__last_info.floor = floor

    def response_to_rms(self) -> None:
        built_location: Location = self.build_location()
        self.__mqtt_client.publish(topic=self.__mqtt_location_publisher_topic, payload=json.dumps(
            built_location.__dict__), qos=self.__mqtt_location_publisher_qos)


__all__ = ['rms_response_location_handler']
