import os
import json

from configparser import ConfigParser

from rclpy.node import Node
from rclpy.timer import Timer
from rclpy.subscription import Subscription
from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import qos_profile_system_default
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import BatteryState
from geometry_msgs.msg import PoseStamped
from robot_status_msgs.msg import SensorStatus
from gps_iao_door_msgs.msg import InOutDoor

from ....mqtt.mqtt_client import Client

from ...common.enum_types import AreaCLSFType
from ...common.service import UUIDService
from ...common.service import TimeService
from ...common.service import ConfigService
from ...common.service import NetworkService
from ...common.domain import Header

from .domain import StatusEventCdType
from .domain import StatusInfoLocation
from .domain import StatusInfoSubLocation
from .domain import ComInfo
from .domain import ComInfoStatusType
from .domain import StatusInfo
from .domain import StatusEvent


class StatusEventHandler():

    def __init__(self, rclpy_node: Node, mqtt_client: Client) -> None:
        self.__rclpy_node: Node = rclpy_node
        self.__mqtt_client: Client = mqtt_client

        self.__script_directory: str = os.path.dirname(os.path.abspath(__file__))
        self.__mqtt_config_file_path: str = '../../../mqtt/mqtt.ini'
        self.__mqtt_config_service: ConfigService = ConfigService(self.__script_directory, self.__mqtt_config_file_path)
        self.__mqtt_config_parser: ConfigParser = self.__mqtt_config_service.read()

        self.__mqtt_status_event_publisher_topic: str = self.__mqtt_config_parser.get('topics', 'status_event')
        self.__mqtt_status_event_publisher_qos: int = int(self.__mqtt_config_parser.get('qos', 'status_event'))

        if self.__mqtt_client.is_connected:
            self.__rclpy_node.get_logger().info(f'MQTT granted publisher\n\ttopic : {self.__mqtt_status_event_publisher_topic}\n\tqos : {self.__mqtt_status_event_publisher_qos}')
        else:
            self.__rclpy_node.get_logger().error(f'MQTT failed to grant publisher\n\ttopic : {self.__mqtt_status_event_publisher_topic}\n\tqos : {self.__mqtt_status_event_publisher_qos}')

        self.__common_config_file_path: str = '../../common/config.ini'
        self.__common_config_service: ConfigService = ConfigService(
            self.__script_directory, self.__common_config_file_path)
        self.__common_config_parser: ConfigParser = self.__common_config_service.read()

        self.__rclpy_battery_state_subscription_topic: str = '/battery/state'
        self.__rclpy_battery_state_subscription_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup()
        self.__rclpy_battery_state_subscription: Subscription = self.__rclpy_node.create_subscription(
            msg_type=BatteryState,
            topic=self.__rclpy_battery_state_subscription_topic,
            qos_profile=qos_profile_sensor_data,
            callback=self.__rclpy_battery_state_subscription_cb,
            callback_group=self.__rclpy_battery_state_subscription_cb_group
        )

        self.__rclpy_imu_data_status_subscription_topic: str = '/imu/data/status'
        self.__rclpy_imu_data_status_subscription_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup()
        self.__rclpy_imu_data_status_subscription: Subscription = self.__rclpy_node.create_subscription(
            msg_type=SensorStatus,
            topic=self.__rclpy_imu_data_status_subscription_topic,
            qos_profile=qos_profile_system_default,
            callback=self.__rclpy_imu_status_subscription_cb,
            callback_group=self.__rclpy_imu_data_status_subscription_cb_group
        )

        self.__rclpy_scan_status_subscriptin_topic: str = '/scan/multi/status'
        self.__rclpy_scan_status_subscription_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup()
        self.__rclpy_scan_status_subscription: Subscription = self.__rclpy_node.create_subscription(
            msg_type=SensorStatus,
            topic=self.__rclpy_scan_status_subscriptin_topic,
            qos_profile=qos_profile_system_default,
            callback=self.__rclpy_scan_status_subscription_cb,
            callback_group=self.__rclpy_scan_status_subscription_cb_group
        )

        self.__rclpy_ublox_fix_status_subscription_topic: str = '/ublox/fix/status'
        self.__rclpy_ublox_fix_status_subscription_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup()
        self.__rclpy_ublox_fix_status_subscription: Subscription = self.__rclpy_node.create_subscription(
            msg_type=SensorStatus,
            topic=self.__rclpy_ublox_fix_status_subscription_topic,
            qos_profile=qos_profile_system_default,
            callback=self.__rclpy_ublox_fix_status_subscription_cb,
            callback_group=self.__rclpy_ublox_fix_status_subscription_cb_group
        )

        self.__rclpy_battery_state_status_subscription_topic: str = '/battery/status'
        self.__rclpy_battery_state_status_subscription_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup()
        self.__rclpy_battery_state_status_status_subscription: Subscription = self.__rclpy_node.create_subscription(
            msg_type=SensorStatus,
            topic=self.__rclpy_battery_state_status_subscription_topic,
            qos_profile=qos_profile_system_default,
            callback=self.__rclpy_battery_state_status_subscription_cb,
            callback_group=self.__rclpy_battery_state_status_subscription_cb_group
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

        self.__rclpy_slam_to_gps_subscription_topic: str = '/slam_to_gps'
        self.__rclpy_slam_to_gps_subscription_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup()
        self.__rclpy_slam_to_gps_subscription: Subscription = self.__rclpy_node.create_subscription(
            msg_type=NavSatFix,
            topic=self.__rclpy_slam_to_gps_subscription_topic,
            qos_profile=qos_profile_sensor_data,
            callback=self.__rclpy_slam_to_gps_subscription_cb,
            callback_group=self.__rclpy_slam_to_gps_subscription_cb_group
        )

        self.__rclpy_ublox_fix_subscription_topic: str = '/ublox/fix'
        self.__rclpy_ublox_fix_subscription_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup()
        self.__rclpy_ublox_fix_subscription: Subscription = self.__rclpy_node.create_subscription(
            msg_type=NavSatFix,
            topic=self.__rclpy_ublox_fix_subscription_topic,
            callback=self.__rclpy_ublox_fix_subscription_cb,
            qos_profile=qos_profile_sensor_data,
            callback_group=self.__rclpy_ublox_fix_subscription_cb_group
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
        
        __rclpy_status_event_timer_period_sec: float = 1.0
        __rclpy_status_event_timer_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup()
        __rclpy_status_event_timer: Timer = self.__rclpy_node.create_timer(
            timer_period_sec = __rclpy_status_event_timer_period_sec,
            callback_group = __rclpy_status_event_timer_cb_group,
            callback = self.__response_to_rms
        )
        
        self.__is_status_event_occurred: bool = False

        self.__network_service: NetworkService = NetworkService()
        self.__time_service: TimeService = TimeService()
        self.__uuid_service: UUIDService = UUIDService()

        self.__header: Header = Header()
        self.__status_info_location: StatusInfoLocation = StatusInfoLocation()
        self.__status_info_sub_location: StatusInfoSubLocation = StatusInfoSubLocation()
        self.__status_info: StatusInfo = StatusInfo()
        self.__com_info: ComInfo = ComInfo()

    def __rclpy_in_out_door_subscription_cb(self, in_out_door_cb: InOutDoor) -> None:
        is_out_door: bool = (in_out_door_cb.determination == True)

        if is_out_door:
            self.__status_info.areaClsf = AreaCLSFType.OUTDOOR.value
        else:
            self.__status_info.areaClsf = AreaCLSFType.INDOOR.value

    def __rclpy_battery_state_subscription_cb(self, battery_state_cb: BatteryState) -> None:
        self.__status_info.batteryLevel = battery_state_cb.percentage

    def __rclpy_imu_status_subscription_cb(self, imu_status_cb: SensorStatus) -> None:
        status_code: int = imu_status_cb.status_code

        if (status_code == -1000):
            self.__is_status_event_occurred = True
            self.__status_info.eventCd = StatusEventCdType.BROKEN.value
            self.__status_info.eventSubCd = 'IMU'
        else:
            self.__is_status_event_occurred = False
            return

    def __rclpy_scan_status_subscription_cb(self, scan_status_cb: SensorStatus) -> None:
        status_code: int = scan_status_cb.status_code

        if (status_code == -1001):
            self.__is_status_event_occurred = True
            self.__status_info.eventCd = StatusEventCdType.BROKEN.value
            self.__status_info.eventSubCd = 'LiDAR'
        else:
            self.__is_status_event_occurred = False
            return

    def __rclpy_ublox_fix_status_subscription_cb(self, gps_status_cb: SensorStatus) -> None:
        status_code: int = gps_status_cb.status_code

        if (status_code == -1002):
            self.__is_status_event_occurred = True
            self.__status_info.eventCd = StatusEventCdType.BROKEN.value
            self.__status_info.eventSubCd = 'GPS'
        else:
            self.__is_status_event_occurred = False
            return

    def __rclpy_battery_state_status_subscription_cb(self, battery_state_status: SensorStatus) -> None:
        status_code: int = battery_state_status.status_code

        if (status_code == -1003):
            self.__is_status_event_occurred = True
            self.__status_info.eventCd = StatusEventCdType.BROKEN.value
            self.__status_info.eventSubCd = 'BATTERY'
        else:
            self.__is_status_event_occurred = False
            return

    def __rclpy_slam_to_gps_subscription_cb(self, slam_to_gps_cb: NavSatFix) -> None:
        self.__status_info_location.xpos = slam_to_gps_cb.longitude
        self.__status_info_location.ypos = slam_to_gps_cb.latitude

    def __rclpy_ublox_fix_subscription_cb(self, ublox_gps_cb: NavSatFix) -> None:
        self.__status_info_sub_location.xpos = ublox_gps_cb.longitude
        self.__status_info_sub_location.ypos = ublox_gps_cb.latitude

    def __rclpy_rtt_odom_subscription_cb(self, rtt_odom_cb: PoseStamped) -> None:
        heading: float = rtt_odom_cb.pose.orientation.y

        self.__status_info_location.heading = heading
        self.__status_info_sub_location.heading = heading

    def __build_status_event(self) -> StatusEvent:
        status_event: StatusEvent = StatusEvent()

        self.__build_header()
        status_event.header = self.__header.__dict__

        self.__build_status_info()
        status_event.statusInfo = self.__status_info.__dict__

        self.__build_com_info()
        status_event.comInfo = self.__com_info.__dict__

        return status_event

    def __build_header(self) -> None:
        robot_corp_id: str = self.__common_config_parser.get('header', 'robotCorpId')
        self.__header.robotCorpId = robot_corp_id

        work_corp_id: str = self.__common_config_parser.get('header', 'workCorpId')
        self.__header.workCorpId = work_corp_id

        work_site_id: str = self.__common_config_parser.get('header', 'workSiteId')
        self.__header.workSiteId = work_site_id

        robot_id: str = self.__common_config_parser.get('header', 'robotId')
        self.__header.robotId = robot_id

        robot_type: str = self.__common_config_parser.get('header', 'robotType')
        self.__header.robotType = robot_type
        
        self.__header.topicUid = self.__uuid_service.generate_uuid()

    def __build_status_info(self) -> None:
        self.__status_info.location = self.__status_info_location.__dict__
        self.__status_info.subLocation = self.__status_info_sub_location.__dict__

        floor: str = '1F'
        self.__status_info.floor = floor
    
    def __build_com_info(self) -> None:
        status: str = ''
        is_mqtt_connected: bool = self.__mqtt_client.is_connected
        if is_mqtt_connected:
            status = ComInfoStatusType.CONNECTED.value
        else:
            status = ComInfoStatusType.DISCONNECTED.value
        self.__com_info.status = status

        self.__com_info.robotIP = self.__network_service.get_local_ip()
        self.__com_info.mqttIP = self.__mqtt_config_parser.get('broker', 'host')
        self.__com_info.mqttPort = self.__mqtt_config_parser.get('broker', 'port')

        battery_event: str = self.__common_config_parser.get('header', 'batteryEvent')
        self.__com_info.batteryEvent = battery_event
        
        self.__com_info.robotTime = self.__time_service.get_current_datetime()

    def __response_to_rms(self) -> None:
        built_status_event: StatusEvent = self.__build_status_event()
        
        if self.__is_status_event_occurred == True:
            self.__mqtt_client.publish(topic=self.__mqtt_status_event_publisher_topic, payload=json.dumps(built_status_event.__dict__), qos=self.__mqtt_status_event_publisher_qos)
        else:
            return


__all__ = ['rms_response_status_event_handler']
