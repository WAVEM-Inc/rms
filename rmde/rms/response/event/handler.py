import os
import json
import rclpy.client

from configparser import ConfigParser
from rclpy.node import Node
from rclpy.subscription import Subscription
from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import qos_profile_system_default
from rclpy.task import Future
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import BatteryState
from geometry_msgs.msg import PoseStamped
from robot_status_msgs.msg import SensorStatus
from robot_status_msgs.msg import NavigationStatus
from gps_iao_door_msgs.msg import InOutDoor

from ....mqtt.mqtt_client import Client

from ...common.service import UUIDService
from ...common.service import TimeService
from ...common.service import ConfigService
from ...common.service import NetworkService

from ...common.enum_types import TaskStatusType
from ...common.enum_types import AreaCLSFType

from ...common.enum_types import JobGroupType
from ...common.enum_types import JobKindType
from ...common.enum_types import JobResultStatusType

from .domain import EventCdType
from .domain import ComInfoStatusType

from ...common.domain import Header
from ...common.domain import Job

from .domain import Event
from .domain import TaskInfo
from .domain import JobResult
from .domain import EventInfo
from .domain import EventInfoLocation
from .domain import EventInfoSubLocation
from .domain import ComInfo



GTS_NAVIGATION_STARTED_CODE: int = 2
GTS_NAVIGATION_COMPLETED_CODE: int = 4

class EventResponseHandler():
    def __init__(self, rclpy_node: Node, mqtt_client: Client) -> None:
        self.__rclpy_node: Node = rclpy_node
        self.__mqtt_client: Client = mqtt_client

        self.__script_directory: str = os.path.dirname(os.path.abspath(__file__))
        self.__mqtt_config_file_path: str = '../../../mqtt/mqtt.ini'
        self.__mqtt_config_service: ConfigService = ConfigService(self.__script_directory, self.__mqtt_config_file_path)
        self.__mqtt_config_parser: ConfigParser = self.__mqtt_config_service.read()
        self.__mqtt_event_publisher_topic: str = self.__mqtt_config_parser.get('topics', 'event')
        self.__mqtt_location_publisher_qos: int = int(self.__mqtt_config_parser.get('qos', 'event'))

        self.__rclpy_node.get_logger().info('MQTT granted publisher\n\ttopic : {%s}\n\tqos : {%d}' % (self.__mqtt_event_publisher_topic, self.__mqtt_location_publisher_qos))
        
        self.__common_config_file_path: str = '../../common/config.ini'
        self.__common_config_service: ConfigService = ConfigService(self.__script_directory, self.__common_config_file_path)
        self.__common_config_parser: ConfigParser = self.__common_config_service.read()

        self.__network_service: NetworkService = NetworkService()
        
        
        self.__rclpy_slam_to_gps_subscription_topic: str = '/slam_to_gps'
        self.__rclpy_slam_to_gps_subscription_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup()
        self.__rclpy_slam_to_gps_subscription: Subscription = self.__rclpy_node.create_subscription(
            msg_type = NavSatFix,
            topic = self.__rclpy_slam_to_gps_subscription_topic,
            qos_profile = qos_profile_sensor_data,
            callback = self.__rclpy_slam_to_gps_subscription_cb,
            callback_group = self.__rclpy_slam_to_gps_subscription_cb_group
        )

        self.__rclpy_ublox_gps_subscription_topic: str = '/ublox/fix'
        self.__rclpy_ublox_gps_subscription_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup()
        self.__rclpy_ublox_gps_subscription: Subscription = self.__rclpy_node.create_subscription(
            msg_type = NavSatFix,
            topic = self.__rclpy_ublox_gps_subscription_topic,
            callback = self.__rclpy_ublox_gps_subscription_cb,
            qos_profile = qos_profile_sensor_data,
            callback_group = self.__rclpy_ublox_gps_subscription_cb_group
        )

        self.__rclpy_rtt_odom_subscription_topic: str = '/rtt_odom'
        self.__rclpy_rtt_odom_subscription_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup()
        self.__rclpy_rtt_odom_subscription: Subscription = self.__rclpy_node.create_subscription(
            msg_type = PoseStamped,
            topic = self.__rclpy_rtt_odom_subscription_topic,
            callback = self.__rclpy_rtt_odom_subscription_cb,
            qos_profile = qos_profile_sensor_data,
            callback_group = self.__rclpy_rtt_odom_subscription_cb_group
        )

        self.__rclpy_battery_state_subscription_topic: str = '/battery/state'
        self.__rclpy_battery_state_subscription_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup()
        self.__rclpy_battery_state_subscription: Subscription = self.__rclpy_node.create_subscription(
            msg_type = BatteryState,
            topic = self.__rclpy_battery_state_subscription_topic,
            qos_profile = qos_profile_sensor_data,
            callback = self.__rclpy_battery_state_subscription_cb,
            callback_group = self.__rclpy_battery_state_subscription_cb_group
        )

        self.__rclpy_imu_status_subscription_topic: str = '/imu/status'
        self.__rclpy_imu_status_subscription_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup()
        self.__rclpy_imu_status_subscription: Subscription = self.__rclpy_node.create_subscription(
            msg_type = SensorStatus,
            topic = self.__rclpy_imu_status_subscription_topic,
            qos_profile = qos_profile_system_default,
            callback = self.__rclpy_imu_status_subscription_cb,
            callback_group = self.__rclpy_imu_status_subscription_cb_group
        )

        self.__rclpy_scan_status_subscriptin_topic: str = '/scan/status'
        self.__rclpy_scan_status_subscription_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup()
        self.__rclpy_scan_status_subscription: Subscription = self.__rclpy_node.create_subscription(
            msg_type = SensorStatus,
            topic = self.__rclpy_scan_status_subscriptin_topic,
            qos_profile = qos_profile_system_default,
            callback = self.__rclpy_scan_status_subscription_cb,
            callback_group = self.__rclpy_scan_status_subscription_cb_group
        )
        
        self.__rclpy_gps_status_subscription_topic: str = '/ublox/status'
        self.__rclpy_gps_status_subscription_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup()
        self.__rclpy_gps_status_subscription: Subscription = self.__rclpy_node.create_subscription(
            msg_type = SensorStatus,
            topic = self.__rclpy_gps_status_subscription_topic,
            qos_profile = qos_profile_system_default,
            callback = self.__rclpy_gps_status_subscription_cb,
            callback_group = self.__rclpy_gps_status_subscription_cb_group
        )
        
        self.__rclpy_battery_state_status_subscription_topic: str = '/battery/status'
        self.__rclpy_battery_state_status_subscription_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup()
        self.__rclpy_battery_state_status_status_subscription: Subscription = self.__rclpy_node.create_subscription(
            msg_type = SensorStatus,
            topic = self.__rclpy_battery_state_status_subscription_topic,
            qos_profile = qos_profile_system_default,
            callback = self.__rclpy_battery_state_status_subscription_cb,
            callback_group = self.__rclpy_battery_state_status_subscription_cb_group
        )

        self.__rclpy_gts_navigation_task_status_subscription_topic: str = '/gts_navigation/task_status'
        self.__rclpy_gts_navigation_task_status_subscription_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup()
        self.__rclpy_gts_navigation_task_status_subscription: Subscription = self.__rclpy_node.create_subscription(
            msg_type = NavigationStatus,
            topic = self.__rclpy_gts_navigation_task_status_subscription_topic,
            qos_profile = qos_profile_system_default,
            callback = self.__rclpy_gts_navigation_task_status_subscription_cb,
            callback_group = self.__rclpy_gts_navigation_task_status_subscription_cb_group
        )

        self.__rclpy_in_out_door_subscription_topic: str = '/in_out_door'
        self.__rclpy_in_out_door_subscription_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup()
        self.__rclpy_in_out_door_subscription: Subscription = self.__rclpy_node.create_subscription(
            msg_type = InOutDoor,
            topic = self.__rclpy_in_out_door_subscription_topic,
            callback = self.__rclpy_in_out_door_subscription_cb,
            qos_profile = qos_profile_sensor_data,
            callback_group = self.__rclpy_in_out_door_subscription_cb_group
        )
        
        self.__ip_address: str = self.__network_service.get_local_ip()
        
        self.__uuid_service: UUIDService = UUIDService()
        self.__time_service: TimeService = TimeService()
        
        self.job_status_code: int = 0
        self.job_group: str = ''
        self.job_kind: str = ''
        self.task_status: str = ''
        self.job_status: str = ''
        self.job_start_time: str = ''
        self.job_end_time: str = ''
        self.job_start_battery_level: float = 0.0
        self.job_end_battery_level: float = 0.0
        self.job_start_dist: float = 0.0
        self.job_end_dist: float = 0.0
        
        self.sensor_status: str = ''
        self.sensor_type: str = ''
        self.current_battery_level: float = 0.0
        self.location_xpos: float = 0.0
        self.location_ypos: float = 0.0
        self.heading: float = 0.0
        self.sub_location_xpos: float = 0.0
        self.sub_location_ypos: float = 0.0
        self.areaClsf: str = ''

        self.job_plan_id: str = ''
        self.job_group_id: str = ''
        self.job_order_id: str = ''
    
    
    def __rclpy_slam_to_gps_subscription_cb(self, slam_to_gps_cb: NavSatFix) -> None:
        self.location_xpos = slam_to_gps_cb.longitude
        self.location_ypos = slam_to_gps_cb.latitude
    

    def __rclpy_ublox_gps_subscription_cb(self, ublox_gps_cb: NavSatFix) -> None:
        self.sub_location_xpos = ublox_gps_cb.longitude
        self.sub_location_ypos = ublox_gps_cb.latitude

    
    def __rclpy_rtt_odom_subscription_cb(self, rtt_odom_cb: PoseStamped) -> None:
        self.heading = rtt_odom_cb.pose.orientation.y
    
    
    def __rclpy_battery_state_subscription_cb(self, battery_state_cb: BatteryState) -> None:
        self.job_start_battery_level = battery_state_cb.percentage
        self.job_end_battery_level = battery_state_cb.percentage
        self.current_battery_level = battery_state_cb.percentage

    
    def __report_sensor_broken_status__(self, sensor_type: str) -> None:
        self.sensor_status = EventCdType.BROKEN.value
        self.sensor_type = sensor_type
        self.__response_to_uvc()
    

    def __rclpy_imu_status_subscription_cb(self, imu_status_cb: SensorStatus) -> None:
        __status_code: int = imu_status_cb.status_code
        
        if (__status_code == -1000):
            self.sensor_status = EventCdType.BROKEN.value
            self.sensor_type = 'IMU'
        else:
            return
        
    
    def __rclpy_scan_status_subscription_cb(self, scan_status_cb: SensorStatus) -> None:
        __status_code: int = scan_status_cb.status_code
        
        if (__status_code == -1001):
            self.sensor_status = EventCdType.BROKEN.value
            self.sensor_type = 'LiDAR'
        else:
            return
        
        
    def __rclpy_gps_status_subscription_cb(self, gps_status_cb: SensorStatus) -> None:
        __status_code: int = gps_status_cb.status_code
        
        if (__status_code == -1002):
            self.sensor_status = EventCdType.BROKEN.value
            self.sensor_type = 'GPS'
        else:
            return
        
    
    def __rclpy_battery_state_status_subscription_cb(self, battery_state_status: SensorStatus) -> None:
        __status_code: int = battery_state_status.status_code
        
        if (__status_code == -1003):
            self.sensor_status = EventCdType.BROKEN.value
            self.sensor_type = 'BATTERY'
        else:
            return
        

    def __rclpy_gts_navigation_task_status_subscription_cb(self, navigation_status: NavigationStatus) -> None:
        self.job_status_code = navigation_status.status_code

        if self.job_status_code == GTS_NAVIGATION_STARTED_CODE:
            self.job_group = JobGroupType.MOVE.value
            self.job_kind = JobKindType.MOVE.value
            self.task_status = JobResultStatusType.SUCCESS.value
        elif self.job_status_code == GTS_NAVIGATION_COMPLETED_CODE:
            self.job_group = JobGroupType.WAIT.value
            self.job_kind = JobKindType.WAIT.value
            self.task_status = JobResultStatusType.SUCCESS.value
        else:
            return
        
        self.job_status = navigation_status.status
        self.job_start_time = navigation_status.start_time
        self.job_end_time = navigation_status.end_time
        self.job_start_battery_level = navigation_status.start_battery_level
        self.job_end_battery_level = navigation_status.end_battery_level
        self.job_start_dist = navigation_status.start_dist
        self.job_end_dist = navigation_status.end_dist
        self.__response_to_uvc()

    
    def __rclpy_in_out_door_subscription_cb(self, in_out_door_cb: InOutDoor) -> None:
        is_out_door: bool = (in_out_door_cb.determination == True)

        if is_out_door:
            self.areaClsf = AreaCLSFType.OUTDOOR.value
        else :
            self.areaClsf = AreaCLSFType.INDOOR.value
        return

        

    def __build_event(self) -> Event:
        __event: Event = Event()

        __header: Header = self.__build_header()
        __header_dict: dict = __header.__dict__
        __event.header = __header_dict

        __taskInfo: TaskInfo = self.__build_task_info()
        __taskInfo_dict: dict = __taskInfo.__dict__
        __event.taskInfo = __taskInfo_dict

        __eventInfo: EventInfo = self.__build_event_info()
        __eventInfo_dict: dict = __eventInfo.__dict__
        __event.eventInfo = __eventInfo_dict

        __comInfo: ComInfo = self.__build_com_info()
        __comInfo_dict: dict = __comInfo.__dict__
        __event.comInfo = __comInfo_dict

        return __event

    
    def __build_header(self) -> Header:
        __header: Header = Header()

        __robotCorpId: str = self.__common_config_parser.get('header', 'robotCorpId')
        __header.robotCorpId = __robotCorpId

        __workCorpId: str = self.__common_config_parser.get('header', 'workCorpId')
        __header.workCorpId = __workCorpId

        __workSiteId: str = self.__common_config_parser.get('header', 'workSiteId')
        __header.workSiteId = __workSiteId

        __robotId: str = self.__common_config_parser.get('header', 'robotId')
        __header.robotId = __robotId

        __robotType: str = self.__common_config_parser.get('header', 'robotType')
        __header.robotType = __robotType

        return __header
    

    def __build_task_info(self) -> TaskInfo:
        __formatted_datetime: str = self.__time_service.get_current_datetime()

        __jobResult: JobResult = JobResult()

        __status: str = self.task_status
        __jobResult.status = __status

        __startTime: str = __formatted_datetime
        __jobResult.startTime = __startTime

        __endTime: str = __formatted_datetime
        __jobResult.endTime = __endTime

        __startBatteryLevel: int = self.job_start_battery_level
        __jobResult.startBatteryLevel = __startBatteryLevel

        __endBatteryLevel: int = self.job_end_battery_level
        __jobResult.endBatteryLevel = __endBatteryLevel

        __dist: int = (self.job_end_dist - self.job_start_dist)
        __jobResult.dist = __dist
    
        __taskInfo: TaskInfo = TaskInfo()

        __jobPlanId: str = Job.jobPlanId
        print(f'jobPlanId : {__jobPlanId}')
        __taskInfo.jobPlanId = __jobPlanId

        __jobGroupId: str = Job.jobGroupId
        print(f'__jobGroupId : {__jobGroupId}')
        __taskInfo.jobGroupId = __jobGroupId

        __jobOrderId: str = Job.jobOrderId
        print(f'__jobOrderId : {__jobOrderId}')
        __taskInfo.jobOrderId = __jobOrderId
        
        __jobGroup: str = self.job_group
        __taskInfo.jobGroup = __jobGroup

        __jobKind: str = self.job_kind
        __taskInfo.jobKind = __jobKind

        __jobResult: dict = __jobResult.__dict__
        __taskInfo.jobResult = __jobResult
        
        return __taskInfo
    
    
    def __build_event_info(self) -> EventInfo:
        __eventLocation: EventInfoLocation = EventInfoLocation()

        __xpos: float = self.location_xpos
        __eventLocation.xpos = __xpos

        __ypos: float = self.location_ypos
        __eventLocation.ypos = __ypos

        __heading: float = self.heading
        __eventLocation.heading = __heading

        __eventLocation_dict: dict = __eventLocation.__dict__

        __eventSubLocation: EventInfoSubLocation = EventInfoSubLocation()

        __sub_xpos: float = self.sub_location_xpos
        __eventSubLocation.xpos = __sub_xpos

        __sub_ypos: float = self.sub_location_ypos
        __eventSubLocation.ypos = __sub_ypos

        __eventSubLocation_dict: dict = __eventSubLocation.__dict__

        __eventInfo: EventInfo = EventInfo()

        __eventId: str = self.__uuid_service.generate_uuid()

        __eventId: str = __eventId
        __eventInfo.eventId = __eventId

        __eventCd: str = self.sensor_status
        __eventInfo.eventCd = __eventCd

        __eventSubCd: str = self.sensor_type
        __eventInfo.eventSubCd = __eventSubCd

        __areaClsf: str = self.areaClsf
        __eventInfo.areaClsf = __areaClsf

        __floor: str = '1F'
        __eventInfo.floor = __floor

        __batteryLevel: int = self.current_battery_level
        __eventInfo.batteryLevel = __batteryLevel

        __eventInfo.location = __eventLocation_dict

        __eventInfo.subLocation = __eventSubLocation_dict

        return __eventInfo
    

    def __build_com_info(self) -> ComInfo:
        __comInfo: ComInfo = ComInfo()

        __status: str = ComInfoStatusType.CONNECTED.value
        __comInfo.status = __status

        __robotIP: str = self.__ip_address
        __comInfo.robotIP = __robotIP

        __mqttIP: str = self.__mqtt_config_parser.get('broker', 'host')
        __comInfo.mqttIP = __mqttIP

        __mqttPort: str = self.__mqtt_config_parser.get('broker', 'port')
        __comInfo.mqttPort = __mqttPort

        return __comInfo


    def __response_to_uvc(self) -> None:
        __built_event: Event = self.__build_event()
        self.__mqtt_client.publish(topic = self.__mqtt_event_publisher_topic, payload = json.dumps(__built_event.__dict__), qos = 0)
        

__all__ = ['rms_response_event_handler']