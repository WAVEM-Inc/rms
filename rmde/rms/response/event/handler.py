import socket
import json

from rclpy.node import Node
from rclpy.subscription import Subscription
from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import qos_profile_system_default
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import BatteryState
from robot_status_msgs.msg import SensorStatus
from robot_status_msgs.msg import VelocityStatus
from robot_status_msgs.msg import NavigationStatus

from ....mqtt.mqtt_client import Client

from ...common.service import UUIDService
from ...common.service import TimeService

from ...common.enum_types import RobotType
from ...common.enum_types import AreaCLSFType

from ...common.enum_types import JobGroupType
from ...common.enum_types import JobKindType
from ...common.enum_types import JobResultStatusType

from .domain import EventCdType
from .domain import ComInfoStatusType

from ...common.domain import Header
from .domain import Event
from .domain import TaskInfo
from .domain import JobResult
from .domain import EventInfo
from .domain import EventInfoLocation
from .domain import ComInfo


class EventResponseHandler():
    rclpy_ublox_fix_subscription_topic: str = '/ublox/fix'
    rclpy_battery_state_subscription_topic: str = '/battery/state'
    rclpy_imu_status_subscription_topic: str = '/imu/status'
    rclpy_scan_status_subscriptin_topic: str = '/scan/status'
    rclpy_ublox_fix_status_subscription_topic: str = '/ublox/fix/status'
    rclpy_battery_state_subscription_topic: str = '/battery/state/status'
    rclpy_gts_navigation_task_status_subscription_topic: str = '/gts_navigation/task_status'
    
    mqtt_event_publisher_topic: str = 'hubilon/atcplus/ros/rco0000000/rbt00000000/event'
    
    
    def __init__(self, rclpy_node: Node, mqtt_client: Client) -> None:
        self.rclpy_node: Node = rclpy_node
        
        self.ublox_fix_subscription_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup()
        self.ublox_fix_subscription: Subscription = self.rclpy_node.create_subscription(
            msg_type = NavSatFix,
            topic = self.rclpy_ublox_fix_subscription_topic,
            qos_profile = qos_profile_sensor_data,
            callback = self.rclpy_ublox_fix_subscription_cb,
            callback_group = self.ublox_fix_subscription_cb_group
        )

        self.battery_state_subscription_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup()
        self.battery_state_subscription: Subscription = self.rclpy_node.create_subscription(
            msg_type = BatteryState,
            topic = self.rclpy_battery_state_subscription_topic,
            qos_profile = qos_profile_sensor_data,
            callback = self.rclpy_battery_state_subscription_cb,
            callback_group = self.battery_state_subscription_cb_group
        )

        self.imu_status_subscription_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup()
        self.imu_status_subscription: Subscription = self.rclpy_node.create_subscription(
            msg_type = SensorStatus,
            topic = self.rclpy_imu_status_subscription_topic,
            qos_profile = qos_profile_system_default,
            callback = self.rclpy_imu_status_subscription_cb,
            callback_group = self.imu_status_subscription_cb_group
        )

        self.scan_status_subscription_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup()
        self.scan_status_subscription: Subscription = self.rclpy_node.create_subscription(
            msg_type = SensorStatus,
            topic = self.rclpy_scan_status_subscriptin_topic,
            qos_profile = qos_profile_system_default,
            callback = self.rclpy_scan_status_subscription_cb,
            callback_group = self.imu_status_subscription_cb_group
        )
        
        self.ublox_fix_status_subscription_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup()
        self.ublox_fix_status_subscription: Subscription = self.rclpy_node.create_subscription(
            msg_type = SensorStatus,
            topic = self.rclpy_ublox_fix_subscription_topic,
            qos_profile = qos_profile_system_default,
            callback = self.rclpy_ublox_fix_status_subscription_cb,
            callback_group = self.ublox_fix_status_subscription_cb_group
        )
        
        self.battery_state_status_subscription_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup()
        self.battery_state_status_subscription: Subscription = self.rclpy_node.create_subscription(
            msg_type = SensorStatus,
            topic = self.rclpy_battery_state_subscription_topic,
            qos_profile = qos_profile_system_default,
            callback = self.rclpy_battery_state_status_subscription_cb,
            callback_group = self.battery_state_status_subscription_cb_group
        )

        self.gts_navigation_task_status_subscription_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup()
        self.gts_navigation_task_status_subscription: Subscription = self.rclpy_node.create_subscription(
            msg_type = NavigationStatus,
            topic = self.rclpy_gts_navigation_task_status_subscription_topic,
            qos_profile = qos_profile_system_default,
            callback = self.rclpy_gts_navigation_task_status_subscription_cb,
            callback_group = self.gts_navigation_task_status_subscription_cb_group
        )

        self.host_name: str = socket.gethostname()
        self.ip_address: str = socket.gethostbyname(self.host_name)

        self.mqtt_client: Client = mqtt_client
        self.uuid_service: UUIDService = UUIDService()
        self.time_service: TimeService = TimeService()
        
        self.job_group: str = ''
        self.job_kind: str = ''
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
    
    
    def rclpy_ublox_fix_subscription_cb(self, ublox_fix_cb: NavSatFix) -> None:        
        self.location_xpos = ublox_fix_cb.latitude
        self.location_ypos = ublox_fix_cb.longitude
        self.heading = ublox_fix_cb.altitude
    
    
    def rclpy_battery_state_subscription_cb(self, battery_state_cb: BatteryState) -> None:
        self.job_start_battery_level = battery_state_cb.percentage
        self.job_end_battery_level = battery_state_cb.percentage
        self.current_battery_level = battery_state_cb.percentage

    
    def __report_sensor_broken_status__(self, sensor_type: str) -> None:
        self.sensor_status = EventCdType.BROKEN.value
        self.sensor_type = sensor_type
        self.response_to_uvc()
    

    def rclpy_imu_status_subscription_cb(self, imu_status_cb: SensorStatus) -> None:
        status_code: int = imu_status_cb.status_code
        
        if (status_code == -1000):
            self.__report_sensor_broken_status__('IMU')
        else:
            return
        
    
    def rclpy_scan_status_subscription_cb(self, scan_status_cb: SensorStatus) -> None:
        status_code: int = scan_status_cb.status_code
        
        if (status_code == -1001):
            self.__report_sensor_broken_status__('SCAN')
        else:
            return
        
        
    def rclpy_ublox_fix_status_subscription_cb(self, ublox_fix_status_cb: SensorStatus) -> None:
        status_code: int = ublox_fix_status_cb.status_code
        
        if (status_code == -1002):
            self.__report_sensor_broken_status__('GPS')
        else:
            return
        
    
    def rclpy_battery_state_status_subscription_cb(self, battery_state_status: SensorStatus) -> None:
        status_code: int = battery_state_status.status_code
        
        if (status_code == -1003):
            self.__report_sensor_broken_status__('BATTERY')
        else:
            return
        

    def rclpy_gts_navigation_task_status_subscription_cb(self, navigation_status: NavigationStatus) -> None:
        self.job_group: str = navigation_status.job_group
        self.job_kind: str = navigation_status.job_kind
        self.job_status: str = navigation_status.status
        self.job_start_time: str = navigation_status.start_time
        self.job_end_time: str = navigation_status.end_time
        self.job_start_battery_level: float = navigation_status.start_battery_level
        self.job_end_battery_level: float = navigation_status.end_battery_level
        self.job_start_dist: float = navigation_status.start_dist
        self.job_end_dist: float = navigation_status.end_dist
        self.response_to_uvc()
        

    def build_event(self) -> Event:
        header: Header = self.__build_header__()
        task_info: TaskInfo = self.__build__task_info__()
        event_info: EventInfo = self.__build_event_info__()
        com_info: ComInfo = self.__build_com_info__()

        event: Event = Event(
            header = header.__dict__,
            taskInfo = task_info.__dict__,
            eventInfo = event_info.__dict__,
            comInfo = com_info.__dict__
        )
        
        return event

    
    def __build_header__(self) -> Header:
        header: Header = Header(
            robotCorpId = 'rco0000001',
            workCorpId = 'wco0000001',
            workSiteId = 'wst0000001',
            robotId = 'rbt0000001',
            robotType = RobotType.AMR.value
        )

        return header
    

    def __build__task_info__(self) -> TaskInfo:
        job_plan_id: str = self.uuid_service.generate_uuid()
        job_group_id: str = self.uuid_service.generate_uuid()
        job_order_id: str = self.uuid_service.generate_uuid()
        
        formatted_datetime: str = self.time_service.get_current_datetime()

        job_result : JobResult = JobResult(
            status = JobResultStatusType.SUCCESS.value,
            startTime = formatted_datetime,
            endTime = formatted_datetime,
            startBatteryLevel = self.job_start_battery_level,
            endBatteryLevel = self.job_end_battery_level,
            dist = (self.job_end_dist - self.job_start_dist)
        )
        
        taskInfo: TaskInfo = TaskInfo(
            jobPlanId = job_plan_id,
            jobGroupId = job_group_id,
            jobOrderId = job_order_id,
            jobGroup = JobGroupType.SUPPLY.value,
            jobKind = JobKindType.MOVE.value,
            jobResult = job_result.__dict__
        )
        
        return taskInfo
    
    
    def __build_event_info__(self) -> EventInfo:
        event_location: EventInfoLocation = EventInfoLocation(
            xpos = self.location_xpos,
            ypos = self.location_ypos,
            heading = self.heading
        )
        
        event_id: str = self.uuid_service.generate_uuid()
        
        event_info: EventInfo = EventInfo(
            eventId = event_id,
            eventCd = self.sensor_status,
            eventSubCd = self.sensor_type,
            areaClsf = AreaCLSFType.INDOOR.value,
            floor = '1F',
            batteryLevel = self.current_battery_level,
            location = event_location.__dict__
        )

        return event_info
    

    def __build_com_info__(self) -> ComInfo:
        com_info: ComInfo = ComInfo(
            status = ComInfoStatusType.CONNECTED.value,
            robotIP = self.ip_address,
            mqttIP = self.mqtt_client.broker_address,
            mqttPort = self.mqtt_client.broker_port
        )

        return com_info


    def response_to_uvc(self) -> None:
        built_event: Event = self.build_event()
        self.mqtt_client.publish(topic = self.mqtt_event_publisher_topic, payload = json.dumps(built_event.__dict__), qos = 0)
        

__all__ = ['event_response_handler']