import rclpy
import json

from rclpy.node import Node
from rclpy.subscription import Subscription
from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import qos_profile_system_default
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from sensor_msgs.msg import NavSatFix
from robot_status_msgs.msg import SensorStatus

from mqtt import broker
from rms.common.application.uuid_service import UUIDService
from rms.common.application.time_service import TimeService
from rms.common.domain.type.robot.robot_type import RobotType
from rms.common.domain.type.area.area_clsf_type import AreaCLSFType
from rms.common.domain.type.job.job_group_type import JobGroupType
from rms.common.domain.type.job.job_kind_type import JobKindType
from rms.common.domain.type.job.job_result_status_type import JobResultStatusType
from rms.response.event.domain.eventInfo.type.event_cd_type import EventCdType
from rms.response.event.domain.comInfo.type.com_info_status import ComInfoStatusType

from rms.common.domain.header import Header
from rms.response.event.domain.event import Event
from rms.response.event.domain.taskInfo.task_info import TaskInfo
from rms.response.event.domain.taskInfo.jobResult.job_result import JobResult
from rms.response.event.domain.eventInfo.event_info import EventInfo
from rms.response.event.domain.eventInfo.location.event_info_location import EventInfoLocation
from rms.response.event.domain.comInfo.com_info import ComInfo


class EventResponseHandler():
    rclpy_ublox_fix_subscription_topic: str = '/ublox/fix'
    rclpy_imu_status_subscription_topic: str = '/imu/status'
    rclpy_scan_status_subscriptin_topic: str = '/scan/status'
    rclpy_ublox_fix_status_subscription_topic: str = '/ublox/fix/status'
    rclpy_battery_state_subscription_topic: str = '/battery/status'
    
    mqtt_event_publisher_topic: str = 'hubilon/atcplus/ros/event'
    
    
    def __init__(self, rclpy_node: Node, mqtt_broker: broker.mqtt_broker) -> None:
        self.rclpy_node: Node = rclpy_node
        
        self.ublox_fix_subscription_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup()
        self.ublox_fix_subscription: Subscription = self.rclpy_node.create_subscription(
            msg_type = NavSatFix,
            topic = self.rclpy_ublox_fix_subscription_topic,
            callback = self.rclpy_ublox_fix_subscription_cb,
            qos_profile = qos_profile_sensor_data,
            callback_group = self.ublox_fix_subscription_cb_group
        )

        self.imu_status_subscription_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup()
        self.imu_status_subscription: Subscription = self.rclpy_node.create_subscription(
            msg_type = SensorStatus,
            topic = self.rclpy_imu_status_subscription_topic,
            callback = self.rclpy_imu_status_subscription_cb,
            qos_profile = qos_profile_system_default,
            callback_group = self.imu_status_subscription_cb_group
        )

        self.scan_status_subscription_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup()
        self.scan_status_subscription: Subscription = self.rclpy_node.create_subscription(
            msg_type = SensorStatus,
            topic = self.rclpy_scan_status_subscriptin_topic,
            callback = self.rclpy_scan_status_subscription_cb,
            qos_profile = qos_profile_system_default,
            callback_group = self.imu_status_subscription_cb_group
        )
        
        self.ublox_fix_status_subscription_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup()
        self.ublox_fix_status_subscription: Subscription = self.rclpy_node.create_subscription(
            msg_type = SensorStatus,
            topic = self.rclpy_ublox_fix_subscription_topic,
            callback = self.rclpy_ublox_fix_status_subscription_cb,
            qos_profile = qos_profile_system_default,
            callback_group = self.ublox_fix_status_subscription_cb_group
        )
        
        self.battery_state_status_subscription_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup()
        self.battery_state_status_subscription: Subscription = self.rclpy_node.create_subscription(
            msg_type = SensorStatus,
            topic = self.rclpy_battery_state_subscription_topic,
            callback = self.rclpy_battery_state_status_subscription_cb,
            qos_profile = qos_profile_system_default,
            callback_group = self.battery_state_status_subscription_cb_group
        )

        self.mqtt_broker: broker.mqtt_broker = mqtt_broker
        self.uuid_service: UUIDService = UUIDService()
        self.time_service: TimeService = TimeService()
        
        self.start_battery_level: int = 0
        self.end_battery_level: int = 0
        self.dist: int = 0
        self.sensor_status: str = ""
        self.location_xpos: float = 0.0
        self.location_ypos: float = 0.0
        self.heading: float = 45.0
    
    
    def rclpy_ublox_fix_subscription_cb(self, ublox_fix_cb: NavSatFix) -> None:
        self.rclpy_node.get_logger().info(
            'Event GPS cb\n\tlat : "%f"\n\tlon : "%f"\n\talt : "%f"' % (
                ublox_fix_cb.latitude,
                ublox_fix_cb.longitude,
                ublox_fix_cb.altitude
            )
        )
        
        self.location_xpos = ublox_fix_cb.latitude
        self.location_ypos = ublox_fix_cb.longitude
        self.heading = ublox_fix_cb.altitude
    
    
    def __report_sensor_broken_status__(self, sensor_name: str) -> None:
        self.sensor_status = EventCdType.BROKEN.value
        self.rclpy_node.get_logger().info('Event %s Sensor Broken "%s"' % (sensor_name, self.sensor_status))
        self.response_to_uvc()
    

    def rclpy_imu_status_subscription_cb(self, imu_status_cb: SensorStatus) -> None:
        status_code: int = imu_status_cb.status_code
        status_message: str = imu_status_cb.status_message
        
        self.rclpy_node.get_logger().info(
            'Event IMU Status\n\tcode : "%d"\n\tmessage : "%s"' % (
                status_code,
                status_message
            )
        )
        
        if (status_code == -1000):
            self.__report_sensor_broken_status__('IMU')
        else:
            return
        
    
    def rclpy_scan_status_subscription_cb(self, scan_status_cb: SensorStatus) -> None:
        status_code: int = scan_status_cb.status_code
        status_message: str = scan_status_cb.status_message
        
        self.rclpy_node.get_logger().info(
            'Event SCAN Status\n\tcode : "%d"\n\tmessage : "%s"' % (
                status_code,
                status_message
            )
        )
        
        if (status_code == -1001):
            self.__report_sensor_broken_status__('SCAN')
        else:
            return
        
    
    def rclpy_ublox_fix_status_subscription_cb(self, ublox_fix_status_cb: SensorStatus) -> None:
        status_code: int = ublox_fix_status_cb.status_code
        status_message: str = ublox_fix_status_cb.status_message
        
        self.rclpy_node.get_logger().info(
            'Event UbloxFix Status\n\tcode : "%d"\n\tmessage : "%s"' % (
                status_code,
                status_message
            )
        )
        
        if (status_code == -1002):
            self.__report_sensor_broken_status__('GPS')
        else:
            return
        
    
    def rclpy_battery_state_status_subscription_cb(self, battery_state_status: SensorStatus) -> None:
        status_code: int = battery_state_status.status_code
        status_message: str = battery_state_status.status_message
        
        self.rclpy_node.get_logger().info(
            'Event BatteryState Status\n\tcode : "%d"\n\tmessage : "%s"' % (
                status_code,
                status_message
            )
        )
        
        if (status_code == -1003):
            self.__report_sensor_broken_status__('BATTERY')
        else:
            return
        

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
            startBatteryLevel = self.start_battery_level,
            endBatteryLevel = self.end_battery_level,
            dist = 300
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
            eventSubCd = 'SUB001',
            areaClsf = AreaCLSFType.INDOOR.value,
            floor = '1F',
            batteryLevel = 50,
            location = event_location.__dict__
        )

        return event_info
    

    def __build_com_info__(self) -> ComInfo:
        com_info: ComInfo = ComInfo(
            status = ComInfoStatusType.CONNECTED.value,
            robotIP = '192.168.0.1',
            mqttIP = '33',
            mqttPort = '1883'
        )

        return com_info


    def response_to_uvc(self) -> None:
        built_event: Event = self.build_event()
        self.mqtt_broker.client.publish(self.mqtt_event_publisher_topic, json.dumps(built_event.__dict__))
        

__all__ = ['event_response_handler']