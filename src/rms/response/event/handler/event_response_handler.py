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
from rms.response.event.domain.event import Event
from rms.common.domain.header import Header
from rms.response.event.domain.taskInfo.task_info import TaskInfo
from rms.response.event.domain.eventInfo.event_info import EventInfo
from rms.response.event.domain.comInfo.com_info import ComInfo
from rms.response.event.domain.taskInfo.jobResult.job_result import JobResult
from rms.common.application.uuid_service import UUIDService
from rms.common.application.time_service import TimeService
from rms.common.application.json_service import JsonEncoder
from rms.common.domain.type.robot_type import RobotType
from rms.common.domain.type.area_clsf_type import AreaCLSFType
from rms.response.event.domain.eventInfo.type.event_cd_type import EventCdType
from rms.response.event.domain.eventInfo.location.event_location import EventLocation
from rms.response.event.domain.comInfo.type.com_info_status import ComInfoStatusType
from rms.response.event.domain.comInfo.com_info import ComInfo

from typing import Dict


class EventResponseHandler():
    rclpy_gps_subscription_topic: str = '/ublox/fix'
    mqtt_event_publisher_topic: str = 'hubilon/atcplus/ros/event'
    
    def __init__(self, rclpy_node: Node, mqtt_broker: broker.mqtt_broker) -> None:
        self.rclpy_node: Node = rclpy_node
        
        self.gps_subscription_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup()
        self.gps_subscription: Subscription = self.rclpy_node.create_subscription(
            msg_type = NavSatFix,
            topic = self.rclpy_gps_subscription_topic,
            callback = self.rclpy_gps_subscription_cb,
            qos_profile = qos_profile_sensor_data,
            callback_group = self.gps_subscription_cb_group
        )

        self.imu_status_subscription_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup()
        self.imu_status_subscription: Subscription = self.rclpy_node.create_subscription(
            msg_type = SensorStatus,
            topic = "/imu/status",
            callback = self.rclpy_imu_status_subscription_cb,
            qos_profile = qos_profile_system_default,
            callback_group = self.imu_status_subscription_cb_group
        )

        self.scan_status_subscription_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup()
        self.scan_status_subscription: Subscription = self.rclpy_node.create_subscription(
            msg_type = SensorStatus,
            topic = "/scan/status",
            callback = self.rclpy_scan_status_subscription_cb,
            qos_profile = qos_profile_system_default,
            callback_group = self.imu_status_subscription_cb_group
        )

        self.mqtt_broker: broker.mqtt_broker = mqtt_broker
        self.uuid_service: UUIDService = UUIDService()
        self.time_service: TimeService = TimeService()
        
        self.location_xpos: float = 0.0
        self.location_ypos: float = 0.0
        self.heading: float = 45.0
    

    def rclpy_imu_status_subscription_cb(self, imu_status_cb: SensorStatus) -> None:
        self.rclpy_node.get_logger().info(
            'IMU Status\n\tcode : "%d"\n\tmessage : "%s"' % (
                imu_status_cb.status_code,
                imu_status_cb.status_message
            )
        )

    
    def rclpy_scan_status_subscription_cb(self, scan_status_cb: SensorStatus) -> None:
        self.rclpy_node.get_logger().info(
            'SCAN Status\n\tcode : "%d"\n\tmessage : "%s"' % (
                scan_status_cb.status_code,
                scan_status_cb.status_message
            )
        )


    def rclpy_gps_subscription_cb(self, gps_cb: NavSatFix) -> None:
        self.rclpy_node.get_logger().info(
            'GPS cb\n\tlat : "%f"\n\tlon : "%f"\n\talt : "%f"' % (
                gps_cb.latitude,
                gps_cb.longitude,
                gps_cb.altitude
            )
        )
        self.location_xpos = gps_cb.latitude
        self.location_ypos = gps_cb.longitude
        self.heading = gps_cb.altitude


    def build_event(self) -> Event:
        header: Header = self.__build_header__()
        taskInfo: TaskInfo = self.__build__task_info__()
        eventInfo: EventInfo = self.__build_event_info__()
        comInfo: ComInfo = self.__build_com_info__()

        event: Event = Event(
            header = header.__dict__,
            taskInfo = taskInfo.__dict__,
            eventInfo = eventInfo.__dict__,
            comInfo = comInfo.__dict__
        )
        
        return event

    
    def __build_header__(self) -> Header:
        header: Header = Header(
            robotCorpId = 'rco0000001',
            workCorpId = 'wco0000001',
            workSiteId = 'wst0000001',
            robotId = 'rbt0000001',
            robotType = str(RobotType.AMR)
        )

        return header
    

    def __build__task_info__(self) -> TaskInfo:
        formatted_datetime: str = self.time_service.get_current_datetime()

        job_result : JobResult = JobResult(
            status = "success",
            startTime = formatted_datetime,
            endTime = formatted_datetime,
            startBatteryLevel = 50,
            endBatteryLevel = 50,
            dist = 300
        )

        jobPlanId: str = self.uuid_service.generate_uuid()
        jobGroupId: str = self.uuid_service.generate_uuid()
        jobOrderId: str = self.uuid_service.generate_uuid()
        
        taskInfo: TaskInfo = TaskInfo(
            jobPlanId = jobPlanId,
            jobGroupId = jobGroupId,
            jobOrderId = jobOrderId,
            jobGroup = 'supply',
            jobKind = 'move',
            jobResult = job_result.__dict__
        )
        
        return taskInfo
    
    
    def __build_event_info__(self) -> EventInfo:
        eventId: str = self.uuid_service.generate_uuid()

        eventLocation: EventLocation = EventLocation(
            xpos = self.location_xpos,
            ypos = self.location_ypos,
            heading = self.heading
        )
        
        eventInfo: EventInfo = EventInfo(
            eventId = eventId,
            eventCd = str(EventCdType.BROKEN),
            eventSubCd = 'SUB001',
            areaClsf = str(AreaCLSFType.INDOOR),
            floor = '1F',
            batteryLevel = 50,
            location = eventLocation.__dict__
        )

        return eventInfo
    

    def __build_com_info__(self) -> ComInfo:
        comInfo: ComInfo = ComInfo(
            status = str(ComInfoStatusType.CONNECTED),
            robotIP = '192.168.0.1',
            mqttIP = '33',
            mqttPort = '1883'
        )

        return comInfo

    
    def response_to_uvc(self) -> None:
        built_event: Event = self.build_event()
        self.mqtt_broker.client.publish(
            self.mqtt_event_publisher_topic,
            json.dumps(built_event.__dict__)
        )