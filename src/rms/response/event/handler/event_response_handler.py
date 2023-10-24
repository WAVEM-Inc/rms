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
            'Event GPS cb\n\tlat : "%f"\n\tlon : "%f"\n\talt : "%f"' % (
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
            robotType = str(RobotType.AMR)
        )

        return header
    

    def __build__task_info__(self) -> TaskInfo:
        job_plan_id: str = self.uuid_service.generate_uuid()
        job_group_id: str = self.uuid_service.generate_uuid()
        job_order_id: str = self.uuid_service.generate_uuid()
        
        formatted_datetime: str = self.time_service.get_current_datetime()

        job_result : JobResult = JobResult(
            status = str(JobResultStatusType.SUCCESS),
            startTime = formatted_datetime,
            endTime = formatted_datetime,
            startBatteryLevel = 50,
            endBatteryLevel = 50,
            dist = 300
        )
        
        taskInfo: TaskInfo = TaskInfo(
            jobPlanId = job_plan_id,
            jobGroupId = job_group_id,
            jobOrderId = job_order_id,
            jobGroup = str(JobGroupType.SUPPLY),
            jobKind = str(JobKindType.MOVE),
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
            eventCd = str(EventCdType.BROKEN),
            eventSubCd = 'SUB001',
            areaClsf = str(AreaCLSFType.INDOOR),
            floor = '1F',
            batteryLevel = 50,
            location = event_location.__dict__
        )

        return event_info
    

    def __build_com_info__(self) -> ComInfo:
        com_info: ComInfo = ComInfo(
            status = str(ComInfoStatusType.CONNECTED),
            robotIP = '192.168.0.1',
            mqttIP = '33',
            mqttPort = '1883'
        )

        return com_info


    def response_to_uvc(self) -> None:
        built_event: Event = self.build_event()
        self.mqtt_broker.client.publish(
            self.mqtt_event_publisher_topic,
            json.dumps(built_event.__dict__)
        )
        

__all__ = ['event_response_handler']