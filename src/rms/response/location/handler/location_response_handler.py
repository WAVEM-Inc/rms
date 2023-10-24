import rclpy
import json

from rclpy.node import Node
from rclpy.subscription import Subscription
from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import qos_profile_system_default
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from sensor_msgs.msg import NavSatFix

from mqtt import broker
from rms.common.application.uuid_service import UUIDService
from rms.common.application.time_service import TimeService
from rms.common.domain.type.robot.robot_type import RobotType
from rms.common.domain.type.job.job_group_type import JobGroupType
from rms.common.domain.type.job.job_kind_type import JobKindType
from rms.common.domain.type.job.task_status_type import TaskStatusType
from rms.common.domain.type.area.area_clsf_type import AreaCLSFType

from rms.common.domain.header import Header
from rms.response.location.domain.location import Location
from rms.response.location.domain.jobInfo.job_info import JobInfo
from rms.response.location.domain.jobInfo.taskInfo.task_info import TaskInfo
from rms.response.location.domain.lastInfo.last_info import LastInfo
from rms.response.location.domain.lastInfo.location.last_info_location import LastInfoLocation


class LocationResponseHandler():
    rclpy_gps_subscription_topic: str = '/ublox/fix'
    mqtt_location_publisher_topic: str = 'hubilon/atcplus/ros/location'
    
    
    def __init__(self, rclpy_node: Node, mqtt_broker: broker.mqtt_broker) -> None:
        self.rclpy_node = rclpy_node
        
        self.gps_subscription_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup()
        self.gps_subscription: Subscription = self.rclpy_node.create_subscription(
            msg_type = NavSatFix,
            topic = self.rclpy_gps_subscription_topic,
            callback = self.rclpy_gps_subscription_cb,
            qos_profile = qos_profile_sensor_data,
            callback_group = self.gps_subscription_cb_group
        )
        
        self.mqtt_broker: broker.mqtt_broker = mqtt_broker
        self.uuid_service: UUIDService = UUIDService()
        self.time_service: TimeService = TimeService()
        
        self.location_xpos: float = 0.0
        self.location_ypos: float = 0.0
        self.heading: float = 45.0
        
        
    def rclpy_gps_subscription_cb(self, gps_cb: NavSatFix) -> None:
        self.rclpy_node.get_logger().info(
            'Location GPS cb\n\tlat : "%f"\n\tlon : "%f"\n\talt : "%f"' % (
                gps_cb.latitude,
                gps_cb.longitude,
                gps_cb.altitude
            )
        )
        self.location_xpos = gps_cb.latitude
        self.location_ypos = gps_cb.longitude
        self.heading = gps_cb.altitude
    
    
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
        header: Header = Header(
            robotCorpId = 'rco0000001',
            workCorpId = 'wco0000001',
            workSiteId = 'wst0000001',
            robotId = 'rbt0000001',
            robotType = str(RobotType.AMR)
        )

        return header
    

    def __build_job_info__(self) -> JobInfo:
        job_plan_id: str = self.uuid_service.generate_uuid()
        job_group_id: str = self.uuid_service.generate_uuid()
        job_order_id: str = self.uuid_service.generate_uuid()
        
        task_info: TaskInfo = TaskInfo(
            jobGroup = str(JobGroupType.SUPPLY),
            jobKind = str(JobKindType.MOVE),
            taskStatus = str(TaskStatusType.ASSIGNED)
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
            areaClsf = str(AreaCLSFType.INDOOR),
            floor = "1F",
            batteryLevel = 50,
            velocity = 1.5,
            totalDist = 300
        )
        
        return last_info
    
    
    def response_to_uvc(self) -> None:
        built_location: Location = self.build_location()
        self.mqtt_broker.client.publish(
            self.mqtt_location_publisher_topic,
            json.dumps(built_location.__dict__)
        )