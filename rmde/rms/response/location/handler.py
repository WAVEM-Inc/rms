import rclpy
import json

from rclpy.node import Node
from rclpy.subscription import Subscription
from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import qos_profile_system_default
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import BatteryState
from robot_status_msgs.msg import VelocityStatus

from ....mqtt import mqtt_client

from ...common.service import UUIDService
from ...common.service import TimeService

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


class LocationResponseHandler():
    rclpy_gps_subscription_topic: str = '/ublox/fix'
    rclpy_battery_state_subscription_topic: str = '/battery/state'
    rclpy_velocity_state_subscription_topic: str = '/velocity/state'
    
    mqtt_location_publisher_topic: str = 'hubilon/atcplus/ros/rco0000000/rbt00000000/location'
    
    
    def __init__(self, rclpy_node: Node, mqtt_broker: mqtt_client.Client) -> None:
        self.rclpy_node = rclpy_node
        
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
    
    
    def rclpy_battery_state_subscription_cb(self, battery_state_cb: BatteryState) -> None:
        self.rclpy_node.get_logger().info(
            'Location BatteryState cb'
        )
    
    
    def rclpy_velocity_state_subscription_cb(self, velocity_state_cb: VelocityStatus) -> None:
        current_velocity: float = velocity_state_cb.current_velocity
        distance: float = velocity_state_cb.distance
        
        self.rclpy_node.get_logger().info(
            'Location VelocityState cb\n\tcurrent_velocity : "%f"\n\tdistance : "%f"' % (
                current_velocity,
                distance
            )
        )
        
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
        header: Header = Header(
            robotCorpId = 'rco0000001',
            workCorpId = 'wco0000001',
            workSiteId = 'wst0000001',
            robotId = 'rbt0000001',
            robotType = RobotType.AMR.value
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
            floor = "1F",
            batteryLevel = self.battery_level,
            velocity = self.velocity,
            totalDist = self.total_dist
        )
        
        return last_info
    
    
    def response_to_uvc(self) -> None:
        built_location: Location = self.build_location()
        self.mqtt_broker.publish(topic = self.mqtt_location_publisher_topic, payload = json.dumps(built_location.__dict__), qos = 0)
        

__all__ = ['location_response_handler']