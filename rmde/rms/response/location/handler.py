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
# from robot_type_msgs.srv import Type
from uuid_gateway_msgs.msg import UUID

from ....mqtt.mqtt_client import Client

from ...common.service import ConfigService

from ...common.enum_types import AreaCLSFType
from ...common.enum_types import JobGroupType
from ...common.enum_types import JobKindType
from ...common.enum_types import TaskStatusType

from ...common.domain import Header
from ...common.domain import JobUUID

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

        self.__script_directory: str = os.path.dirname(os.path.abspath(__file__))
        self.__mqtt_config_file_path: str = '../../../mqtt/mqtt.ini'
        self.__mqtt_config_service: ConfigService = ConfigService(self.__script_directory, self.__mqtt_config_file_path)
        self.__mqtt_config_parser: ConfigParser = self.__mqtt_config_service.read()
        self.__mqtt_location_publisher_topic: str = self.__mqtt_config_parser.get('topics', 'location')
        self.__mqtt_location_publisher_qos: int = int(self.__mqtt_config_parser.get('qos', 'location'))

        self.__rclpy_node.get_logger().info('MQTT granted publisher\n\ttopic : {%s}\n\tqos : {%d}' % (self.__mqtt_location_publisher_topic, self.__mqtt_location_publisher_qos))

        self.__common_config_file_path: str = '../../common/config.ini'
        self.__common_config_service: ConfigService = ConfigService(self.__script_directory, self.__common_config_file_path)
        self.__common_config_parser: ConfigParser = self.__common_config_service.read()
        
        # self.__rclpy_robot_type_select_service_server_name: str = '/robot_type/service'
        # self.__rclpy_robot_type_select_service_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup()
        # self.__rclpy_robot_type_select_service_client: Client = self.__rclpy_node.create_client(
        #     srv_type = Type,
        #     srv_name = self.__rclpy_robot_type_select_service_server_name,
        #     qos_profile = qos_profile_system_default,
        #     callback_group = self.__rclpy_robot_type_select_service_cb_group
        # )

        self.__rclpy_slam_to_gps_subscription_topic: str = '/slam_to_gps'
        self.__rclpy_slam_to_gps_subscription_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup()
        self.__rclpy_slam_to_gps_subscription: Subscription = self.__rclpy_node.create_subscription(
            msg_type = NavSatFix,
            topic = self.__rclpy_slam_to_gps_subscription_topic,
            callback = self.__rclpy_slam_to_gps_subscription_cb,
            qos_profile = qos_profile_sensor_data,
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
            callback = self.__rclpy_battery_state_subscription_cb,
            qos_profile = qos_profile_sensor_data,
            callback_group = self.__rclpy_battery_state_subscription_cb_group
        )
        
        self.__rclpy_velocity_state_subscription_topic: str = '/velocity/state'
        self.__rclpy_velocity_state_subscription_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup()
        self.__rclpy_velocity_state_subscription: Subscription = self.__rclpy_node.create_subscription(
            msg_type = VelocityStatus,
            topic = self.__rclpy_velocity_state_subscription_topic,
            callback = self.__rclpy_velocity_state_subscription_cb,
            qos_profile = qos_profile_sensor_data,
            callback_group = self.__rclpy_velocity_state_subscription_cb_group
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

        self.__rclpy_get_uuid_subscription_topic: str = '/uuid_gateway_server/get_uuid'
        self.__rclpy_get_uuid_subscription_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup()
        self.__rclpy_get_uuid_subscription: Subscription = self.__rclpy_node.create_subscription(
            msg_type = UUID,
            topic = self.__rclpy_get_uuid_subscription_topic,
            qos_profile = qos_profile_system_default,
            callback_group = self.__rclpy_get_uuid_subscription_cb_group,
            callback = self.__rclpy_get_uuid_subscription_cb
        )
        
        self.location_xpos: float = 0.0
        self.location_ypos: float = 0.0
        self.heading: float = 0.0
        self.sub_location_xpos: float = 0.0
        self.sub_location_ypos: float = 0.0
        self.battery_level: float = 0.0
        self.velocity: float = 0.0
        self.total_dist: int = 0
        self.areaClsf: str = ''
        self.job_status_code: int = 0
        self.job_group: str = ''
        self.job_kind: str = ''
        self.task_status: str = ''

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
        self.battery_level = battery_state_cb.percentage
    
    
    def __rclpy_velocity_state_subscription_cb(self, velocity_state_cb: VelocityStatus) -> None:
        __current_velocity: float = velocity_state_cb.current_velocity
        __distance: float = velocity_state_cb.distance
        
        self.velocity = __current_velocity
        self.total_dist = __distance


    def __rclpy_gts_navigation_task_status_subscription_cb(self, navigation_status: NavigationStatus) -> None:
        self.job_status_code = navigation_status.status_code

        if self.job_status_code == GTS_NAVIGATION_STARTED_CODE:
            self.job_group = JobGroupType.MOVE.value
            self.job_kind = JobKindType.MOVE.value
            self.task_status = TaskStatusType.ASSIGNED.value
        elif self.job_status_code == GTS_NAVIGATION_COMPLETED_CODE:
            self.job_group = JobGroupType.WAIT.value
            self.job_kind = JobKindType.WAIT.value
            self.task_status = TaskStatusType.UNASSIGNED.value
        else:
            return

    
    def __rclpy_in_out_door_subscription_cb(self, in_out_door_cb: InOutDoor) -> None:
        is_out_door: bool = (in_out_door_cb.determination == True)

        if is_out_door:
            self.areaClsf = AreaCLSFType.OUTDOOR.value
        else :
            self.areaClsf = AreaCLSFType.INDOOR.value
        return
    

    def __rclpy_get_uuid_subscription_cb(self, get_uuid_cb: UUID) -> None:
        self.job_plan_id = get_uuid_cb.job_plan_id
        self.job_group_id = get_uuid_cb.job_group_id
        self.job_order_id = get_uuid_cb.job_order_id

    
    def build_location(self) -> Location:
        __location: Location = Location()

        __header: Header = self.__build_header()
        __header_dict: dict = __header.__dict__
        __location.header = __header_dict

        __jobInfo: JobInfo = self.__build_job_info()
        __jobInfo_dict: dict = __jobInfo.__dict__
        __location.jobInfo = __jobInfo_dict
        
        __lastInfo: LastInfo = self.__build_last_info()
        __lastInfo_dict: dict = __lastInfo.__dict__
        __location.lastInfo = __lastInfo_dict
        
        return __location
        
        
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
    

    def __build_job_info(self) -> JobInfo:
        __taskInfo: TaskInfo = TaskInfo()

        __job_group: str = self.job_group
        __taskInfo.jobGroup = __job_group

        __job_kind: str = self.job_kind
        __taskInfo.jobKind = __job_kind

        __task_status: str = self.task_status
        __taskInfo.taskStatus = __task_status

        __task_info_dict: dict = __taskInfo.__dict__

        __jobInfo: JobInfo = JobInfo()

        __jobPlanId: str = self.job_plan_id
        __jobInfo.jobPlanId = __jobPlanId

        __jobGroupId: str = self.job_group_id
        __jobInfo.jobGroupId = __jobGroupId

        __jobOrderId: str = self.job_order_id
        __jobInfo.jobOrderId = __jobOrderId

        __taskInfo: dict = __task_info_dict
        __jobInfo.taskInfo = __taskInfo

        return __jobInfo
    
    
    def __build_last_info(self) -> LastInfo:
        __lastInfoLocation: LastInfoLocation = LastInfoLocation()

        __xpos: float = self.location_xpos
        __lastInfoLocation.xpos = __xpos

        __ypos: float = self.location_ypos
        __lastInfoLocation.ypos = __ypos

        __heading: float = self.heading
        __lastInfoLocation.heading = __heading

        __lastInfoLocation_dict: dict = __lastInfoLocation.__dict__

        __lastInfoSubLocation: LastInfoSubLocation = LastInfoSubLocation()
        
        __sub_xpos: float = self.sub_location_xpos
        __lastInfoSubLocation.xpos = __sub_xpos

        __sub_ypos: float = self.sub_location_ypos
        __lastInfoSubLocation.ypos = __sub_ypos

        __lastInfoSubLocation_dict: dict = __lastInfoSubLocation.__dict__
        
        __lastInfo: LastInfo = LastInfo()

        __lastInfo.location = __lastInfoLocation_dict

        __lastInfo.subLocation = __lastInfoSubLocation_dict

        __lastInfo.areaClsf = self.areaClsf
        
        __floor: str = '1F'
        __lastInfo.floor = __floor

        __batteryLevel: int = self.battery_level
        __lastInfo.batteryLevel = __batteryLevel

        __velocity: float = abs(self.velocity)
        __lastInfo.velocity = __velocity
        
        __totalDist: int = self.total_dist
        __lastInfo.totalDist = __totalDist
        
        return __lastInfo
    
    
    def response_to_uvc(self) -> None:
        built_location: Location = self.build_location()
        self.__mqtt_client.publish(topic = self.__mqtt_location_publisher_topic, payload = json.dumps(built_location.__dict__), qos = self.__mqtt_location_publisher_qos)
        

__all__ = ['rms_response_location_handler']