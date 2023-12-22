import os
import json
import rclpy.client

from configparser import ConfigParser
from rclpy.node import Node
from rclpy.subscription import Subscription
from rclpy.qos import qos_profile_system_default
from rclpy.qos import qos_profile_services_default
from rclpy.task import Future
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from robot_status_msgs.msg import NavigationStatus
from robot_status_msgs.msg import TaskStatus
from robot_status_msgs.srv import RegisterTask

from ....mqtt.mqtt_client import Client

from ...common.service import ConfigService
from ...common.service import UUIDService

from ...common.domain import Header

from .domain import TaskEvent
from .domain import TaskEventInfo
from .domain import JobResult

from typing import Any


GTS_NAVIGATION_STARTED_CODE: int = 2
GTS_NAVIGATION_COMPLETED_CODE: int = 4

TASK_STATUS_REGISTER_KEY: str = 'qAqwmrwskdfliqnwkfnlasdkfnlas'


class TaskEventResponseHandler():

    def __init__(self, rclpy_node: Node, mqtt_client: Client) -> None:
        self.__rclpy_node: Node = rclpy_node
        self.__mqtt_client: Client = mqtt_client

        self.__script_directory: str = os.path.dirname(os.path.abspath(__file__))
        self.__mqtt_config_file_path: str = '../../../mqtt/mqtt.ini'
        self.__mqtt_config_service: ConfigService = ConfigService(self.__script_directory, self.__mqtt_config_file_path)
        self.__mqtt_config_parser: ConfigParser = self.__mqtt_config_service.read()

        self.__mqtt_task_event_publisher_topic: str = self.__mqtt_config_parser.get('topics', 'task_event')
        self.__mqtt_task_event_publisher_qos: int = int(self.__mqtt_config_parser.get('qos', 'task_event'))

        if self.__mqtt_client.is_connected:
            self.__rclpy_node.get_logger().info(f'MQTT granted publisher\n\ttopic : {self.__mqtt_task_event_publisher_topic}\n\tqos : {self.__mqtt_task_event_publisher_qos}')
        else:
            self.__rclpy_node.get_logger().error(f'MQTT failed to grant publisher\n\ttopic : {self.__mqtt_task_event_publisher_topic}\n\tqos : {self.__mqtt_task_event_publisher_qos}')

        self.__common_config_file_path: str = '../../common/config.ini'
        self.__common_config_service: ConfigService = ConfigService(self.__script_directory, self.__common_config_file_path)
        self.__common_config_parser: ConfigParser = self.__common_config_service.read()

        self.__rclpy_gts_navigation_task_status_subscription_topic: str = '/gts_navigation/task_status'
        self.__rclpy_gts_navigation_task_status_subscription_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup()
        self.__rclpy_gts_navigation_task_status_subscription: Subscription = self.__rclpy_node.create_subscription(
            msg_type=NavigationStatus,
            topic=self.__rclpy_gts_navigation_task_status_subscription_topic,
            qos_profile=qos_profile_system_default,
            callback=self.__rclpy_gts_navigation_task_status_subscription_cb,
            callback_group=self.__rclpy_gts_navigation_task_status_subscription_cb_group
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

        self.__rclpy_register_task_service_server_name: str = '/robot_task/register'
        self.__rclpy_register_task_service_client_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup()
        self.__rclpy_register_task_service_client: rclpy.client.Client = self.__rclpy_node.create_client(
            srv_name=self.__rclpy_register_task_service_server_name,
            srv_type=RegisterTask,
            qos_profile=qos_profile_services_default,
            callback_group=self.__rclpy_register_task_service_client_cb_group
        )
        self.__uuid_service: UUIDService = UUIDService()
        
        self.__header: Header = Header()
        self.__job_result: JobResult = JobResult()
        self.__task_event_info: TaskEventInfo = TaskEventInfo()

        self.__job_group: str = ''
        self.__job_kind: str = ''
        self.__job_plan_id: str = ''
        self.__job_group_id: str = ''
        self.__job_order_id: str = ''

    def __rclpy_gts_navigation_task_status_subscription_cb(self, navigation_status: NavigationStatus) -> None:
        self.job_status_code = navigation_status.status_code

        if self.job_status_code == GTS_NAVIGATION_STARTED_CODE:
            self.__task_event_info.jobGroup = navigation_status.job_group
            self.__task_event_info.jobKind = navigation_status.job_kind
        elif self.job_status_code == GTS_NAVIGATION_COMPLETED_CODE:
            import time
            time.sleep(0.7)
            self.__task_event_info.jobPlanId = self.__job_plan_id
            self.__task_event_info.jobGroupId = self.__job_group_id
            self.__task_event_info.jobOrderId = self.__job_order_id
            self.__task_event_info.jobGroup = self.__job_group
            self.__task_event_info.jobKind = self.__job_kind
            self.__job_result.status = navigation_status.status
            self.__job_result.startTime = navigation_status.start_time
            self.__job_result.endTime = navigation_status.end_time
            self.__job_result.startBatteryLevel = navigation_status.start_battery_level
            self.__job_result.endBatteryLevel = navigation_status.end_battery_level
            self.__job_result.dist = abs(navigation_status.end_dist - navigation_status.start_dist)
            self.__task_event_info.jobResult = self.__job_result.__dict__
            self.__rclpy_node.get_logger().info(f'TaskEventHandler [{self.__task_event_info.jobOrderId}] task completed with status : {self.__job_result.status}')
            self.__response_to_rms()
            self.__rclpy_register_task_service_request()
        else:
            return

    def __rclpy_task_status_subscription_cb(self, task_status_cb: TaskStatus) -> None:
        self.__job_group = task_status_cb.job_group
        self.__job_kind = task_status_cb.job_kind
        self.__job_plan_id = task_status_cb.job_plan_id
        self.__job_group_id = task_status_cb.job_group_id
        self.__job_order_id = task_status_cb.job_order_id

    def __rclpy_register_task_service_request(self) -> Any:
        self.__rclpy_node.get_logger().info('TaskEventHandler Task has been completed')

        rclpy_register_task_request: RegisterTask.Request = RegisterTask.Request()
        rclpy_register_task_request.register_key = TASK_STATUS_REGISTER_KEY
        rclpy_register_task_request.job_group = ''
        rclpy_register_task_request.job_kind = ''
        rclpy_register_task_request.job_plan_id = ''
        rclpy_register_task_request.job_group_id = ''
        rclpy_register_task_request.job_order_id = ''

        rclpy_register_task_request_future: Future = self.__rclpy_register_task_service_client.call_async(rclpy_register_task_request)

        return rclpy_register_task_request_future.result()

    def __build_task_event(self) -> TaskEvent:
        task_event: TaskEvent = TaskEvent()

        self.__build_header()
        task_event.header = self.__header.__dict__

        task_event.taskEventInfo = self.__task_event_info.__dict__

        return task_event

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

    def __response_to_rms(self) -> None:
        build_task_event: TaskEvent = self.__build_task_event()
        self.__mqtt_client.publish(topic=self.__mqtt_task_event_publisher_topic, payload=json.dumps(build_task_event.__dict__), qos=self.__mqtt_task_event_publisher_qos)


__all__ = ['rms_response_task_event_handler']
