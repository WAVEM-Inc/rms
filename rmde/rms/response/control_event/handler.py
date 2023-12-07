import os
import json
import paho.mqtt.client as mqtt

from configparser import ConfigParser
from rclpy.node import Node
from rclpy.subscription import Subscription
from rclpy.qos import qos_profile_system_default
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from ....mqtt.mqtt_client import Client
from ...common.service import ConfigService

from robot_status_msgs.msg import TaskStatus

from ...common.domain import Header
from .domain import ControlResult
from .domain import TaskEventInfo
from .domain import ControlEvent


class ControlEventHandler():

    def __init__(self, rclpy_node: Node, mqtt_client: Client) -> None:
        self.__rclpy_node: Node = rclpy_node
        self.__mqtt_client: Client = mqtt_client

        self.__script_directory: str = os.path.dirname(
            os.path.abspath(__file__))
        self.__mqtt_config_file_path: str = '../../../mqtt/mqtt.ini'
        self.__mqtt_config_service: ConfigService = ConfigService(
            self.__script_directory, self.__mqtt_config_file_path)
        self.__mqtt_config_parser: ConfigParser = self.__mqtt_config_service.read()

        self.__mqtt_control_event_publisher_topic: str = self.__mqtt_config_parser.get(
            'topics', 'control_event')
        self.__mqtt_control_event_publisher_qos: int = int(
            self.__mqtt_config_parser.get('qos', 'control_event'))

        if self.__mqtt_client.is_connected:
            self.__rclpy_node.get_logger().info(
                f'MQTT granted publisher\n\ttopic : {self.__mqtt_control_event_publisher_topic}\n\tqos : {self.__mqtt_control_event_publisher_qos}')
        else:
            self.__rclpy_node.get_logger().error(
                f'MQTT failed to grant publisher\n\ttopic : {self.__mqtt_control_event_publisher_topic}\n\tqos : {self.__mqtt_control_event_publisher_topic}')

        self.__common_config_file_path: str = '../../common/config.ini'
        self.__common_config_service: ConfigService = ConfigService(
            self.__script_directory, self.__common_config_file_path)
        self.__common_config_parser: ConfigParser = self.__common_config_service.read()

        self.__rclpy_task_status_subscription_topic: str = '/robot_task/status'
        self.__rclpy_task_status_subscription_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup()
        self.__rclpy_task_status_subscription: Subscription = self.__rclpy_node.create_subscription(
            msg_type=TaskStatus,
            topic=self.__rclpy_task_status_subscription_topic,
            qos_profile=qos_profile_system_default,
            callback_group=self.__rclpy_task_status_subscription_cb_group,
            callback=self.__rclpy_task_status_subscription_cb
        )

        self.__header: Header = Header()
        self.control_result: ControlResult = ControlResult()
        self.__task_event_info: TaskEventInfo = TaskEventInfo()

        self.__job_group: str = ''
        self.__job_kind: str = ''
        self.__job_plan_id: str = ''
        self.__job_group_id: str = ''
        self.__job_order_id: str = ''

    def __rclpy_task_status_subscription_cb(self, task_status_cb: TaskStatus) -> None:
        self.__job_group = task_status_cb.job_group
        self.__job_kind = task_status_cb.job_kind
        self.__job_plan_id = task_status_cb.job_plan_id
        self.__job_group_id = task_status_cb.job_group_id
        self.__job_order_id = task_status_cb.job_order_id

    def __build_control_event(self) -> ControlEvent:
        control_event: ControlEvent = ControlEvent()

        self.__build_header()
        control_event.header = self.__header.__dict__
        control_event.controlResult = self.control_result.__dict__

        self.__build_task_event_info()
        control_event.taskEventInfo = self.__task_event_info.__dict__

        return control_event

    def __build_header(self) -> None:
        robot_corp_id: str = self.__common_config_parser.get(
            'header', 'robotCorpId')
        self.__header.robotCorpId = robot_corp_id

        work_corp_id: str = self.__common_config_parser.get(
            'header', 'workCorpId')
        self.__header.workCorpId = work_corp_id

        work_site_id: str = self.__common_config_parser.get(
            'header', 'workSiteId')
        self.__header.workSiteId = work_site_id

        robot_id: str = self.__common_config_parser.get('header', 'robotId')
        self.__header.robotId = robot_id

        robot_type: str = self.__common_config_parser.get(
            'header', 'robotType')
        self.__header.robotType = robot_type

    def __build_task_event_info(self) -> None:
        self.__task_event_info.jobPlanId = self.__job_plan_id
        self.__task_event_info.jobGroupId = self.__job_group_id
        self.__task_event_info.jobOrderId = self.__job_order_id
        self.__task_event_info.jobGroup = self.__job_group
        self.__task_event_info.jobKind = self.__job_kind

    def response_to_rms(self) -> None:
        built_control_event: ControlEvent = self.__build_control_event()
        self.__mqtt_client.publish(topic=self.__mqtt_control_event_publisher_topic, payload=json.dumps(
            built_control_event.__dict__), qos=self.__mqtt_control_event_publisher_qos)


__all__ = ['rms_response_control_event_handler']
