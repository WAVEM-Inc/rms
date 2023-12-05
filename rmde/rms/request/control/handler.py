import os
import json
import paho.mqtt.client as mqtt

from configparser import ConfigParser
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.subscription import Subscription
from rclpy.qos import qos_profile_system_default
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from gts_navigation_msgs.msg import NavigationControl
from robot_status_msgs.msg import NavigationStatus

from ....mqtt.mqtt_client import Client
from ...common.service import ConfigService
from ...common.service import TimeService

from ...response.control_event.handler import ControlEventHandler
from ...response.control_event.domain import ControlCmdType

from ...response.cmd_response.handler import CmdRepsonseHandler
from ...response.cmd_response.domain import CmdResponse
from ...response.cmd_response.domain import CmdResult
from ...response.cmd_response.domain import CmdResultTopicKindType

from .domain import Control
from .domain import ControlInfo
from .domain import ControlCmdType

from typing import Any
from typing import Dict


RCLPY_FLAG: str = 'RCLPY'
MQTT_FLAG: str = 'MQTT'
GTS_NAVIGATION_STOPPED_CODE: int = -1
GTS_NAVIGATION_RESUMED_CODE: int = 1


class ControlRequestHandler():

    def __init__(self, rclpy_node: Node, mqtt_client: Client) -> None:
        self.__rclpy_node: Node = rclpy_node
        self.__mqtt_client: Client = mqtt_client

        self.__script_directory: str = os.path.dirname(
            os.path.abspath(__file__))
        self.__mqtt_config_file_path: str = '../../../mqtt/mqtt.ini'
        self.__mqtt_config_service: ConfigService = ConfigService(
            self.__script_directory, self.__mqtt_config_file_path)
        self.__mqtt_config_parser: ConfigParser = self.__mqtt_config_service.read()

        self.__mqtt_control_subscription_topic: str = self.__mqtt_config_parser.get(
            'topics', 'control')
        self.__mqtt_control_subscription_qos: int = int(
            self.__mqtt_config_parser.get('qos', 'control'))

        self.__rclpy_gts_navigation_control_publisher_topic: str = '/gts_navigation/control'
        self.__rclpy_gts_navigation_control_publisher_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup()
        self.__rclpy_gts_navigation_control_publisher: Publisher = self.__rclpy_node.create_publisher(
            msg_type=NavigationControl,
            topic=self.__rclpy_gts_navigation_control_publisher_topic,
            qos_profile=qos_profile_system_default,
            callback_group=self.__rclpy_gts_navigation_control_publisher_cb_group
        )

        self.__rclpy_gts_navigation_task_status_subscription_topic: str = '/gts_navigation/task_status'
        self.__rclpy_gts_navigation_task_status_subscription_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup()
        self.__rclpy_gts_navigation_task_status_subscription: Subscription = self.__rclpy_node.create_subscription(
            msg_type=NavigationStatus,
            topic=self.__rclpy_gts_navigation_task_status_subscription_topic,
            qos_profile=qos_profile_system_default,
            callback=self.__rclpy_gts_navigation_task_status_subscription_cb,
            callback_group=self.__rclpy_gts_navigation_task_status_subscription_cb_group
        )

        self.__time_service: TimeService = TimeService()

        self.__control: Control = Control()
        self.__control_info: ControlInfo = ControlInfo()

        self.__control_event_handler: ControlEventHandler = ControlEventHandler(
            rclpy_node=self.__rclpy_node, mqtt_client=self.__mqtt_client)
        self.__cmd_repsonse_handler: CmdRepsonseHandler = CmdRepsonseHandler(
            rclpy_node=self.__rclpy_node, mqtt_client=self.__mqtt_client)

    def request_to_uvc(self) -> None:
        def __mqtt_control_subscription_cb(mqtt_client: mqtt.Client, mqtt_user_data: Dict, mqtt_message: mqtt.MQTTMessage) -> None:
            try:
                mqtt_topic: str = mqtt_message.topic
                mqtt_decoded_payload: str = mqtt_message.payload.decode()
                mqtt_json: Any = json.loads(mqtt_message.payload)

                self.__rclpy_node.get_logger().info(
                    f'{MQTT_FLAG} subscription cb payload [{mqtt_decoded_payload}] from [{mqtt_topic}]')
                self.__rclpy_node.get_logger().info(
                    f'{MQTT_FLAG} subscription cb json [{mqtt_decoded_payload}] from [{mqtt_topic}]')

                header_dict: dict = mqtt_json['header']
                self.__control.header = header_dict

                cmd_response: CmdResponse = CmdResponse()
                cmd_response.header = header_dict

                cmd_result: CmdResult = CmdResult()
                cmd_result.status = 'success'
                cmd_result.startTime = self.__time_service.get_current_datetime()
                cmd_result.topicKind = CmdResultTopicKindType.CONTROL.value

                cmd_response.cmdResult = cmd_result.__dict__

                self.__cmd_repsonse_handler.response_to_rms(
                    cmd_response=cmd_response)

                control_info_dict: dict = mqtt_json['controlInfo']
                self.__control.controlInfo = control_info_dict

                controlId: str = control_info_dict['controlId']
                self.__control_info.controlId = controlId
                self.__control_event_handler.control_info.controlId = controlId

                self.__rclpy_node.get_logger().info(
                    f'ControlEventHandler controlInfo.controlId {self.__control_event_handler.control_info.controlId}')

                controlCmd: str = self.__control.controlInfo['controlCmd']
                self.__control_info.controlCmd = controlCmd

                self.__judge_control_cmd()

            except KeyError as ke:
                self.__rclpy_node.get_logger().error(
                    f'Invalid JSON Key in MQTT {self.__mqtt_control_subscription_topic} subscription callback: {ke}')

            except json.JSONDecodeError as jde:
                self.__rclpy_node.get_logger().error(
                    f'Invalid JSON format in MQTT {self.__mqtt_control_subscription_topic} subscription callback: {jde.msg}')

            except Exception as e:
                self.__rclpy_node.get_logger().error(
                    f'Exception in MQTT {self.__mqtt_control_subscription_topic} subscription callback: {e}')
                raise

        self.__mqtt_client.subscribe(
            topic=self.__mqtt_control_subscription_topic, qos=self.__mqtt_control_subscription_qos)
        self.__mqtt_client.client.message_callback_add(
            sub=self.__mqtt_control_subscription_topic, callback=__mqtt_control_subscription_cb)

    def __judge_control_cmd(self) -> None:
        is_cmd_reset: bool = (
            self.__control_info.controlCmd == ControlCmdType.RESET.value)
        is_cmd_go: bool = (self.__control_info.controlCmd ==
                           ControlCmdType.GO.value)
        is_cmd_stop: bool = (
            self.__control_info.controlCmd == ControlCmdType.STOP.value)

        if (is_cmd_reset and is_cmd_go and is_cmd_stop):
            return
        elif (is_cmd_reset and is_cmd_go):
            return
        elif (is_cmd_go and is_cmd_stop):
            return
        elif (is_cmd_reset and is_cmd_stop):
            return
        elif (is_cmd_stop and not is_cmd_reset and not is_cmd_go):
            self.__rclpy_node.get_logger().info(
                'judge gts_navigation/control command is stop')

            rclpy_gts_navigation_control: NavigationControl = NavigationControl()
            rclpy_gts_navigation_control.cancel_navigation = True
            rclpy_gts_navigation_control.resume_navigation = False

            self.__control_event_handler.control_result.startTime = self.__time_service.get_current_datetime()
            self.__rclpy_gts_navigation_control_publisher.publish(
                rclpy_gts_navigation_control)
        elif (is_cmd_go and not is_cmd_stop and not is_cmd_reset):
            self.__rclpy_node.get_logger().info(
                'judge gts_navigation/control command is move')

            rclpy_gts_navigation_control: NavigationControl = NavigationControl()
            rclpy_gts_navigation_control.cancel_navigation = False
            rclpy_gts_navigation_control.resume_navigation = True

            self.__control_event_handler.control_result.startTime = self.__time_service.get_current_datetime()
            self.__rclpy_gts_navigation_control_publisher.publish(
                rclpy_gts_navigation_control)
        else:
            return

    def __rclpy_gts_navigation_task_status_subscription_cb(self, navigation_status: NavigationStatus) -> None:
        self.job_status_code = navigation_status.status_code

        if self.job_status_code == GTS_NAVIGATION_STOPPED_CODE:
            self.__control_event_handler.control_result.status = 'success'
            self.__control_event_handler.control_result.endTime = self.__time_service.get_current_datetime()
            self.__control_event_handler.control_info.controlCmd = ControlCmdType.STOP.value
            self.__control_event_handler.response_to_rms()
        elif self.job_status_code == GTS_NAVIGATION_RESUMED_CODE:
            self.__control_event_handler.control_result.status = 'success'
            self.__control_event_handler.control_result.endTime = self.__time_service.get_current_datetime()
            self.__control_event_handler.control_info.controlCmd = ControlCmdType.GO.value
            self.__control_event_handler.response_to_rms()
        else:
            return


__all__ = ['rms_request_control_handler']
