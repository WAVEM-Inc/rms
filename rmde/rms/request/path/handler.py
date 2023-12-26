import os
import json
import rclpy.client
import importlib
import paho.mqtt.client as mqtt

from configparser import ConfigParser
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.qos import qos_profile_system_default
from rclpy.qos import qos_profile_services_default
from rclpy.task import Future
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rosbridge_library.internal import message_conversion

from gts_navigation_msgs.msg import GoalWaypoints
from robot_status_msgs.srv import RegisterTask

from ....mqtt.mqtt_client import Client
from ...common.service import ConfigService
from ...common.service import TimeService

from ...response.cmd_response.handler import CmdRepsonseHandler
from ...response.cmd_response.domain import CmdResponse
from ...response.cmd_response.domain import CmdResult
from ...response.cmd_response.domain import CmdResultTopicKindType

from ...common.domain import JobUUID

from .domain import JobInfo
from .domain import JobPath
from .domain import Path

from typing import Any
from typing import Dict


RCLPY_FLAG: str = 'RCLPY'
MQTT_FLAG: str = 'MQTT'

UUID_REGISTER_KEY: str = 'qAqwmrwskdfliqnwkfnlasdkfnlas'

UUID_REGISTER_KEY_EMPTY_CODE: int = -100
UUID_REGISTER_KEY_EMPTY_MESSAGE: str = 'register_key is empty'

UUID_REGISTER_KEY_INCONSISTENCY_CODE: int = -101
UUID_REGISTER_KEY_INCONSISTENCY_MESSAGE: str = 'register_key is not equals'

UUID_REGISTER_SUCCESS_CODE: int = 100
UUID_REGISTER_SUCCESS_MESSAGE: str = 'uuid has been registered successfully'


class PathRequestHandler():

    def __init__(self, rclpy_node: Node, mqtt_client: Client) -> None:
        self.__rclpy_node: Node = rclpy_node
        self.__mqtt_client: Client = mqtt_client

        self.__script_directory: str = os.path.dirname(os.path.abspath(__file__))

        self.__mqtt_config_file_path: str = '../../../mqtt/mqtt.ini'
        self.__mqtt_config_service: ConfigService = ConfigService(self.__script_directory, self.__mqtt_config_file_path)
        self.__mqtt_config_parser: ConfigParser = self.__mqtt_config_service.read()

        self.__mqtt_path_subscription_topic: str = self.__mqtt_config_parser.get('topics', 'path')
        self.__mqtt_path_subscription_qos: int = int(self.__mqtt_config_parser.get('qos', 'path'))

        self.__common_config_file_path: str = '../../common/config.ini'
        self.__common_config_service: ConfigService = ConfigService(self.__script_directory, self.__common_config_file_path)
        self.__common_config_parser: ConfigParser = self.__common_config_service.read()

        self.__rclpy_goal_waypoints_publisher_topic: str = '/gts_navigation/waypoints'
        self.__rclpy_goal_waypoints_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup()
        self.__rclpy_goal_waypoints_publisher: Publisher = self.__rclpy_node.create_publisher(
            msg_type=GoalWaypoints,
            topic=self.__rclpy_goal_waypoints_publisher_topic,
            qos_profile=qos_profile_system_default,
            callback_group=self.__rclpy_goal_waypoints_cb_group
        )

        self.__rclpy_register_task_service_server_name: str = '/robot_task/register'
        self.__rclpy_register_task_service_client_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup()
        self.__rclpy_register_task_service_client: rclpy.client.Client = self.__rclpy_node.create_client(
            srv_name=self.__rclpy_register_task_service_server_name,
            srv_type=RegisterTask,
            qos_profile=qos_profile_services_default,
            callback_group=self.__rclpy_register_task_service_client_cb_group
        )

        self.__time_service: TimeService = TimeService()

        self.__path: Path = Path()
        self.__jobInfo: JobInfo = JobInfo()
        self.__jobPath: JobPath = JobPath()
        self.__cmd_repsonse_handler: CmdRepsonseHandler = CmdRepsonseHandler(rclpy_node=self.__rclpy_node, mqtt_client=self.__mqtt_client)
        
        self.__job_start_time: str = ''

    def request_to_uvc(self) -> None:
        def __mqtt_path_subscription_cb(mqtt_client: mqtt.Client, mqtt_user_data: Dict, mqtt_message: mqtt.MQTTMessage) -> None:
            try:
                mqtt_topic: str = mqtt_message.topic
                mqtt_decoded_payload: str = mqtt_message.payload.decode()
                mqtt_json: Any = json.loads(mqtt_message.payload)

                self.__rclpy_node.get_logger().info(f'{MQTT_FLAG} subscription cb payload [{mqtt_decoded_payload}] from [{mqtt_topic}]')
                self.__rclpy_node.get_logger().info(f'{MQTT_FLAG} subscription cb json [{mqtt_decoded_payload}] from [{mqtt_topic}]')

                header_dict: dict = mqtt_json['header']
                self.__path.header = header_dict
                
                cmd_response: CmdResponse = CmdResponse()
                cmd_result: CmdResult = CmdResult()
                cmd_result.status = 'success'
                cmd_result.startTime = self.__time_service.get_current_datetime()
                cmd_result.topicKind = CmdResultTopicKindType.PATH.value
                cmd_result.resCmdId = header_dict['topicUid']

                cmd_response.cmdResult = cmd_result.__dict__

                self.__cmd_repsonse_handler.response_to_rms(cmd_response=cmd_response)

                job_info_dict: dict = mqtt_json['jobInfo']
                self.__path.jobInfo = job_info_dict

                job_path_dict: dict = mqtt_json['jobPath']
                self.__path.jobPath = job_path_dict

                job_uuid: JobUUID = JobUUID()
                job_plan_id: str = self.__path.jobInfo['jobPlanId']
                self.__jobInfo.jobPlanId = job_plan_id
                job_uuid.jobPlanId = job_plan_id

                job_group_id: str = self.__path.jobInfo['jobGroupId']
                self.__jobInfo.jobGroupId = job_group_id
                job_uuid.jobGroupId = job_group_id

                job_order_id: str = self.__path.jobInfo['jobOrderId']
                self.__jobInfo.jobOrderId = job_order_id
                job_uuid.jobOrderId = job_order_id

                job_group: str = self.__path.jobInfo['jobGroup']
                self.__jobInfo.jobGroup = job_group
                job_uuid.jobGroup = job_group

                job_kind: str = self.__path.jobInfo['jobKind']
                self.__jobInfo.jobKind = job_kind
                job_uuid.jobKind = job_kind

                self.__job_start_time = self.__time_service.get_current_datetime()
                rclpy_request_register_uuid_service_result: Any = self.__rclpy_register_task_service_request(job_uuid)
                self.__rclpy_node.get_logger().info(f'PathRequestHandler request_register_uuid_result : {rclpy_request_register_uuid_service_result}')

                area_clsf: str = self.__path.jobPath['areaClsf']
                self.__jobPath.areaClsf = area_clsf

                location_list: list = self.__path.jobPath['locationList']
                self.__jobPath.locationList = location_list

                job_kind_type_dict: dict = self.__path.jobPath['jobKindType']
                self.__jobPath.jobKindType = job_kind_type_dict

                self.__rclpy_node.get_logger().info(f'PathRequestHandler LocationList {self.__jobPath.locationList}')
                self.__rclpy_publish_goal_waypoints_list(self.__jobPath.locationList)

            except KeyError as ke:
                self.__rclpy_node.get_logger().error(f'PathRequestHandler Invalid JSON Key in MQTT {self.__mqtt_path_subscription_topic} subscription callback: {ke}')

            except json.JSONDecodeError as jde:
                self.__rclpy_node.get_logger().error(f'PathRequestHandler Invalid JSON format in MQTT {self.__mqtt_path_subscription_topic} subscription callback: {jde.msg}')

            except Exception as e:
                self.__rclpy_node.get_logger().error(f'PathRequestHandler Exception in MQTT {self.__mqtt_path_subscription_topic} subscription callback: {e}')
                raise

        self.__mqtt_client.subscribe(topic=self.__mqtt_path_subscription_topic, qos=self.__mqtt_path_subscription_qos)
        self.__mqtt_client.client.message_callback_add(sub=self.__mqtt_path_subscription_topic, callback=__mqtt_path_subscription_cb)

    def __rclpy_lookup_messages(self, module_name: str, module_class_name: str) -> Any:
        self.__rclpy_node.get_logger().info(f'{RCLPY_FLAG} lookup object module_name : {module_name}')
        self.__rclpy_node.get_logger().info(f'{RCLPY_FLAG} lookup object module_class_name : {module_class_name}')

        message_path = importlib.import_module(module_name, self.__rclpy_node.get_name())
        message_object: Any = getattr(message_path, module_class_name)

        return message_object

    def __rclpy_register_task_service_request(self, job_uuid: JobUUID) -> Any:
        self.__rclpy_node.get_logger().info(f'PathRequestHandler request_register_uuid job_plan_id : {job_uuid.jobPlanId}')
        self.__rclpy_node.get_logger().info(f'PathRequestHandler request_register_uuid job_group_id : {job_uuid.jobGroupId}')
        self.__rclpy_node.get_logger().info(f'PathRequestHandler request_register_uuid job_order_id : {job_uuid.jobOrderId}')

        rclpy_register_task_request: RegisterTask.Request = RegisterTask.Request()
        rclpy_register_task_request.register_key = UUID_REGISTER_KEY
        rclpy_register_task_request.job_start_time = self.__job_start_time
        rclpy_register_task_request.job_group = job_uuid.jobGroup
        rclpy_register_task_request.job_kind = job_uuid.jobKind
        rclpy_register_task_request.job_plan_id = job_uuid.jobPlanId
        rclpy_register_task_request.job_group_id = job_uuid.jobGroupId
        rclpy_register_task_request.job_order_id = job_uuid.jobOrderId

        rclpy_register_task_request_future: Future = self.__rclpy_register_task_service_client.call_async(rclpy_register_task_request)

        return rclpy_register_task_request_future.result()

    def __rclpy_publish_goal_waypoints_list(self, location_list: list) -> None:
        rclpy_goal_waypoints_list: list = []

        for location in location_list:
            rclpy_nav_sat_fix_obj: Any = self.__rclpy_lookup_messages('sensor_msgs.msg', 'NavSatFix')

            rclpy_location_to_nav_sat_fix_dict: dict = {
                'longitude': location['xpos'],
                'latitude': location['ypos'],
                'altitude': 0.0
            }

            rclpy_nav_sat_fix: Any = message_conversion.populate_instance(rclpy_location_to_nav_sat_fix_dict, rclpy_nav_sat_fix_obj())
            rclpy_goal_waypoints_list.append(rclpy_nav_sat_fix)

        self.__rclpy_node.get_logger().info('GoalWaypoints List : {}'.format(rclpy_goal_waypoints_list))

        rclpy_goal_waypoints: GoalWaypoints = GoalWaypoints()
        rclpy_goal_waypoints.goal_waypoints_list = rclpy_goal_waypoints_list

        self.__rclpy_node.get_logger().info('GoalWaypoints : {}'.format(rclpy_goal_waypoints.goal_waypoints_list))
        self.__rclpy_goal_waypoints_publisher.publish(rclpy_goal_waypoints)


__all__ = ['rms_request_path_handler']
