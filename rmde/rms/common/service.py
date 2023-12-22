import os
import uuid
import socket

from rclpy.node import Node
from rclpy.qos import qos_profile_services_default
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.client import Client
from rclpy.task import Future
from robot_type_msgs.srv import Type

from configparser import ConfigParser
from datetime import datetime
from typing import Dict
from typing import Any


def empty_dict() -> Dict:
    return {}


class TimeService():

    def __init__(self) -> None:
        pass

    def get_current_datetime(self) -> str:
        __current_datetime: datetime = datetime.now()
        __formatted_datetime: str = __current_datetime.strftime("%Y%m%d%H%M%S")

        return __formatted_datetime


class UUIDService():

    def __init__(self) -> None:
        pass

    def generate_uuid(self) -> str:
        __uuid_obj: uuid.UUID = uuid.uuid4()
        __uuid_str: str = str(__uuid_obj)

        return __uuid_str


class ConfigService():

    def __init__(self, file_path: str, file_name: str) -> None:
        self.config_parser: ConfigParser = ConfigParser()
        self.__config_file_path: str = os.path.join(file_path, file_name)

    @property
    def config_file_path(self) -> str:
        return self.__config_file_path

    def read(self) -> ConfigParser:
        self.config_parser.read(self.__config_file_path)

        return self.config_parser


class NetworkService():

    def __init__(self) -> None:
        pass

    def get_local_ip(self) -> str:
        __s: socket.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        __s.connect(("8.8.8.8", 80))
        __ip_address: str = __s.getsockname()[0]

        return __ip_address


class RobotTypeService():

    def __init__(self, rclpy_node: Node) -> None:
        self.__script_directory: str = os.path.dirname(os.path.abspath(__file__))
        self.__common_config_file_path: str = '../common/config.ini'
        self.__common_config_service: ConfigService = ConfigService(self.__script_directory, self.__common_config_file_path)
        self.__common_config_parser: ConfigParser = self.__common_config_service.read()

        self.__rclpy_node: Node = rclpy_node
        self.__rclpy_robot_type_select_service_server_name: str = '/robot_type/service'
        self.__rclpy_robot_type_select_service_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup()
        self.__rclpy_robot_type_select_service_client: Client = self.__rclpy_node.create_client(
            srv_type=Type,
            srv_name=self.__rclpy_robot_type_select_service_server_name,
            qos_profile=qos_profile_services_default,
            callback_group=self.__rclpy_robot_type_select_service_cb_group
        )

    def __rclpy_robot_type_select_service_request(self, check: bool, type: int) -> Any:
        is___rclpy_robot_type_select_service_client_ready: bool = self.__rclpy_robot_type_select_service_client.wait_for_service(timeout_sec=1.0)
        __rclpy_robot_type_select_request_msgs: Type.Request = Type.Request()

        if not is___rclpy_robot_type_select_service_client_ready:
            return None
        else:
            __rclpy_robot_type_select_request_msgs.check = check
            __rclpy_robot_type_select_request_msgs.type = type
            __rclpy_robot_type_select_service_call_future: Future = self.__rclpy_robot_type_select_service_client.call_async(__rclpy_robot_type_select_request_msgs)
            __is_rclpy_robot_type_select_service_call_future_done: bool = __rclpy_robot_type_select_service_call_future.done()
            __rclpy_robot_type_select_service_call_future_result: Any = __rclpy_robot_type_select_service_call_future.result()

            return __rclpy_robot_type_select_service_call_future_result

    def select_current_robot_type(self) -> None:
        __robotType: str = ''
        __rclpy_robot_type_select_response: Any = self.__rclpy_robot_type_select_service_request(False, 0)
        __is_rclpy_robot_type_select_response_none: bool = (__rclpy_robot_type_select_response == None)

        if __is_rclpy_robot_type_select_response_none:
            return
        else:
            __robotType = __rclpy_robot_type_select_response.description
            self.__write_common_config_file(__robotType)

    def __write_common_config_file(self, robot_type: str) -> None:
        self.__common_config_parser.set('header', 'robotType', robot_type)

        with open(self.__common_config_service.config_file_path, 'w') as common_config_file:
            self.__common_config_parser.write(common_config_file)


__all__ = ['rms_common_service']
