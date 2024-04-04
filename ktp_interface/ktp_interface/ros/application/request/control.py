import json;
import rclpy;

from rclpy.node import Node;
from rclpy.client import Client;
from rclpy.task import Future;
from rclpy.qos import qos_profile_services_default;
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup;
from rosbridge_library.internal import message_conversion;

from ktp_data_msgs.msg import Control;
from ktp_data_msgs.srv import AssignControl;
from ktp_interface.tcp.application.service import control_callback_flag;

from typing import Any;

DATA_MANAGER_NODE_NAME: str = "ktp_data_manager";
ASSIGN_CONTROL_SERVICE_NAME: str = f"/{DATA_MANAGER_NODE_NAME}/assign/control";


class ControlManager:

    def __init__(self, node: Node) -> None:
        self.__node: Node = node;

        assign_control_service_client_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup();
        self.__assign_control_service_client: Client = self.__node.create_client(
            srv_type=AssignControl,
            srv_name=ASSIGN_CONTROL_SERVICE_NAME,
            qos_profile=qos_profile_services_default,
            callback_group=assign_control_service_client_cb_group
        );

    def deliver_control_callback_json(self, control_callback_json: Any) -> None:
        try:
            self.__node.get_logger().info(
                f"Control Callback From KTP : {json.dumps(obj=control_callback_json, indent=4)}");
            control: Control = message_conversion.populate_instance(msg=control_callback_json, inst=Control());
            self.__assign_control_request(control=control);
        except message_conversion.NonexistentFieldException as nefe:
            self.__node.get_logger().error(f"Control Callback : {nefe}");
            return;

    def __assign_control_request(self, control: Control) -> None:
        assign_control_request: AssignControl.Request = AssignControl.Request();
        assign_control_request.control = control;

        self.__node.get_logger().info(f"Assign Control Request Message : {assign_control_request}");

        is_assign_control_service_server_ready: bool = self.__assign_control_service_client.wait_for_service(timeout_sec=0.75);

        if is_assign_control_service_server_ready:
            call_future: Future = self.__assign_control_service_client.call_async(request=assign_control_request);
            result: Any = call_future.result();
            self.__node.get_logger().info(f"{ASSIGN_CONTROL_SERVICE_NAME} request result : {result}");
        else:
            self.__node.get_logger().error(f"{ASSIGN_CONTROL_SERVICE_NAME} is not ready");
            return;



__all__ = ["ControlManager"];