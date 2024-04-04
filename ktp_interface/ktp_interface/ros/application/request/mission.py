#-*- coding:utf-8 -*-

import json;
from rclpy.node import Node;
from rclpy.client import Client;
from rclpy.subscription import Subscription;
from rclpy.task import Future;
from rclpy.qos import qos_profile_system_default
from rclpy.qos import qos_profile_services_default;
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup;
from rosbridge_library.internal import message_conversion;
from typing import Any;

from std_msgs.msg import String;
from ktp_data_msgs.msg import Mission;
from ktp_data_msgs.srv import AssignMission;
from ktp_interface.tcp.application.service import mission_callback_flag;


DATA_MANAGER_NODE_NAME: str = "ktp_data_manager";
ASSIGN_MISSION_SERVICE_NAME: str = f"/{DATA_MANAGER_NODE_NAME}/assign/mission";


class MissionManager:

    def __init__(self, node: Node) -> None:
        self.__node: Node = node;

        assign_mission_service_client_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup();
        self.__assign_mission_service_client: Client = self.__node.create_client(
            srv_type=AssignMission,
            srv_name=ASSIGN_MISSION_SERVICE_NAME,
            qos_profile=qos_profile_services_default,
            callback_group=assign_mission_service_client_cb_group
        );

    def deliver_mission_callback_json(self, mission_callback_json: Any) -> None:
        try:
            self.__node.get_logger().info(
                f"Mission Callback From KTP : {json.dumps(obj=mission_callback_json, indent=4)}");
            mission: Mission = message_conversion.populate_instance(msg=mission_callback_json, inst=Mission());
            self.__assign_mission_request(mission=mission);
        except message_conversion.NonexistentFieldException as nefe:
            self.__node.get_logger().error(f"Mission Callback : {nefe}");
            return;

    def __assign_mission_request(self, mission: Mission) -> None:
        assign_mission_request: AssignMission.Request = AssignMission.Request();
        assign_mission_request.mission = mission;

        self.__node.get_logger().info(f"Assign Mission Request Message : {assign_mission_request}");

        is_assign_mission_service_server_ready: bool = self.__assign_mission_service_client.wait_for_service(timeout_sec=0.75);

        if is_assign_mission_service_server_ready:
            call_future: Future = self.__assign_mission_service_client.call_async(request=assign_mission_request);
            result: Any = call_future.result();
            self.__node.get_logger().info(f"{ASSIGN_MISSION_SERVICE_NAME} request result : {result}");
        else:
            self.__node.get_logger().error(f"{ASSIGN_MISSION_SERVICE_NAME} is not ready");
            return;


__all__ = ["MissionManager"];
