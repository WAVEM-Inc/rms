#-*- coding:utf-8 -*-

import json

import rclpy;

from rclpy.node import Node;
from rclpy.client import Client;
from rclpy.qos import qos_profile_system_default;
from rclpy.qos import qos_profile_services_default;
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup;
from rosbridge_library.internal import message_conversion;
from typing import Any;

from std_msgs.msg import String;
from ktp_data_msgs.srv import AssignMission;


DATA_MANAGER_NODE_NAME: str = "ktp_data_manager";
ASSIGN_MISSION_TO_DATA_MGR_SERVICE_NAME: str = f"/{DATA_MANAGER_NODE_NAME}/assign/mission";


class MissionManager:

    def __init__(self, node: Node) -> None:
        self.__node: Node = node;

        __assign_mission_service_client_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup();
        self.__assign_mission_service_client: Client = self.__node.create_client(
            srv_type=AssignMission,
            srv_name=ASSIGN_MISSION_TO_DATA_MGR_SERVICE_NAME,
            qos_profile=qos_profile_services_default,
            callback_group=__assign_mission_service_client_cb_group
        );

        self.__test_sub = self.__node.create_subscription(
            topic="/chatter",
            msg_type=String,
            qos_profile=qos_profile_system_default,
            callback_group=MutuallyExclusiveCallbackGroup(),
            callback=self.test_cb
        );

    def test_cb(self, string: String) -> None:
        self.__node.get_logger().info(f"chatter cb : {string}");
        pass;

    def request_assign_mission(self, mission_json: Any) -> None:
        self.__node.get_logger().info(
            f"---------------------------- Assign Mission Request ----------------------------");
        self.__node.get_logger().info(f"mission_json : {json.dumps(mission_json)}");

        assign_mission_request: Any = message_conversion.populate_instance(mission_json, AssignMission.Request());


__all__ = ["MissionManager"];
