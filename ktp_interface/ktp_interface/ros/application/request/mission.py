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
        
        self.test_sub: Subscription = self.__node.create_subscription(
            topic="/rms/ktp/itf/mission/test",
            msg_type=Mission,
            qos_profile=qos_profile_system_default,
            callback_group=MutuallyExclusiveCallbackGroup(),
            callback=self.test_cb
        );
        
    def test_cb(self, mission_cb: Mission) -> None:
        mission_dumped = json.dumps(obj=message_conversion.extract_values(inst=mission_cb), indent=4);
        print(f"mission_dumped : {mission_dumped}");
        
        mission: Any = message_conversion.populate_instance(json.loads(mission_dumped), Mission());
        self.__assign_mission_request(mission=mission);

    def deliver_mission_callback_json(self, mission_callback_json: Any) -> None:
        mission: Any = message_conversion.populate_instance(json.loads(mission_callback_json), Mission());
        self.__node.get_logger().info(f"Mission Callback From KTP : {mission}");
        self.__assign_mission_request(mission=mission);

    def __assign_mission_request(self, mission: Any) -> None:
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
