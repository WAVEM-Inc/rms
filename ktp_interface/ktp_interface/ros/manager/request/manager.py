from rclpy.node import Node;
from rclpy.timer import Timer;
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup;

from typing import Any;

from ktp_interface.ros.application.request.control import ControlManager;
from ktp_interface.ros.application.request.detected_object import DetectedObjectManager;
from ktp_interface.ros.application.request.mission import MissionManager;

from ktp_interface.tcp.application.service import get_control_callback_flag;
from ktp_interface.tcp.application.service import get_mission_callback_flag;
from ktp_interface.tcp.application.service import get_detected_object_flag;
from ktp_interface.tcp.application.service import get_control;
from ktp_interface.tcp.application.service import get_mission;
from ktp_interface.tcp.application.service import get_detected_object;


class RequestManager:

    def __init__(self, node: Node) -> None:
        self.__node: Node = node;

        self.__polling_timer: Timer = self.__node.create_timer(
            timer_period_sec=0.5,
            callback_group=MutuallyExclusiveCallbackGroup(),
            callback=self.__polling_timer_cb
        );

        self.__control_manager: ControlManager = ControlManager(node=self.__node);
        self.__detected_object_manager: DetectedObjectManager = DetectedObjectManager(node=self.__node);
        self.__mission_manager: MissionManager = MissionManager(node=self.__node);

    def __polling_timer_cb(self) -> None:
        print("\n");
        self.__node.get_logger().info(f"control_callback_flag : {get_control_callback_flag()}, control : {get_control()}");
        self.__node.get_logger().info(f"mission_callback_flag : {get_mission_callback_flag()}, mission : {get_mission()}");
        self.__node.get_logger().info(f"detected_object_callback_flag : {get_detected_object_flag()}, detected_object : {get_detected_object()}");
        print("\n");

    def manage_assign_mission(self, mission_json: Any) -> None:
        self.__mission_manager.request_assign_mission(mission_json=mission_json);


__all__ = ["RequestManager"];
