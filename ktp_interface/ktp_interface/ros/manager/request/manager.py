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
        if get_control_callback_flag():
            self.__control_manager.deliver_control_callback_json(control_callback_json=get_control());

        if get_mission_callback_flag():
            self.__mission_manager.deliver_mission_callback_json(mission_callback_json=get_mission());

        if get_detected_object_flag():
            self.__detected_object_manager.deliver_detected_object_callback_json(detected_object_callback_json=get_detected_object());


__all__ = ["RequestManager"];
