from rclpy.node import Node;

from typing import Any;

from ktp_interface.ros.application.request.control import ControlManager;
from ktp_interface.ros.application.request.detected_object import DetectedObjectManager;
from ktp_interface.ros.application.request.mission import MissionManager;


class RequestManager:

    def __init__(self, node: Node) -> None:
        self.__node: Node = node;

        self.__control_manager: ControlManager = ControlManager(node=self.__node);
        self.__detected_object_manager: DetectedObjectManager = DetectedObjectManager(node=self.__node);
        self.__mission_manager: MissionManager = MissionManager(node=self.__node);

    def manage_assign_mission(self, mission_json: Any) -> None:
        self.__mission_manager.request_assign_mission(mission_json=mission_json);


__all__ = ["RequestManager"];
