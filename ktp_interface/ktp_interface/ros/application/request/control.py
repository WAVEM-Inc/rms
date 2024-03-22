import rclpy;

from rclpy.node import Node;
from rclpy.qos import qos_profile_system_default;
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup;
from rosbridge_library.internal import message_conversion;

from ktp_interface.tcp.application.service import get_control_callback_flag;
from ktp_interface.tcp.application.service import get_mission_callback_flag;
from ktp_interface.tcp.application.service import get_detected_object_flag;
from ktp_interface.tcp.application.service import get_control;
from ktp_interface.tcp.application.service import get_mission;
from ktp_interface.tcp.application.service import get_detected_object;


class ControlManager:

    def __init__(self, node: Node) -> None:
        self.__node: Node = node;


__all__ = ["ControlManager"];