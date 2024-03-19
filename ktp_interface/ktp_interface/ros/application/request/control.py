import rclpy;

from rclpy.node import Node;
from rclpy.qos import qos_profile_system_default;
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup;
from rosbridge_library.internal import message_conversion;


class ControlManager:

    def __init__(self, node: Node) -> None:
        self.__node: Node = node;


__all__ = ["ControlManager"];