import rclpy;

from rclpy.node import Node;
from rclpy.qos import qos_profile_system_default;
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup;
from rosbridge_library.internal import message_conversion;

from ktp_interface.tcp.application.service import TCP;
from ktp_interface.tcp.application.service import TCPService;


class ResponseManager:

    def __init__(self, node: Node, tcp_service: TCP) -> None:
        self.__node: Node = node;
        self.__tcp_service: TCPService = tcp_service;


__all__ = ["ResponseManager"];
