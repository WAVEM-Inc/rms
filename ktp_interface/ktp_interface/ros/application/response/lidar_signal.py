#-*- coding:utf-8 -*-

import json;
import rclpy;

from typing import Any;

from rclpy.node import Node;
from rclpy.service import Service;
from rclpy.qos import qos_profile_services_default;
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup;
from rosbridge_library.internal import message_conversion;

from ktp_interface.tcp.application.service import tcp_send_resource;

from ktp_data_msgs.srv import AssignLiDARSignal;


ASSIGN_LIDAR_SIGNAL_SERVICE_SERVER_NAME: str = "/ktp_interface/assign/lidar_signal";
KTP_TCP_RESOURCE_ID: str = "rbt_lidar_signal";


class LiDARSignalManager:

    def __init__(self, node: Node) -> None:
        self.__node: Node = node;

        assign_lidar_signal_service_server_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup();
        self.__assign_lidar_signal_service_server: Service = self.__node.create_service(
            srv_type=AssignLiDARSignal,
            srv_name=ASSIGN_LIDAR_SIGNAL_SERVICE_SERVER_NAME,
            callback_group=assign_lidar_signal_service_server_cb_group,
            callback=self.__assign_lidar_signal_service_server_cb,
            qos_profile=qos_profile_services_default
        );

    def __assign_lidar_signal_service_server_cb(self, request: AssignLiDARSignal.Request, response: AssignLiDARSignal.Response) -> AssignLiDARSignal.Response:
        self.__node.get_logger().info(f"assign lidar_signal request : {request}");

        response.result = True;

        return response;


__all__ = ["LiDARSignalManager"];
