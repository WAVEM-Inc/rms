#-*- coding:utf-8 -*-

import json;
import rclpy;

from datetime import datetime;
from typing import Any;

from rclpy.node import Node;
from rclpy.subscription import Subscription;
from rclpy.qos import qos_profile_system_default;
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup;
from rosbridge_library.internal import message_conversion;

from ktp_interface.tcp.application.service import tcp_send_resource;

from ktp_data_msgs.msg import ErrorReport;


ERROR_REPORT_FROM_MGR_TOPIC_NAME: str = "/rms/ktp/data/error_report";
KTP_TCP_RESOURCE_ID: str = "rbt_error_report";


class ErrorReportManager:

    def __init__(self, node: Node) -> None:
        self.__node: Node = node;

        error_report_subscription_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup();
        self.__error_report_subscription: Subscription = self.__node.create_subscription(
            msg_type=ErrorReport,
            topic=ERROR_REPORT_FROM_MGR_TOPIC_NAME,
            callback_group=error_report_subscription_cb_group,
            callback=self.__error_report_subscription_cb,
            qos_profile=qos_profile_system_default
        );

    def __error_report_subscription_cb(self, error_report_cb: ErrorReport) -> None:
        deserialized_error_report: Any = message_conversion.extract_values(error_report_cb);
        self.__node.get_logger().info(f"error_report_cb : {deserialized_error_report}");

        rc: int = tcp_send_resource(resource_id=KTP_TCP_RESOURCE_ID, properties=deserialized_error_report);
        if rc < 0:
            self.__node.get_logger().error(f"Failed to Sending Resource to id : {KTP_TCP_RESOURCE_ID}");
        else:
            self.__node.get_logger().info(f"Succeeded to Sending Resource to id : {KTP_TCP_RESOURCE_ID}");


__all__ = ["ErrorReportManager"];
