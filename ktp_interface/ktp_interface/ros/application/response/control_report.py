#-*- coding:utf-8 -*-

import json
import rclpy;
from datetime import datetime;

from rclpy.node import Node;
from rclpy.subscription import Subscription;
from rclpy.qos import qos_profile_system_default;
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup;
from rosbridge_library.internal import message_conversion;

from ktp_interface.tcp.application.service import tcp_send_resource;

from ktp_data_msgs.msg import Control;
from ktp_data_msgs.msg import ControlReport;

from typing import Any;

CONTROL_REPORT_FROM_MGR_TOPIC_NAME: str = "/rms/ktp/data/control_report";
KTP_TCP_RESOURCE_ID: str = "rbt_control_report";


class ControlReportManager:

    def __init__(self, node: Node) -> None:
        self.__node: Node = node;

        control_report_subscription_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup();
        self.__control_report_subscription: Subscription = self.__node.create_subscription(
            msg_type=ControlReport,
            topic=CONTROL_REPORT_FROM_MGR_TOPIC_NAME,
            callback_group=control_report_subscription_cb_group,
            callback=self.__control_report_subscription_cb,
            qos_profile=qos_profile_system_default
        );

    def __control_report_subscription_cb(self, control_report_cb: ControlReport) -> None:
        deserialized_control_report: Any = message_conversion.extract_values(control_report_cb);
        self.__node.get_logger().info(f"control_report_cb : {deserialized_control_report}");

        rc: int = tcp_send_resource(resource_id=KTP_TCP_RESOURCE_ID, properties=deserialized_control_report);
        if rc < 0:
            self.__node.get_logger().error(f"Failed to Sending Resource to id : {KTP_TCP_RESOURCE_ID}");
        else:
            self.__node.get_logger().info(f"Succeeded to Sending Resource to id : {KTP_TCP_RESOURCE_ID}");


__all__ = ["ControlReportManager"];
