#-*- coding:utf-8 -*-

from rclpy.node import Node;
from rclpy.subscription import Subscription;
from rclpy.qos import qos_profile_system_default;
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup;
from rosbridge_library.internal import message_conversion;

from ktp_interface.tcp.application.service import tcp_send_resource;

from ktp_data_msgs.msg import Status;
from ktp_data_msgs.msg import StatusService;

from typing import Any;

RBT_STATUS_FROM_DATA_MGR_TOPIC_NAME: str = "/rms/ktp/data/rbt_status";
KTP_TCP_RESOURCE_ID: str = "rbt_status";


class RbtStatusManager:

    def __init__(self, node: Node) -> None:
        self.__node: Node = node;

        rbt_status_subscription_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup();
        self.__rbt_status_subscription: Subscription = self.__node.create_subscription(
            msg_type=Status,
            topic=RBT_STATUS_FROM_DATA_MGR_TOPIC_NAME,
            callback_group=rbt_status_subscription_cb_group,
            callback=self.__rbt_status_subscription_cb,
            qos_profile=qos_profile_system_default
        );

    def __rbt_status_subscription_cb(self, rbt_status_cb: Status) -> None:
        deserialized_rbt_status: Any = message_conversion.extract_values(rbt_status_cb);
        self.__node.get_logger().info(f"rbt_status_cb : {deserialized_rbt_status}");

        rc: int = tcp_send_resource(resource_id=KTP_TCP_RESOURCE_ID, properties=deserialized_rbt_status);
        if rc < 0:
            self.__node.get_logger().error(f"Failed to Sending Resource to id : {KTP_TCP_RESOURCE_ID}");
        else:
            self.__node.get_logger().info(f"Succeeded to Sending Resource to id : {KTP_TCP_RESOURCE_ID}");


__all__ = ["RbtStatusManager"];
