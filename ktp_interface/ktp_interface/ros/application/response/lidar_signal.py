#-*- coding:utf-8 -*-

import json;
import rclpy;

from typing import Any;

from rclpy.node import Node;
from rclpy.subscription import Subscription;
from rclpy.qos import qos_profile_system_default;
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup;
from rosbridge_library.internal import message_conversion;

from ktp_interface.tcp.application.service import tcp_send_resource;

from ktp_data_msgs.msg import LiDARSignal;


LIDAR_SIGNAL_FROM_MGR_TOPIC_NAME: str = "/rms/ktp/data/lidar_signal";
KTP_TCP_RESOURCE_ID: str = "rbt_lidar_signal";


class LiDARSignalManager:

    def __init__(self, node: Node) -> None:
        self.__node: Node = node;

        lidar_signal_subscription_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup();
        self.__lidar_signal_subscription: Subscription = self.__node.create_subscription(
            msg_type=LiDARSignal,
            topic=LIDAR_SIGNAL_FROM_MGR_TOPIC_NAME,
            callback_group=lidar_signal_subscription_cb_group,
            callback=self.__lidar_signal_subscription_cb,
            qos_profile=qos_profile_system_default
        );

    def __lidar_signal_subscription_cb(self, lidar_signal_cb: LiDARSignal) -> None:
        deserialized_lidar_signal: Any = message_conversion.extract_values(lidar_signal_cb);
        self.__node.get_logger().info(f"lidar_signal_cb : {deserialized_lidar_signal}");

        rc: int = tcp_send_resource(resource_id=KTP_TCP_RESOURCE_ID, properties=deserialized_lidar_signal);
        if rc < 0:
            self.__node.get_logger().error(f"Failed to Sending Resource to id : {KTP_TCP_RESOURCE_ID}");
        else:
            self.__node.get_logger().info(f"Succeeded to Sending Resource to id : {KTP_TCP_RESOURCE_ID}");


__all__ = ["LiDARSignalManager"];
