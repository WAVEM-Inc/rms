#-*- coding:utf-8 -*-

from typing import Any;

from rclpy.node import Node;
from rclpy.subscription import Subscription;
from rclpy.qos import qos_profile_system_default;
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup;
from rosbridge_library.internal import message_conversion;

from ktp_interface.tcp.application.service import tcp_send_resource;

from ktp_data_msgs.msg import ServiceStatus;


SERVICE_STATUS_FROM_MGR_TOPIC_NAME: str = "/rms/ktp/data/service_status";
KTP_TCP_RESOURCE_ID: str = "rbt_service_status";


class ServiceStatusManager:

    def __init__(self, node: Node) -> None:
        self.__node: Node = node;

        service_status_subscription_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup();
        self.__service_status_subscription: Subscription = self.__node.create_subscription(
            msg_type=ServiceStatus,
            topic=SERVICE_STATUS_FROM_MGR_TOPIC_NAME,
            callback_group=service_status_subscription_cb_group,
            callback=self.__service_status_subscription_cb,
            qos_profile=qos_profile_system_default
        );

    def __service_status_subscription_cb(self, service_status_cb: ServiceStatus) -> None:
        deserialized_service_status: Any = message_conversion.extract_values(service_status_cb);
        self.__node.get_logger().info(f"service_status_cb : {deserialized_service_status}");

        rc: int = tcp_send_resource(resource_id=KTP_TCP_RESOURCE_ID, properties=deserialized_service_status);
        if rc < 0:
            self.__node.get_logger().error(f"Failed to Sending Resource to id : {KTP_TCP_RESOURCE_ID}");
        else:
            self.__node.get_logger().info(f"Succeeded to Sending Resource to id : {KTP_TCP_RESOURCE_ID}");


__all__: list[str] = ["ServiceStatusManager"];
