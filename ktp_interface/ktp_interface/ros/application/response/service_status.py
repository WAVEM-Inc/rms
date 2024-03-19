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

from ktp_data_msgs.msg import ServiceStatus;
from ktp_data_msgs.msg import ServiceStatusTask;
from ktp_data_msgs.msg import ServiceStatusTaskData;
from ktp_data_msgs.msg import Mission;


SERVICE_STATUS_FROM_MGR_TOPIC_NAME: str = "/rms/ktp/data/service_status";
KTP_TCP_RESOURCE_ID: str = "rbt_service_status";
IM_DEV_ID = "KECDSEMITB001";


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

    def test(self, mission: Mission) -> None:
        test_service_status: ServiceStatus = ServiceStatus();
        test_service_status.create_time = datetime.now().strftime("%y%m%d%H%M%S%f")[:-3];
        test_service_status.mission_code = mission.mission_code;
        test_service_status.mission_id = mission.mission_id;
        test_service_status.owner = mission.owner;
        test_service_status.reserve = "";

        test_service_status_task: ServiceStatusTask = ServiceStatusTask()
        for mission_task in mission.task:
            test_service_status_task.task_id = mission_task.task_id;
            test_service_status_task.task_code = mission_task.task_code;
            test_service_status_task.seq = mission_task.seq;

            for mission_task_data in mission_task.task.task_data:
                test_service_status_task.task_data.map_id = mission_task_data.map_id;
                test_service_status_task.task_data.goal = mission_task_data.goal;
                test_service_status_task.task_data.source = mission_task_data.source;

        test_service_status.task = test_service_status_task;

        # self.__tcp_service.send_resource(KTP_TCP_RESOURCE_ID, message_conversion.extract_values(test_service_status));


__all__ = ["ServiceStatusManager"];
