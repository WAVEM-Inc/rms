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

from std_msgs.msg import String;


GRAPH_LIST_FROM_MGR_TOPIC_NAME: str = "/rms/ktp/data/graph_list";
KTP_TCP_RESOURCE_ID: str = "rbt_graph_list";


class GraphListManager:

    def __init__(self, node: Node) -> None:
        self.__node: Node = node;

        graph_list_subscription_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup();
        self.__graph_list_subscription: Subscription = self.__node.create_subscription(
            msg_type=String,
            topic=GRAPH_LIST_FROM_MGR_TOPIC_NAME,
            callback_group=graph_list_subscription_cb_group,
            callback=self.__graph_list_subscription_cb,
            qos_profile=qos_profile_system_default
        );

    def __graph_list_subscription_cb(self, graph_list_cb: String) -> None:
        try:
            deserialized_graph_list: Any = json.loads(graph_list_cb.data);
            self.__node.get_logger().info(f"deserialized_graph_list : {json.dumps(deserialized_graph_list, indent=4)}");

            rc: int = tcp_send_resource(resource_id=KTP_TCP_RESOURCE_ID, properties=deserialized_graph_list);
            if rc < 0:
                self.__node.get_logger().error(f"Failed to Sending Resource to id : {KTP_TCP_RESOURCE_ID}");
            else:
                self.__node.get_logger().info(f"Succeeded to Sending Resource to id : {KTP_TCP_RESOURCE_ID}");
        except AttributeError as ae:
            self.__node.get_logger().error(f"{KTP_TCP_RESOURCE_ID} failed : {ae}");
            pass;


__all__ = ["GraphListManager"];
