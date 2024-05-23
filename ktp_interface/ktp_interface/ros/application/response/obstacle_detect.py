from typing import Any;
from rclpy.node import Node;
from rclpy.subscription import Subscription;
from rclpy.qos import qos_profile_system_default;
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup;
from rosbridge_library.internal import message_conversion;
from ktp_interface.tcp.application.service import tcp_send_resource;
from ktp_data_msgs.msg import ObstacleDetect;


OBSTACLE_DETECT_FROM_MGR_TOPIC_NAME: str = "/rms/ktp/data/obstacle_detect";
KTP_TCP_RESOURCE_ID: str = "rbt_obstacle_detect";


class ObstacleDetectManager:

    def __init__(self, node: Node) -> None:
        self.__node: Node = node;

        obstacle_detect_subscription_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup();
        self.__obstacle_detect_subscription: Subscription = self.__node.create_subscription(
            msg_type=ObstacleDetect,
            topic=OBSTACLE_DETECT_FROM_MGR_TOPIC_NAME,
            callback_group=obstacle_detect_subscription_cb_group,
            callback=self.__obstacle_detect_subscription_cb,
            qos_profile=qos_profile_system_default
        );

    def __obstacle_detect_subscription_cb(self, obstacle_detect_cb: ObstacleDetect) -> None:
        deserialized_obstacle_detect: Any = message_conversion.extract_values(obstacle_detect_cb);
        self.__node.get_logger().info(f"obstacle_detect_cb : {deserialized_obstacle_detect}");

        rc: int = tcp_send_resource(resource_id=KTP_TCP_RESOURCE_ID, properties=deserialized_obstacle_detect);
        if rc < 0:
            self.__node.get_logger().error(f"Failed to Sending Resource to id : {KTP_TCP_RESOURCE_ID}");
        else:
            self.__node.get_logger().info(f"Succeeded to Sending Resource to id : {KTP_TCP_RESOURCE_ID}");


__all__: list[str] = ["ObstacleDetectManager"];
