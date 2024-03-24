import rclpy;

from rclpy.node import Node;
from rclpy.publisher import Publisher;
from rclpy.qos import qos_profile_system_default;
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup;
from rosbridge_library.internal import message_conversion;
from typing import Any;

from ktp_data_msgs.msg import DetectedObject;

DETECTED_OBJECT_PUBLISHER_NAME: str = "/rms/ktp/inf/detected_object";


class DetectedObjectManager:

    def __init__(self, node: Node) -> None:
        self.__node: Node = node;

        detected_object_publisher_topic: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup();
        self.__detected_object_publisher: Publisher = self.__node.create_publisher(
            topic=DETECTED_OBJECT_PUBLISHER_NAME,
            msg_type=DetectedObject,
            qos_profile=qos_profile_system_default,
            callback_group=detected_object_publisher_topic
        );

    def deliver_detected_object_callback_json(self, detected_object_callback_json: Any) -> None:
        detected_object: Any = message_conversion.populate_instance(detected_object_callback_json, DetectedObject());
        self.__node.get_logger().info(f"Detected Object Callback From KTP : {detected_object}");
        self.__detected_object_publish(detected_object=detected_object);

    def __detected_object_publish(self, detected_object: Any) -> None:
        self.__node.get_logger().info(f"Detected Object Publish Message : {detected_object}");
        self.__detected_object_publisher.publish(detected_object);


__all__ = ["DetectedObjectManager"];
