import json;
from rclpy.node import Node;
from rclpy.publisher import Publisher;
from rclpy.qos import qos_profile_system_default;
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup;
from rosbridge_library.internal import message_conversion;
from typing import Any;
from ktp_data_msgs.msg import DetectedObject;
from ktp_interface.tcp.application.service import detected_object_callback_flag;


DETECTED_OBJECT_PUBLISHER_NAME: str = "/rms/ktp/itf/detected_object";


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
        try:
            self.__node.get_logger().info(
                f"Detected Object Callback From KTP : {json.dumps(obj=detected_object_callback_json, indent=4)}");
            detected_object: DetectedObject = message_conversion.populate_instance(msg=detected_object_callback_json, inst=DetectedObject());
            self.__detected_object_publish(detected_object=detected_object);
        except message_conversion.NonexistentFieldException as nefe:
            self.__node.get_logger().error(f"DetectedObject Callback : {nefe}");
            return;

    def __detected_object_publish(self, detected_object: DetectedObject) -> None:
        self.__node.get_logger().info(f"Detected Object Publish Message : {detected_object}");
        self.__detected_object_publisher.publish(msg=detected_object);


__all__: list[str] = ["DetectedObjectManager"];
