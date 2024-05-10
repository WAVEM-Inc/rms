from rclpy.node import Node;
from rclpy.publisher import Publisher;
from rclpy.qos import qos_profile_sensor_data;
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup;
from ktp_data_msgs.msg import DetectedObject;


OBJECT_DETECT_TOPIC: str = "/drive/object_detect";


class DetectedObjectService:
    
    def __init__(self, node: Node) -> None:
        self.__node: Node = node;
        
        object_detect_publisher_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup();
        self.__object_detect_publisher: Publisher = self.__node.create_publisher(
            topic=OBJECT_DETECT_TOPIC,
            msg_type=DetectedObject,
            qos_profile=qos_profile_sensor_data,
            callback_group=object_detect_publisher_cb_group
        );
        
    def object_detect_publish(self, detected_object: DetectedObject) -> None:
        self.__object_detect_publisher.publish(msg=detected_object);
    
    
__all__: list[str] = ["DetectedObjectService"];