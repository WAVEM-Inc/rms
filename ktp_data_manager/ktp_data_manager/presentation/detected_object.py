from rclpy.node import Node;
from rclpy.subscription import Subscription;
from rclpy.qos import qos_profile_sensor_data;
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup;
from ktp_data_msgs.msg import DetectedObject;
from ktp_data_manager.application.detected_object import DetectedObjectService;


DETECTED_OBJECT_FROM_ITF_TOPIC: str = "/rms/ktp/itf/detected_object";


class DetectedObjectController:
    
    def __init__(self, node: Node) -> None:
        self.__node: Node = node;
        self.__detected_object_service: DetectedObjectService = DetectedObjectService(node=self.__node);
        
        detected_object_from_itf_subscription_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup();
        self.__detected_object_from_itf_subscription: Subscription = self.__node.create_subscription(
            topic=DETECTED_OBJECT_FROM_ITF_TOPIC,
            msg_type=DetectedObject,
            qos_profile=qos_profile_sensor_data,
            callback_group=detected_object_from_itf_subscription_cb_group,
            callback=self.detected_object_from_itf_subscription_cb
        );
        
    def detected_object_from_itf_subscription_cb(self, detected_object_cb: DetectedObject) -> None:
        self.__detected_object_service.object_detect_publish(detected_object=detected_object_cb);


__all__: list[str] = ["DetectedObjectController"];