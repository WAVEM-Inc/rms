from rclpy.node import Node;
from rclpy.subscription import Subscription;
from rclpy.qos import qos_profile_sensor_data;
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup;
from ktp_data_msgs.msg import ObstacleDetect;
from ktp_data_manager.utils import get_current_time;


OBSTACLE_DETECT_TOPIC: str = "/rms/ktp/task/notify/obstacle_detect";


class ObstacleDetectService:
    
    def __init__(self, node: Node) -> None:
        self.__node: Node = node;
        self.__obstacle_detect: ObstacleDetect = None;
        
        obstacle_detect_subscription_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup();
        self.__obstacle_detect_subscription: Subscription = self.__node.create_subscription(
            topic=OBSTACLE_DETECT_TOPIC,
            msg_type=ObstacleDetect,
            qos_profile=qos_profile_sensor_data,
            callback_group=obstacle_detect_subscription_cb_group,
            callback=self.obstacle_detect_subscription_cb
        );
        
    @property
    def obstacle_detect(self) -> ObstacleDetect:
        return self.__obstacle_detect;
    
    @obstacle_detect.setter
    def obstacle_detect(self, obstacle_detect: ObstacleDetect) -> None:
        self.__obstacle_detect = obstacle_detect;
    
    def obstacle_detect_subscription_cb(self, obstacle_detect_cb: ObstacleDetect) -> None:
        obstacle_detect: ObstacleDetect = obstacle_detect_cb;
        obstacle_detect.create_time = get_current_time();
        self.obstacle_detect = obstacle_detect;
        
        
__all__: list[str] = ["ObstacleDetectService"];