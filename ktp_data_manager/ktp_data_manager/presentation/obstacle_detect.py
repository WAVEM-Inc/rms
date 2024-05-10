from rclpy.node import Node;
from rclpy.publisher import Publisher;
from rclpy.timer import Timer;
from rclpy.qos import qos_profile_sensor_data;
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup;
from ktp_data_msgs.msg import ObstacleDetect;
from ktp_data_manager.application.obstacle_detect import ObstacleDetectService;


OBSTACLE_DETECT_TO_ITF_RATE: float = 0.55;
OBSTACLE_DETECT_TO_ITF_TOPIC: str = "/rms/ktp/data/obstacle_detect";


class ObstacleDetectController:
    
    def __init__(self, node: Node) -> None:
        self.__node: Node = node;
        self.__obstacle_detect_service: ObstacleDetectService = ObstacleDetectService(node=self.__node);
        
        obstalce_detect_to_itf_publisher_timer_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup();
        self.__obstalce_detect_to_itf_publisher_timer: Timer = self.__node.create_timer(
            timer_period_sec=OBSTACLE_DETECT_TO_ITF_RATE,
            callback_group=obstalce_detect_to_itf_publisher_timer_cb_group,
            callback=self.obstalce_detect_to_itf_publisher_timer_cb
        );
        
        obstacle_detect_to_itf_publisher_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup();
        self.__obstacle_detect_to_itf_publisher: Publisher = self.__node.create_publisher(
            topic=OBSTACLE_DETECT_TO_ITF_TOPIC,
            msg_type=ObstacleDetect,
            qos_profile=qos_profile_sensor_data,
            callback_group=obstacle_detect_to_itf_publisher_cb_group
        );
        
    def obstalce_detect_to_itf_publisher_timer_cb(self) -> None:
        obstacle_detect: ObstacleDetect = self.__obstacle_detect_service.obstacle_detect;
        
        if obstacle_detect is not None:
            self.__obstacle_detect_to_itf_publisher.publish(msg=obstacle_detect);
            self.__obstacle_detect_service.obstacle_detect = None;
        else:
            return;
    
    
__all__: list[str] = ["ObstalceDetectController"];