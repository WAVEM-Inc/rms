from rclpy.node import Node;
from rclpy.impl.rcutils_logger import RcutilsLogger;
from rclpy.publisher import Publisher;
from rclpy.subscription import Subscription;
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup;
from rclpy.qos import qos_profile_system_default;
from ktp_data_msgs.msg import ObstacleDetect;
import obstacle_msgs.msg as obstacle;
from ktp_task_controller.utils import get_current_time;


NOTIFY_OBSTACLE_DETECT_TOPIC_NAME: str = "/rms/ktp/task/notify/obstacle_detect";
DRIVE_OBSTACLE_TOPIC: str = "/drive/obstacle/event";


class ObstacleService:
    
    def __init__(self, node: Node) -> None:
        self.__node: Node = node;
        self.__log: RcutilsLogger = self.__node.get_logger();
        
        obstacle_status_subscription_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup();
        self.__obstacle_status_subscription: Subscription = self.__node.create_subscription(
            topic=DRIVE_OBSTACLE_TOPIC,
            msg_type=obstacle.Status,
            qos_profile=qos_profile_system_default,
            callback_group=obstacle_status_subscription_cb_group,
            callback=self.obstacle_status_subscription_cb
        );

        obstacle_detect_publisher_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup();
        self.__obstacle_detect_publisher: Publisher = self.__node.create_publisher(
            topic=NOTIFY_OBSTACLE_DETECT_TOPIC_NAME,
            msg_type=ObstacleDetect,
            qos_profile=qos_profile_system_default,
            callback_group=obstacle_detect_publisher_cb_group
        );
    
    def obstacle_status_subscription_cb(self, obstacle_status_cb: obstacle.Status) -> None:
        obstacle_detect: ObstacleDetect = ObstacleDetect();
        obstacle_detect.create_time = get_current_time();
        obstacle_detect.event_type = "STOP";
        obstacle_detect.object_id = obstacle_status_cb.obstacle_id;

        obstacle_status: int = obstacle_status_cb.obstacle_status;
        
        if obstacle_status == 0:
            obstacle_detect.event_reason_code = "ahead_detect";
            obstacle_detect.is_cooperative = False;
        elif obstacle_status_cb.obstacle_status == 1:
            obstacle_detect.event_reason_code = "ambient_detect";
            obstacle_detect.is_cooperative = False;
        elif obstacle_status_cb.obstacle_status == 2:
            obstacle_detect.event_reason_code = "cooperative_detect";
            obstacle_detect.is_cooperative = True;
        else:
            obstacle_detect.event_reason_code = "";
            obstacle_detect.is_cooperative = False;

        self.__obstacle_detect_publisher.publish(msg=obstacle_detect);
    
__all__: list[str] = ["ObstacleService"];