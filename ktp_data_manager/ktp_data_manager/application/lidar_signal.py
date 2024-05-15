from rclpy.node import Node;
from rclpy.subscription import Subscription;
from rclpy.timer import Timer;
from rclpy.qos import qos_profile_system_default;
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup;
from std_msgs.msg import String;
from ktp_data_msgs.msg import LiDARSignal;
from ktp_data_manager.utils import get_current_time;


OBSTACLE_COOPERATIVE_TOPIC: str = "/drive/obstacle/cooperative";


class LiDARSignalService:
    
    def __init__(self, node: Node) -> None:
        self.__node: Node = node;
        self.__lidar_signal: LiDARSignal = None;
        
        obstacle_cooperative_subscription_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup();
        self.__obstacle_cooperative_subscription: Subscription = self.__node.create_subscription(
            topic=OBSTACLE_COOPERATIVE_TOPIC,
            msg_type=String,
            qos_profile=qos_profile_system_default,
            callback_group=obstacle_cooperative_subscription_cb_group,
            callback=self.obstacle_cooperative_subscription_cb
        );
        
    @property
    def lidar_signal(self) -> LiDARSignal:
        return self.__lidar_signal;
    
    @lidar_signal.setter
    def lidar_signal(self, lidar_signal: LiDARSignal) -> None:
        self.__lidar_signal = lidar_signal;
        
    def obstacle_cooperative_subscription_cb(self, obstacle_cooperative_cb: String) -> None:
        lidar_signal: LiDARSignal = LiDARSignal();
        lidar_signal.create_time = get_current_time();
        lidar_signal.signal_type = obstacle_cooperative_cb.data;
        
        self.lidar_signal = lidar_signal;
        
        
__all__: list[str] = ["LiDARSignalService"];