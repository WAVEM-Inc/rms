from rclpy.node import Node;
from rclpy.publisher import Publisher;
from rclpy.timer import Timer;
from rclpy.qos import qos_profile_system_default;
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup;
from ktp_data_msgs.msg import LiDARSignal;
from ktp_data_manager.application.lidar_signal import LiDARSignalService;


LIDAR_SIGNAL_TO_ITF_RATE: float = 0.7;
LIDAR_SIGNAL_TO_ITF_TOPIC: str = "/rms/ktp/data/lidar_signal";


class LiDARSignalController:
    
    def __init__(self, node: Node) -> None:
        self.__node: Node = node;
        self.__lidar_signal_service: LiDARSignalService = LiDARSignalService(node=self.__node);
        
        lidar_signl_to_itf_publish_timer_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup();
        self.__lidar_signal_to_itf_publish_timer: Timer = self.__node.create_timer(
            timer_period_sec=LIDAR_SIGNAL_TO_ITF_RATE,
            callback_group=lidar_signl_to_itf_publish_timer_cb_group,
            callback=self.lidar_signal_to_itf_publish_timer_cb
        );
        
        lidar_signal_to_itf_publisher_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup();
        self.__lidar_signal_to_itf_publisher: Publisher = self.__node.create_publisher(
            topic=LIDAR_SIGNAL_TO_ITF_TOPIC,
            msg_type=LiDARSignal,
            qos_profile=qos_profile_system_default,
            callback_group=lidar_signal_to_itf_publisher_cb_group
        );
        
    def lidar_signal_to_itf_publish(self, lidar_signal: LiDARSignal) -> None:
        self.__lidar_signal_to_itf_publisher.publish(msg=lidar_signal);
    
    def lidar_signal_to_itf_publish_timer_cb(self) -> None:
        lidar_signal: LiDARSignal = self.__lidar_signal_service.lidar_signal;
        
        if lidar_signal is not None:
            self.__lidar_signal_to_itf_publisher.publish(msg=lidar_signal);
            self.__lidar_signal_service.lidar_signal = None;
        else:
            return;
        
__all__: list[str] = ["LiDARSignalController"];