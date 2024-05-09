from rclpy.node import Node;
from rclpy.timer import Timer;
from rclpy.publisher import Publisher;
from rclpy.qos import qos_profile_system_default;
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup;
from ktp_data_msgs.msg import Status;
from ktp_data_manager.utils import get_current_time;
from ktp_data_manager.application.rbt_status import RbtStatusService;


RBT_STATUS_TO_ITF_TOPIC: str = "/rms/ktp/data/rbt_status";
RBT_STATUS_TO_ITF_RATE: float = 0.85;


class RbtStatusController:
    
    def __init__(self, node: Node) -> None:
        self.__node: Node = node;
        self.__param_map_id: str = self.__node.get_parameter(name="map_id").get_parameter_value().string_value;
        self.__param_firmware_version: str = self.__node.get_parameter(name="firmware_version").get_parameter_value().string_value;
        
        self.__rbt_status_service: RbtStatusService = RbtStatusService(node=self.__node);
        
        rbt_status_to_itf_publisher_timer_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup();
        self.__rbt_status_to_itf_publisher_timer: Timer = self.__node.create_timer(
            timer_period_sec=RBT_STATUS_TO_ITF_RATE,
            callback_group=rbt_status_to_itf_publisher_timer_cb_group,
            callback=self.rbt_status_to_itf_publisher_timer_cb
        );
        
        rbt_status_to_itf_publisher_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup();
        self.__rbt_status_to_itf_publisher: Publisher = self.__node.create_publisher(
            topic=RBT_STATUS_TO_ITF_TOPIC,
            msg_type=Status,
            qos_profile=qos_profile_system_default,
            callback_group=rbt_status_to_itf_publisher_cb_group
        );
    
    def rbt_status_to_itf_publisher_timer_cb(self) -> None:
        rbt_status: Status = self.__rbt_status_service.rbt_status;
        rbt_status.map_id = self.__param_map_id;
        rbt_status.create_time = get_current_time();
        rbt_status.firmware_version = self.__param_firmware_version;
        self.__rbt_status_to_itf_publisher.publish(msg=rbt_status);


__all__: list[str] = ["RbtStatusController"];