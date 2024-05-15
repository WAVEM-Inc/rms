from rclpy.node import Node;
from rclpy.subscription import Subscription;
from rclpy.qos import qos_profile_system_default;
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup;
from ktp_data_msgs.msg import ServiceStatus;


MISSION_STATUS_FROM_TASK_CTRL_TOPIC: str = "/rms/ktp/task/notify/mission/status";


class ServiceStatusService:
    
    def __init__(self, node: Node) -> None:
        self.__node: Node = node;
        self.__service_status: ServiceStatus = None;
        
        mission_status_subscription_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup();
        self.__mission_status_subscription: Subscription = self.__node.create_subscription(
            topic=MISSION_STATUS_FROM_TASK_CTRL_TOPIC,
            msg_type=ServiceStatus,
            qos_profile=qos_profile_system_default,
            callback_group=mission_status_subscription_cb_group,
            callback=self.mission_status_subscription_cb
        );
        
    @property
    def service_status(self) -> ServiceStatus:
        return self.__service_status;
    
    @service_status.setter
    def service_status(self, service_status: ServiceStatus) -> None:
        self.__service_status = service_status;
        
    def mission_status_subscription_cb(self, mission_status_cb: ServiceStatus) -> None:
        self.__service_status = mission_status_cb;
    

__all__: list[str] = ["ServiceStatusService"];