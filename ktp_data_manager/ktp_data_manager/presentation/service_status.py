from rclpy.node import Node;
from rclpy.timer import Timer;
from rclpy.publisher import Publisher;
from rclpy.qos import qos_profile_system_default;
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup;
from ktp_data_msgs.msg import ServiceStatus;
from ktp_data_manager.application.service_status import ServiceStatusService;


SERVICE_STATUS_TO_ITF_RATE: float = 0.5;
SERVICE_STATUS_TO_ITF_TOPIC: str = "/rms/ktp/data/service_status";


class ServiceStatusController:
    
    def __init__(self, node: Node) -> None:
        self.__node: Node = node;
        self.__service_status_service: ServiceStatusService = ServiceStatusService(node=self.__node);
        
        service_status_to_itf_publisher_timer_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup();
        self.__service_status_to_itf_publisher_timer: Timer = self.__node.create_timer(
            timer_period_sec=SERVICE_STATUS_TO_ITF_RATE,
            callback_group=service_status_to_itf_publisher_timer_cb_group,
            callback=self.service_status_to_itf_publisher_timer_cb
        );

        service_status_to_itf_publisher_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup();
        self.__service_status_to_itf_publisher: Publisher = self.__node.create_publisher(
            topic=SERVICE_STATUS_TO_ITF_TOPIC,
            msg_type=ServiceStatus,
            qos_profile=qos_profile_system_default,
            callback_group=service_status_to_itf_publisher_cb_group
        );
    
    def service_status_to_itf_publisher_timer_cb(self) -> None:
        service_status: ServiceStatus = self.__service_status_service.service_status;
        if service_status is not None:
            self.__service_status_to_itf_publisher.publish(msg=service_status);
            self.__service_status_service.service_status = None;
        else:
            return;

__all__: list[str] = ["ServiceStatusController"];