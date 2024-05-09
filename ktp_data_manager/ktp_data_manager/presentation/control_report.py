from rclpy.node import Node;
from rclpy.impl.rcutils_logger import RcutilsLogger;
from rclpy.timer import Timer;
from rclpy.publisher import Publisher;
from rclpy.subscription import Subscription;
from rclpy.qos import qos_profile_system_default;
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup;
from ktp_data_msgs.msg import ControlReport;
from ktp_data_manager.application.control_report import ControlReportService;


CONTROL_REPORT_TO_ITF_RATE: float = 0.5;
CONTROL_REPORT_TO_ITF_TOPIC: str = "/rms/ktp/data/control_report";


class ControlReportController:
    
    def __init__(self, node: Node) -> None:
        self.__node: Node = node;
        
        self.__control_report_service: ControlReportService = ControlReportService(node=self.__node);
        
        control_report_to_itf_publisher_timer_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup();
        self.__control_report_to_itf_publisher_timer: Timer = self.__node.create_timer(
            timer_period_sec=CONTROL_REPORT_TO_ITF_RATE,
            callback_group=control_report_to_itf_publisher_timer_cb_group,
            callback=self.control_report_to_itf_publisher_timer_cb
        );
        
        control_report_to_itf_publisher_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup();
        self.__control_report_to_itf_publisher: Publisher = self.__node.create_publisher(
            topic=CONTROL_REPORT_TO_ITF_TOPIC,
            msg_type=ControlReport,
            qos_profile=qos_profile_system_default,
            callback_group=control_report_to_itf_publisher_cb_group
        );
    
    def control_report_to_itf_publisher_timer_cb(self) -> None:
        control_report: ControlReport = self.__control_report_service.control_report;
        
        if control_report is not None:
            self.__control_report_to_itf_publisher.publish(msg=control_report);
            self.__control_report_service.control_report = None;
        else:
            return;


__all__: list[str] = ["ControlReportController"];