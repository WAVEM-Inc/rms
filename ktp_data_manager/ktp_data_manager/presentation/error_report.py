from rclpy.node import Node;
from rclpy.timer import Timer;
from rclpy.publisher import Publisher;
from rclpy.subscription import Subscription;
from rclpy.qos import qos_profile_system_default;
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup;
from std_msgs.msg import String;
from ktp_data_msgs.msg import ErrorReport;
from ktp_data_manager.utils import get_current_time;
from ktp_data_manager.application.error_report import ErrorReportService;


ERROR_REPORT_TO_ITF_RATE: float = 0.7;
ERROR_REPORT_TO_ITF_TOPIC: str = "/rms/ktp/data/error_report";



class ErrorReportController:
    
    def __init__(self, node: Node) -> None:
        self.__node: Node = node;
        self.__error_report_service: ErrorReportService = ErrorReportService(node=self.__node);
        
        error_report_to_itf_publisher_timer_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup();
        self.__error_report_to_itf_publisher_timer: Timer = self.__node.create_timer(
            timer_period_sec=ERROR_REPORT_TO_ITF_RATE,
            callback_group=error_report_to_itf_publisher_timer_cb_group,
            callback=self.error_report_to_itf_publisher_timer_cb
        );
        
        error_report_to_itf_publisher_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup();
        self.__error_report_to_itf_publisher: Publisher = self.__node.create_publisher(
            topic=ERROR_REPORT_TO_ITF_TOPIC,
            msg_type=ErrorReport,
            qos_profile=qos_profile_system_default,
            callback_group=error_report_to_itf_publisher_cb_group
        );
    
    def error_report_to_itf_publisher_timer_cb(self) -> None:
        error_report: ErrorReport = self.__error_report_service.error_report;
        
        if error_report is not None:
            self.__error_report_to_itf_publisher.publish(msg=error_report);
            self.__error_report_service.error_report = None;
        else:
            return;

__all__: list[str] = ["ErrorReportController"];