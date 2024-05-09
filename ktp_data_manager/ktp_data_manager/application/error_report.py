from rclpy.node import Node;
from rclpy.impl.rcutils_logger import RcutilsLogger;
from rclpy.publisher import Publisher;
from rclpy.subscription import Subscription;
from rclpy.qos import qos_profile_system_default;
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup;
from std_msgs.msg import String;
from ktp_data_msgs.msg import ErrorReport;
from ktp_data_manager.utils import get_current_time;


NOTIFY_ERROR_STATUS_TOPIC: str = "/rms/ktp/data/notify/error/status";
ERROR_REPORT_TO_ITF_TOPIC: str = "/rms/ktp/data/error_report";


class ErrorReportService:
    
    def __init__(self, node: Node) -> None:
        self.__node: Node = node;
        self.__error_report: ErrorReport = None;
        
        notify_error_status_subscription_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup();
        self.__notify_error_status_subscription: Subscription = self.__node.create_subscription(
            topic=NOTIFY_ERROR_STATUS_TOPIC,
            msg_type=String,
            qos_profile=qos_profile_system_default,
            callback_group=notify_error_status_subscription_cb_group,
            callback=self.notify_error_status_subscription_cb
        );
        
    @property
    def error_report(self) -> ErrorReport:
        return self.__error_report;
    
    @error_report.setter
    def error_report(self, error_report: ErrorReport) -> None:
        self.__error_report = error_report;
        
    def notify_error_status_subscription_cb(self, error_report_cb: String) -> None:
        error_report: ErrorReport = ErrorReport();
        error_report.create_time = get_current_time();
        error_report.error_code = error_report_cb.data;
        
        self.__error_report = error_report;


__all__: list[str] = ["ErrorReportService"];